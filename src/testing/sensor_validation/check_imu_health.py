#!/usr/bin/env python3
"""What did the BNO085 think of itself during a run?

Three questions, one pass over the bag:

  1. Did the chip's per-sensor calibration accuracy change mid-run?
     Read from /imu/diagnostics. A magnetometer level that drops to
     0 or 1 means the chip stopped trusting mag, and the rotation
     vector degenerates toward free-running gyro yaw.

  2. Did orientation_covariance[8] actually adapt, or was it a
     constant the driver wrote once?

  3. How much yaw did the IMU accumulate, unwrapped, versus how much
     the robot physically turned? A closed loop must come back to
     where it started. Anything else is drift.

Usage:
    python3 check_imu_health.py <bag>

Requires: pip3 install rosbags numpy
"""

import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

_PKG = Path.home() / 'saads_ws' / 'src' / 'testing' / 'sensor_validation'
SEARCH_ROOTS = [
    _PKG / 'results' / 'bags',
    Path.cwd() / 'results' / 'bags',
    Path.cwd(),
    Path.home() / 'Desktop',
    Path.home(),
]

IMU_TOPIC = '/imu/data'
DIAG_TOPIC = '/imu/diagnostics'
GPS_TOPIC = '/gps/fix'


def find_bag(spec):
    p = Path(spec).expanduser()
    if p.is_dir() and list(p.glob('*.db3')):
        return p
    for root in SEARCH_ROOTS:
        if not root.is_dir():
            continue
        direct = root / spec
        if direct.is_dir() and list(direct.glob('*.db3')):
            return direct
    raise SystemExit(f'Could not find a bag called "{spec}".')


def quat_to_yaw(x, y, z, w):
    return math.degrees(math.atan2(2 * (w * z + x * y),
                                   1 - 2 * (y * y + z * z)))


def latlon_to_local(lat, lon, lat0, lon0):
    R = 6378137.0
    return (math.radians(lon - lon0) * R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * R)


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag>')

    bag = find_bag(sys.argv[1])
    ts = get_typestore(Stores.ROS2_HUMBLE)

    diag, imu, gps = [], [], []
    with AnyReader([bag], default_typestore=ts) as reader:
        wanted = {IMU_TOPIC, DIAG_TOPIC, GPS_TOPIC}
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(f'Nothing useful in bag. Found: {found}')

        for conn, stamp, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)

            if conn.topic == DIAG_TOPIC:
                t = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                     if hasattr(m, 'header') else stamp * 1e-9)
                fields = {}
                for st in m.status:
                    for kv in st.values:
                        fields[f'{st.name}/{kv.key}' if st.name else kv.key] \
                            = kv.value
                    fields.setdefault(f'{st.name}:level', str(st.level))
                diag.append({'t': t, 'fields': fields})

            elif conn.topic == IMU_TOPIC:
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                o = m.orientation
                imu.append({'t': t,
                            'yaw': quat_to_yaw(o.x, o.y, o.z, o.w),
                            'cov8': m.orientation_covariance[8],
                            'gz': m.angular_velocity.z})

            else:
                if m.status.status < 0 or math.isnan(m.latitude):
                    continue
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                gps.append({'t': t, 'lat': m.latitude, 'lon': m.longitude})

    print(f'bag                {bag}')
    print(f'diagnostics        {len(diag)}')
    print(f'imu                {len(imu)}')
    print(f'gps                {len(gps)}')

    t0 = min([r['t'] for r in (diag + imu + gps)]) if (diag or imu or gps) \
        else 0.0

    # ------------------------------------------------------------------
    # 1. calibration accuracy change log
    # ------------------------------------------------------------------
    print('\n' + '=' * 62)
    print('1. CALIBRATION ACCURACY - change log')
    print('=' * 62)
    if not diag:
        print('  no /imu/diagnostics in this bag')
    else:
        print('  keys present: ' + ', '.join(sorted(diag[0]['fields'])))
        print()
        previous = {}
        changes = 0
        for rec in diag:
            for key, val in rec['fields'].items():
                if previous.get(key) != val:
                    if key in previous:
                        print(f'  t={rec["t"] - t0:7.1f}s   {key:<34} '
                              f'{previous[key]} -> {val}')
                        changes += 1
                    else:
                        print(f'  t={rec["t"] - t0:7.1f}s   {key:<34} '
                              f'= {val}   (first seen)')
                    previous[key] = val
        print(f'\n  {changes} change(s) after the initial values.')
        if changes == 0:
            print('  -> Every level held steady. The chip did not report '
                  'losing confidence\n     in any sensor during the run.')

    # ------------------------------------------------------------------
    # 2. did the covariance adapt
    # ------------------------------------------------------------------
    print('\n' + '=' * 62)
    print('2. ORIENTATION COVARIANCE [8] - did it adapt?')
    print('=' * 62)
    if not imu:
        print('  no /imu/data')
    else:
        cov = np.array([r['cov8'] for r in imu])
        uniq = np.unique(np.round(cov, 12))
        sig = np.degrees(np.sqrt(cov))
        print(f'  samples            {len(cov)}')
        print(f'  distinct values    {len(uniq)}')
        print(f'  variance           {cov.min():.6e} to {cov.max():.6e} rad^2')
        print(f'  as 1 sigma         {sig.min():.2f} to {sig.max():.2f} deg')
        if len(uniq) <= 2:
            print('\n  -> CONSTANT. The adaptive path did not run. The EKF was '
                  'given a\n     fixed heading confidence regardless of what '
                  'the chip actually knew.')
        else:
            print(f'\n  -> Adapting across {len(uniq)} distinct values. '
                  'The chip drove this.')

    # ------------------------------------------------------------------
    # 3. yaw accumulation over a closed loop
    # ------------------------------------------------------------------
    print('\n' + '=' * 62)
    print('3. YAW ACCUMULATION - unwrapped')
    print('=' * 62)
    if len(imu) < 10:
        print('  not enough IMU data')
    else:
        yaw = np.unwrap(np.radians([r['yaw'] for r in imu]))
        total_turn = math.degrees(yaw[-1] - yaw[0])
        gz = np.array([r['gz'] for r in imu])
        t = np.array([r['t'] for r in imu])
        gyro_integral = math.degrees(np.trapezoid(gz, t)) \
            if hasattr(np, 'trapezoid') else math.degrees(np.trapz(gz, t))

        print(f'  fused yaw, start to end    {total_turn:+10.1f} deg')
        print(f'  raw gyro z integrated      {gyro_integral:+10.1f} deg')
        print(f'  difference                 {total_turn - gyro_integral:+10.1f}'
              f' deg   (what the fusion corrected)')

        if gps and len(gps) > 10:
            lat0, lon0 = gps[0]['lat'], gps[0]['lon']
            pts = [latlon_to_local(g['lat'], g['lon'], lat0, lon0)
                   for g in gps]
            closure = math.hypot(pts[-1][0] - pts[0][0],
                                 pts[-1][1] - pts[0][1])
            course = np.unwrap([math.atan2(pts[i + 1][1] - pts[i][1],
                                           pts[i + 1][0] - pts[i][0])
                                for i in range(len(pts) - 1)
                                if math.hypot(pts[i + 1][0] - pts[i][0],
                                              pts[i + 1][1] - pts[i][1]) > 1.0])
            if len(course) > 2:
                gps_turn = math.degrees(course[-1] - course[0])
                print(f'  GPS course, start to end   {gps_turn:+10.1f} deg')
                print(f'  loop closure               {closure:10.2f} m')
                err = total_turn - gps_turn
                print(f'\n  IMU gained {err:+.1f} deg over the loop.')
                if closure < 5.0 and abs(err) > 20.0:
                    print('  -> The robot physically returned to its start, so '
                          'the true net\n     rotation is a multiple of 360. '
                          f'{abs(err):.0f} deg of unexplained yaw is '
                          'DRIFT,\n     not a mounting offset and not soft '
                          'iron.')
                elif abs(err) <= 20.0:
                    print('  -> Within 20 deg. Yaw tracked the physical '
                          'rotation.')

    print()


if __name__ == '__main__':
    main()
