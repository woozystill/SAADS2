#!/usr/bin/env python3
"""Full localization analysis - every output on one sheet.

Six panels covering what the stack produces and how well:

  1  Tracks        raw GPS, local EKF and global EKF on one set of axes.
                   The global EKF should follow GPS but smoother; the
                   local EKF has no absolute reference and will drift
                   away, which is the point of having two.

  2  Heading       IMU yaw against GPS course over ground. Course comes
                   from position change, so it is immune to magnetic
                   interference and is the only honest reference for
                   heading on a moving vehicle.

  3  Offset        that difference against heading. Flat means a
                   constant mounting rotation, correctable with one
                   number in the static transform. A sinusoid means
                   soft-iron distortion, which no parameter fixes.

  4  Uncertainty   GPS reported, global EKF, local EKF. The sawtooth on
                   the global EKF is it predicting between fixes and
                   being corrected by each one.

  5  Speed         from GPS, from the local EKF. Divergence means the
                   velocity inputs disagree with reality.

  6  Rates         publish rate of every topic over the run, which
                   catches dropouts a summary average would hide.

Usage:
    python3 analyse_localization.py <bag>
    python3 analyse_localization.py <bag> --min-move 5

Requires: pip3 install matplotlib rosbags numpy
"""

import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from _paths import out, graph, find_bag, stem

DPI = 150
R_EARTH = 6378137.0

C_GPS = '#1D7A5F'
C_GLOBAL = '#7C5CBF'
C_LOCAL = '#B0741A'
C_IMU = '#2E5C9A'
C_WARN = '#C0392B'

plt.rcParams.update({
    'font.size': 9.5, 'axes.titlesize': 11, 'axes.labelsize': 10,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def load(bag_path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    gps, glob, loc, imu = [], [], [], []
    wanted = {'/gps/fix', '/odometry/global', '/odometry/local', '/imu/data'}

    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(f'Nothing usable. Bag has: {found}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

            if conn.topic == '/gps/fix':
                if m.status.status < 0 or math.isnan(m.latitude):
                    continue
                gps.append({'t': t, 'lat': m.latitude, 'lon': m.longitude,
                            'var': m.position_covariance[0]})
            elif conn.topic == '/imu/data':
                o = m.orientation
                imu.append({
                    't': t,
                    'yaw': math.degrees(math.atan2(
                        2 * (o.w * o.z + o.x * o.y),
                        1 - 2 * (o.y * o.y + o.z * o.z))) % 360.0,
                    'yaw_var': m.orientation_covariance[8],
                })
            else:
                p = m.pose.pose.position
                rec = {'t': t, 'x': p.x, 'y': p.y,
                       'var': m.pose.covariance[0],
                       'vx': m.twist.twist.linear.x}
                (glob if conn.topic == '/odometry/global' else loc).append(rec)

    return gps, glob, loc, imu


def to_metres(gps):
    lat0, lon0 = gps[0]['lat'], gps[0]['lon']
    k = math.cos(math.radians(lat0))
    for f in gps:
        f['x'] = math.radians(f['lon'] - lon0) * R_EARTH * k
        f['y'] = math.radians(f['lat'] - lat0) * R_EARTH
    return gps


def segments(gps, min_move):
    """Split the GPS track into legs of at least min_move metres.

    Course over a short baseline is dominated by position noise, so the
    baseline has to be long enough that real movement outweighs it.
    """
    segs, anchor = [], gps[0]
    for f in gps[1:]:
        dx, dy = f['x'] - anchor['x'], f['y'] - anchor['y']
        d = math.hypot(dx, dy)
        if d >= min_move:
            segs.append({
                't': (anchor['t'] + f['t']) / 2.0,
                'course': math.degrees(math.atan2(dy, dx)) % 360.0,
                'dist': d,
            })
            anchor = f
    return segs


def nearest_yaw(imu, t):
    if not imu:
        return None
    i = min(range(len(imu)), key=lambda k: abs(imu[k]['t'] - t))
    return imu[i]['yaw'] if abs(imu[i]['t'] - t) < 1.0 else None


def ang_diff(a, b):
    return (a - b + 540.0) % 360.0 - 180.0


def rate_series(rows, window=20):
    """Instantaneous publish rate, so dropouts show as dips."""
    if len(rows) < window + 1:
        return [], []
    t, hz = [], []
    for i in range(window, len(rows)):
        span = rows[i]['t'] - rows[i - window]['t']
        if span > 0:
            t.append(rows[i]['t'])
            hz.append(window / span)
    return t, hz


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag> [--min-move M]')

    bag = find_bag(sys.argv[1])
    name = stem(bag)
    min_move = 5.0
    if '--min-move' in sys.argv:
        min_move = float(sys.argv[sys.argv.index('--min-move') + 1])

    gps, glob, loc, imu = load(bag)
    print(f'{len(gps)} GPS fixes, {len(glob)} global, {len(loc)} local, '
          f'{len(imu)} IMU')
    if not gps:
        raise SystemExit('No valid GPS fixes - was this recorded outdoors?')
    gps = to_metres(gps)

    t0 = min([r['t'] for r in gps] + [r['t'] for r in (glob or gps)])
    rel = lambda rows: np.array([r['t'] - t0 for r in rows])

    fig = plt.figure(figsize=(16, 10))

    # ---- 1. tracks ----
    ax = fig.add_subplot(2, 3, 1)
    ax.plot([f['x'] for f in gps], [f['y'] for f in gps], '.-',
            color=C_GPS, markersize=2, linewidth=0.7, label='raw GPS')
    if glob:
        ax.plot([r['x'] for r in glob], [r['y'] for r in glob],
                color=C_GLOBAL, linewidth=1.3, label='global EKF')
    if loc:
        ax.plot([r['x'] for r in loc], [r['y'] for r in loc],
                color=C_LOCAL, linewidth=1.1, alpha=0.8,
                label='local EKF (no absolute ref)')
    ax.plot(gps[0]['x'], gps[0]['y'], 'o', color='#1D7A5F', markersize=10,
            label='start')
    ax.plot(gps[-1]['x'], gps[-1]['y'], 's', color=C_WARN, markersize=9,
            label='end')
    dist = sum(math.hypot(gps[i]['x'] - gps[i-1]['x'],
                          gps[i]['y'] - gps[i-1]['y'])
               for i in range(1, len(gps)))
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.set_title(f'1. Tracks, {dist:.0f} m travelled')
    ax.set_aspect('equal')
    ax.legend(fontsize=8)

    # ---- 2. heading ----
    segs = segments(gps, min_move)
    ax = fig.add_subplot(2, 3, 2)
    if segs:
        ax.plot([s['t'] - t0 for s in segs], [s['course'] for s in segs],
                'o-', color=C_GPS, markersize=3.5, linewidth=0.9,
                label='GPS course')
    if imu:
        ax.plot(rel(imu), [r['yaw'] for r in imu], color=C_IMU,
                linewidth=0.9, alpha=0.8, label='IMU yaw')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('heading (deg)')
    ax.set_title('2. Heading - IMU vs GPS course')
    ax.legend(fontsize=8)

    # ---- 3. offset vs heading ----
    ax = fig.add_subplot(2, 3, 3)
    offs, courses = [], []
    for s in segs:
        y = nearest_yaw(imu, s['t'])
        if y is None:
            continue
        offs.append(ang_diff(y, s['course']))
        courses.append(s['course'])
    if offs:
        offs_a, courses_a = np.array(offs), np.array(courses)
        ax.scatter(courses_a, offs_a, s=28, color=C_GPS, alpha=0.75,
                   label='measured')
        mean_off = float(np.mean(offs_a))
        ax.axhline(mean_off, color=C_WARN, linestyle='--', linewidth=1.3,
                   label=f'constant {mean_off:+.1f} deg')
        rms = float(np.sqrt(np.mean((offs_a - mean_off) ** 2)))
        ax.set_title(f'3. Offset vs heading, rms {rms:.1f} deg')
    else:
        mean_off, rms = float('nan'), float('nan')
        ax.set_title('3. Offset vs heading - no usable segments')
    ax.set_xlabel('GPS course (deg)')
    ax.set_ylabel('IMU yaw - GPS course (deg)')
    ax.set_xlim(0, 360)
    ax.legend(fontsize=8)

    # ---- 4. uncertainty ----
    ax = fig.add_subplot(2, 3, 4)
    ax.plot(rel(gps), np.sqrt([f['var'] for f in gps]), '.-',
            color=C_GPS, markersize=2, linewidth=0.7,
            label='GPS reported')
    if glob:
        ax.plot(rel(glob), np.sqrt([r['var'] for r in glob]),
                color=C_GLOBAL, linewidth=1.1, label='global EKF')
    if loc:
        ax.plot(rel(loc), np.sqrt([r['var'] for r in loc]),
                color=C_LOCAL, linewidth=1.0, alpha=0.8, label='local EKF')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position 1$\\sigma$ (m)')
    ax.set_title('4. Uncertainty - sawtooth is predict then correct')
    ax.set_yscale('log')
    ax.legend(fontsize=8)

    # ---- 5. speed ----
    ax = fig.add_subplot(2, 3, 5)
    if len(gps) > 2:
        gt, gv = [], []
        for i in range(1, len(gps)):
            dt = gps[i]['t'] - gps[i-1]['t']
            if dt <= 0:
                continue
            d = math.hypot(gps[i]['x'] - gps[i-1]['x'],
                           gps[i]['y'] - gps[i-1]['y'])
            gt.append(gps[i]['t'] - t0)
            gv.append(d / dt)
        ax.plot(gt, gv, '.', color=C_GPS, markersize=2.5, alpha=0.5,
                label='from GPS positions')
    if loc:
        ax.plot(rel(loc), [r['vx'] for r in loc], color=C_LOCAL,
                linewidth=1.1, label='local EKF vx')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('speed (m/s)')
    ax.set_title('5. Speed')
    ax.legend(fontsize=8)

    # ---- 6. rates ----
    ax = fig.add_subplot(2, 3, 6)
    for rows, lab, col in ((gps, 'GPS', C_GPS), (imu, 'IMU', C_IMU),
                           (glob, 'global EKF', C_GLOBAL),
                           (loc, 'local EKF', C_LOCAL)):
        rt, rh = rate_series(rows)
        if rt:
            ax.plot(np.array(rt) - t0, rh, linewidth=1.0, color=col,
                    label=lab)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('Hz')
    ax.set_title('6. Publish rates - dips are dropouts')
    ax.set_yscale('log')
    ax.legend(fontsize=8)

    fig.savefig(graph(f'{name}_localization.png'), dpi=DPI)

    # ---------------- report ----------------
    def closure(rows):
        if not rows:
            return float('nan')
        return math.hypot(rows[-1]['x'] - rows[0]['x'],
                          rows[-1]['y'] - rows[0]['y'])

    dur = gps[-1]['t'] - gps[0]['t']
    gps_sig = np.sqrt([f['var'] for f in gps])

    lines = [
        f'bag                   {name}',
        f'duration              {dur:.1f} s',
        f'distance travelled    {dist:.1f} m',
        '',
        'RATES',
        f'  GPS                 {len(gps)/dur:.2f} Hz   ({len(gps)} fixes)',
    ]
    if imu:
        lines.append(f'  IMU                 {len(imu)/dur:.1f} Hz')
    if glob:
        lines.append(f'  global EKF          {len(glob)/dur:.1f} Hz')
    if loc:
        lines.append(f'  local EKF           {len(loc)/dur:.1f} Hz')

    lines += ['', 'LOOP CLOSURE   (meaningful only if you returned to start)',
              f'  raw GPS             {closure(gps):.2f} m']
    if glob:
        lines.append(f'  global EKF          {closure(glob):.2f} m')
        if closure(glob) < closure(gps):
            lines.append('  -> fused estimate closed tighter than raw GPS')
    if loc:
        lines.append(f'  local EKF           {closure(loc):.2f} m'
                     '   (expected to drift - no absolute reference)')

    lines += ['', 'UNCERTAINTY',
              f'  GPS reported        {np.mean(gps_sig):.2f} m mean, '
              f'{np.min(gps_sig):.2f} - {np.max(gps_sig):.2f}']
    if glob:
        gs = np.sqrt([r['var'] for r in glob])
        lines.append(f'  global EKF          {np.mean(gs):.2f} m mean, '
                     f'{np.min(gs):.2f} - {np.max(gs):.2f}')
        if np.mean(gs) < np.mean(gps_sig):
            lines.append('  -> filter is more confident than its own input, '
                         'which is what fusing is for')

    lines += ['', 'HEADING']
    if offs:
        lines += [
            f'  segments used       {len(offs)}  (min move {min_move} m)',
            f'  mean offset         {mean_off:+.2f} deg',
            f'  rms about that      {rms:.2f} deg',
            '',
        ]
        if rms < 5:
            lines += [
                '  CONSTANT OFFSET. The error does not depend on heading, so',
                '  it is a fixed mounting rotation plus declination. Put it',
                '  in the base_link -> imu_link static transform as a yaw',
                '  rotation - NOT in navsat_transform yaw_offset, which was',
                '  tested and rotates the map frame by its own value',
                '  regardless of the measured offset.',
            ]
        else:
            lines += [
                f'  rms {rms:.1f} deg is high. Either the offset varies with',
                '  heading - soft iron, which no single parameter corrects -',
                '  or segments spanning turns are inflating it. Check panel 3',
                '  for a sinusoid: a wave means distortion, scatter means',
                '  turning segments.',
            ]
    else:
        lines.append('  no usable segments - too little movement, or '
                     'min-move set too high')

    report = '\n'.join(lines)
    print()
    print(report)
    with open(out(f'{name}_localization.txt'), 'w') as fh:
        fh.write(report + '\n')
    print(f'\n  wrote results/graphs/{name}_localization.png and .txt')


if __name__ == '__main__':
    main()
