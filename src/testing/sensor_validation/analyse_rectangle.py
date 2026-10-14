#!/usr/bin/env python3
"""Analyse a closed-loop walk: loop closure and heading-offset behaviour.

A rectangle is a stronger test than two separate straight lines, because
every heading is captured in ONE recording with the sensor never moved or
re-mounted. That removes the confound where an offset difference between
sessions is really just a difference in how the sensor was carried.

The decisive plot is offset against heading:

  - a FLAT line means the offset is a single constant - mounting rotation
    plus magnetic declination - and one yaw_offset parameter corrects it
  - a SINUSOID means soft-iron distortion or interference, where the error
    depends on which way you face, and no single parameter corrects it

Usage:
    python3 analyse_rectangle.py <bag_directory>
    python3 analyse_rectangle.py <bag_directory> --min-move 5

Requires: pip3 install rosbags matplotlib
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

DPI = 160

plt.rcParams.update({
    'font.size': 12, 'axes.titlesize': 14, 'axes.labelsize': 12,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def quat_to_yaw(x, y, z, w):
    return math.degrees(math.atan2(2 * (w * z + x * y),
                                   1 - 2 * (y * y + z * z))) % 360.0


def angle_diff(a, b):
    return (a - b + 540.0) % 360.0 - 180.0


def latlon_to_local(lat, lon, lat0, lon0):
    R = 6378137.0
    return (math.radians(lon - lon0) * R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * R)


def load(bag_path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    gps, imu, odom = [], [], []
    wanted = {'/gps/fix', '/imu/data', '/odometry/global'}
    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(f'Nothing usable. Found: {found}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == '/gps/fix':
                if m.status.status < 0 or math.isnan(m.latitude):
                    continue
                gps.append({'t': t, 'lat': m.latitude, 'lon': m.longitude,
                            'cov': m.position_covariance[0]})
            elif conn.topic == '/imu/data':
                o = m.orientation
                imu.append({'t': t, 'yaw': quat_to_yaw(o.x, o.y, o.z, o.w)})
            else:
                p = m.pose.pose.position
                odom.append({'t': t, 'x': p.x, 'y': p.y})
    return gps, imu, odom


def imu_yaw_at(imu, t):
    best = min(imu, key=lambda r: abs(r['t'] - t))
    return best['yaw'] if abs(best['t'] - t) < 0.5 else None


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag_directory> [--min-move M]')

    bag = sys.argv[1]
    min_move = 5.0
    if '--min-move' in sys.argv:
        min_move = float(sys.argv[sys.argv.index('--min-move') + 1])

    bag = find_bag(bag)
    name = stem(bag)
    gps, imu, odom = load(bag)
    print(f'{len(gps)} GPS fixes, {len(imu)} IMU samples, '
          f'{len(odom)} odometry samples')
    if len(gps) < 10 or len(imu) < 10:
        raise SystemExit('Not enough data.')

    lat0, lon0 = gps[0]['lat'], gps[0]['lon']
    for g in gps:
        g['x'], g['y'] = latlon_to_local(g['lat'], g['lon'], lat0, lon0)

    # Course over ground, with a baseline long enough that GPS position
    # noise does not dominate the direction estimate.
    segs = []
    anchor = gps[0]
    for g in gps[1:]:
        dx, dy = g['x'] - anchor['x'], g['y'] - anchor['y']
        dist = math.hypot(dx, dy)
        if dist < min_move:
            continue
        course = math.degrees(math.atan2(dy, dx)) % 360.0
        yaw = imu_yaw_at(imu, g['t'])
        if yaw is not None:
            segs.append({'t': g['t'] - gps[0]['t'], 'course': course,
                         'yaw': yaw, 'offset': angle_diff(yaw, course),
                         'dist': dist})
        anchor = g

    if len(segs) < 4:
        raise SystemExit(
            f'Only {len(segs)} segments at --min-move {min_move}. '
            f'Try a smaller value.')

    # Segments that span a corner are not measurements of anything: the
    # GPS course averages over the whole turn while the IMU yaw is an
    # instant reading, so the two legitimately disagree. Keep only
    # segments whose course matches at least one neighbour, which is
    # true on a straight and false through a turn.
    def on_straight(i):
        c = segs[i]['course']
        near = []
        if i > 0:
            near.append(abs(angle_diff(c, segs[i - 1]['course'])))
        if i < len(segs) - 1:
            near.append(abs(angle_diff(c, segs[i + 1]['course'])))
        return any(d < 25.0 for d in near)

    straight = [s for i, s in enumerate(segs) if on_straight(i)]
    n_corner = len(segs) - len(straight)
    if len(straight) >= 4:
        segs_fit = straight
    else:
        segs_fit = segs
        n_corner = 0

    offsets = np.array([s['offset'] for s in segs_fit])
    courses = np.array([s['course'] for s in segs_fit])

    ref = offsets[0]
    mean_off = ref + np.mean([angle_diff(o, ref) for o in offsets])
    mean_off = (mean_off + 180) % 360 - 180
    resid = np.array([angle_diff(o, mean_off) for o in offsets])
    const_rms = float(np.sqrt(np.mean(resid ** 2)))

    # A heading-dependent (soft-iron) error appears as a once-per-revolution
    # sinusoid in offset vs heading. Fit one and compare how much better it
    # explains the data than a flat line.
    c = np.radians(courses)
    A = np.column_stack([np.ones_like(c), np.cos(c), np.sin(c)])
    coef, *_ = np.linalg.lstsq(A, offsets, rcond=None)
    sin_fit = A @ coef
    sin_rms = float(np.sqrt(np.mean((offsets - sin_fit) ** 2)))
    sin_amp = float(math.hypot(coef[1], coef[2]))

    # Loop closure from GPS.
    close_gps = math.hypot(gps[-1]['x'] - gps[0]['x'],
                           gps[-1]['y'] - gps[0]['y'])
    perimeter = sum(math.hypot(gps[i]['x'] - gps[i - 1]['x'],
                               gps[i]['y'] - gps[i - 1]['y'])
                    for i in range(1, len(gps)))

    close_odom = None
    if len(odom) > 10:
        close_odom = math.hypot(odom[-1]['x'] - odom[0]['x'],
                                odom[-1]['y'] - odom[0]['y'])

    # ---------------- figure ----------------
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 5))

    xs = [g['x'] for g in gps]
    ys = [g['y'] for g in gps]
    ax1.plot(xs, ys, '.-', color='#1D9E75', markersize=4, linewidth=1,
             label='GPS')
    if len(odom) > 10:
        ox = [o['x'] - odom[0]['x'] for o in odom]
        oy = [o['y'] - odom[0]['y'] for o in odom]
        ax1.plot(ox, oy, '-', color='#7C5CBF', linewidth=1.2, alpha=0.8,
                 label='EKF global')
    ax1.plot(xs[0], ys[0], 'o', color='#2E7D32', markersize=11, label='start')
    ax1.plot(xs[-1], ys[-1], 's', color='#C0392B', markersize=9, label='end')
    ax1.set_xlabel('east (m)')
    ax1.set_ylabel('north (m)')
    ax1.set_title(f'Track, {perimeter:.0f} m walked')
    ax1.legend(fontsize=9)
    ax1.set_aspect('equal', adjustable='datalim')

    order = np.argsort(courses)
    ax2.plot(courses[order], offsets[order], 'o', color='#1D9E75',
             markersize=7, label='measured')
    ax2.axhline(mean_off, color='#C0392B', linestyle='--',
                label=f'constant {mean_off:+.1f} deg')
    grid = np.linspace(0, 360, 200)
    gr = np.radians(grid)
    ax2.plot(grid, coef[0] + coef[1] * np.cos(gr) + coef[2] * np.sin(gr),
             color='#7C5CBF', linewidth=1.5,
             label=f'sinusoid, amp {sin_amp:.1f} deg')
    ax2.set_xlabel('GPS course (deg)')
    ax2.set_ylabel('IMU yaw - GPS course (deg)')
    ax2.set_title('Constant, or heading-dependent?')
    ax2.set_xlim(-10, 370)
    ax2.legend(fontsize=9)

    t = [s['t'] for s in segs]
    ax3.plot(t, [s['course'] for s in segs], 'o-', color='#2E6FBF',
             markersize=4, label='GPS course')
    ax3.plot(t, [s['yaw'] for s in segs], 's-', color='#1D9E75',
             markersize=4, label='IMU yaw')
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('heading (deg)')
    ax3.set_title('Both headings over the walk')
    ax3.legend(fontsize=9)

    fig.savefig(graph(f'{name}_rectangle.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = [
        f'bag                {bag}',
        f'GPS fixes          {len(gps)}',
        f'segments           {len(segs)}  (min move {min_move} m)',
        f'  used for fit     {len(segs_fit)}  ({n_corner} corner segments excluded)',
        f'distance walked    {perimeter:.1f} m',
        f'GPS covariance     {np.mean([g["cov"] for g in gps]):.1f} m^2 mean',
        '',
        'LOOP CLOSURE',
        f'  GPS start to end   {close_gps:.2f} m',
    ]
    if close_odom is not None:
        lines.append(f'  EKF start to end   {close_odom:.2f} m')
    lines += [
        f'  as % of perimeter  {close_gps / perimeter * 100:.1f}%',
        '',
        'HEADING OFFSET',
        '',
        'seg   course    IMU yaw     offset',
        '-' * 40,
    ]
    for i, s in enumerate(segs):
        mark = '' if s in segs_fit else '   <- corner, excluded'
        lines.append(f'{i + 1:<5} {s["course"]:7.1f}  {s["yaw"]:9.1f}  '
                     f'{s["offset"]:+9.1f}{mark}')
    lines += [
        '',
        f'constant fit       {mean_off:+.2f} deg,  RMS residual {const_rms:.2f} deg',
        f'sinusoid fit       amplitude {sin_amp:.2f} deg,  RMS residual {sin_rms:.2f} deg',
        '',
    ]

    headings_spanned = float(np.ptp(courses))
    if headings_spanned < 120:
        lines += [
            f'INCONCLUSIVE - the walk only covered {headings_spanned:.0f} deg of',
            'heading. Distinguishing a constant from a sinusoid needs at least',
            'three well-separated directions. Walk a full rectangle or a loop.',
        ]
    elif sin_amp < 10.0 or sin_rms > const_rms * 0.7:
        lines += [
            'CONSTANT OFFSET - the sensor is behaving.',
            '',
            'The sinusoid fit is no better than a flat line, so the error',
            'does not depend on which way the robot faces. That rules out',
            'soft-iron distortion and nearby interference.',
            '',
            f'The {mean_off:+.1f} deg is the angle between the IMU x-axis and',
            'the direction of travel - a mounting rotation, plus local',
            'magnetic declination. It changes whenever the sensor is',
            'physically moved, so it describes this mounting only.',
            '',
            'DO NOT put this into navsat_transform yaw_offset.',
            'That was tested directly: yaw_offset rotated the map frame by',
            'its own value regardless of the measured offset, so any',
            'non-zero setting skewed the track. Leave it at 0.0.',
            '',
            'What this number is good for: confirming the offset is',
            'constant, and telling you how far the sensor is rotated',
            'relative to the robot if you want to straighten the mounting.',
        ]
    else:
        lines += [
            'HEADING-DEPENDENT ERROR.',
            '',
            f'The offset varies sinusoidally with heading, amplitude',
            f'{sin_amp:.1f} deg. The sinusoid fit explains the data far better',
            f'than a constant ({sin_rms:.1f} vs {const_rms:.1f} deg RMS).',
            '',
            'This is the signature of soft-iron distortion or interference',
            'from something fixed relative to the sensor. No single parameter',
            'corrects it - the fix is to relocate the sensor away from the',
            'interfering object, then re-measure.',
            '',
            f'For reference, a constant fit would give {mean_off:+.1f} deg,',
            'but it would be wrong by up to the sinusoid amplitude depending',
            'on which way the robot faces.',
        ]

    with open(out(f'{name}_rectangle.txt'), 'w') as fh:
        fh.write('\n'.join(lines) + '\n')

    print('\n'.join(lines))
    print(f'\n  wrote results/graphs/{name}_rectangle.png and .txt')


if __name__ == '__main__':
    main()
