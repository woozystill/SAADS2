#!/usr/bin/env python3
"""Extract yaw_offset by comparing IMU heading against GPS course-over-ground.

GPS course derived from consecutive fixes is an absolute heading reference
that is immune to magnetic interference. Comparing it against the IMU's
reported yaw gives the constant that belongs in navsat_transform.

The distinction that matters:
  - a CONSTANT difference is yaw_offset plus magnetic declination
  - a difference that VARIES with heading means magnetic interference,
    which no single parameter corrects

Usage:
    python3 analyse_straight_line.py <bag_directory>
    python3 analyse_straight_line.py <bag_directory> --min-move 0.5

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
    'font.size': 12, 'axes.titlesize': 15, 'axes.labelsize': 13,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def quat_to_yaw(x, y, z, w):
    """Yaw in degrees, 0-360, ENU convention (0 = east, CCW positive)."""
    return math.degrees(math.atan2(2 * (w * z + x * y),
                                   1 - 2 * (y * y + z * z))) % 360.0


def angle_diff(a, b):
    return (a - b + 540.0) % 360.0 - 180.0


def latlon_to_local(lat, lon, lat0, lon0):
    """Equirectangular projection to metres. Fine over tens of metres."""
    R = 6378137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def load(bag_path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    gps, imu = [], []
    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        wanted = {'/gps/fix', '/imu/data'}
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(f'Need /gps/fix and /imu/data. Found: {found}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == '/gps/fix':
                if m.status.status < 0 or math.isnan(m.latitude):
                    continue
                gps.append({'t': t, 'lat': m.latitude, 'lon': m.longitude,
                            'cov': m.position_covariance[0]})
            else:
                o = m.orientation
                imu.append({'t': t, 'yaw': quat_to_yaw(o.x, o.y, o.z, o.w)})
    return gps, imu


def imu_yaw_at(imu, t):
    """Nearest IMU sample to a given time."""
    best = min(imu, key=lambda r: abs(r['t'] - t))
    return best['yaw'] if abs(best['t'] - t) < 0.5 else None


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag_directory> [--min-move M]')

    bag = sys.argv[1]
    min_move = 0.5
    if '--min-move' in sys.argv:
        min_move = float(sys.argv[sys.argv.index('--min-move') + 1])

    gps, imu = load(bag)
    print(f'{len(gps)} GPS fixes, {len(imu)} IMU samples')
    if len(gps) < 5 or len(imu) < 5:
        raise SystemExit('Not enough data.')

    lat0, lon0 = gps[0]['lat'], gps[0]['lon']
    for g in gps:
        g['x'], g['y'] = latlon_to_local(g['lat'], g['lon'], lat0, lon0)

    # Course over ground between consecutive fixes far enough apart that
    # GPS noise does not dominate the direction.
    samples = []
    anchor = gps[0]
    for g in gps[1:]:
        dx, dy = g['x'] - anchor['x'], g['y'] - anchor['y']
        dist = math.hypot(dx, dy)
        if dist < min_move:
            continue
        # atan2(dy, dx) is ENU yaw: 0 = east, CCW positive - same
        # convention the IMU reports, so they are directly comparable.
        course = math.degrees(math.atan2(dy, dx)) % 360.0
        yaw = imu_yaw_at(imu, g['t'])
        if yaw is not None:
            samples.append({'t': g['t'] - gps[0]['t'], 'course': course,
                            'yaw': yaw, 'offset': angle_diff(yaw, course),
                            'dist': dist, 'x': g['x'], 'y': g['y']})
        anchor = g

    if len(samples) < 3:
        raise SystemExit(
            f'Only {len(samples)} usable segments. Either the walk was too '
            f'short, or --min-move {min_move} is too large for the distance '
            f'covered. Try --min-move 0.3')

    offsets = [s['offset'] for s in samples]
    ref = offsets[0]
    mean_off = (ref + np.mean([angle_diff(o, ref) for o in offsets])) % 360.0
    mean_off = (mean_off + 180) % 360 - 180
    spread = max(offsets) - min(offsets)
    total = math.hypot(gps[-1]['x'] - gps[0]['x'], gps[-1]['y'] - gps[0]['y'])

    # ---------------- figure ----------------
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 5))

    xs = [g['x'] for g in gps]
    ys = [g['y'] for g in gps]
    ax1.plot(xs, ys, '.-', color='#1D9E75', markersize=4, linewidth=1)
    ax1.plot(xs[0], ys[0], 'o', color='#2E7D32', markersize=10, label='start')
    ax1.plot(xs[-1], ys[-1], 's', color='#C0392B', markersize=9, label='end')
    ax1.set_xlabel('east (m)')
    ax1.set_ylabel('north (m)')
    ax1.set_title(f'GPS track, {total:.1f} m end to end')
    ax1.legend(fontsize=10)
    ax1.set_aspect('equal', adjustable='datalim')

    t = [s['t'] for s in samples]
    ax2.plot(t, [s['course'] for s in samples], 'o-', color='#2E6FBF',
             markersize=5, label='GPS course')
    ax2.plot(t, [s['yaw'] for s in samples], 's-', color='#1D9E75',
             markersize=5, label='IMU yaw')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('heading (deg)')
    ax2.set_title('Heading, two independent sources')
    ax2.legend(fontsize=10)

    ax3.plot(t, offsets, 'o-', color='#7C5CBF', markersize=6)
    ax3.axhline(mean_off, color='#C0392B', linestyle='--',
                label=f'mean {mean_off:+.1f} deg')
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('IMU yaw - GPS course (deg)')
    ax3.set_title('Offset')
    ax3.legend(fontsize=10)
    pad = max(5.0, spread * 0.6)
    ax3.set_ylim(min(offsets) - pad, max(offsets) + pad)
    ax3.text(0.5, 0.04, f'spread {spread:.1f} deg',
             transform=ax3.transAxes, ha='center',
             bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))

    fig.savefig(graph(f'{name}_straight.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = [
        f'bag                {name}',
        f'GPS fixes          {len(gps)}',
        f'usable segments    {len(samples)}  (min move {min_move} m)',
        f'track length       {total:.1f} m end to end',
        f'GPS covariance     {np.mean([g["cov"] for g in gps]):.1f} m^2 mean',
        '',
        'seg   dist    GPS course    IMU yaw     offset',
        '-' * 50,
    ]
    for i, s in enumerate(samples):
        lines.append(f'{i + 1:<5} {s["dist"]:5.1f}m  {s["course"]:9.1f}  '
                     f'{s["yaw"]:10.1f}  {s["offset"]:+9.1f}')
    lines += [
        '',
        f'MEAN OFFSET        {mean_off:+.2f} deg',
        f'spread             {spread:.2f} deg',
        '',
    ]

    if spread < 25.0:
        lines += [
            'Offset is reasonably consistent across this run.',
            '',
            f'The {mean_off:+.1f} deg is the angle between the IMU x-axis and',
            'the direction of travel - mounting rotation plus declination.',
            'It changes whenever the sensor is physically moved.',
            '',
            'DO NOT set navsat_transform yaw_offset to this. Tested',
            'directly: yaw_offset rotates the map frame by its own value',
            'regardless of the measured offset, so any non-zero setting',
            'skews the track. Leave it at 0.0.',
            '',
            'One heading only tells you the offset is consistent along',
            'this line. Walk a rectangle to test whether it is constant',
            'across all headings - that is what rules out soft iron.',
        ]
    else:
        lines += [
            f'Offset varies by {spread:.0f} deg between segments. Two possible',
            'causes, and they need different responses:',
            '',
            '  1. The walk was not straight enough, or segments were too short',
            '     for GPS noise to average out. Try a longer, straighter run,',
            '     or raise --min-move.',
            '  2. Genuine magnetic interference, in which case no single',
            '     parameter corrects it and the sensor needs relocating.',
            '',
            'Distinguish them by walking two straight lines in different',
            'directions. Consistent offset between them means the sensor is',
            'fine and the first run was just noisy.',
        ]

    with open(out(f'{name}_straight.txt'), 'w') as fh:
        fh.write('\n'.join(lines) + '\n')

    print('\n'.join(lines))
    print(f'\n  wrote results/graphs/{name}_straight.png and .txt')


if __name__ == '__main__':
    main()
