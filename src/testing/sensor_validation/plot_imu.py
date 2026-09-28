#!/usr/bin/env python3
"""Presentation plots and raw data exports for a fused IMU.

Every run writes three artifacts:

    <name>.csv   every sample, one row each - portable raw data
    <name>.png   the figure for slides
    <name>.txt   computed statistics, plain text

Usage:
    python3 plot_imu.py drift    [seconds]   default 300, stationary
    python3 plot_imu.py cardinal             interactive, 4 headings
    python3 plot_imu.py gravity              interactive, 6 orientations
    python3 plot_imu.py live                 rolling trace, for demos

    python3 plot_imu.py drift --bag mybag    plot from a recorded bag
    python3 plot_imu.py replot file.csv      re-plot without hardware

Requires: pip3 install matplotlib rosbags
"""

import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from _paths import out, graph, find_bag, stem

TOPIC = '/imu/data'
DPI = 160
G = 9.807

plt.rcParams.update({
    'font.size': 12,
    'axes.titlesize': 15,
    'axes.labelsize': 13,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.autolayout': True,
})

CSV_FIELDS = ['t', 'qx', 'qy', 'qz', 'qw', 'yaw', 'pitch', 'roll',
              'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'accel_mag', 'gyro_mag']

# The six faces of a box. Together these exercise every accelerometer axis
# in both directions, which is what makes a scale or bias error visible.
ORIENTATIONS = [
    ('flat, label up', 'lay it flat on the desk'),
    ('flat, label down', 'flip it over, still flat'),
    ('on long edge, left', 'stand it on its long edge'),
    ('on long edge, right', 'stand it on the opposite long edge'),
    ('on short edge, near', 'stand it on its short edge'),
    ('on short edge, far', 'stand it on the opposite short edge'),
]


# ---------------------------------------------------------------------------

def quat_to_euler(x, y, z, w):
    roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                   1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                  1 - 2 * (y * y + z * z))) % 360.0
    return roll, pitch, yaw


def angle_diff(a, b):
    return (a - b + 540.0) % 360.0 - 180.0


def unwrap(yaws):
    out = [yaws[0]]
    for y in yaws[1:]:
        out.append(out[-1] + angle_diff(y, out[-1] % 360.0))
    return out


def circular_mean(yaws):
    """Mean of angles, handling the 359/1 wrap correctly."""
    ref = yaws[0]
    return (ref + statistics.mean([angle_diff(v, ref) for v in yaws])) % 360.0


def make_row(t, quat, gyro, accel):
    qx, qy, qz, qw = quat
    roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)
    return {
        't': t, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        'yaw': yaw, 'pitch': pitch, 'roll': roll,
        'gx': gyro[0], 'gy': gyro[1], 'gz': gyro[2],
        'ax': accel[0], 'ay': accel[1], 'az': accel[2],
        'accel_mag': math.sqrt(sum(c * c for c in accel)),
        'gyro_mag': math.sqrt(sum(c * c for c in gyro)),
    }


# ---------------------------------------------------------------------------

def save_csv(rows, name, extra_fields=()):
    fields = list(extra_fields) + CSV_FIELDS
    path = out(f'{name}.csv')
    with open(path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in fields})
    print(f'  wrote {path}  ({len(rows)} rows)')


def save_stats(lines, name):
    path = out(f'{name}.txt')
    with open(path, 'w') as fh:
        fh.write(f'{name}\n{datetime.now().isoformat(timespec="seconds")}\n')
        fh.write('=' * 60 + '\n' + '\n'.join(lines) + '\n')
    print(f'  wrote {path}')
    for ln in lines:
        print(f'    {ln}')


def load_csv(path):
    with open(path) as fh:
        rows = []
        for r in csv.DictReader(fh):
            rows.append({k: (float(v) if k in CSV_FIELDS else v)
                         for k, v in r.items()})
        return rows


def load_bag(bag_path, topic=TOPIC):
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    rows = []
    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise SystemExit(
                f'{topic} not in bag. Found: '
                f'{sorted({c.topic for c in reader.connections})}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            rows.append(make_row(
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                (m.orientation.x, m.orientation.y,
                 m.orientation.z, m.orientation.w),
                (m.angular_velocity.x, m.angular_velocity.y,
                 m.angular_velocity.z),
                (m.linear_acceleration.x, m.linear_acceleration.y,
                 m.linear_acceleration.z)))
    print(f'  read {len(rows)} samples from {bag_path}')
    return rows


def live_collector():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu

    class Collector(Node):
        def __init__(self):
            super().__init__('imu_plotter')
            self.rows = []
            # The driver publishes BEST_EFFORT; a RELIABLE subscription
            # connects to nothing and receives nothing.
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
            self.create_subscription(Imu, TOPIC, self._cb, qos)

        def _cb(self, msg):
            o, a, g = (msg.orientation, msg.linear_acceleration,
                       msg.angular_velocity)
            self.rows.append(make_row(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                (o.x, o.y, o.z, o.w), (g.x, g.y, g.z), (a.x, a.y, a.z)))

        def gather(self, seconds, label=''):
            self.rows = []
            end = time.time() + seconds
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if label:
                    print(f'  {label}: {end - time.time():4.1f}s  ',
                          end='\r', flush=True)
            print(' ' * 50, end='\r')
            return list(self.rows)

    rclpy.init()
    return rclpy, Collector()


# ---------------------------------------------------------------------------
# drift
# ---------------------------------------------------------------------------

def figure_drift(rows, name):
    if len(rows) < 10:
        raise SystemExit('Too few samples.')

    t = np.array([r['t'] - rows[0]['t'] for r in rows])
    y = np.array(unwrap([r['yaw'] for r in rows]))
    y_rel = y - y[0]
    span = float(t[-1] - t[0])
    rate = float((y[-1] - y[0]) / span * 60.0) if span > 0 else 0.0
    envelope = float(np.max(y_rel) - np.min(y_rel))

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, y_rel, linewidth=1.4, color='#1D9E75', label='measured', zorder=3)
    ax.axhline(0, color='#888', linewidth=0.8, linestyle='--')
    ax.fill_between(t, np.min(y_rel), np.max(y_rel), color='#1D9E75',
                    alpha=0.12, label=f'envelope {envelope:.2f} deg')
    ax.plot(t, t / 60.0 * 2.0, linewidth=1.5, color='#C0392B',
            linestyle=':', label='typical unfused gyro (2 deg/min)')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('yaw change (deg)')
    ax.set_title('Stationary yaw: bounded, not accumulating')
    ax.legend(loc='upper left')
    ax.text(0.98, 0.05,
            f'net {rate:+.3f} deg/min\n'
            f'envelope {envelope:.3f} deg\n'
            f'{len(rows)} samples over {span:.0f} s',
            transform=ax.transAxes, ha='right', va='bottom',
            bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))
    fig.savefig(graph(f'{name}.png'), dpi=DPI)
    print(f'  wrote results/graphs/{name}.png')

    return [
        f'samples          {len(rows)}',
        f'duration         {span:.1f} s',
        f'rate             {len(rows) / span:.2f} Hz',
        f'net change       {y[-1] - y[0]:+.3f} deg',
        f'net rate         {rate:+.4f} deg/min',
        f'envelope         {envelope:.4f} deg (max - min)',
        f'noise 1 sigma    {float(np.std(y_rel)):.4f} deg',
        '',
        'PASS' if envelope < 2.0 else 'FAIL',
        '',
        'An unfused gyro accumulates 1-10 deg/min without bound. A',
        'magnetometer-referenced fusion settles into a small envelope and',
        'stays there. The envelope figure is the meaningful one: it says',
        'the error is bounded, not that it is zero.',
    ]


# ---------------------------------------------------------------------------
# cardinal
# ---------------------------------------------------------------------------

def figure_cardinal(measured, name):
    expected = {'North': 90.0, 'East': 0.0, 'South': 270.0, 'West': 180.0}
    names = list(expected)
    offsets = [angle_diff(measured[n], expected[n]) for n in names]
    mean_off = statistics.mean(offsets)
    spread = max(offsets) - min(offsets)
    corrected = {n: (measured[n] - mean_off) % 360.0 for n in names}
    residual = [angle_diff(corrected[n], expected[n]) for n in names]

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    idx = np.arange(len(names))

    # Panel 1: raw measurement against the ENU expectation. East expects 0,
    # which would draw a zero-height bar and read as missing data, so the
    # expected series is drawn as markers rather than bars.
    ax = axes[0]
    ax.bar(idx, [measured[n] for n in names], 0.55,
           color='#1D9E75', label='measured')
    ax.scatter(idx, [expected[n] for n in names], marker='_', s=900,
               color='#C0392B', linewidths=3, label='expected (ENU)', zorder=4)
    ax.set_xticks(idx)
    ax.set_xticklabels(names)
    ax.set_ylabel('yaw (deg)')
    ax.set_ylim(-20, 380)
    ax.set_title('Raw heading vs ENU expectation')
    ax.legend(loc='upper left', fontsize=10)

    # Panel 2: the offsets, which is where consistency shows.
    ax = axes[1]
    ax.bar(idx, offsets, 0.55, color='#7C5CBF')
    ax.axhline(mean_off, color='#C0392B', linestyle='--',
               label=f'mean {mean_off:+.2f} deg')
    ax.set_xticks(idx)
    ax.set_xticklabels(names)
    ax.set_ylabel('offset from expected (deg)')
    pad = max(2.0, spread)
    ax.set_ylim(min(offsets) - pad, max(offsets) + pad)
    ax.set_title('Offset is constant, not heading-dependent')
    ax.legend(fontsize=10)
    ax.text(0.5, 0.04, f'spread {spread:.2f} deg',
            transform=ax.transAxes, ha='center',
            bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))

    # Panel 3: what the heading looks like once the constant is removed.
    ax = axes[2]
    ax.bar(idx, residual, 0.55, color='#1D9E75')
    ax.axhline(0, color='#888', linewidth=0.8)
    ax.set_xticks(idx)
    ax.set_xticklabels(names)
    ax.set_ylabel('residual error (deg)')
    lim = max(3.0, max(abs(r) for r in residual) * 1.6)
    ax.set_ylim(-lim, lim)
    ax.set_title(f'After subtracting {mean_off:+.1f} deg')
    ax.text(0.5, 0.92,
            f'max residual {max(abs(r) for r in residual):.2f} deg',
            transform=ax.transAxes, ha='center', va='top',
            bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))

    fig.savefig(graph(f'{name}.png'), dpi=DPI)
    print(f'  wrote results/graphs/{name}.png')

    lines = ['heading   measured   expected    offset   corrected  residual',
             '-' * 62]
    for n, off, res in zip(names, offsets, residual):
        lines.append(f'{n:<9} {measured[n]:8.2f}   {expected[n]:8.1f}  '
                     f'{off:+8.2f}   {corrected[n]:8.2f}  {res:+8.2f}')
    lines += [
        '',
        f'mean offset      {mean_off:+.2f} deg',
        f'spread           {spread:.2f} deg',
        f'max residual     {max(abs(r) for r in residual):.2f} deg',
        '',
        'PASS' if spread < 15.0 else 'FAIL',
        '',
        'The offset is not an error to remove from the sensor. It is the',
        'board mounting rotation plus local magnetic declination, and it',
        'belongs in navsat_transform:',
        f'  yaw_offset: {math.radians(mean_off):.4f}',
        '',
        'What matters is that it is CONSTANT. A heading-dependent offset',
        'would mean soft-iron distortion, which no single parameter fixes.',
    ]
    return lines


def run_cardinal(node, name='imu_cardinal'):
    measured, all_rows = {}, []
    print('\nPoint the board X axis at each direction. Keep it level.')
    print('Use a phone compass or a map.\n')

    for d in ['North', 'East', 'South', 'West']:
        input(f'  Point X at {d.upper()}, hold level, press Enter...')
        rows = node.gather(5.0, d)
        if len(rows) < 3:
            raise SystemExit(f'No data while measuring {d}')
        for r in rows:
            r2 = dict(r)
            r2['heading'] = d
            all_rows.append(r2)
        measured[d] = circular_mean([r['yaw'] for r in rows])
        print(f'  {d}: {measured[d]:.2f} deg')

    save_csv(all_rows, name, extra_fields=['heading'])
    save_stats(figure_cardinal(measured, name), name)


# ---------------------------------------------------------------------------
# gravity
# ---------------------------------------------------------------------------

def figure_gravity(per_pos, name):
    """per_pos: list of (label, [magnitudes])"""
    labels = [p[0] for p in per_pos]
    means = [float(np.mean(p[1])) for p in per_pos]
    sds = [float(np.std(p[1])) for p in per_pos]
    errors = [m - G for m in means]

    overall = float(np.mean(means))
    spread = max(means) - min(means)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.5))

    idx = np.arange(len(labels))
    short = [l.replace(', ', '\n') for l in labels]

    ax1.bar(idx, means, 0.55, yerr=sds, capsize=4, color='#1D9E75')
    ax1.axhline(G, color='#C0392B', linestyle='--',
                label=f'g = {G}', linewidth=1.6)
    ax1.set_xticks(idx)
    ax1.set_xticklabels(short, fontsize=9)
    ax1.set_ylabel('|acceleration| (m/s$^2$)')
    lo = min(min(means) - 0.3, G - 0.3)
    hi = max(max(means) + 0.3, G + 0.3)
    ax1.set_ylim(lo, hi)
    ax1.set_title('Magnitude held still in six orientations')
    ax1.legend(fontsize=10)

    ax2.bar(idx, errors, 0.55,
            color=['#1D9E75' if abs(e) < 0.2 else '#E8A33D' for e in errors])
    ax2.axhline(0, color='#888', linewidth=0.9)
    ax2.axhline(0.2, color='#C0392B', linestyle=':', linewidth=1.2,
                label='+/- 0.2 m/s$^2$ (2%)')
    ax2.axhline(-0.2, color='#C0392B', linestyle=':', linewidth=1.2)
    ax2.set_xticks(idx)
    ax2.set_xticklabels(short, fontsize=9)
    ax2.set_ylabel('error from g (m/s$^2$)')
    lim = max(0.35, max(abs(e) for e in errors) * 1.5)
    ax2.set_ylim(-lim, lim)
    ax2.set_title('Per-orientation error')
    ax2.legend(fontsize=10)
    ax2.text(0.5, 0.04,
             f'mean {overall:.3f}   spread across orientations {spread:.3f}',
             transform=ax2.transAxes, ha='center',
             bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))

    fig.savefig(graph(f'{name}.png'), dpi=DPI)
    print(f'  wrote results/graphs/{name}.png')

    lines = ['orientation              mean      sd     error', '-' * 52]
    for l, m, s, e in zip(labels, means, sds, errors):
        lines.append(f'{l:<24} {m:7.3f}  {s:6.3f}  {e:+7.3f}')
    lines += [
        '',
        f'overall mean     {overall:.4f} m/s^2',
        f'expected         {G} m/s^2',
        f'error            {overall - G:+.4f} ({(overall - G) / G * 100:+.2f}%)',
        f'spread           {spread:.4f} m/s^2 across orientations',
        '',
        'PASS' if abs(overall - G) < 0.2 and spread < 0.3 else 'FAIL',
        '',
        'Two separate things are being checked. The overall mean tests',
        'scale: is the accelerometer reading the right magnitude at all.',
        'The spread tests per-axis consistency: if one axis had a scale or',
        'bias error, the orientations that load that axis would read',
        'differently from the others, and "down" would be computed wrong.',
        '',
        'That matters for heading, not just tilt. Extracting yaw from a',
        'magnetometer requires tilt compensation, which requires knowing',
        'which way down is. A wrong gravity vector corrupts heading even',
        'with a perfect magnetometer.',
    ]
    return lines


def run_gravity(node, name='imu_gravity'):
    print('\nSix-position accelerometer test.')
    print('Hold each orientation COMPLETELY STILL for 4 seconds.')
    print('Stillness is what matters - any movement adds real acceleration')
    print('that is not sensor error.\n')

    per_pos, all_rows = [], []
    for label, instruction in ORIENTATIONS:
        input(f'  {instruction.upper()}, then press Enter...')
        rows = node.gather(4.0, label)
        if len(rows) < 5:
            raise SystemExit(f'No data while measuring {label}')

        # Discard anything taken while still moving.
        still = [r for r in rows if r['gyro_mag'] < 0.05]
        if len(still) < 3:
            print(f'    only {len(still)} still samples - hold more steadily')
            still = rows

        for r in still:
            r2 = dict(r)
            r2['position'] = label
            all_rows.append(r2)

        mags = [r['accel_mag'] for r in still]
        per_pos.append((label, mags))
        print(f'    {label}: {np.mean(mags):.3f} m/s^2  '
              f'({len(still)} still samples)')

    save_csv(all_rows, name, extra_fields=['position'])
    save_stats(figure_gravity(per_pos, name), name)


# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)

    mode = sys.argv[1]
    args = sys.argv[2:]

    bag = None
    if '--bag' in args:
        i = args.index('--bag')
        bag = args[i + 1]
        args = args[:i] + args[i + 2:]
    seconds = float(args[0]) if args and args[0].replace('.', '').isdigit() \
        else None

    if mode == 'replot':
        if not args:
            raise SystemExit('usage: plot_imu.py replot <file.csv>')
        rows = load_csv(args[0])
        base = os.path.splitext(args[0])[0] + '_replot'
        save_stats(figure_drift(rows, base), base)
        return

    if bag:
        bag = find_bag(bag)
        rows = load_bag(bag)
        name = stem(bag)
        if mode != 'drift':
            raise SystemExit('--bag supports drift only; the other tests '
                             'need interactive prompts')
        save_csv(rows, name)
        save_stats(figure_drift(rows, name), name)
        return

    rclpy, node = live_collector()
    try:
        if mode == 'drift':
            secs = seconds or 300
            print(f'\nPlace the sensor down and DO NOT TOUCH IT '
                  f'for {secs:.0f} s.')
            input('Press Enter to start...')
            rows = node.gather(secs, 'recording')
            save_csv(rows, 'imu_drift')
            save_stats(figure_drift(rows, 'imu_drift'), 'imu_drift')
        elif mode == 'cardinal':
            run_cardinal(node)
        elif mode == 'gravity':
            run_gravity(node)
        elif mode == 'live':
            print('Recording. Ctrl+C to stop and save.')
            buf = []
            try:
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if node.rows:
                        buf.extend(node.rows)
                        node.rows = []
                        print(f'  yaw {buf[-1]["yaw"]:7.2f} deg   '
                              f'{len(buf)} samples', end='\r', flush=True)
            except KeyboardInterrupt:
                print()
            save_csv(buf, 'imu_live')
            save_stats(figure_drift(buf, 'imu_live'), 'imu_live')
        else:
            raise SystemExit(__doc__)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
