#!/usr/bin/env python3
"""Accelerometer gravity-magnitude test.

Records the accelerometer while the sensor sits completely still and
compares the measured magnitude against standard gravity.

The measurement is simple: a stationary accelerometer senses exactly one
thing, the reaction to gravity, so |a| must equal 9.807 m/s^2. Anything
else is scale or bias error.

Stillness is the whole method. Any movement adds real acceleration that is
not sensor error, which is what made an earlier hand-rotated version of
this test unrepeatable. Here the sensor never leaves the surface.

Usage:
    python3 test_gravity.py            60 s, one position
    python3 test_gravity.py 120        longer
    python3 test_gravity.py 40 4       40 s each in 4 flat rotations

Requires: pip3 install matplotlib
"""

import math
import statistics
import sys
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from _paths import out, graph

TOPIC = '/imu/data'
G = 9.807
DPI = 160

plt.rcParams.update({
    'font.size': 12, 'axes.titlesize': 15, 'axes.labelsize': 13,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})

COLOURS = ['#1D9E75', '#2E6FBF', '#7C5CBF', '#E8A33D',
           '#C0392B', '#0F766E', '#9AA0A6', '#B45309']


class Collector(Node):

    def __init__(self):
        super().__init__('gravity_test')
        self.rows = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, TOPIC, self._cb, qos)

    def _cb(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        self.rows.append({
            't': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'ax': a.x, 'ay': a.y, 'az': a.z,
            'mag': math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
            'gyro': math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z),
        })

    def gather(self, seconds, label=''):
        import time
        self.rows = []
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if label:
                print(f'  {label}: {end - time.time():5.1f}s  ',
                      end='\r', flush=True)
        print(' ' * 50, end='\r')
        return list(self.rows)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    n_pos = int(sys.argv[2]) if len(sys.argv) > 2 else 1

    print(f'\nGravity magnitude test.')
    if n_pos == 1:
        print(f'Place the sensor FLAT and do not touch it for {secs:.0f} s.\n')
    else:
        print(f'{n_pos} flat rotations, {secs:.0f} s each.')
        print('Keep it flat on the same surface; rotate in place between '
              'positions.\n')

    rclpy.init()
    node = Collector()
    groups = []

    try:
        for i in range(n_pos):
            if n_pos == 1:
                input('  Press Enter when the sensor is settled...')
                label = 'stationary'
            else:
                input(f'  Position {i + 1} of {n_pos}, flat, press Enter...')
                label = f'position {i + 1}'
            rows = node.gather(secs, label)
            if len(rows) < 10:
                raise SystemExit('No data - is the driver running?')

            # Discard anything captured while still moving.
            still = [r for r in rows if r['gyro'] < 0.05]
            if len(still) < 10:
                print(f'    only {len(still)} still samples; using all')
                still = rows
            groups.append((label, still))
            m = [r['mag'] for r in still]
            print(f'    {label}: {statistics.mean(m):.4f} m/s^2  '
                  f'({len(still)} still samples)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    all_mags = np.array([r['mag'] for _, rows in groups for r in rows])
    mean = float(np.mean(all_mags))
    sd = float(np.std(all_mags))
    err = mean - G
    t0 = groups[0][1][0]['t']

    # ---------------- figure ----------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5),
                                   gridspec_kw={'width_ratios': [1.5, 1]})

    for i, (label, rows) in enumerate(groups):
        t = [r['t'] - t0 for r in rows]
        m = [r['mag'] for r in rows]
        ax1.plot(t, m, '.', markersize=2.5, color=COLOURS[i % len(COLOURS)],
                 label=label if n_pos > 1 else 'measured')
    ax1.axhline(G, color='#C0392B', linestyle='--', linewidth=1.8,
                label=f'g = {G} m/s$^2$', zorder=5)
    ax1.axhline(mean, color='#333', linestyle='-', linewidth=1.2,
                label=f'mean {mean:.4f}', zorder=4)
    ax1.fill_between([0, max(r['t'] - t0 for _, rs in groups for r in rs)],
                     G * 0.99, G * 1.01, color='#C0392B', alpha=0.08,
                     label='$\\pm$1% band')
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('|acceleration| (m/s$^2$)')
    ax1.set_title('Accelerometer magnitude, sensor stationary')
    ax1.legend(fontsize=9, loc='best', markerscale=4)
    span = max(0.12, abs(err) * 3, sd * 5)
    ax1.set_ylim(G - span, G + span)

    ax2.hist(all_mags, bins=40, color='#1D9E75', edgecolor='white')
    ax2.axvline(G, color='#C0392B', linestyle='--', linewidth=1.8,
                label=f'g = {G}')
    ax2.axvline(mean, color='#333', linewidth=1.4, label=f'mean {mean:.4f}')
    ax2.set_xlabel('|acceleration| (m/s$^2$)')
    ax2.set_ylabel('count')
    ax2.set_title('Distribution')
    ax2.legend(fontsize=10)
    ax2.text(0.03, 0.96,
             f'error {err:+.4f} m/s$^2$\n'
             f'({err / G * 100:+.2f}% of g)\n'
             f'$\\sigma$ = {sd:.4f}\n'
             f'{len(all_mags)} samples',
             transform=ax2.transAxes, va='top', fontsize=11,
             bbox=dict(boxstyle='round', facecolor='#F2F2F2', alpha=0.9))

    fig.savefig(graph('imu_gravity_static.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = [f'positions        {n_pos}',
             f'duration each    {secs:.0f} s',
             f'total samples    {len(all_mags)}',
             '']
    if n_pos > 1:
        lines += ['position          mean       sd', '-' * 34]
        for label, rows in groups:
            m = [r['mag'] for r in rows]
            lines.append(f'{label:<16} {statistics.mean(m):8.4f} '
                         f'{statistics.pstdev(m):8.4f}')
        lines.append('')
    lines += [
        f'measured mean    {mean:.4f} m/s^2',
        f'expected         {G} m/s^2',
        f'error            {err:+.4f} m/s^2 ({err / G * 100:+.2f}%)',
        f'noise 1 sigma    {sd:.4f} m/s^2',
        '',
        'PASS' if abs(err) < 0.2 else 'MARGINAL',
        '',
        'A stationary accelerometer senses exactly one thing: the reaction',
        'to gravity. So the magnitude must equal 9.807 m/s^2, and any',
        'departure is scale or bias error. Consumer MEMS parts are',
        'typically specified to a few percent, so an error under 1% is a',
        'good result rather than a marginal one.',
    ]

    with open(out('imu_gravity_static.txt'), 'w') as fh:
        fh.write(f'imu_gravity_static\n'
                 f'{datetime.now().isoformat(timespec="seconds")}\n')
        fh.write('=' * 60 + '\n' + '\n'.join(lines) + '\n')

    print('\n'.join(lines))
    print('\n  wrote results/graphs/imu_gravity_static.png and .txt')


if __name__ == '__main__':
    main()
