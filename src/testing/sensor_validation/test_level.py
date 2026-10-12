#!/usr/bin/env python3
"""Accelerometer tilt-bias test.

Measures whether the sensor computes "down" correctly - which is the thing
that actually matters, since magnetometer tilt compensation depends on it.

WHY THIS BEATS THE SIX-FACE MAGNITUDE TEST

The six-face test asks whether |acceleration| equals g in every orientation.
In principle that is sound. In practice you cannot stand a small breakout
board on its edge repeatably enough, and the positioning error swamps the
sensor error - run-to-run scale estimates varied by 3.4 percentage points,
which is larger than the effect being measured.

This test instead keeps the sensor on ONE flat surface and rotates it about
the vertical axis into four positions 90 degrees apart. The surface tilt is
identical in all four, so averaging the four readings cancels it exactly and
leaves the sensor's own bias. The variation around that mean is the surface
tilt, reported separately as a by-product.

Positioning error is near zero because "lying flat on a table" is far easier
to reproduce than "balanced on an edge".

Usage:
    python3 test_level.py              4 positions, 5 s each
    python3 test_level.py 8            8 positions, finer

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


def quat_to_euler(x, y, z, w):
    roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                   1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                  1 - 2 * (y * y + z * z))) % 360.0
    return roll, pitch, yaw


class Collector(Node):

    def __init__(self):
        super().__init__('level_test')
        self.rows = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, TOPIC, self._cb, qos)

    def _cb(self, msg):
        o, a, g = (msg.orientation, msg.linear_acceleration,
                   msg.angular_velocity)
        roll, pitch, yaw = quat_to_euler(o.x, o.y, o.z, o.w)
        self.rows.append({
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'ax': a.x, 'ay': a.y, 'az': a.z,
            'accel_mag': math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
            'gyro_mag': math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z),
        })

    def gather(self, seconds, label=''):
        import time
        self.rows = []
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if label:
                print(f'  {label}: {end - time.time():4.1f}s  ',
                      end='\r', flush=True)
        print(' ' * 50, end='\r')
        return [r for r in self.rows if r['gyro_mag'] < 0.05] or list(self.rows)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    if n < 3:
        raise SystemExit('need at least 3 positions')
    step = 360.0 / n

    print(f'\nTilt-bias test, {n} positions.')
    print('Keep the sensor FLAT ON ONE SURFACE throughout. Do not lift it,')
    print('do not change surfaces. Just rotate it in place about the')
    print(f'vertical axis by roughly {step:.0f} degrees each time.\n')
    print('The surface does not need to be level - its tilt cancels out.\n')

    rclpy.init()
    node = Collector()
    results = []

    try:
        for i in range(n):
            input(f'  Position {i + 1} of {n} '
                  f'(rotate ~{step:.0f} deg from the last), press Enter...')
            rows = node.gather(5.0, f'pos {i + 1}')
            if len(rows) < 5:
                raise SystemExit('No data - is the driver running?')
            results.append({
                'roll': statistics.mean(r['roll'] for r in rows),
                'pitch': statistics.mean(r['pitch'] for r in rows),
                'yaw': statistics.mean(r['yaw'] for r in rows),
                'mag': statistics.mean(r['accel_mag'] for r in rows),
                'n': len(rows),
            })
            print(f'    roll {results[-1]["roll"]:+7.3f}  '
                  f'pitch {results[-1]["pitch"]:+7.3f}  '
                  f'|a| {results[-1]["mag"]:.3f}  ({len(rows)} samples)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rolls = [r['roll'] for r in results]
    pitches = [r['pitch'] for r in results]
    mags = [r['mag'] for r in results]

    # Averaging evenly-spaced rotations on one surface cancels the surface
    # tilt, because its contribution to roll and pitch traces a full circle.
    # What survives is the sensor's own bias.
    roll_bias = statistics.mean(rolls)
    pitch_bias = statistics.mean(pitches)
    tilt_bias = math.hypot(roll_bias, pitch_bias)

    # The variation around that mean is the surface, not the sensor.
    surface_tilt = math.hypot(
        (max(rolls) - min(rolls)) / 2, (max(pitches) - min(pitches)) / 2)

    mag_mean = statistics.mean(mags)
    mag_spread = max(mags) - min(mags)

    # ---------------- figure ----------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.5))

    ax1.plot(rolls, pitches, 'o-', color='#1D9E75', markersize=9,
             linewidth=1.4, label='measured positions', zorder=3)
    for i, (r, p) in enumerate(zip(rolls, pitches)):
        ax1.annotate(str(i + 1), (r, p), textcoords='offset points',
                     xytext=(8, 6), fontsize=10)
    ax1.plot(roll_bias, pitch_bias, 'X', color='#7C5CBF', markersize=16,
             label=f'mean = sensor bias ({tilt_bias:.3f} deg)', zorder=4)
    ax1.plot(0, 0, '+', color='#C0392B', markersize=16, markeredgewidth=2.5,
             label='true level', zorder=4)
    ax1.set_xlabel('roll (deg)')
    ax1.set_ylabel('pitch (deg)')
    ax1.set_title('Rotating on one surface')
    ax1.legend(fontsize=9, loc='best')
    ax1.set_aspect('equal', adjustable='datalim')

    idx = np.arange(len(results))
    width = 0.38
    ax2.bar(idx - width / 2, rolls, width, color='#1D9E75', label='roll')
    ax2.bar(idx + width / 2, pitches, width, color='#2E6FBF', label='pitch')
    ax2.axhline(roll_bias, color='#1D9E75', linestyle='--', linewidth=1.4,
                label=f'roll bias {roll_bias:+.3f}')
    ax2.axhline(pitch_bias, color='#2E6FBF', linestyle='--', linewidth=1.4,
                label=f'pitch bias {pitch_bias:+.3f}')
    ax2.axhline(0, color='#888', linewidth=0.9)
    ax2.set_xticks(idx)
    ax2.set_xticklabels([f'pos {i + 1}' for i in idx])
    ax2.set_ylabel('angle (deg)')
    ax2.set_title('Per-position roll and pitch')
    ax2.legend(fontsize=9)

    fig.savefig(graph('imu_level.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = [
        f'positions        {n}',
        '',
        'pos      roll     pitch       |a|',
        '-' * 38,
    ]
    for i, r in enumerate(results):
        lines.append(f'{i + 1:<6} {r["roll"]:+8.3f} {r["pitch"]:+8.3f} '
                     f'{r["mag"]:9.3f}')
    lines += [
        '',
        f'SENSOR TILT BIAS     {tilt_bias:.3f} deg',
        f'  roll component     {roll_bias:+.3f} deg',
        f'  pitch component    {pitch_bias:+.3f} deg',
        '',
        f'surface tilt         {surface_tilt:.3f} deg  (cancels; not sensor error)',
        '',
        f'magnitude mean       {mag_mean:.4f} m/s^2 '
        f'({(mag_mean / G - 1) * 100:+.2f}% of g)',
        f'magnitude spread     {mag_spread:.4f} m/s^2',
        '',
    ]

    if tilt_bias < 1.0:
        lines += [
            'PASS',
            '',
            f'The sensor computes "down" to within {tilt_bias:.2f} deg. That is the',
            'quantity magnetometer tilt compensation depends on, so this',
            'bounds the heading error contributed by the accelerometer.',
        ]
    else:
        lines += [
            'MARGINAL',
            '',
            f'A {tilt_bias:.2f} deg tilt bias propagates into heading through tilt',
            'compensation. Worth applying as an offset if heading accuracy',
            'better than a degree is needed.',
        ]

    lines += [
        '',
        'HOW TO READ THIS',
        'Rotating on a single surface keeps the surface tilt constant while',
        'the sensor axes turn beneath it. Averaging evenly-spaced positions',
        'cancels that constant exactly, so the mean is the sensor bias and',
        'the scatter around it is the surface. Positioning error is minimal',
        'because lying flat is far easier to reproduce than balancing on an',
        'edge - which is what made the six-face magnitude test unrepeatable.',
    ]

    with open(out('imu_level.txt'), 'w') as fh:
        fh.write(f'imu_level\n{datetime.now().isoformat(timespec="seconds")}\n')
        fh.write('=' * 60 + '\n' + '\n'.join(lines) + '\n')

    print('\n'.join(lines))
    print('\n  wrote results/graphs/imu_level.png and results/imu_level.txt')


if __name__ == '__main__':
    main()
