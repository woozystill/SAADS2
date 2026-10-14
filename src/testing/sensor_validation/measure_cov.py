#!/usr/bin/env python3
"""Derive IMU covariances from measured stationary noise.

The gyroscope and accelerometer covariances in the driver started as
values chosen to be plausible - not datasheet figures, and not measured.
This replaces them with numbers taken from the sensor's own output.

Why this and not the chip's own estimate: the rotation vector report
carries a per-reading accuracy in radians, which the driver now reads.
The gyroscope and accelerometer reports do not - they are 10 bytes, all
of which are the three axis values, with no room for an accuracy field.
The only thing the chip offers for those is a 0-3 quality level, which
is qualitative. Converting that to rad^2/s^2 would mean inventing a
mapping, which is exactly what we are trying to get away from.

With the sensor stationary, everything the gyroscope reports is noise
plus bias, and everything the accelerometer reports beyond gravity is
noise. So the variance of a stationary recording IS the measurement
noise, for this sensor, in this mounting, at this temperature.

Usage:
    python3 measure_covariance.py results/imu_drift.csv
    python3 measure_covariance.py <bag_directory>

Requires: pip3 install numpy rosbags
"""

import csv
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

try:
    from _paths import out, graph, find_bag, stem
except ImportError:                       # standalone use
    def out(name):
        return name

    def graph(name):
        return name

    def find_bag(name):
        return name

    def stem(p):
        return os.path.basename(os.path.normpath(p))


def load_csv(path):
    """Read the CSV written by plot_imu.py."""
    gyro, accel, quat = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            gyro.append([float(row['gx']), float(row['gy']),
                         float(row['gz'])])
            accel.append([float(row['ax']), float(row['ay']),
                          float(row['az'])])
            quat.append([float(row['qx']), float(row['qy']),
                         float(row['qz']), float(row['qw'])])
    return np.array(gyro), np.array(accel), np.array(quat)


def load_bag(path):
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_HUMBLE)
    gyro, accel, quat = [], [], []
    with AnyReader([Path(path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == '/imu/data']
        if not conns:
            raise SystemExit('/imu/data not in bag')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            g, a, o = (m.angular_velocity, m.linear_acceleration,
                       m.orientation)
            gyro.append([g.x, g.y, g.z])
            accel.append([a.x, a.y, a.z])
            quat.append([o.x, o.y, o.z, o.w])
    return np.array(gyro), np.array(accel), np.array(quat)


def quat_to_yaw(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y),
                                   1 - 2 * (y * y + z * z))) % 360.0


def plot(gyro, accel, name):
    r"""Time series and histograms for both raw sensors.

    The time series shows whether the recording was genuinely steady -
    a trend means the sensor was drifting or being moved, and the
    variance would then include that rather than only noise.

    The histogram shows whether the noise is Gaussian, which is what a
    Kalman filter assumes. A clean bell curve means a single variance
    describes the sensor honestly; a skewed or multi-peaked one means it
    does not, and the number should be treated as approximate.
    """
    plt.rcParams.update({
        'font.size': 10, 'axes.titlesize': 12, 'axes.labelsize': 10.5,
        'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
    })

    t = np.arange(len(gyro))
    colours = ['#1D7A5F', '#2E5C9A', '#B0741A']
    labels = ['x', 'y', 'z']

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))

    # --- gyroscope ---------------------------------------------------
    ax = axes[0][0]
    for i in range(3):
        ax.plot(t, gyro[:, i], linewidth=0.6, color=colours[i],
                alpha=0.8, label=f'gyro {labels[i]}')
    ax.set_ylabel('rad/s')
    ax.set_xlabel('sample')
    ax.set_title('Gyroscope over the recording')
    ax.legend(fontsize=9, ncol=3)

    ax = axes[0][1]
    for i in range(3):
        centred = gyro[:, i] - gyro[:, i].mean()
        ax.hist(centred, bins=80, histtype='step', linewidth=1.4,
                color=colours[i],
                label=rf'{labels[i]}  $\sigma$={gyro[:, i].std():.5f}')
    ax.set_xlabel('rad/s, bias removed')
    ax.set_ylabel('count')
    ax.set_title('Gyroscope noise distribution')
    ax.legend(fontsize=9)

    # --- accelerometer -----------------------------------------------
    ax = axes[1][0]
    for i in range(3):
        ax.plot(t, accel[:, i], linewidth=0.6, color=colours[i],
                alpha=0.8, label=f'accel {labels[i]}')
    ax.set_ylabel('m/s$^2$')
    ax.set_xlabel('sample')
    ax.set_title('Accelerometer over the recording (gravity included)')
    ax.legend(fontsize=9, ncol=3)

    ax = axes[1][1]
    for i in range(3):
        centred = accel[:, i] - accel[:, i].mean()
        ax.hist(centred, bins=80, histtype='step', linewidth=1.4,
                color=colours[i],
                label=rf'{labels[i]}  $\sigma$={accel[:, i].std():.5f}')
    ax.set_xlabel('m/s$^2$, mean removed')
    ax.set_ylabel('count')
    ax.set_title('Accelerometer noise distribution')
    ax.legend(fontsize=9)

    path = graph(f'{name}_noise.png')
    fig.savefig(path, dpi=160)
    print(f'  wrote {path}')


def main():
    if len(sys.argv) < 2:
        raise SystemExit(
            f'usage: {sys.argv[0]} <stationary.csv | bag_directory>')

    src = sys.argv[1]
    if src.endswith('.csv'):
        gyro, accel, quat = load_csv(src)
        name = os.path.splitext(os.path.basename(src))[0]
    else:
        path = find_bag(src)
        gyro, accel, quat = load_bag(path)
        name = stem(path)

    n = len(gyro)
    if n < 100:
        raise SystemExit(f'Only {n} samples - need a longer recording.')

    # --- sanity: was it actually stationary? -------------------------
    rate = np.linalg.norm(gyro, axis=1)
    moved = float(np.percentile(rate, 99))
    if moved > 0.05:
        print(f'WARNING: 99th percentile angular rate is {moved:.3f} rad/s.')
        print('The sensor was moving. These numbers will be too large.\n')

    # --- gyroscope ----------------------------------------------------
    # Stationary, so the mean is bias and the spread is noise. Only the
    # spread belongs in the covariance; bias is a separate correction.
    g_bias = gyro.mean(axis=0)
    g_var = gyro.var(axis=0)

    # --- accelerometer ------------------------------------------------
    # Gravity sits in these axes, so per-axis variance about the mean is
    # the noise, with the constant gravity component removed by taking
    # the variance rather than the raw magnitude.
    a_var = accel.var(axis=0)
    a_mag = np.linalg.norm(accel, axis=1)

    # --- heading ------------------------------------------------------
    yaws = np.array([quat_to_yaw(q) for q in quat])
    ref = yaws[0]
    dev = np.array([(y - ref + 540) % 360 - 180 for y in yaws])
    yaw_sigma_deg = float(dev.std())

    # Use the worst axis for each block: the covariance is a single
    # number on the diagonal, so it has to cover the noisiest axis
    # rather than the average, or the filter over-trusts that axis.
    g_worst = float(g_var.max())
    a_worst = float(a_var.max())
    yaw_var = math.radians(yaw_sigma_deg) ** 2

    lines = [
        f'source              {name}',
        f'samples             {n}',
        '',
        'GYROSCOPE, rad/s',
        f'  bias        x {g_bias[0]:+.6f}  y {g_bias[1]:+.6f}  '
        f'z {g_bias[2]:+.6f}',
        f'  sigma       x {math.sqrt(g_var[0]):.6f}  '
        f'y {math.sqrt(g_var[1]):.6f}  z {math.sqrt(g_var[2]):.6f}',
        f'  variance    x {g_var[0]:.3e}  y {g_var[1]:.3e}  '
        f'z {g_var[2]:.3e}',
        '',
        'ACCELEROMETER, m/s^2',
        f'  sigma       x {math.sqrt(a_var[0]):.6f}  '
        f'y {math.sqrt(a_var[1]):.6f}  z {math.sqrt(a_var[2]):.6f}',
        f'  variance    x {a_var[0]:.3e}  y {a_var[1]:.3e}  '
        f'z {a_var[2]:.3e}',
        f'  magnitude   {a_mag.mean():.4f} m/s^2 '
        f'({(a_mag.mean() / 9.807 - 1) * 100:+.2f}% of g)',
        '',
        'HEADING, from the fused quaternion',
        f'  sigma       {yaw_sigma_deg:.4f} deg over the recording',
        f'  variance    {yaw_var:.3e} rad^2',
        '',
        '=' * 62,
        'MEASURED VALUES FOR THE DRIVER',
        '=' * 62,
        '',
        'Each is the worst axis, since the covariance is one number on',
        'the diagonal and has to cover the noisiest axis rather than the',
        'average.',
        '',
        f'  angular_velocity_variance:    {g_worst:.3e}',
        f'  linear_acceleration_variance: {a_worst:.3e}',
        '',
        'Set them with:',
        '',
        f'  ros2 launch bno085_driver bno085.launch.py \\',
        f'    --ros-args -p angular_velocity_variance:={g_worst:.3e} \\',
        f'               -p linear_acceleration_variance:={a_worst:.3e}',
        '',
        'or edit the defaults in bno085_node.py.',
        '',
        'CAVEATS worth stating alongside these numbers:',
        '',
        '  These are the noise floor at rest, in this mounting, at this',
        '  temperature. Under vibration the true noise is higher, so',
        '  these are a lower bound rather than a worst case.',
        '',
        '  MEMS gyroscope bias drifts with temperature, so the bias',
        '  figures above hold for the conditions of this recording only.',
        '  The variance is more stable than the bias.',
        '',
        '  Heading sigma here is drift over a stationary run, not the',
        '  same quantity as the chip\'s per-reading accuracy estimate.',
        '  The driver publishes the chip\'s estimate for heading and uses',
        '  these measured figures for the gyroscope and accelerometer.',
    ]

    plot(gyro, accel, name)

    report = '\n'.join(lines)
    print(report)

    path = out(f'{name}_covariance.txt')
    with open(path, 'w') as fh:
        fh.write(report + '\n')
    print(f'\n  wrote {path}')


if __name__ == '__main__':
    main()
