#!/usr/bin/env python3
"""Plot how confidence evolves during a run.

Answers the question "how much do you trust each sensor, and does that
trust change mid-run?" with measured data rather than assertion.

Three things are plotted, and they behave differently:

  GPS reported covariance
      Comes from the receiver, derived from satellite geometry. Changes
      continuously as satellites move and sky view changes. The filter
      weights every fix by this, so it already trusts GPS less on a bad
      day without anyone intervening.

  EKF position uncertainty
      The filter's own estimate of how wrong it might be. Grows during
      prediction, collapses when a measurement arrives. Between 1 Hz GPS
      fixes it grows for a full second - the sawtooth is the filter
      saying "I am getting less sure" and then being corrected.

  IMU orientation covariance
      Fixed in the driver, from the datasheet. Does NOT adapt. Worth
      naming as a limitation: if the magnetometer is disturbed, the
      filter has no way to know it should trust heading less.

Usage:
    python3 plot_uncertainty.py <bag>
    python3 plot_uncertainty.py <bag> --dropout 40 55

The --dropout option shades a window where GPS was deliberately blocked,
so the covariance growth during the outage is easy to point at.

Requires: pip3 install matplotlib rosbags
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
    'font.size': 11, 'axes.titlesize': 13, 'axes.labelsize': 11.5,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def load(bag_path):
    """Pull covariance fields out of the three topics that carry them."""
    ts = get_typestore(Stores.ROS2_HUMBLE)
    gps, glob, loc, imu = [], [], [], []
    wanted = {'/gps/fix', '/odometry/global', '/odometry/local', '/imu/data'}

    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in wanted]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(f'Nothing usable in bag. Found: {found}')

        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

            if conn.topic == '/gps/fix':
                gps.append({
                    't': t,
                    'status': m.status.status,
                    # position_covariance is 3x3 row-major: [0]=east, [4]=north
                    'var_e': m.position_covariance[0],
                    'var_n': m.position_covariance[4],
                })
            elif conn.topic == '/imu/data':
                # orientation_covariance is 3x3: [8] = yaw variance
                imu.append({'t': t, 'var_yaw': m.orientation_covariance[8]})
            else:
                # pose.covariance is 6x6 row-major:
                #   [0]=x, [7]=y, [35]=yaw
                rec = {
                    't': t,
                    'var_x': m.pose.covariance[0],
                    'var_y': m.pose.covariance[7],
                    'var_yaw': m.pose.covariance[35],
                }
                (glob if conn.topic == '/odometry/global' else loc).append(rec)

    return gps, glob, loc, imu


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag> [--dropout START END]')

    bag = find_bag(sys.argv[1])
    name = stem(bag)

    dropout = None
    if '--dropout' in sys.argv:
        i = sys.argv.index('--dropout')
        dropout = (float(sys.argv[i + 1]), float(sys.argv[i + 2]))

    gps, glob, loc, imu = load(bag)
    print(f'{len(gps)} GPS, {len(glob)} global, {len(loc)} local, '
          f'{len(imu)} IMU')
    if not glob:
        raise SystemExit('No /odometry/global in bag - nothing to plot.')

    t0 = min([r['t'] for r in (gps or glob)])

    def rel(rows):
        return np.array([r['t'] - t0 for r in rows])

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10),
                                        sharex=True)

    # ---------------- 1. what the GPS says about itself ----------------
    if gps:
        tg = rel(gps)
        # Report as 1-sigma in metres - a variance in m^2 means nothing
        # to most of an audience, but "plus or minus 3.5 m" does.
        sigma = np.sqrt([r['var_e'] for r in gps])
        ax1.plot(tg, sigma, '.-', color='#2E5C9A', markersize=4,
                 linewidth=1, label='GPS reported 1$\\sigma$')
        lost = [r for r in gps if r['status'] < 0]
        if lost:
            ax1.plot(rel(lost), [0] * len(lost), 'x', color='#C0392B',
                     markersize=7, label='no fix')
        ax1.axhline(float(np.mean(sigma)), color='#5A5A5A',
                    linestyle='--', linewidth=1,
                    label=f'mean {np.mean(sigma):.2f} m')
        ax1.set_ylabel('GPS 1$\\sigma$ (m)')
        ax1.set_title('1. What the GPS says about itself — '
                      'receiver-reported, changes with satellite geometry')
        ax1.legend(fontsize=9, loc='upper right')

    # ---------------- 2. what the filter says about itself -------------
    tG = rel(glob)
    sx = np.sqrt([r['var_x'] for r in glob])
    sy = np.sqrt([r['var_y'] for r in glob])
    ax2.plot(tG, sx, linewidth=1.2, color='#7C5CBF', label='global EKF, x')
    ax2.plot(tG, sy, linewidth=1.2, color='#1D7A5F', alpha=0.75,
             label='global EKF, y')
    if loc:
        tL = rel(loc)
        sxl = np.sqrt([r['var_x'] for r in loc])
        ax2.plot(tL, sxl, linewidth=1.0, color='#B0741A', alpha=0.8,
                 label='local EKF, x (no absolute reference)')
    ax2.set_ylabel('position 1$\\sigma$ (m)')
    ax2.set_title('2. What the filter says about itself — '
                  'grows on prediction, collapses on each GPS fix')
    ax2.legend(fontsize=9, loc='upper left')

    # ---------------- 3. the fixed one -------------------------------
    if imu:
        ti = rel(imu)
        syaw = np.degrees(np.sqrt([r['var_yaw'] for r in imu]))
        ax3.plot(ti, syaw, linewidth=1.4, color='#C0392B',
                 label='IMU heading 1$\\sigma$ (fixed in driver)')
    tg_yaw = np.degrees(np.sqrt([r['var_yaw'] for r in glob]))
    ax3.plot(tG, tg_yaw, linewidth=1.2, color='#7C5CBF',
             label='global EKF heading 1$\\sigma$')
    ax3.set_ylabel('heading 1$\\sigma$ (deg)')
    ax3.set_xlabel('time (s)')
    ax3.set_title('3. Heading confidence — the IMU line is constant '
                  'because the driver hardcodes it')
    ax3.legend(fontsize=9, loc='upper right')

    if dropout:
        for a in (ax1, ax2, ax3):
            a.axvspan(dropout[0], dropout[1], color='#C0392B', alpha=0.10)
        ax2.text(sum(dropout) / 2,
                 float(np.max(sx)) * 0.9, 'GPS blocked',
                 ha='center', fontsize=10, color='#C0392B')

    fig.savefig(graph(f'{name}_uncertainty.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = [f'bag                {name}', '']
    if gps:
        sigma = np.sqrt([r['var_e'] for r in gps])
        nofix = sum(1 for r in gps if r['status'] < 0)
        lines += [
            'GPS, receiver-reported',
            f'  fixes               {len(gps)}  ({nofix} with no fix)',
            f'  1 sigma mean        {np.mean(sigma):.2f} m',
            f'  1 sigma range       {np.min(sigma):.2f} - {np.max(sigma):.2f} m',
            '  -> varies during the run; the filter weights each fix by it',
            '',
        ]
    lines += [
        'Global EKF position uncertainty',
        f'  1 sigma mean        {np.mean(sx):.2f} m',
        f'  1 sigma range       {np.min(sx):.2f} - {np.max(sx):.2f} m',
        '  -> sawtooth: grows between fixes, collapses when one arrives',
        '',
    ]
    if imu:
        syaw = np.degrees(np.sqrt([r['var_yaw'] for r in imu]))
        lines += [
            'IMU heading uncertainty',
            f'  1 sigma             {np.mean(syaw):.2f} deg, constant',
            '  -> hardcoded in the driver from the datasheet. Does NOT',
            '     adapt. If the magnetometer is disturbed by a motor, the',
            '     filter has no way to know it should trust heading less.',
            '     Named as a limitation, not a defect - adaptive IMU',
            '     covariance is future work.',
            '',
        ]
    lines += [
        'HOW THE FILTER USES THESE',
        'Each measurement is weighted by the inverse of its covariance, so',
        'a fix reported as 4 m accurate moves the estimate less than one',
        'reported as 1 m accurate. Nobody tunes this mid-run; it follows',
        'from the numbers the sensors publish.',
        '',
        'Between fixes the filter has no position measurement at all, so',
        'its uncertainty grows. Wheel odometry will slow that growth by',
        'supplying velocity at 50 Hz.',
    ]

    with open(out(f'{name}_uncertainty.txt'), 'w') as fh:
        fh.write('\n'.join(lines) + '\n')

    print('\n'.join(lines))
    print(f'\n  wrote results/graphs/{name}_uncertainty.png and .txt')


if __name__ == '__main__':
    main()
