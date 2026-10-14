#!/usr/bin/env python3
"""Magnetometer circle test - hard and soft iron, measured.

Rotate the robot through a full turn while recording /imu/mag, and the
horizontal field components trace a shape. What that shape is tells you
what is wrong:

    CIRCLE, centred on the origin      clean. The sensor sees only the
                                       Earth's field.

    CIRCLE, centred away from origin   HARD IRON. Something is adding a
                                       constant field - a magnet, a
                                       magnetised bracket, a battery.
                                       The offset vector points at it.
                                       The chip corrects this.

    ELLIPSE                            SOFT IRON. Ferrous material is
                                       bending the field, so the
                                       distortion depends on heading.
                                       The chip corrects this too, but
                                       only if it has calibrated in this
                                       configuration.

    SCATTER, no shape                  interference that changes with
                                       time - motor current, switching
                                       electronics. Nothing corrects
                                       this. The sensor has to move.

The radius is the local field strength. Tucson is about 47 uT; a radius
far from that means the field is not predominantly the Earth's.

REQUIRES the magnetometer report, which the runtime launch leaves off:

    ros2 launch bno085_driver bno085.launch.py \\
        --ros-args -p enable_magnetometer:=true

Usage:
    python3 analyse_magnetometer.py <bag>
    python3 analyse_magnetometer.py <bag> --expect 47.0

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

DPI = 160

plt.rcParams.update({
    'font.size': 10.5, 'axes.titlesize': 12.5, 'axes.labelsize': 11,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def load(bag_path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    mag, yaw = [], []
    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections
                 if c.topic in ('/imu/mag', '/imu/data')]
        if not conns:
            found = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f'Neither /imu/mag nor /imu/data in bag. Found: {found}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if conn.topic == '/imu/mag':
                f = m.magnetic_field
                # Published in tesla; microtesla is the readable unit.
                mag.append((t, f.x * 1e6, f.y * 1e6, f.z * 1e6))
            else:
                o = m.orientation
                yaw.append((t, math.degrees(math.atan2(
                    2 * (o.w * o.z + o.x * o.y),
                    1 - 2 * (o.y * o.y + o.z * o.z))) % 360))
    if not mag:
        raise SystemExit(
            '/imu/mag is empty. Relaunch the driver with\n'
            '  --ros-args -p enable_magnetometer:=true\n'
            'and record again.')
    return np.array(mag), np.array(yaw)


def fit_circle(x, y):
    """Algebraic circle fit. Returns centre, radius, rms residual.

    Solves for the centre that makes all points equidistant, which is
    linear in the unknowns once written as x^2+y^2 = 2ax + 2by + c.
    """
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(sol[2] + cx ** 2 + cy ** 2)
    resid = np.hypot(x - cx, y - cy) - r
    return cx, cy, r, float(np.sqrt(np.mean(resid ** 2)))


def fit_ellipse_axes(x, y, cx, cy):
    """Principal axes of the point cloud about a given centre.

    The ratio of the two axis lengths is the soft-iron indicator: 1.0 is
    a circle, anything much larger means the field is being stretched in
    one direction.
    """
    pts = np.column_stack([x - cx, y - cy])
    cov = np.cov(pts.T)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    vals, vecs = vals[order], vecs[:, order]
    major, minor = 2 * np.sqrt(vals)
    angle = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    return major, minor, angle


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag> [--expect uT]')

    bag = find_bag(sys.argv[1])
    name = stem(bag)

    expect = 47.0        # Tucson, roughly
    if '--expect' in sys.argv:
        expect = float(sys.argv[sys.argv.index('--expect') + 1])

    mag, yaw = load(bag)
    t, mx, my, mz = mag[:, 0], mag[:, 1], mag[:, 2], mag[:, 3]
    t = t - t[0]
    n = len(mx)

    cx, cy, r, rms = fit_circle(mx, my)
    major, minor, angle = fit_ellipse_axes(mx, my, cx, cy)
    offset = math.hypot(cx, cy)
    ratio = major / minor if minor > 0 else float('inf')
    total = np.sqrt(mx ** 2 + my ** 2 + mz ** 2)

    # --- did the recording actually cover a full turn? ---------------
    heading = np.degrees(np.arctan2(my - cy, mx - cx)) % 360
    covered = len(np.unique((heading // 15).astype(int))) / 24.0 * 100

    # ---------------- figure ----------------
    fig = plt.figure(figsize=(14, 9))

    # 1. the circle itself
    ax = fig.add_subplot(2, 2, 1)
    ax.scatter(mx, my, s=4, alpha=0.4, color='#2E5C9A', label='measured')
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(cx + r * np.cos(th), cy + r * np.sin(th), '--',
            color='#C0392B', linewidth=1.4,
            label=f'fitted circle, r={r:.1f} $\\mu$T')
    ax.plot(cx, cy, 'x', color='#C0392B', markersize=11, markeredgewidth=2)
    ax.plot(0, 0, '+', color='#1D7A5F', markersize=13, markeredgewidth=2,
            label='origin')
    if offset > 0.5:
        ax.annotate('', xy=(cx, cy), xytext=(0, 0),
                    arrowprops=dict(arrowstyle='->', color='#B0741A', lw=1.8))
        ax.text(cx / 2, cy / 2, f'  hard iron\n  {offset:.1f} $\\mu$T',
                fontsize=9.5, color='#B0741A')
    ax.set_xlabel('mag X ($\\mu$T)')
    ax.set_ylabel('mag Y ($\\mu$T)')
    ax.set_title('Horizontal field - the circle test')
    ax.set_aspect('equal')
    ax.legend(fontsize=9, loc='best')

    # 2. residual against heading - soft iron shows as a 2-cycle wave
    ax = fig.add_subplot(2, 2, 2)
    resid = np.hypot(mx - cx, my - cy) - r
    ax.scatter(heading, resid, s=4, alpha=0.4, color='#1D7A5F')
    ax.axhline(0, color='#5A5A5A', linestyle='--', linewidth=1)
    ax.set_xlabel('heading in the magnetic frame (deg)')
    ax.set_ylabel('radius error ($\\mu$T)')
    ax.set_title('Residual vs heading - a wave here means soft iron')
    ax.set_xlim(0, 360)

    # 3. total field magnitude over the run
    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, total, linewidth=0.8, color='#7C5CBF')
    ax.axhline(expect, color='#C0392B', linestyle='--', linewidth=1.2,
               label=f'expected {expect:.0f} $\\mu$T')
    ax.axhline(float(np.mean(total)), color='#5A5A5A', linestyle=':',
               linewidth=1.2, label=f'mean {np.mean(total):.1f} $\\mu$T')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('|B| ($\\mu$T)')
    ax.set_title('Total field strength - should be steady while rotating')
    ax.legend(fontsize=9)

    # 4. the three components
    ax = fig.add_subplot(2, 2, 4)
    for arr, lab, col in ((mx, 'X', '#1D7A5F'), (my, 'Y', '#2E5C9A'),
                          (mz, 'Z', '#B0741A')):
        ax.plot(t, arr, linewidth=0.8, color=col, label=lab)
    ax.set_xlabel('time (s)')
    ax.set_ylabel('$\\mu$T')
    ax.set_title('Components - X and Y should swing, Z stay flat')
    ax.legend(fontsize=9, ncol=3)

    fig.savefig(graph(f'{name}_magnetometer.png'), dpi=DPI)

    # ---------------- verdict ----------------
    lines = [
        f'bag                 {name}',
        f'samples             {n}',
        f'duration            {t[-1]:.1f} s',
        f'heading coverage    {covered:.0f}% of a full turn',
        '',
        'CIRCLE FIT, horizontal plane',
        f'  radius            {r:.2f} uT      (local field strength)',
        f'  centre            ({cx:+.2f}, {cy:+.2f}) uT',
        f'  offset from zero  {offset:.2f} uT   <- HARD IRON',
        f'  rms residual      {rms:.2f} uT',
        '',
        'ELLIPSE FIT',
        f'  major / minor     {ratio:.3f}        <- SOFT IRON if >> 1',
        f'  major axis at     {angle:+.0f} deg',
        '',
        'TOTAL FIELD',
        f'  mean              {np.mean(total):.2f} uT',
        f'  spread            {np.std(total):.2f} uT',
        f'  expected here     {expect:.1f} uT',
        '',
    ]

    if covered < 70:
        lines += [
            'COVERAGE TOO LOW',
            '',
            f'Only {covered:.0f}% of headings were visited, so the fit is',
            'extrapolating. Rotate through a full turn and record again.',
            '',
        ]

    verdict = []
    if offset < 5.0:
        verdict.append('  hard iron    SMALL - under 5 uT, nothing to chase')
    elif offset < 20.0:
        verdict.append('  hard iron    MODERATE - the chip corrects this, but '
                       'the source is close')
    else:
        verdict.append('  hard iron    LARGE - something magnetic is near the '
                       'sensor. Find it.')

    if ratio < 1.10:
        verdict.append('  soft iron    SMALL - circle is round, no '
                       'heading-dependent distortion')
    elif ratio < 1.30:
        verdict.append('  soft iron    MODERATE - some stretching, heading '
                       'error will vary by direction')
    else:
        verdict.append('  soft iron    LARGE - the field is badly distorted. '
                       'Relocate the sensor.')

    dev = abs(np.mean(total) - expect) / expect * 100
    if dev < 15:
        verdict.append('  field        matches the expected local value')
    else:
        verdict.append(f'  field        {dev:.0f}% off the expected value - '
                       f'the field is not predominantly the Earth\'s')

    if np.std(total) > 5.0:
        verdict.append('  stability    field strength MOVED during the run - '
                       'that is time-varying interference, which no '
                       'calibration corrects')

    lines += ['VERDICT'] + verdict + [
        '',
        'WHAT THESE MEAN',
        '',
        '  Hard iron is a constant field added by something fixed to the',
        '  robot. It shifts the circle off the origin. The chip estimates',
        '  and removes it, so a moderate offset is not fatal - but it',
        '  tells you something magnetic is close.',
        '',
        '  Soft iron is ferrous material bending the field, which makes',
        '  the circle an ellipse and the heading error depend on which',
        '  way the robot faces. The chip corrects this too, provided it',
        '  calibrated in this configuration.',
        '',
        '  Neither is corrected by the yaw_offset parameter, and neither',
        '  is the same as motor interference - that varies with current,',
        '  so it appears as scatter rather than a shifted or stretched',
        '  circle, and no calibration removes it.',
    ]

    report = '\n'.join(lines)
    print(report)
    with open(out(f'{name}_magnetometer.txt'), 'w') as fh:
        fh.write(report + '\n')
    print(f'\n  wrote results/graphs/{name}_magnetometer.png and .txt')


if __name__ == '__main__':
    main()
