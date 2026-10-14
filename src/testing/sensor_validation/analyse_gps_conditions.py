#!/usr/bin/env python3
"""Compare GPS behaviour across sky conditions.

Record the receiver STATIONARY in several places - open sky, under a
tree, beside a building, indoors - then run this with all of them. It
answers two different questions:

  How accurate is it?      scatter of the reported positions about
                           their own mean. The receiver is not moving,
                           so every bit of spread is error.

  Does it know?            the covariance it reports, against that
                           measured scatter. A receiver that reports 4 m
                           while scattering by 20 m is confidently
                           wrong, and that is worth knowing about before
                           a filter trusts it.

That second comparison is the point. HDOP measures satellite geometry,
not signal quality, so a multipath fix bouncing off a building can show
good geometry and a badly wrong position. The covariance is flagged
position_covariance_type 1 - approximated - for exactly this reason.

Usage:
    python3 analyse_gps_conditions.py open_sky tree building indoor
    python3 analyse_gps_conditions.py gps_open:"Open sky" gps_tree:"Tree"

Labels after a colon are used in the plot; without one the bag name is
used. Each bag should be at least two minutes, receiver still.

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
R_EARTH = 6378137.0

COLOURS = ['#1D7A5F', '#2E5C9A', '#B0741A', '#C0392B', '#7C5CBF', '#00838F']

plt.rcParams.update({
    'font.size': 10.5, 'axes.titlesize': 12.5, 'axes.labelsize': 11,
    'axes.grid': True, 'grid.alpha': 0.3, 'figure.autolayout': True,
})


def load(bag_path):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    fixes = []
    with AnyReader([Path(bag_path)], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == '/gps/fix']
        if not conns:
            raise SystemExit(f'/gps/fix not in {bag_path}')
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            fixes.append({
                't': t,
                'status': m.status.status,
                'lat': m.latitude,
                'lon': m.longitude,
                'var': m.position_covariance[0],
                'cov_type': m.position_covariance_type,
            })
    return fixes


def analyse(fixes, label):
    total = len(fixes)
    good = [f for f in fixes
            if f['status'] >= 0 and not math.isnan(f['lat'])]
    if len(good) < 10:
        return {'label': label, 'total': total, 'n': len(good),
                'usable': False}

    lat0 = np.mean([f['lat'] for f in good])
    lon0 = np.mean([f['lon'] for f in good])
    k = math.cos(math.radians(lat0))
    x = np.array([math.radians(f['lon'] - lon0) * R_EARTH * k for f in good])
    y = np.array([math.radians(f['lat'] - lat0) * R_EARTH for f in good])

    # Stationary receiver, so distance from the mean IS the error.
    d = np.hypot(x, y)
    reported = np.sqrt([f['var'] for f in good])
    dur = good[-1]['t'] - good[0]['t']

    return {
        'label': label,
        'usable': True,
        'total': total,
        'n': len(good),
        'nofix': total - len(good),
        'duration': dur,
        'rate': total / dur if dur > 0 else 0.0,
        'x': x, 'y': y,
        'measured_rms': float(np.sqrt(np.mean(d ** 2))),
        'measured_95': float(np.percentile(d, 95)),
        'max_err': float(np.max(d)),
        'reported_mean': float(np.mean(reported)),
        'reported_min': float(np.min(reported)),
        'reported_max': float(np.max(reported)),
        'cov_type': good[0]['cov_type'],
    }


def main():
    if len(sys.argv) < 2:
        raise SystemExit(
            f'usage: {sys.argv[0]} <bag[:label]> [<bag[:label]> ...]')

    results = []
    for arg in sys.argv[1:]:
        if arg.startswith('--'):
            continue
        if ':' in arg:
            bagname, label = arg.split(':', 1)
        else:
            bagname, label = arg, arg
        path = find_bag(bagname)
        results.append(analyse(load(path), label))

    usable = [r for r in results if r['usable']]
    if not usable:
        raise SystemExit('No bag had enough valid fixes.')

    # ---------------- figure ----------------
    fig = plt.figure(figsize=(14, 9))

    # 1. scatter, all conditions on one set of axes
    ax = fig.add_subplot(2, 2, 1)
    for i, r in enumerate(usable):
        c = COLOURS[i % len(COLOURS)]
        ax.scatter(r['x'], r['y'], s=5, alpha=0.35, color=c,
                   label=f"{r['label']}  rms {r['measured_rms']:.1f} m")
    ax.axhline(0, color='#999999', linewidth=0.8)
    ax.axvline(0, color='#999999', linewidth=0.8)
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.set_title('Scatter about each run\'s own mean, receiver stationary')
    ax.set_aspect('equal')
    ax.legend(fontsize=8.5, loc='best')

    # 2. reported vs measured - the honesty check
    ax = fig.add_subplot(2, 2, 2)
    lim = max(max(r['reported_mean'] for r in usable),
              max(r['measured_rms'] for r in usable)) * 1.25
    ax.plot([0, lim], [0, lim], '--', color='#5A5A5A', linewidth=1.2,
            label='honest: reported = measured')
    for i, r in enumerate(usable):
        c = COLOURS[i % len(COLOURS)]
        ax.scatter(r['reported_mean'], r['measured_rms'], s=90, color=c,
                   zorder=3, label=r['label'])
        ax.annotate(r['label'], (r['reported_mean'], r['measured_rms']),
                    textcoords='offset points', xytext=(7, 5), fontsize=9)
    ax.set_xlabel('reported 1$\\sigma$ (m)')
    ax.set_ylabel('measured rms error (m)')
    ax.set_title('Above the line = overconfident')
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.legend(fontsize=8.5, loc='upper left')

    # 3. bars, reported against measured
    ax = fig.add_subplot(2, 2, 3)
    idx = np.arange(len(usable))
    w = 0.38
    ax.bar(idx - w / 2, [r['reported_mean'] for r in usable], w,
           color='#2E5C9A', label='reported 1$\\sigma$')
    ax.bar(idx + w / 2, [r['measured_rms'] for r in usable], w,
           color='#C0392B', label='measured rms')
    ax.set_xticks(idx)
    ax.set_xticklabels([r['label'] for r in usable], fontsize=9,
                       rotation=15, ha='right')
    ax.set_ylabel('metres')
    ax.set_title('What it claims vs what it does')
    ax.legend(fontsize=9)

    # 4. fix availability
    ax = fig.add_subplot(2, 2, 4)
    ax.bar(idx, [r['n'] / r['total'] * 100 for r in usable],
           color='#1D7A5F')
    for i, r in enumerate(usable):
        ax.text(i, r['n'] / r['total'] * 100 + 1,
                f"{r['rate']:.1f} Hz", ha='center', fontsize=9)
    ax.set_xticks(idx)
    ax.set_xticklabels([r['label'] for r in usable], fontsize=9,
                       rotation=15, ha='right')
    ax.set_ylabel('% of messages with a fix')
    ax.set_ylim(0, 108)
    ax.set_title('Fix availability')

    fig.savefig(graph('gps_conditions.png'), dpi=DPI)

    # ---------------- report ----------------
    lines = ['GPS ACROSS CONDITIONS', '',
             '%-18s %7s %7s %9s %9s %9s %8s' %
             ('condition', 'fixes', 'rate', 'reported', 'measured',
              'max err', 'ratio'),
             '-' * 76]
    for r in results:
        if not r['usable']:
            lines.append('%-18s %7s  %s' %
                         (r['label'], r['n'], 'too few valid fixes'))
            continue
        ratio = r['measured_rms'] / r['reported_mean'] \
            if r['reported_mean'] > 0 else float('nan')
        lines.append('%-18s %7d %6.2fHz %8.2fm %8.2fm %8.2fm %7.2fx' %
                     (r['label'], r['n'], r['rate'], r['reported_mean'],
                      r['measured_rms'], r['max_err'], ratio))

    lines += ['', 'READING THIS', '']
    for r in usable:
        ratio = r['measured_rms'] / r['reported_mean'] \
            if r['reported_mean'] > 0 else float('nan')
        if ratio < 1.3:
            note = 'honest - reported uncertainty covers the real error'
        elif ratio < 2.5:
            note = 'optimistic - real error exceeds what it claims'
        else:
            note = ('OVERCONFIDENT - claims good accuracy while being '
                    'badly wrong')
        lines.append(f'  {r["label"]:<16} {note}')

    lines += [
        '',
        'The covariance is derived from HDOP, which describes satellite',
        'GEOMETRY and not signal quality. A fix bouncing off a building',
        'can come from well-spread satellites and still be tens of metres',
        'out, so geometry looks fine while the position is wrong. That is',
        'why the message flags it position_covariance_type 1, meaning',
        'approximated, and why a cross-check against an independent',
        'source matters more than self-reported confidence.',
        '',
        'The UBX binary protocol carries hAcc, computed by the receiver\'s',
        'own filter rather than inferred from geometry, which would give',
        'type 2. That is the fix for the overconfident rows above.',
    ]

    report = '\n'.join(lines)
    print(report)
    with open(out('gps_conditions.txt'), 'w') as fh:
        fh.write(report + '\n')
    print('\n  wrote results/graphs/gps_conditions.png and '
          'results/gps_conditions.txt')


if __name__ == '__main__':
    main()
