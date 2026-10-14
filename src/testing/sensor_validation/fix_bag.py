#!/usr/bin/env python3
"""Rebuild a bag's metadata.yaml from its .db3.

A bag killed with anything harsher than a single Ctrl+C ends up with an
empty metadata.yaml, and then nothing can read it - ros2 bag info, ros2
bag play, and the analysis scripts all fail. Everything needed to
rebuild that file is already in the database, so there is no reason to
lose the recording.

Usage:
    python3 fix_bag.py results/bags/mag_motor_thr
"""

import os
import sqlite3
import sys


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f'usage: {sys.argv[0]} <bag_directory>')

    bag = sys.argv[1].rstrip('/')
    if not os.path.isdir(bag):
        raise SystemExit(f'Not a directory: {bag}')

    dbs = [f for f in sorted(os.listdir(bag)) if f.endswith('.db3')]
    if not dbs:
        raise SystemExit(f'No .db3 file in {bag}')

    conn = sqlite3.connect(os.path.join(bag, dbs[0]))
    topics = list(conn.execute(
        'SELECT id, name, type, serialization_format FROM topics'))
    lo, hi = conn.execute(
        'SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
    total = conn.execute('SELECT COUNT(*) FROM messages').fetchone()[0]

    if not total:
        raise SystemExit('The database has no messages.')

    lines = [
        'rosbag2_bagfile_information:',
        '  version: 4',
        '  storage_identifier: sqlite3',
        '  relative_file_paths:',
    ]
    for d in dbs:
        lines.append(f'    - {d}')
    lines += [
        '  duration:',
        f'    nanoseconds: {hi - lo}',
        '  starting_time:',
        f'    nanoseconds_since_epoch: {lo}',
        f'  message_count: {total}',
        '  topics_with_message_count:',
    ]
    for tid, name, typ, fmt in topics:
        n = conn.execute(
            'SELECT COUNT(*) FROM messages WHERE topic_id=?',
            (tid,)).fetchone()[0]
        lines += [
            '    - topic_metadata:',
            f'        name: {name}',
            f'        type: {typ}',
            f'        serialization_format: {fmt}',
            '        offered_qos_profiles: ""',
            f'      message_count: {n}',
        ]
    lines += [
        '  compression_format: ""',
        '  compression_mode: ""',
        '',
    ]

    path = os.path.join(bag, 'metadata.yaml')
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))

    print(f'wrote {path}')
    print(f'  {total} messages over {(hi - lo) / 1e9:.1f} s')
    for tid, name, typ, _ in topics:
        n = conn.execute(
            'SELECT COUNT(*) FROM messages WHERE topic_id=?',
            (tid,)).fetchone()[0]
        print(f'  {name:<24} {n} msgs')


if __name__ == '__main__':
    main()
