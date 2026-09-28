"""Shared paths for the sensor validation scripts.

Every script writes into results/ next to this file, regardless of which
directory it was run from. That removes a whole class of confusion - on
the laptop, outputs landed wherever the shell happened to be, and one
rectangle's plot overwrote another's because both used a fixed filename.

Bags are looked up in the current directory first, then results/bags, so
a bag recorded into either place is found by its bare name.
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, 'results')
BAGS = os.path.join(RESULTS, 'bags')
GRAPHS = os.path.join(RESULTS, 'graphs')

os.makedirs(BAGS, exist_ok=True)
os.makedirs(GRAPHS, exist_ok=True)


def out(name):
    """Absolute path for a data or report file inside results/.

    PNGs go to graphs/ instead - see graph().
    """
    return os.path.join(RESULTS, name)


def graph(name):
    """Absolute path for a figure inside results/graphs/."""
    return os.path.join(GRAPHS, name)


def find_bag(name):
    """Locate a bag directory by bare name, path, or under results/bags."""
    candidates = [
        name,
        os.path.join(BAGS, name),
        os.path.join(RESULTS, name),
        os.path.join(HERE, name),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    raise SystemExit(
        f"Bag '{name}' not found. Looked in:\n  "
        + '\n  '.join(candidates)
        + f"\n\nRecord bags with:\n  cd {BAGS}\n"
        f"  ros2 bag record /gps/fix /imu/data /odometry/local "
        f"/odometry/global /odometry/gps /tf /tf_static -o <name>")


def stem(bag_path):
    """Bag directory name, for naming outputs after their source."""
    return os.path.basename(os.path.normpath(bag_path))
