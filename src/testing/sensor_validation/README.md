# Sensor validation

Test and analysis scripts for the localization stack. Every script writes
into `results/` next to this file, regardless of which directory you run
it from, and names its outputs after the bag they came from so runs do
not overwrite each other.

    sensor_validation/
      *.py                 the scripts
      results/
        graphs/            PNG figures
        bags/              recorded rosbags
        *.txt              reports and statistics
        *.csv              raw samples, opens in Excel

These are plain Python scripts, not a ROS package - there is a
`COLCON_IGNORE` here so colcon does not try to build them or scan the
recorded bags.

## Setup, once

    pip3 install matplotlib rosbags

## Live check

    python3 show_yaw.py

Prints heading in place on one line: ENU yaw (0 = east, 90 = north,
counter-clockwise), compass bearing, and roll/pitch. Use it to sanity
check the IMU, and to watch for interference - bring metal near the
sensor and see whether the heading moves.

## Bench tests, no GPS needed

Run these with the IMU publishing. Each writes a PNG and a TXT into
`results/`.

    python3 verify_imu.py all        # full suite, ~10 min, writes a report
    python3 plot_imu.py drift 300    # 5 min stationary - the headline test
    python3 test_level.py            # tilt bias, 4 flat rotations
    python3 test_gravity.py 60       # accelerometer scale, stationary

**drift** is the most informative. An unfused gyro accumulates 1-10
deg/min without bound; a magnetometer-referenced fusion settles into a
small envelope and stays there. Measured on the laptop: 0.036 deg/min
over 5 minutes.

**test_level** rotates the sensor on one flat surface. Averaging four
evenly spaced positions cancels the surface tilt exactly, leaving the
sensor's own bias. This beats the six-face magnitude test, which could
not be repeated well enough to measure anything - balancing a breakout
board on its edge is not reproducible.

**test_gravity** keeps the sensor still. A stationary accelerometer
senses exactly one thing, so the magnitude must be 9.807 m/s^2.

## Field tests, GPS needed

Record into `results/bags` so the analysis scripts find bags by bare
name:

    cd results/bags
    ros2 bag record /gps/fix /imu/data /odometry/local /odometry/global \
        /odometry/gps /tf /tf_static -o rect_01

Stop with a SINGLE Ctrl+C and wait for "Recording stopped" - killing it
harder leaves the bag without its metadata.yaml, and it then needs
reindexing plus two hand patches before anything can read it.

Then, from this directory:

    python3 analyse_rectangle.py rect_01 --min-move 5
    python3 analyse_straight_line.py line_01 --min-move 10

### Rectangle

Walk a closed loop, four sides, returning to the exact start. This is
the strongest single test: every heading is captured in one recording
with the sensor never moved, so an offset difference between sides
cannot be blamed on remounting.

It reports:

- **loop closure** - how far the estimate ended from where it started,
  for both raw GPS and the fused estimate
- **offset vs heading** - flat means a constant offset; a sinusoid means
  soft-iron distortion, which no parameter corrects

Segments that span a corner are detected and excluded. Through a turn
the GPS course averages over the whole corner while the IMU yaw is an
instant reading, so they legitimately disagree and would otherwise
inflate the residual.

### Straight line

80-100 m in one direction, as straight as you can manage. Use
`--min-move 10` so each course estimate is taken over a baseline longer
than the GPS position noise - shorter segments measure noise, not
heading.

## On yaw_offset

Both analysis scripts report the angle between the IMU x-axis and the
direction of travel. **Do not put that number into
`navsat_transform`'s `yaw_offset`.**

This was tested directly. Setting +38.5 deg rotated the map frame by
-39 deg; setting -48.7 deg rotated it by +48.7 deg. The rotation tracked
the parameter, not the measured offset, so any non-zero value skewed the
track. `yaw_offset: 0.0` produced a fused track that overlays raw GPS.

What the measured offset is good for: confirming it stays constant
across all headings, which rules out magnetic distortion, and telling
you how far the sensor is rotated if you want to straighten the
mounting.

## Reference results, laptop bench

For comparison when you re-run these with the sensors mounted:

| Test | Result |
| --- | --- |
| Stationary yaw drift, 5 min | 0.036 deg/min |
| Heading residual after offset | 0.70 deg |
| Offset consistency, 4 rectangles | sinusoid amplitude 0.5-0.6 deg |
| Rotation closure, full turn | 0.94 deg |
| Tilt bias | 0.72 deg |
| Accelerometer magnitude | -0.82% of g |
| Loop closure, 220 m, fused | 4.15 m (raw GPS 5.33 m) |

These describe a board taped to a laptop. Heading accuracy depends on
the magnetic environment, so they must be re-measured once the sensor is
on the robot.

## On the robot, before trusting anything

Run this first, before measuring offsets:

    python3 show_yaw.py
    # robot stationary, wheels clear, ramp motors 0 -> full -> 0

If heading moves with throttle, the sensor is reading motor current.
Calibration cannot fix that - it corrects static fields, and motor
current is not static. Relocate the sensor and test again.
