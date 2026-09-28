#!/usr/bin/env python3
"""Verification suite for a fused IMU publishing sensor_msgs/Imu.

Runs objective tests and writes a timestamped report you can hand to
someone else. Each test targets a specific failure mode, so a pass set is
evidence rather than an assertion.

Usage:
    python3 verify_imu.py drift      [seconds]   default 300
    python3 verify_imu.py cardinal               interactive, 4 headings
    python3 verify_imu.py gravity    [seconds]   default 60
    python3 verify_imu.py loop                   interactive, 360 return
    python3 verify_imu.py all                    everything in sequence

Report is appended to imu_verification_report.txt in the working directory.
"""

import math
import statistics
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

from _paths import out

REPORT = out('imu_verification_report.txt')
TOPIC = '/imu/data'


def quat_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in degrees. Yaw wrapped to [0, 360)."""
    roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                   1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                  1 - 2 * (y * y + z * z))) % 360.0
    return roll, pitch, yaw


def angle_diff(a, b):
    """Signed shortest difference a - b, in degrees."""
    return (a - b + 540.0) % 360.0 - 180.0


class Collector(Node):
    """Subscribes to the IMU topic and buffers whatever arrives."""

    def __init__(self):
        super().__init__('imu_verifier')
        self.samples = []
        # The driver publishes BEST_EFFORT. A RELIABLE subscription
        # will not connect to it and receives nothing, silently.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, TOPIC, self._cb, qos)

    def _cb(self, msg):
        o = msg.orientation
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.samples.append({
            't': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'quat': (o.x, o.y, o.z, o.w),
            'accel': (a.x, a.y, a.z),
            'gyro': (g.x, g.y, g.z),
            'frame': msg.header.frame_id,
            'ocov': msg.orientation_covariance[0],
            'gcov': msg.angular_velocity_covariance[0],
            'acov': msg.linear_acceleration_covariance[0],
        })

    def gather(self, seconds, label=None):
        """Spin for a fixed duration, returning the samples collected."""
        self.samples = []
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if label:
                left = end - time.time()
                print(f'  {label}: {left:5.1f}s remaining   ',
                      end='\r', flush=True)
        if label:
            print(' ' * 50, end='\r')
        return list(self.samples)


def write(lines):
    text = '\n'.join(lines)
    print(text)
    with open(REPORT, 'a') as fh:
        fh.write(text + '\n')


def header(title):
    return [
        '',
        '=' * 68,
        f'{title}',
        f'{datetime.now().isoformat(timespec="seconds")}',
        '=' * 68,
    ]


# ---------------------------------------------------------------------------
# Test 1: static drift
# ---------------------------------------------------------------------------

def test_drift(node, seconds):
    """Yaw must not wander while the sensor is stationary.

    This is the single most informative test. A 6-axis IMU, or a 9-axis one
    whose magnetometer is not being used, integrates gyro bias and drifts
    steadily - typically degrees per minute. A working magnetometer-
    referenced fusion holds yaw indefinitely.
    """
    out = header('TEST 1 - STATIC YAW DRIFT')
    out.append('Sensor must be STATIONARY for the whole test.')
    out.append(f'Duration: {seconds} s')
    out.append('')
    print('\n'.join(out[:-1]))

    samples = node.gather(seconds, 'collecting')
    if len(samples) < 10:
        write(out + ['FAIL - too few samples. Is the driver running?'])
        return False

    yaws = [quat_to_euler(*s['quat'])[2] for s in samples]
    t0 = samples[0]['t']
    span = samples[-1]['t'] - t0

    # Unwrap so a 359->1 crossing is not read as a 358 degree jump.
    unwrapped = [yaws[0]]
    for y in yaws[1:]:
        unwrapped.append(unwrapped[-1] + angle_diff(y, unwrapped[-1] % 360))

    total = unwrapped[-1] - unwrapped[0]
    rate = total / span * 60.0 if span > 0 else 0.0
    noise = statistics.pstdev(
        [angle_diff(y, statistics.median(yaws)) for y in yaws])

    out += [
        f'Samples:            {len(samples)}',
        f'Rate:               {len(samples)/span:.1f} Hz',
        f'Start yaw:          {yaws[0]:.2f} deg',
        f'End yaw:            {yaws[-1]:.2f} deg',
        f'Total drift:        {total:+.2f} deg over {span:.0f} s',
        f'Drift rate:         {rate:+.3f} deg/min',
        f'Noise (1 sigma):    {noise:.3f} deg',
        '',
        'Reference: an unfused gyro drifts 1-10 deg/min or worse.',
        'A magnetometer-referenced fusion should stay under 0.1 deg/min.',
        '',
    ]
    ok = abs(rate) < 0.5
    out.append('PASS - yaw is magnetically referenced, not free-running.'
               if ok else
               f'FAIL - drifting {rate:+.2f} deg/min. Magnetometer may not '
               'be contributing.')
    write(out)
    return ok


# ---------------------------------------------------------------------------
# Test 2: cardinal headings
# ---------------------------------------------------------------------------

def test_cardinal(node):
    """Yaw at four known headings must match the expected convention.

    Four readings 90 degrees apart distinguish three things at once: the
    world-frame convention (ENU vs NED), the sign of rotation, and the
    constant offset. A consistent offset across all four is magnetic
    declination; an inconsistent one is interference or a bad fit.
    """
    out = header('TEST 2 - CARDINAL HEADINGS')
    out.append('Point the sensor X axis at each direction using a compass')
    out.append('or map. Keep it level. 5 s average at each heading.')
    out.append('')
    print('\n'.join(out))

    expected = {'North': 90.0, 'East': 0.0, 'South': 270.0, 'West': 180.0}
    measured = {}

    for name, exp in expected.items():
        input(f'  Point X at {name.upper()}, hold level, press Enter...')
        s = node.gather(5.0, f'{name}')
        if len(s) < 5:
            write(out + [f'FAIL - no data while measuring {name}'])
            return False
        yaws = [quat_to_euler(*x['quat'])[2] for x in s]
        ref = yaws[0]
        avg = (ref + statistics.mean(
            [angle_diff(y, ref) for y in yaws])) % 360.0
        measured[name] = avg
        print(f'  {name}: {avg:.2f} deg')

    out += ['', 'Heading      measured   expected   offset', '-' * 44]
    offsets = []
    for name, exp in expected.items():
        off = angle_diff(measured[name], exp)
        offsets.append(off)
        out.append(f'{name:<12} {measured[name]:8.2f}   {exp:8.1f}   {off:+7.2f}')

    mean_off = statistics.mean(offsets)
    spread = max(offsets) - min(offsets)

    out += [
        '',
        f'Mean offset:        {mean_off:+.2f} deg',
        f'Spread:             {spread:.2f} deg',
        '',
    ]

    if spread < 15.0:
        out += [
            'PASS - offsets are consistent across all four headings.',
            'A consistent offset is magnetic declination, not sensor error.',
            f'Suggested navsat_transform setting:',
            f'  magnetic_declination_radians: {math.radians(mean_off):.4f}',
            f'  yaw_offset: 0.0   (ENU convention confirmed)',
        ]
        ok = True
    else:
        out += [
            f'FAIL - offsets vary by {spread:.1f} deg between headings.',
            'A heading-dependent error means soft-iron distortion or',
            'nearby interference, not declination.',
        ]
        ok = False
    write(out)
    return ok


# ---------------------------------------------------------------------------
# Test 3: gravity magnitude
# ---------------------------------------------------------------------------

def test_gravity(node, seconds):
    """Accelerometer magnitude must equal g regardless of orientation.

    Rotate the sensor slowly throughout. If scale factors or biases are
    wrong the magnitude changes with attitude; if they are right it stays
    at 9.81 m/s^2 whichever way the sensor faces.
    """
    out = header('TEST 3 - GRAVITY MAGNITUDE')
    out.append('Rotate the sensor SLOWLY through many orientations.')
    out.append(f'Duration: {seconds} s')
    out.append('')
    print('\n'.join(out))

    samples = node.gather(seconds, 'rotate slowly')
    if len(samples) < 20:
        write(out + ['FAIL - too few samples.'])
        return False

    mags = [math.sqrt(sum(c * c for c in s['accel'])) for s in samples]
    mean = statistics.mean(mags)
    sd = statistics.pstdev(mags)

    out += [
        f'Samples:            {len(samples)}',
        f'Mean magnitude:     {mean:.3f} m/s^2',
        f'Std deviation:      {sd:.3f} m/s^2',
        f'Min / max:          {min(mags):.3f} / {max(mags):.3f}',
        f'Expected:           9.807 m/s^2',
        f'Error:              {mean - 9.807:+.3f} m/s^2 '
        f'({(mean - 9.807) / 9.807 * 100:+.2f}%)',
        '',
    ]
    ok = abs(mean - 9.807) < 0.3 and sd < 0.6
    out.append('PASS - accelerometer scale is correct in all orientations.'
               if ok else
               'FAIL - magnitude varies with orientation or is offset. '
               'Suggests a scale or bias error.')
    write(out)
    return ok


# ---------------------------------------------------------------------------
# Test 4: rotation closure
# ---------------------------------------------------------------------------

def test_loop(node):
    """A full turn returning to the start must report the same yaw.

    Catches accumulation error: a fusion that scales rotation incorrectly
    will not close the loop even though it looks smooth while turning.
    """
    out = header('TEST 4 - ROTATION CLOSURE')
    out.append('Mark the sensor position. Rotate a full 360 degrees and')
    out.append('return it to exactly the same spot.')
    out.append('')
    print('\n'.join(out))

    input('  Place at start position, press Enter...')
    s = node.gather(3.0, 'start')
    if len(s) < 5:
        write(out + ['FAIL - no data.'])
        return False
    start = quat_to_euler(*s[-1]['quat'])[2]
    print(f'  start yaw: {start:.2f} deg')

    input('  Rotate 360 degrees, return to the same spot, press Enter...')
    s = node.gather(3.0, 'end')
    end = quat_to_euler(*s[-1]['quat'])[2]

    err = angle_diff(end, start)
    out += [
        f'Start yaw:          {start:.2f} deg',
        f'End yaw:            {end:.2f} deg',
        f'Closure error:      {err:+.2f} deg',
        '',
    ]
    ok = abs(err) < 5.0
    out.append('PASS - full rotation closes. No accumulation error.'
               if ok else
               f'FAIL - {err:+.1f} deg of error after one turn.')
    write(out)
    return ok


# ---------------------------------------------------------------------------
# Message structure
# ---------------------------------------------------------------------------

def test_message(node):
    """Every field robot_localization reads must be present and non-zero."""
    out = header('TEST 0 - MESSAGE STRUCTURE')
    s = node.gather(3.0, 'sampling')
    if not s:
        write(out + [f'FAIL - nothing published on {TOPIC}'])
        return False

    m = s[-1]
    q = m['quat']
    norm = math.sqrt(sum(c * c for c in q))

    checks = [
        ('topic publishing', True, f'{len(s)} msgs in 3 s'),
        ('frame_id set', bool(m['frame']), m['frame']),
        ('quaternion is unit', abs(norm - 1.0) < 0.01, f'norm {norm:.5f}'),
        ('orientation covariance', m['ocov'] > 0, f'{m["ocov"]:g}'),
        ('angular vel covariance', m['gcov'] > 0, f'{m["gcov"]:g}'),
        ('linear acc covariance', m['acov'] > 0, f'{m["acov"]:g}'),
    ]
    for label, passed, detail in checks:
        out.append(f'  [{"ok" if passed else "XX"}] {label:<26} {detail}')

    ok = all(c[1] for c in checks)
    out += ['', 'PASS - message is complete for robot_localization.' if ok
            else 'FAIL - missing or zero fields.']
    write(out)
    return ok


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)

    mode = sys.argv[1]
    arg = float(sys.argv[2]) if len(sys.argv) > 2 else None

    rclpy.init()
    node = Collector()
    results = {}

    try:
        if mode in ('message', 'all'):
            results['structure'] = test_message(node)
        if mode in ('drift', 'all'):
            results['drift'] = test_drift(node, arg or 300)
        if mode in ('cardinal', 'all'):
            results['cardinal'] = test_cardinal(node)
        if mode in ('gravity', 'all'):
            results['gravity'] = test_gravity(node, arg or 60)
        if mode in ('loop', 'all'):
            results['closure'] = test_loop(node)

        if not results:
            raise SystemExit(__doc__)

        if len(results) > 1:
            summary = header('SUMMARY')
            for k, v in results.items():
                summary.append(f'  {k:<12} {"PASS" if v else "FAIL"}')
            summary += ['', 'ALL TESTS PASSED' if all(results.values())
                        else 'SOME TESTS FAILED']
            write(summary)

        print(f'\nReport appended to {REPORT}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
