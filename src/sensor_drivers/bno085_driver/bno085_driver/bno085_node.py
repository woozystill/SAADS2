#!/usr/bin/env python3
"""BNO085 IMU driver for ROS 2.

Publishes sensor_msgs/Imu on /imu/data with a fused, magnetometer-
referenced orientation quaternion plus angular velocity and linear
acceleration. The chip runs sensor fusion on-board, so this feeds
robot_localization directly - no imu_filter_madgwick stage.

The report enabled is ROTATION_VECTOR, the magnetometer-referenced one
giving absolute heading. Do NOT switch to GAME_ROTATION_VECTOR: it
excludes the magnetometer deliberately and its yaw drifts without bound.

--------------------------------------------------------------------------
CALIBRATION
--------------------------------------------------------------------------
Symptom this addresses: heading differing by up to 20 degrees between
boots at the same physical orientation. The chip re-derives its magnetic
calibration each time it powers up, converging on whatever field it sees
during those first seconds. A different convergence gives a different
heading reference.

Three things fix that, and all three are needed together:

  begin_calibration()      explicitly enables dynamic calibration for
                           accelerometer, gyroscope and magnetometer.
                           CEVA documents accel and mag as default-on,
                           but calling it removes the assumption.

  BNO_REPORT_MAGNETOMETER  CEVA's procedure requires magnetometer output
                           at roughly 50 Hz for calibration to converge.
                           It is also the only report that updates
                           calibration_status - without it that value
                           reads 0 forever regardless of the true state.

  save_calibration_data()  writes the converged calibration to the chip's
                           flash, so the next boot starts from it instead
                           of re-deriving. This is the part that stops
                           heading moving between power cycles.

Call the save service once, with the sensor in its final mounted
position and the robot in its normal magnetic state:

    ros2 service call /bno085_node/save_calibration std_srvs/srv/Trigger

Move the sensor through figure-eights and 180-degree rotations about
each axis first, and wait for the logged accuracy to reach 2 or 3.
Re-run it whenever the sensor is physically moved or the magnetic
environment changes materially.

--------------------------------------------------------------------------
report_interval_us
--------------------------------------------------------------------------
The BNO085 does not wait to be polled. It STREAMS reports at a rate set
by report_interval_us, and the reader must drain them at least that fast
or it falls permanently behind - the queue grows and the node silently
stops publishing.

Measured through an MCP2221A USB-I2C bridge, roughly 57 ms per
transaction:

    interval    chip produces    reader drains    margin
    200000 us      5.0 Hz          33.9 Hz         6.8x
    150000 us      6.7 Hz          28.7 Hz         4.3x
    100000 us     10.0 Hz          12.9 Hz         1.3x
     66000 us     15.2 Hz          12.4 Hz         0.8x  <- FAILS

The default below is sized for native I2C on a Jetson header, where the
drain rate is orders of magnitude higher. Behind a USB bridge, raise it
to 150000 and drop publish_rate to 15.
--------------------------------------------------------------------------
"""

import math
from struct import unpack_from

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger

import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

# The SH-2 rotation vector report is 14 bytes: 4 bytes of header,
# 8 bytes of quaternion, then a 2-byte accuracy estimate in radians
# at Q-point 12. Adafruit's parser reads only the quaternion and
# discards the accuracy, so we read it off the raw report ourselves.
_ROTATION_VECTOR_LEN = 14
_ACCURACY_OFFSET = 12
_Q_POINT_12 = 2 ** -12

ACCURACY = {0: 'unreliable', 1: 'low', 2: 'medium', 3: 'high'}


class Bno085Node(Node):

    def __init__(self):
        super().__init__('bno085_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('i2c_address', 0x4A)

        # Native I2C default. Behind a USB bridge use 150000.
        self.declare_parameter('report_interval_us', 20000)

        # Magnetometer report. Needed for calibration to converge and for
        # calibration_status to mean anything. Costs a fourth stream on
        # the bus - if the publish rate collapses after enabling this,
        # the bus is saturated and the interval needs raising.
        self.declare_parameter('enable_magnetometer', True)
        self.declare_parameter('mag_report_interval_us', 20000)

        # Explicitly enable dynamic calibration at startup.
        self.declare_parameter('dynamic_calibration', True)

        # Seconds between calibration accuracy log lines. 0 disables.
        self.declare_parameter('calibration_log_period', 15.0)

        # Adaptive orientation covariance. When on, the heading
        # variance published with each message reflects how confident
        # the chip currently is, instead of a fixed datasheet number.
        # Two sources, in order of preference:
        #
        #   1. the per-reading accuracy estimate in radians that the
        #      chip sends with every rotation vector report
        #   2. the 0-3 magnetometer accuracy level, mapped below
        #
        # This is what lets robot_localization down-weight heading by
        # itself when the magnetic environment degrades - a motor
        # running, or a boot that converged badly - rather than trusting
        # it equally always.
        self.declare_parameter('adaptive_covariance', True)

        # Floor on reported heading sigma, degrees. The chip
        # occasionally reports implausibly small values; without a floor
        # those become near-infinite confidence and the filter stops
        # correcting heading at all.
        self.declare_parameter('min_orientation_sigma_deg', 1.0)

        # Fallback mapping from the 0-3 accuracy level to heading sigma
        # in degrees, used when the per-reading estimate is unavailable.
        # Only the 'high' entry is a datasheet figure (2.5 deg dynamic
        # heading accuracy); the rest are engineering judgment.
        self.declare_parameter('sigma_deg_by_accuracy', [45.0, 15.0, 5.0, 2.5])

        # Fixed fallback, used when adaptive_covariance is off or no
        # accuracy information is available at all.
        self.declare_parameter('orientation_variance', 0.0019)
        self.declare_parameter('angular_velocity_variance', 2.5e-5)
        self.declare_parameter('linear_acceleration_variance', 2.5e-3)

        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = self.get_parameter('publish_rate').value
        address = self.get_parameter('i2c_address').value
        interval = int(self.get_parameter('report_interval_us').value)
        self.use_mag = bool(self.get_parameter('enable_magnetometer').value)
        mag_interval = int(
            self.get_parameter('mag_report_interval_us').value)
        do_cal = bool(self.get_parameter('dynamic_calibration').value)
        cal_period = float(
            self.get_parameter('calibration_log_period').value)

        self.adaptive_cov = bool(
            self.get_parameter('adaptive_covariance').value)
        self.min_sigma = math.radians(float(
            self.get_parameter('min_orientation_sigma_deg').value))
        self.sigma_by_accuracy = [
            math.radians(v) for v in
            self.get_parameter('sigma_deg_by_accuracy').value]
        self.orient_var = float(
            self.get_parameter('orientation_variance').value)

        # Updated by the report hook below; None until the first
        # rotation vector report carrying a usable estimate arrives.
        self._heading_sigma = None
        self.gyro_var = float(
            self.get_parameter('angular_velocity_variance').value)
        self.accel_var = float(
            self.get_parameter('linear_acceleration_variance').value)

        produced_hz = 1e6 / interval
        if rate_hz > produced_hz * 3:
            self.get_logger().warn(
                f'publish_rate {rate_hz} Hz is far above the '
                f'{produced_hz:.1f} Hz the chip produces per feature. Most '
                f'published messages will repeat cached values.')

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = BNO08X_I2C(i2c, address=address)
        except Exception as exc:
            self.get_logger().error(
                f'Could not open BNO085 at 0x{address:02X}: {exc}')
            self.get_logger().error(
                'On a Jetson, check i2cdetect -y -r 7 shows 4a. If the chip '
                'answers but features will not enable, its protocol state is '
                'stuck - disconnect ALL FOUR wires for 10 s, since leaving '
                'SDA and SCL connected keeps it alive through the bus '
                'pull-ups and a VIN-only cycle does nothing.')
            raise

        for feature in (BNO_REPORT_ROTATION_VECTOR,
                        BNO_REPORT_GYROSCOPE,
                        BNO_REPORT_ACCELEROMETER):
            self.sensor.enable_feature(feature, report_interval=interval)

        if self.use_mag:
            self.sensor.enable_feature(BNO_REPORT_MAGNETOMETER,
                                       report_interval=mag_interval)

        if self.adaptive_cov:
            self._install_accuracy_hook()

        if do_cal:
            try:
                self.sensor.begin_calibration()
                self.get_logger().info(
                    'Dynamic calibration enabled (accel, gyro, mag)')
            except Exception as exc:
                self.get_logger().warn(
                    f'begin_calibration() failed: {exc}')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(Imu, 'imu/data', qos)
        self.mag_publisher = self.create_publisher(
            MagneticField, 'imu/mag', qos)

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_reading)

        # Writes the converged calibration to the chip's flash. Call it
        # once the logged accuracy reaches 2 or 3, with the sensor in its
        # final mounted position.
        self.save_srv = self.create_service(
            Trigger, '~/save_calibration', self.save_calibration)

        if cal_period > 0 and self.use_mag:
            self.create_timer(cal_period, self.log_accuracy)

        self.read_failures = 0

        self.get_logger().info(
            f'BNO085 up at 0x{address:02X}, reports every {interval} us '
            f'({produced_hz:.1f} Hz/feature), mag '
            f'{"on" if self.use_mag else "off"}, covariance '
            f'{"adaptive" if self.adaptive_cov else "fixed"}, publishing imu/data at '
            f'{rate_hz} Hz in frame {self.frame_id}')

    # ------------------------------------------------------------------
    def _install_accuracy_hook(self):
        """Capture the per-reading heading accuracy the library discards.

        Wraps the sensor's report handler on this instance only, reads
        the accuracy estimate out of each rotation vector report, then
        hands the report on unchanged. If the field is missing or the
        library's internals differ, the wrapper degrades to a no-op and
        the covariance falls back to the accuracy-level mapping.
        """
        original = self.sensor._process_report

        def hooked(report_id, report_bytes):
            if (report_id == BNO_REPORT_ROTATION_VECTOR
                    and len(report_bytes) >= _ROTATION_VECTOR_LEN):
                try:
                    raw = unpack_from('<h', report_bytes,
                                      offset=_ACCURACY_OFFSET)[0]
                    sigma = abs(raw * _Q_POINT_12)
                    # A zero estimate means the chip has not formed one
                    # yet, not that heading is perfect.
                    if sigma > 0.0:
                        self._heading_sigma = sigma
                except Exception:
                    pass
            return original(report_id, report_bytes)

        try:
            self.sensor._process_report = hooked
            self.get_logger().info(
                'Adaptive covariance on: reading the per-report heading '
                'accuracy estimate')
        except Exception as exc:
            self.get_logger().warn(
                f'Could not hook report handler ({exc}); falling back to '
                f'the accuracy-level mapping')

    def _orientation_variance(self):
        """Heading variance to publish with this message, in rad^2."""
        if not self.adaptive_cov:
            return self.orient_var

        sigma = self._heading_sigma
        if sigma is None:
            # No per-reading estimate yet - fall back to the 0-3 level.
            try:
                level = self.sensor.calibration_status
            except Exception:
                return self.orient_var
            if not 0 <= level < len(self.sigma_by_accuracy):
                return self.orient_var
            sigma = self.sigma_by_accuracy[level]

        return max(sigma, self.min_sigma) ** 2

    # ------------------------------------------------------------------
    def log_accuracy(self):
        """Report the chip's own magnetometer calibration accuracy.

        Only meaningful while the magnetometer report is enabled - the
        library updates this value from that report and nothing else.
        """
        try:
            level = self.sensor.calibration_status
        except Exception:
            return
        text = f'Mag calibration: {ACCURACY.get(level, level)} ({level}/3)'
        if level >= 2:
            self.get_logger().info(
                text + ' - ready to save: ros2 service call '
                '/bno085_node/save_calibration std_srvs/srv/Trigger')
        else:
            self.get_logger().warn(
                text + ' - rotate through figure-eights and 180 deg on '
                'each axis to converge')

    def save_calibration(self, request, response):
        """Write the current calibration to the chip's flash.

        Without this the chip re-derives its magnetic calibration on
        every power-up, which is why heading can differ by tens of
        degrees between boots at the same physical orientation.
        """
        try:
            level = self.sensor.calibration_status
        except Exception:
            level = -1

        if 0 <= level < 2:
            response.success = False
            response.message = (
                f'Accuracy only {level}/3. Move the sensor through '
                f'figure-eights and 180 deg rotations on each axis until '
                f'it reaches 2 or 3, then call again. Saving a poor '
                f'calibration makes every future boot start from it.')
            self.get_logger().warn(response.message)
            return response

        try:
            self.sensor.save_calibration_data()
        except Exception as exc:
            response.success = False
            response.message = f'save_calibration_data() failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f'Calibration saved to flash at accuracy {level}/3. Future '
            f'boots will start from this instead of re-deriving.')
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    def publish_reading(self):
        try:
            qx, qy, qz, qw = self.sensor.quaternion
            gx, gy, gz = self.sensor.gyro
            ax, ay, az = self.sensor.acceleration
            self.read_failures = 0
        except (OSError, RuntimeError, KeyError) as exc:
            self.read_failures += 1
            if self.read_failures in (1, 10, 100):
                self.get_logger().warn(
                    f'I2C read failed ({self.read_failures}x): {exc}')
            return

        # A dropped SHTP packet can leave a zero quaternion, which is not
        # a valid rotation. Skip rather than publish nonsense.
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 0.9 or norm > 1.1:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        # Roll and pitch come from gravity and are consistently good;
        # only heading degrades with the magnetic environment, so only
        # the yaw term adapts.
        yaw_var = self._orientation_variance()
        msg.orientation_covariance = [
            self.orient_var, 0.0, 0.0,
            0.0, self.orient_var, 0.0,
            0.0, 0.0, yaw_var,
        ]
        msg.angular_velocity_covariance = [
            self.gyro_var, 0.0, 0.0,
            0.0, self.gyro_var, 0.0,
            0.0, 0.0, self.gyro_var,
        ]
        msg.linear_acceleration_covariance = [
            self.accel_var, 0.0, 0.0,
            0.0, self.accel_var, 0.0,
            0.0, 0.0, self.accel_var,
        ]

        self.publisher.publish(msg)

        if self.use_mag:
            try:
                mx, my, mz = self.sensor.magnetic
            except (OSError, RuntimeError, KeyError):
                return
            mag = MagneticField()
            mag.header = msg.header
            # The chip reports microtesla; MagneticField is in tesla.
            mag.magnetic_field.x = float(mx) * 1e-6
            mag.magnetic_field.y = float(my) * 1e-6
            mag.magnetic_field.z = float(mz) * 1e-6
            self.mag_publisher.publish(mag)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bno085Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
