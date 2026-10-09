#!/usr/bin/env python3

"""
Wheel odometry for SAADS.

The ESP32 sends four signed wheel linear velocities over I2C.

I2C packet, little-endian:

    float32 front_left_mps
    float32 front_right_mps
    float32 rear_left_mps
    float32 rear_right_mps
    uint32  sequence

Total packet size: 20 bytes.

The ESP32 has already handled:

    Hall transitions
    motor direction
    motor RPM
    6.7:1 drivetrain reduction
    10.25 inch wheel diameter
    conversion to m/s

This node converts the four wheel velocities into:

    linear.x   [m/s]
    angular.z  [rad/s]

and publishes:

    /wheel_odom

The integrated pose in this message exists only for visualization and
debugging. robot_localization is configured to fuse only the twist.
"""

import math
import struct

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from smbus2 import SMBus, i2c_msg


# ============================================================
# ESP32 I2C PACKET
# ============================================================

PACKET_FORMAT = "<ffffI"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


class WheelOdometry(Node):

    def __init__(self):
        super().__init__("wheel_odometry")

        # ====================================================
        # PARAMETERS
        # ====================================================

        # Change this to the /dev/i2c-X bus connected to ESP32.
        self.declare_parameter("i2c_bus", 1)

        # ESP32 I2C slave address.
        self.declare_parameter("i2c_address", 0x12)

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # Effective left-right track width in metres.
        #
        # 0.45 m is currently a placeholder.
        # Measure and later calibrate this using known rotations.
        self.declare_parameter("track_width", 0.595)

        # ESP32 should already report vehicle-forward as positive.
        # These are available as emergency software corrections.
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        # Measurement variances.
        # These are initial values and should eventually be measured.
        self.declare_parameter("vx_variance", 0.01)
        self.declare_parameter("vyaw_variance", 0.02)

        # ESP32 creates a fresh velocity packet at 10 Hz.
        #
        # Poll at 20 Hz so we do not miss a packet because the
        # Jetson and ESP32 timers are not synchronized.
        #
        # Duplicate sequence numbers are discarded below.
        self.declare_parameter("poll_rate_hz", 20.0)

        # Reject obviously corrupt binary packets.
        self.declare_parameter("max_abs_wheel_speed_mps", 15.0)

        # ====================================================
        # LOAD PARAMETERS
        # ====================================================

        self.bus_number = int(
            self.get_parameter("i2c_bus").value
        )

        self.address = int(
            self.get_parameter("i2c_address").value
        )

        self.frame_id = str(
            self.get_parameter("frame_id").value
        )

        self.child_frame_id = str(
            self.get_parameter("child_frame_id").value
        )

        self.track_width = float(
            self.get_parameter("track_width").value
        )

        self.invert_left = bool(
            self.get_parameter("invert_left").value
        )

        self.invert_right = bool(
            self.get_parameter("invert_right").value
        )

        self.vx_variance = float(
            self.get_parameter("vx_variance").value
        )

        self.vyaw_variance = float(
            self.get_parameter("vyaw_variance").value
        )

        self.poll_rate_hz = float(
            self.get_parameter("poll_rate_hz").value
        )

        self.max_abs_wheel_speed = float(
            self.get_parameter("max_abs_wheel_speed_mps").value
        )

        if self.track_width <= 0.0:
            raise ValueError(
                "track_width must be greater than zero"
            )

        if self.poll_rate_hz <= 0.0:
            raise ValueError(
                "poll_rate_hz must be greater than zero"
            )

        # ====================================================
        # OPEN I2C BUS
        # ====================================================

        try:
            self.bus = SMBus(self.bus_number)

        except Exception as exc:
            self.get_logger().error(
                f"Could not open /dev/i2c-{self.bus_number}: {exc}"
            )
            raise

        # ====================================================
        # ROS PUBLISHER
        # ====================================================

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.publisher = self.create_publisher(
            Odometry,
            "/wheel_odom",
            qos,
        )

        # ====================================================
        # DEBUG / INTEGRATED POSE
        # ====================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_publish_time = None

        # ====================================================
        # PACKET MONITORING
        # ====================================================

        self.last_sequence = None

        self.good_packets = 0
        self.bad_packets = 0
        self.duplicate_packets = 0

        # ====================================================
        # POLLING TIMER
        # ====================================================

        self.timer = self.create_timer(
            1.0 / self.poll_rate_hz,
            self.update,
        )

        self.get_logger().info(
            "Wheel odometry started: "
            f"/dev/i2c-{self.bus_number}, "
            f"address 0x{self.address:02X}, "
            f"{PACKET_SIZE}-byte packets, "
            f"polling at {self.poll_rate_hz:.1f} Hz, "
            f"track width {self.track_width:.3f} m"
        )

    # ========================================================
    # READ ESP32
    # ========================================================

    def read_esp32(self):

        try:
            message = i2c_msg.read(
                self.address,
                PACKET_SIZE,
            )

            self.bus.i2c_rdwr(message)

            raw = bytes(message)

        except Exception as exc:
            self.bad_packets += 1

            if self.bad_packets in (1, 10, 100, 1000):
                self.get_logger().warn(
                    f"I2C read failed "
                    f"({self.bad_packets} failures): {exc}"
                )

            return None

        # ----------------------------------------------------
        # VERIFY PACKET LENGTH
        # ----------------------------------------------------

        if len(raw) != PACKET_SIZE:
            self.bad_packets += 1

            self.get_logger().warn(
                f"Expected {PACKET_SIZE} bytes, "
                f"received {len(raw)}"
            )

            return None

        # ----------------------------------------------------
        # UNPACK
        # ----------------------------------------------------

        try:
            (
                front_left,
                front_right,
                rear_left,
                rear_right,
                sequence,
            ) = struct.unpack(
                PACKET_FORMAT,
                raw,
            )

        except struct.error as exc:
            self.bad_packets += 1

            self.get_logger().warn(
                f"Could not unpack I2C packet: {exc}"
            )

            return None

        # ----------------------------------------------------
        # VALIDATE VALUES
        # ----------------------------------------------------

        wheel_speeds = (
            front_left,
            front_right,
            rear_left,
            rear_right,
        )

        for speed in wheel_speeds:

            if not math.isfinite(speed):
                self.bad_packets += 1

                self.get_logger().warn(
                    "Received non-finite wheel velocity"
                )

                return None

            if abs(speed) > self.max_abs_wheel_speed:
                self.bad_packets += 1

                self.get_logger().warn(
                    f"Rejected unreasonable wheel velocity "
                    f"{speed:.3f} m/s"
                )

                return None

        # ----------------------------------------------------
        # DISCARD DUPLICATE ESP32 PACKETS
        # ----------------------------------------------------

        if self.last_sequence is not None:

            if sequence == self.last_sequence:
                self.duplicate_packets += 1
                return None

        self.last_sequence = sequence
        self.good_packets += 1

        return (
            front_left,
            front_right,
            rear_left,
            rear_right,
        )

    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):

        packet = self.read_esp32()

        if packet is None:
            return

        (
            front_left,
            front_right,
            rear_left,
            rear_right,
        ) = packet

        # ----------------------------------------------------
        # OPTIONAL SIGN CORRECTION
        # ----------------------------------------------------

        if self.invert_left:
            front_left = -front_left
            rear_left = -rear_left

        if self.invert_right:
            front_right = -front_right
            rear_right = -rear_right

        # ----------------------------------------------------
        # SIDE VELOCITIES
        #
        # All four values are already in m/s.
        # ----------------------------------------------------

        left_velocity = (
            front_left + rear_left
        ) / 2.0

        right_velocity = (
            front_right + rear_right
        ) / 2.0

        # ----------------------------------------------------
        # SKID-STEER KINEMATICS
        #
        # vx = forward velocity [m/s]
        #
        # wz = yaw rate [rad/s]
        # ----------------------------------------------------

        vx = (
            left_velocity + right_velocity
        ) / 2.0

        wz = (
            right_velocity - left_velocity
        ) / self.track_width

        self.publish_odometry(
            vx,
            wz,
        )

    # ========================================================
    # PUBLISH ODOMETRY
    # ========================================================

    def publish_odometry(
        self,
        vx,
        wz,
    ):

        now = self.get_clock().now()

        current_time = (
            now.nanoseconds * 1e-9
        )

        # ----------------------------------------------------
        # INTEGRATE POSE
        #
        # Debugging / RViz only.
        # The EKFs do NOT fuse this pose.
        # ----------------------------------------------------

        if self.last_publish_time is not None:

            dt = (
                current_time
                - self.last_publish_time
            )

            if 0.0 < dt < 1.0:

                midpoint_yaw = (
                    self.yaw
                    + 0.5 * wz * dt
                )

                self.x += (
                    vx
                    * math.cos(midpoint_yaw)
                    * dt
                )

                self.y += (
                    vx
                    * math.sin(midpoint_yaw)
                    * dt
                )

                self.yaw += (
                    wz * dt
                )

                # Keep yaw bounded to [-pi, pi].
                self.yaw = math.atan2(
                    math.sin(self.yaw),
                    math.cos(self.yaw),
                )

        self.last_publish_time = current_time

        # ----------------------------------------------------
        # BUILD ODOMETRY MESSAGE
        # ----------------------------------------------------

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        # ----------------------------------------------------
        # DEBUG POSE
        # ----------------------------------------------------

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation = yaw_to_quaternion(
            self.yaw
        )

        # Pose is intentionally treated as very uncertain.
        msg.pose.covariance[0] = 1e3
        msg.pose.covariance[7] = 1e3
        msg.pose.covariance[14] = 1e3
        msg.pose.covariance[21] = 1e3
        msg.pose.covariance[28] = 1e3
        msg.pose.covariance[35] = 1e3

        # ----------------------------------------------------
        # TWIST
        # ----------------------------------------------------

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        # ----------------------------------------------------
        # TWIST COVARIANCE
        # ----------------------------------------------------

        # VX
        msg.twist.covariance[0] = self.vx_variance

        # VY
        msg.twist.covariance[7] = 1e3

        # VZ
        msg.twist.covariance[14] = 1e3

        # Vroll
        msg.twist.covariance[21] = 1e3

        # Vpitch
        msg.twist.covariance[28] = 1e3

        # Vyaw
        msg.twist.covariance[35] = self.vyaw_variance

        self.publisher.publish(msg)

    # ========================================================
    # CLEANUP
    # ========================================================

    def destroy_node(self):

        try:
            self.bus.close()

        except Exception:
            pass

        super().destroy_node()


# ============================================================
# YAW -> QUATERNION
# ============================================================

def yaw_to_quaternion(yaw):

    quaternion = Quaternion()

    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)

    return quaternion


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = WheelOdometry()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
