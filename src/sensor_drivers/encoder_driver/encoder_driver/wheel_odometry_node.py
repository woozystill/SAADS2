#!/usr/bin/env python3

"""
SAADS wheel odometry + ESP32 I2C motor bridge.

The Jetson is the I2C master and the ESP32 is slave address 0x12.
The same address is used in both directions:

ESP32 -> Jetson telemetry (20 bytes, little-endian):
    float32 front_left_mps
    float32 front_right_mps
    float32 rear_left_mps
    float32 rear_right_mps
    uint32  sequence

Jetson -> ESP32 command (8 bytes):
    uint8   magic = 0xA5
    uint8   sequence
    int8    front_left_percent
    int8    front_right_percent
    int8    rear_left_percent
    int8    rear_right_percent
    uint8   flags
    uint8   checksum = XOR(bytes 0..6)

ROS interfaces:
    publishes /wheel_odom
    publishes /motor/wheel_speeds
    publishes /motor/controller_status

    subscribes /motor/setpoints
    subscribes /motor/arm
    subscribes /motor/stop
    subscribes /motor/reset_halls

Only this node should open /dev/i2c-1 while it is running.
"""

import json
import math
import struct

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32MultiArray, String

from smbus2 import SMBus, i2c_msg


# ============================================================
# ESP32 -> JETSON TELEMETRY PACKET
# ============================================================

TELEMETRY_FORMAT = "<ffffI"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)


# ============================================================
# JETSON -> ESP32 MOTOR COMMAND PACKET
# ============================================================

COMMAND_MAGIC = 0xA5
COMMAND_FORMAT = "<BBbbbbBB"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

FLAG_ARM = 0x01
FLAG_STOP = 0x02
FLAG_RESET_HALL = 0x04


class WheelOdometry(Node):

    def __init__(self):
        super().__init__("wheel_odometry")

        # ====================================================
        # PARAMETERS
        # ====================================================

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x12)

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        # Current measured/selected effective left-right track width.
        self.declare_parameter("track_width", 0.595)

        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        self.declare_parameter("vx_variance", 0.01)
        self.declare_parameter("vyaw_variance", 0.02)

        # ESP32 telemetry is refreshed at 50 Hz. Polling at 20 Hz is
        # sufficient for the current ROS odometry/control loop.
        self.declare_parameter("poll_rate_hz", 20.0)

        self.declare_parameter("max_abs_wheel_speed_mps", 15.0)

        # If no fresh setpoint arrives from the GUI/ROS command source,
        # command all four motors back to zero.
        self.declare_parameter("setpoint_timeout_s", 0.50)

        # ====================================================
        # LOAD PARAMETERS
        # ====================================================

        self.bus_number = int(self.get_parameter("i2c_bus").value)
        self.address = int(self.get_parameter("i2c_address").value)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(
            self.get_parameter("child_frame_id").value
        )

        self.track_width = float(self.get_parameter("track_width").value)

        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)

        self.vx_variance = float(self.get_parameter("vx_variance").value)
        self.vyaw_variance = float(
            self.get_parameter("vyaw_variance").value
        )

        self.poll_rate_hz = float(
            self.get_parameter("poll_rate_hz").value
        )

        self.max_abs_wheel_speed = float(
            self.get_parameter("max_abs_wheel_speed_mps").value
        )

        self.setpoint_timeout_s = float(
            self.get_parameter("setpoint_timeout_s").value
        )

        if self.track_width <= 0.0:
            raise ValueError("track_width must be greater than zero")

        if self.poll_rate_hz <= 0.0:
            raise ValueError("poll_rate_hz must be greater than zero")

        if self.setpoint_timeout_s <= 0.0:
            raise ValueError("setpoint_timeout_s must be greater than zero")

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
        # ROS PUBLISHERS
        # ====================================================

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/wheel_odom",
            qos,
        )

        self.wheel_speed_publisher = self.create_publisher(
            Float32MultiArray,
            "/motor/wheel_speeds",
            qos,
        )

        self.status_publisher = self.create_publisher(
            String,
            "/motor/controller_status",
            qos,
        )

        # ====================================================
        # ROS COMMAND SUBSCRIBERS
        # ====================================================

        self.create_subscription(
            Float32MultiArray,
            "/motor/setpoints",
            self.on_setpoints,
            qos,
        )

        self.create_subscription(
            Empty,
            "/motor/arm",
            self.on_arm,
            qos,
        )

        self.create_subscription(
            Empty,
            "/motor/stop",
            self.on_stop,
            qos,
        )

        self.create_subscription(
            Empty,
            "/motor/reset_halls",
            self.on_reset_halls,
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
        # TELEMETRY MONITORING
        # ====================================================

        self.last_telemetry_sequence = None
        self.good_packets = 0
        self.bad_packets = 0
        self.duplicate_packets = 0

        self.last_wheel_speeds = [0.0, 0.0, 0.0, 0.0]

        # ====================================================
        # COMMAND STATE
        # ====================================================

        self.command_control_active = False
        self.remote_armed = False
        self.remote_stopped = True

        self.desired_percent = [0.0, 0.0, 0.0, 0.0]
        self.last_setpoint_time = None

        self.command_sequence = 0
        self.command_writes = 0
        self.command_write_failures = 0
        self.pending_hall_reset = False

        # ====================================================
        # MAIN TIMER
        # ====================================================

        self.timer = self.create_timer(
            1.0 / self.poll_rate_hz,
            self.update,
        )

        self.get_logger().info(
            "Wheel odometry + motor bridge started: "
            f"/dev/i2c-{self.bus_number}, "
            f"address 0x{self.address:02X}, "
            f"telemetry {TELEMETRY_SIZE} bytes, "
            f"command {COMMAND_SIZE} bytes, "
            f"loop {self.poll_rate_hz:.1f} Hz, "
            f"track width {self.track_width:.3f} m"
        )

    # ========================================================
    # ROS COMMAND CALLBACKS
    # ========================================================

    def on_setpoints(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(
                "/motor/setpoints must contain exactly 4 values: "
                "FL, FR, RL, RR"
            )
            return

        values = [float(value) for value in msg.data]

        for value in values:
            if not math.isfinite(value) or value < -100.0 or value > 100.0:
                self.get_logger().warn(
                    "Rejected motor setpoint outside [-100, 100] percent"
                )
                return

        self.desired_percent = values
        self.last_setpoint_time = self.get_clock().now()

    def on_arm(self, _msg):
        # Activates Jetson ownership of motor commands. The first packets
        # contain ARM with zero output unless fresh setpoints are available.
        self.command_control_active = True
        self.remote_armed = True
        self.remote_stopped = False
        self.desired_percent = [0.0, 0.0, 0.0, 0.0]
        self.last_setpoint_time = self.get_clock().now()

        self.get_logger().info(
            "Remote motor control ARM requested; outputs held at 0%"
        )

    def on_stop(self, _msg):
        self.command_control_active = True
        self.remote_armed = False
        self.remote_stopped = True
        self.desired_percent = [0.0, 0.0, 0.0, 0.0]
        self.last_setpoint_time = self.get_clock().now()

        self.get_logger().warn(
            "Remote STOP requested; commanding all motors to neutral"
        )

    def on_reset_halls(self, _msg):
        self.command_control_active = True
        self.pending_hall_reset = True

    # ========================================================
    # COMMAND PACKET HELPERS
    # ========================================================

    @staticmethod
    def xor_checksum(data):
        value = 0
        for byte in data:
            value ^= byte
        return value

    def setpoint_is_fresh(self, now):
        if self.last_setpoint_time is None:
            return False

        age_s = (
            now - self.last_setpoint_time
        ).nanoseconds / 1_000_000_000.0

        return age_s <= self.setpoint_timeout_s

    def build_command_packet(self, now):
        if not self.command_control_active:
            return None, False

        fresh = self.setpoint_is_fresh(now)

        if (
            self.remote_stopped
            or not self.remote_armed
            or not fresh
        ):
            effective = [0, 0, 0, 0]
        else:
            effective = [
                int(round(value))
                for value in self.desired_percent
            ]

        flags = 0

        if self.remote_armed:
            flags |= FLAG_ARM

        if self.remote_stopped:
            flags |= FLAG_STOP

        if self.pending_hall_reset:
            flags |= FLAG_RESET_HALL

        self.command_sequence = (self.command_sequence + 1) & 0xFF

        first_seven = struct.pack(
            "<BBbbbbB",
            COMMAND_MAGIC,
            self.command_sequence,
            effective[0],
            effective[1],
            effective[2],
            effective[3],
            flags,
        )

        checksum = self.xor_checksum(first_seven)

        packet = first_seven + bytes([checksum])

        return packet, fresh

    def write_esp32_command(self, now):
        packet, fresh = self.build_command_packet(now)

        if packet is None:
            return fresh

        try:
            message = i2c_msg.write(
                self.address,
                packet,
            )

            self.bus.i2c_rdwr(message)
            self.command_writes += 1

            # The reset request is one-shot after a successful write.
            self.pending_hall_reset = False

        except Exception as exc:
            self.command_write_failures += 1

            if self.command_write_failures in (1, 10, 100, 1000):
                self.get_logger().warn(
                    "I2C motor-command write failed "
                    f"({self.command_write_failures} failures): {exc}"
                )

        return fresh

    # ========================================================
    # READ ESP32 TELEMETRY
    # ========================================================

    def read_esp32(self):
        try:
            message = i2c_msg.read(
                self.address,
                TELEMETRY_SIZE,
            )

            self.bus.i2c_rdwr(message)
            raw = bytes(message)

        except Exception as exc:
            self.bad_packets += 1

            if self.bad_packets in (1, 10, 100, 1000):
                self.get_logger().warn(
                    "I2C telemetry read failed "
                    f"({self.bad_packets} failures): {exc}"
                )

            return None

        if len(raw) != TELEMETRY_SIZE:
            self.bad_packets += 1
            self.get_logger().warn(
                f"Expected {TELEMETRY_SIZE} telemetry bytes, "
                f"received {len(raw)}"
            )
            return None

        try:
            (
                front_left,
                front_right,
                rear_left,
                rear_right,
                sequence,
            ) = struct.unpack(
                TELEMETRY_FORMAT,
                raw,
            )

        except struct.error as exc:
            self.bad_packets += 1
            self.get_logger().warn(
                f"Could not unpack I2C telemetry packet: {exc}"
            )
            return None

        wheel_speeds = [
            float(front_left),
            float(front_right),
            float(rear_left),
            float(rear_right),
        ]

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
                    f"Rejected implausible wheel speed {speed:.3f} m/s"
                )
                return None

        self.good_packets += 1

        return wheel_speeds, int(sequence)

    # ========================================================
    # ODOMETRY
    # ========================================================

    def publish_wheel_speeds(self, wheel_speeds):
        msg = Float32MultiArray()
        msg.data = [float(value) for value in wheel_speeds]
        self.wheel_speed_publisher.publish(msg)

    def publish_odometry(self, wheel_speeds, now):
        fl, fr, rl, rr = wheel_speeds

        if self.invert_left:
            fl = -fl
            rl = -rl

        if self.invert_right:
            fr = -fr
            rr = -rr

        left_mps = 0.5 * (fl + rl)
        right_mps = 0.5 * (fr + rr)

        vx = 0.5 * (left_mps + right_mps)
        wz = (right_mps - left_mps) / self.track_width

        if self.last_publish_time is not None:
            dt = (
                now - self.last_publish_time
            ).nanoseconds / 1_000_000_000.0

            # Ignore unreasonable integration intervals after startup,
            # clock jumps, pauses, or debugging.
            if 0.0 < dt < 1.0:
                yaw_mid = self.yaw + 0.5 * wz * dt

                self.x += vx * math.cos(yaw_mid) * dt
                self.y += vx * math.sin(yaw_mid) * dt
                self.yaw += wz * dt

                self.yaw = math.atan2(
                    math.sin(self.yaw),
                    math.cos(self.yaw),
                )

        self.last_publish_time = now

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = q

        # Integrated wheel pose is for debugging/visualization only.
        # Give all pose components deliberately large uncertainty.
        msg.pose.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 1000.0

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        msg.twist.covariance = [0.0] * 36
        msg.twist.covariance[0] = self.vx_variance
        msg.twist.covariance[7] = 1000.0
        msg.twist.covariance[14] = 1000.0
        msg.twist.covariance[21] = 1000.0
        msg.twist.covariance[28] = 1000.0
        msg.twist.covariance[35] = self.vyaw_variance

        self.odom_publisher.publish(msg)

    # ========================================================
    # STATUS
    # ========================================================

    def publish_status(self, now, setpoint_fresh):
        if self.last_setpoint_time is None:
            setpoint_age_s = None
        else:
            setpoint_age_s = max(
                0.0,
                (
                    now - self.last_setpoint_time
                ).nanoseconds / 1_000_000_000.0,
            )

        status = {
            "i2c_bus": self.bus_number,
            "i2c_address": f"0x{self.address:02X}",
            "good_packets": self.good_packets,
            "bad_packets": self.bad_packets,
            "duplicate_packets": self.duplicate_packets,
            "last_telemetry_sequence": self.last_telemetry_sequence,
            "command_control_active": self.command_control_active,
            "remote_armed": self.remote_armed,
            "remote_stopped": self.remote_stopped,
            "setpoint_fresh": bool(setpoint_fresh),
            "setpoint_age_s": setpoint_age_s,
            "desired_percent": self.desired_percent,
            "command_writes": self.command_writes,
            "command_write_failures": self.command_write_failures,
        }

        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(msg)

    # ========================================================
    # MAIN LOOP
    # ========================================================

    def update(self):
        now = self.get_clock().now()

        # Send motor command first. A command packet is only sent after
        # Jetson motor control has been activated by /motor/arm, /motor/stop,
        # or /motor/reset_halls.
        setpoint_fresh = self.write_esp32_command(now)

        result = self.read_esp32()

        if result is None:
            self.publish_status(now, setpoint_fresh)
            return

        wheel_speeds, sequence = result

        if self.last_telemetry_sequence == sequence:
            self.duplicate_packets += 1
            self.publish_status(now, setpoint_fresh)
            return

        self.last_telemetry_sequence = sequence
        self.last_wheel_speeds = wheel_speeds

        self.publish_wheel_speeds(wheel_speeds)
        self.publish_odometry(wheel_speeds, now)
        self.publish_status(now, setpoint_fresh)

    # ========================================================
    # CLEANUP
    # ========================================================

    def destroy_node(self):
        # Best-effort neutral command before giving up ownership.
        try:
            if self.command_control_active:
                self.remote_armed = False
                self.remote_stopped = True
                self.desired_percent = [0.0, 0.0, 0.0, 0.0]
                now = self.get_clock().now()

                for _ in range(3):
                    self.write_esp32_command(now)
        except Exception:
            pass

        try:
            self.bus.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
