import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class MockLocalization(Node):
    """Provides deterministic localization and motion for SAADS Navigation CIL."""

    def __init__(self):
        super().__init__('mock_localization')

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Temporary CIL starting pose inside cil_test_map.
        self.x = -3.0
        self.y = 0.0
        self.yaw = 0.0

        # Current commanded velocities.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Stop the mock vehicle if velocity commands become stale.
        self.cmd_vel_timeout = 0.5
        self.last_cmd_vel_time = None

        # Simulation update period: 20 Hz.
        self.dt = 0.05

        self.publish_map_to_odom()

        self.timer = self.create_timer(
            self.dt,
            self.update_and_publish
        )

        self.get_logger().info(
            'SAADS CIL mock localization and motion started.'
        )

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def publish_map_to_odom(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(transform)

    def update_and_publish(self):
        # Stop if the most recent velocity command is stale.
        if self.last_cmd_vel_time is not None:
            command_age = (
                self.get_clock().now() - self.last_cmd_vel_time
            ).nanoseconds / 1e9

            if command_age > self.cmd_vel_timeout:
                self.linear_velocity = 0.0
                self.angular_velocity = 0.0

        # Simple planar differential-drive-style motion model.
        self.x += (
            self.linear_velocity *
            math.cos(self.yaw) *
            self.dt
        )

        self.y += (
            self.linear_velocity *
            math.sin(self.yaw) *
            self.dt
        )

        self.yaw += self.angular_velocity * self.dt

        # Keep yaw in the range [-pi, pi].
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        now = self.get_clock().now().to_msg()

        half_yaw = self.yaw / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MockLocalization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
