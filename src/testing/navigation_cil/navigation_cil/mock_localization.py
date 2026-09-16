import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class MockLocalization(Node):
    """Provides deterministic localization data for SAADS Navigation CIL."""

    def __init__(self):
        super().__init__('mock_localization')

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Temporary CIL starting pose inside cil_test_map.
        self.x = -3.0
        self.y = 0.0
        self.yaw = 0.0

        self.publish_map_to_odom()

        # Publish mock odometry and odom -> base_link at 20 Hz.
        self.timer = self.create_timer(0.05, self.publish_localization)

        self.get_logger().info(
            'SAADS CIL mock localization started.'
        )

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

    def publish_localization(self):
        now = self.get_clock().now().to_msg()

        # odom -> base_link
        transform = TransformStamped()

        transform.header.stamp = now
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # yaw = 0 for the initial planner-only CIL test.
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

        # /odom
        odom = Odometry()

        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

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
