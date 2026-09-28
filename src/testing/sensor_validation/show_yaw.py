#!/usr/bin/env python3
"""Print live heading from /imu/data, updating in place on one line.

Shows ENU yaw (0 = east, 90 = north, counter-clockwise), the equivalent
compass bearing, and roll/pitch so you can tell whether the sensor is
level enough for the yaw to be trustworthy - a large tilt mixes the
Earth's vertical field component into the horizontal calculation.

Usage:
    python3 show_yaw.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


class ShowYaw(Node):

    def __init__(self):
        super().__init__('show_yaw')
        # The driver publishes BEST_EFFORT. A RELIABLE subscription will
        # not connect to it, and receives nothing without saying so.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, '/imu/data', self.cb, qos)
        print('waiting for /imu/data ...')

    def cb(self, msg):
        o = msg.orientation
        yaw = math.degrees(math.atan2(
            2 * (o.w * o.z + o.x * o.y),
            1 - 2 * (o.y * o.y + o.z * o.z))) % 360
        pitch = math.degrees(math.asin(
            max(-1, min(1, 2 * (o.w * o.y - o.z * o.x)))))
        roll = math.degrees(math.atan2(
            2 * (o.w * o.x + o.y * o.z),
            1 - 2 * (o.x * o.x + o.y * o.y)))
        print('yaw %6.1f  (compass %6.1f)   pitch %+6.1f  roll %+6.1f   '
              % (yaw, (90 - yaw) % 360, pitch, roll), end='\r', flush=True)


def main():
    rclpy.init()
    node = ShowYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
