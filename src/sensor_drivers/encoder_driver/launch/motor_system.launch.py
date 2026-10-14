#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="encoder_driver",
                executable="wheel_odometry",
                name="wheel_odometry",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "i2c_bus": 1,
                        "i2c_address": 0x12,
                        "track_width": 0.595,
                        "poll_rate_hz": 20.0,
                        "setpoint_timeout_s": 0.50,
                    }
                ],
            ),
            Node(
                package="encoder_driver",
                executable="motor_dashboard",
                name="motor_dashboard",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "http_port": 8080,
                        "browser_timeout_s": 0.60,
                    }
                ],
            ),
        ]
    )
