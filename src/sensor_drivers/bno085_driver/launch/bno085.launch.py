from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno085_driver',
            executable='bno085_node',
            name='bno085_node',
            output='screen',
            parameters=[{
                # Native I2C on the Jetson header drains far faster than
                # the MCP2221A bridge did, so 20000 us (50 Hz per feature)
                # is comfortable here. On the laptop bridge this had to be
                # 150000 or the reader fell permanently behind.
                'report_interval_us': 20000,
                'publish_rate': 50.0,
                'frame_id': 'imu_link',
                'i2c_address': 0x4A,
            }],
        ),
    ])
