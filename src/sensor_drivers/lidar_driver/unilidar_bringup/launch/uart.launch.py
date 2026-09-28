# Run the L2 over UART.   ros2 launch ~/unitree_ws/uart.launch.py [rviz:=true] [serial_port:=/dev/ttyUSB0]
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, 'lidar.launch.py')),
            launch_arguments={'connection': 'uart', 'switch_to': 'uart'}.items(),
        ),
    ])
