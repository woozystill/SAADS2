import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar_launch = os.path.join(
        get_package_share_directory('unilidar_bringup'),
        'launch',
        'lidar.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch),
            launch_arguments={
                'connection': 'eth',
                'rviz': 'false',
            }.items(),
        ),
    ])
