from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('saads_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    platform_launch = os.path.join(launch_dir, 'platform_launch.py')
    sensors_launch = os.path.join(launch_dir, 'sensors_launch.py')
    processing_launch = os.path.join(launch_dir, 'processing_launch.py')
    localization_launch = os.path.join(launch_dir, 'localization_launch.py')
    navigation_launch = os.path.join(launch_dir, 'navigation_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(platform_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(processing_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch)
        ),
    ])
