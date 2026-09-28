import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('localization_bringup'),
        'config', 'ekf_dual.yaml')

    start_drivers = LaunchConfiguration('start_drivers')

    return LaunchDescription([

        DeclareLaunchArgument(
            'start_drivers', default_value='true',
            description='Also start the GPS and IMU drivers. Set false if '
                        'they are already running in their own terminals.'),

        # ------------------------------------------------------------------
        # Sensor drivers - each package's own launch file, so their
        # settings live in one place. No BLINKA environment variables:
        # those were for the laptop's USB bridge. The Jetson uses native
        # I2C and needs none.
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('bno085_driver'),
                'launch', 'bno085.launch.py')),
            condition=IfCondition(start_drivers),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gps_driver'),
                'launch', 'gps.launch.py')),
            condition=IfCondition(start_drivers),
        ),

        # ------------------------------------------------------------------
        # Where each sensor sits on the robot. Zeros for now - replace
        # with measured values once the sensors are mounted. Rotation
        # matters far more than translation at this robot's scale.
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--frame-id', 'base_link',
                       '--child-frame-id', 'imu_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['--frame-id', 'base_link',
                       '--child-frame-id', 'gps_link'],
        ),

        # ------------------------------------------------------------------
        # Local EKF - odom frame. Continuous, never jumps, drifts.
        # Publishes odom -> base_link.
        # ------------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[config],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),

        # ------------------------------------------------------------------
        # Global EKF - map frame. Adds GPS, allowed to jump.
        # Publishes map -> odom. Runs in parallel with the local EKF at the
        # topic level, but needs its odom -> base_link through TF.
        # ------------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[config],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),

        # ------------------------------------------------------------------
        # navsat_transform - GPS lat/lon into map-frame metres.
        # Consumes the global EKF's output and feeds /odometry/gps back
        # into it; the loop is intentional.
        #
        # The imu remap is ('imu', ...), verified on Humble: with
        # ('imu/data', ...) the node subscribed to /imu, never saw the IMU,
        # and /odometry/gps stayed silent.
        # ------------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        ),
    ])
