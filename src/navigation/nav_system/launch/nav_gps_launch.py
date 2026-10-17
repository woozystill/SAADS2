#!/usr/bin/env python3
"""Nav2 bringup - OUTDOOR GPS WAYPOINT PROFILE.

Runs alongside Trevor's nav_launch.py (map-based CIL). Neither replaces
the other; pick one per session.

    ros2 launch nav_system nav_launch.py        <- map-based, indoor CIL
    ros2 launch nav_system nav_gps_launch.py    <- outdoor GPS waypoints

Does NOT start localization. Run that first - Nav2 needs map -> odom and
odom -> base_link to exist before it can activate:

    ros2 launch localization_bringup localization.launch.py

Or, on the bench with no wheels:

    ros2 run nav_system sim_robot
    ros2 launch localization_bringup localization.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('nav_system')
    params_file = os.path.join(pkg, 'config', 'nav2_params_gps.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # No map_server, no amcl.
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    common = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=common,
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=common,
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=common,
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=common,
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=common,
        ),

        # Without this every server above sits unconfigured, silently
        # ignoring goals while looking perfectly healthy.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0,
            }],
        ),

        # lat/lon -> map frame -> NavigateToPose. Humble has no
        # FollowGPSWaypoints action, so this does the conversion through
        # robot_localization's /fromLL service.
        Node(
            package='nav_system',
            executable='gps_waypoint_node',
            name='gps_waypoint_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
