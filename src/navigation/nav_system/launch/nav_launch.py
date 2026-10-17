from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('nav_system')

    params_file = os.path.join(
        package_dir,
        'config',
        'nav2_params.yaml'
    )

    default_map = os.path.join(
        package_dir,
        'maps',
        'cil_test_map.yaml'
    )

    map_file = LaunchConfiguration('map')

    lifecycle_nodes = [
        'map_server',
        'planner_server',
	'controller_server',
	'behavior_server',
	'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Absolute path to the Nav2 occupancy map YAML'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': map_file}
            ]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # Retained for the full SAADS Nav2 architecture.
        # These will be fully configured in the controller/BT CIL phase.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

	Node(
	    package='nav2_behaviors',
	    executable='behavior_server',
	    name='behavior_server',
	    output='screen',
	    parameters=[params_file]
	),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes
            }]
        ),
    ])
