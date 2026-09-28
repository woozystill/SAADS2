import os, glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

UART_BIT = 8  # work_mode bit 3: 0 = Ethernet, 1 = UART


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context).strip().lower()
    conn = arg('connection')
    target = arg('switch_to') or conn
    for v in (conn, target):
        if v not in ('eth', 'uart'):
            raise RuntimeError(f"Use 'eth' or 'uart', got '{v}'")
    work_mode = (int(arg('extra_mode_bits')) & ~UART_BIT) | (UART_BIT if target == 'uart' else 0)

    actions = [LogInfo(msg=f"[lidar] talking over {conn.upper()}, work_mode={work_mode}")]
    actions.append(Node(
        package='unitree_lidar_ros2', executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node', output='screen',
        parameters=[{
            'initialize_type': 1 if conn == 'uart' else 2,
            'work_mode': work_mode,
            'use_system_timestamp': True,
            'range_min': 0.0, 'range_max': 100.0, 'cloud_scan_num': 18,
            'serial_port': LaunchConfiguration('serial_port').perform(context),
            'baudrate': 4000000,
            'lidar_port': 6101, 'lidar_ip': LaunchConfiguration('lidar_ip').perform(context),
            'local_port': 6201, 'local_ip': LaunchConfiguration('local_ip').perform(context),
            'cloud_frame': 'unilidar_lidar', 'cloud_topic': 'unilidar/cloud',
            'imu_frame': 'unilidar_imu', 'imu_topic': 'unilidar/imu',
        }],
    ))

    if arg('rviz') in ('true', '1', 'yes'):
        rviz_args = ['-f', 'unilidar_lidar']
        try:  # use Unitree's own RViz view if the package ships one
            share = get_package_share_directory('unitree_lidar_ros2')
            cfgs = glob.glob(os.path.join(share, '**', '*.rviz'), recursive=True)
            if cfgs:
                rviz_args = ['-d', cfgs[0]]
        except Exception:
            pass
        actions.append(Node(package='rviz2', executable='rviz2', output='screen', arguments=rviz_args))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('connection', default_value='eth'),
        DeclareLaunchArgument('switch_to', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.62'),
        DeclareLaunchArgument('local_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('extra_mode_bits', default_value='0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
