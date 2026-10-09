import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ========================================================
    # CONFIG FILE
    # ========================================================

    config = os.path.join(
        get_package_share_directory(
            "localization_bringup"
        ),
        "config",
        "ekf_dual.yaml",
    )


    # ========================================================
    # LAUNCH ARGUMENTS
    # ========================================================

    start_drivers = LaunchConfiguration(
        "start_drivers"
    )

    encoder_i2c_bus = LaunchConfiguration(
        "encoder_i2c_bus"
    )


    return LaunchDescription([

        # ====================================================
        # ARGUMENTS
        # ====================================================

        DeclareLaunchArgument(
            "start_drivers",
            default_value="true",
            description=(
                "Start wheel odometry, GPS, and IMU drivers. "
                "Set false if they are already running."
            ),
        ),

        DeclareLaunchArgument(
            "encoder_i2c_bus",
            default_value="1",
            description=(
                "Jetson I2C bus connected to the ESP32. "
                "Example: /dev/i2c-1 -> use 1."
            ),
        ),


        # ====================================================
        # WHEEL ODOMETRY
        #
        # ESP32
        #   -> I2C
        #   -> wheel_odometry
        #   -> /wheel_odom
        # ====================================================

        Node(
            package="encoder_driver",
            executable="wheel_odometry",
            name="wheel_odometry",
            output="screen",

            condition=IfCondition(
                start_drivers
            ),

            parameters=[{
                "i2c_bus": ParameterValue(
                    encoder_i2c_bus,
                    value_type=int,
                ),

                "i2c_address": 0x12,

                # Placeholder until measured/calibrated.
                "track_width": 0.595,

                # Poll ESP32 faster than its 10 Hz packet
                # generation rate. Duplicate packets are
                # discarded using the sequence number.
                "poll_rate_hz": 20.0,

                "invert_left": False,
                "invert_right": False,

                "vx_variance": 0.01,
                "vyaw_variance": 0.02,

                "max_abs_wheel_speed_mps": 15.0,
            }],
        ),


        # ====================================================
        # BNO085 IMU
        # ====================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "bno085_driver"
                    ),
                    "launch",
                    "bno085.launch.py",
                )
            ),

            condition=IfCondition(
                start_drivers
            ),
        ),


        # ====================================================
        # GPS
        # ====================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "gps_driver"
                    ),
                    "launch",
                    "gps.launch.py",
                )
            ),

            condition=IfCondition(
                start_drivers
            ),
        ),


        # ====================================================
        # SENSOR STATIC TRANSFORMS
        #
        # Zero translation / rotation for now.
        #
        # Replace these later with the actual sensor
        # mounting positions and orientations.
        # ====================================================

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu",

            arguments=[
                "--frame-id",
                "base_link",

                "--child-frame-id",
                "imu_link",
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_gps",

            arguments=[
                "--frame-id",
                "base_link",

                "--child-frame-id",
                "gps_link",
            ],
        ),


        # ====================================================
        # LOCAL EKF
        #
        # world_frame = odom
        #
        # Inputs:
        #
        #   /wheel_odom
        #   /imu/data
        #
        # Output:
        #
        #   /odometry/local
        #
        # TF:
        #
        #   odom -> base_link
        # ====================================================

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",

            output="screen",

            parameters=[
                config
            ],

            remappings=[
                (
                    "odometry/filtered",
                    "/odometry/local",
                ),
            ],
        ),


        # ====================================================
        # GLOBAL EKF
        #
        # world_frame = map
        #
        # Inputs:
        #
        #   /wheel_odom
        #   /imu/data
        #   /odometry/gps
        #
        # Output:
        #
        #   /odometry/global
        #
        # TF:
        #
        #   map -> odom
        # ====================================================

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",

            output="screen",

            parameters=[
                config
            ],

            remappings=[
                (
                    "odometry/filtered",
                    "/odometry/global",
                ),
            ],
        ),


        # ====================================================
        # NAVSAT TRANSFORM
        #
        # Uses:
        #
        #   /imu/data
        #   /gps/fix
        #   /odometry/global
        #
        # Produces:
        #
        #   /odometry/gps
        #
        # /odometry/gps feeds back into the GLOBAL EKF.
        #
        # This feedback path is the standard
        # robot_localization dual-EKF/navsat architecture.
        # ====================================================

        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",

            output="screen",

            parameters=[
                config
            ],

            remappings=[
                (
                    "imu",
                    "/imu/data",
                ),

                (
                    "gps/fix",
                    "/gps/fix",
                ),

                (
                    "odometry/filtered",
                    "/odometry/global",
                ),

                (
                    "odometry/gps",
                    "/odometry/gps",
                ),

                (
                    "gps/filtered",
                    "/gps/filtered",
                ),
            ],
        ),

    ])
