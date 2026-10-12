"""Normal operation. Publishes /imu/data, /imu/mag and /imu/diagnostics.

Settings here override the node's own defaults. They match the node, so
ros2 run and ros2 launch behave the same - but this file is the one to
edit, because ros2 launch ignores --ros-args -p entirely.

To calibrate, use bno085_calib_gyro.launch.py (parked) and then
bno085_calib_mag.launch.py (rotating), and save to flash between runs.
"""

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
                # the MCP2221A bridge did, so 20000 us - 50 Hz per
                # feature - is comfortable. Behind a USB bridge this had
                # to be 150000 or the reader fell permanently behind.
                'report_interval_us': 20000,
                'publish_rate': 50.0,
                'frame_id': 'imu_link',
                'i2c_address': 0x4A,

                # --- calibration -------------------------------------
                # NOTE: dynamic_calibration false does NOT disable
                # calibration. It means "send no command", so the chip
                # falls back to its own defaults - accelerometer and
                # magnetometer calibrating, gyroscope not. To control it
                # at all, this has to be true and the per-sensor flags
                # below do the choosing.
                'dynamic_calibration': True,

                # Chip default, stable, and not fused anyway.
                'calibrate_accelerometer': False,

                # On. The datasheet warns this can mis-converge without
                # hand-like tremor, but tested here parked and still the
                # level climbed 0 -> 3, so it converges on this
                # hardware. Worth having, since gyroscope bias drifts
                # with temperature and the odom EKF fuses yaw rate and
                # nothing else.
                'calibrate_gyroscope': True,

                # OFF, and this one is measured rather than inferred. A
                # 947 m run with it enabled showed 84 degrees of heading
                # drift across a single straight leg as the chip
                # adjusted its model while driving. The flash
                # calibration is better than anything it re-derives on
                # the move.
                'calibrate_magnetometer': False,

                # Planar accelerometer calibration suits a vehicle
                # constrained to a plane - the datasheet names robot
                # vacuums. Leave false while the amphibious case is
                # still open, since a pitching hull is not that case.
                'planar_accelerometer': False,

                # --- reporting ---------------------------------------
                # The magnetometer stream costs a quarter of the I2C
                # traffic and robot_localization never reads it. Kept on
                # because it is the only thing that makes the per-sensor
                # accuracy levels available to the health dashboard, and
                # the rate held at 50 Hz with it enabled.
                'enable_magnetometer': True,
                'mag_report_interval_us': 20000,

                # Silence the periodic accuracy log - the dashboard
                # shows it live instead.
                'calibration_log_period': 0.0,

                # --- covariance --------------------------------------
                # Heading variance comes from the chip's own per-reading
                # accuracy estimate, read off bytes the driver library
                # discards. It is the one value here that adapts.
                'adaptive_covariance': True,
                'min_orientation_sigma_deg': 1.0,

                # Measured from a 5.6 minute stationary recording with
                # the sensor mounted - 16843 samples, worst axis of
                # three. The chip exposes only a qualitative 0-3 level
                # for these two, so measuring beat inventing a mapping.
                'angular_velocity_variance': 3.022e-06,
                'linear_acceleration_variance': 4.844e-04,
            }],
        ),
    ])
