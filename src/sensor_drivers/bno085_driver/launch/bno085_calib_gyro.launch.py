"""Gyroscope zero-rate-offset calibration. Robot PARKED and untouched.

A gyroscope at rest should read zero on every axis. Any deviation is
zero-rate offset, and left uncorrected it appears as heading drift.

WHY THIS IS SEPARATE FROM THE MAGNETOMETER STEP

The datasheet warns that gyroscope ZRO calibration can mis-converge
when the device has insufficient tremor, and that slow horizontal
rotation about the gravity vector can fool the algorithm outright.
Driving a robot in slow circles - which is exactly the magnetometer
procedure - is that failure case.

It also says ZRO "will always be corrected when the device becomes very
stable (such as when laid on a table)". A parked robot is a table. So
gyroscope calibration is done parked, on its own, and then turned off
before anything moves.

PROCEDURE

  1. Robot assembled, powered, parked on a solid surface.
  2. Launch this and DO NOT TOUCH IT for two minutes.
  3. Ctrl+C.
  4. Go on to bno085_calibrate.launch.py for the magnetometer, which
     leaves gyroscope calibration off.

Gyroscope ZRO is a property of the chip, not of its surroundings, so
unlike the magnetometer this step does not have to be done outdoors.
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
                'report_interval_us': 20000,
                'publish_rate': 50.0,
                'frame_id': 'imu_link',
                'i2c_address': 0x4A,

                'dynamic_calibration': True,
                'calibrate_gyroscope': True,     # the point of this file
                'calibrate_accelerometer': True,
                'calibrate_magnetometer': False,  # not while parked

                # The magnetometer stream is what makes
                # calibration_status update, so keep it on to see
                # progress even though mag calibration is off.
                'enable_magnetometer': True,
                'calibration_log_period': 5.0,
            }],
        ),
    ])
