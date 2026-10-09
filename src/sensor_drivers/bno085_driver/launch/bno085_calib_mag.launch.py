"""Calibration mode. Use this when setting up, not when running.

Turns on everything the chip needs to build a magnetic model of its
surroundings, and reports progress every 5 seconds.

PROCEDURE

  1. Sensor in its FINAL mounted position, robot assembled and powered
     as it will normally be. What you save describes the magnetic
     environment it was saved in, so an indoor calibration is worthless
     outdoors and a hand-held one is worthless once mounted.

  2. Outdoors, away from buildings. A room with a steel frame is close
     to worst case - the same sensor reported 2 deg heading uncertainty
     on a clean bench and 65 deg in a software lab.

  3. Launch this, then rotate the robot through a full circle, slowly,
     two or three times. Uneven ground helps: it adds pitch and roll
     coverage without lifting anything.

  4. Watch for "high (3/3)" and hold it for a minute.

  5. Save:
         ros2 service call /bno085_node/save_calibration \\
             std_srvs/srv/Trigger

     It refuses below 2/3, because a saved bad calibration becomes the
     starting point for every future boot.

  6. Kill this, and use bno085.launch.py from then on.

If it plateaus at 2/3, save anyway - 2/3 is usable. If it will not
reach 2/3 at all, the sensor is probably too close to the battery or
frame for the chip to model the field, and moving it is the fix rather
than more rotating.
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

                # The three settings that make calibration work. All
                # three are needed together: without the magnetometer
                # stream, calibration_status reads 0 forever no matter
                # what the chip is actually doing.
                'dynamic_calibration': True,
                'calibrate_magnetometer': True,
                'enable_magnetometer': True,
                'mag_report_interval_us': 20000,

                # Progress every 5 s while you rotate.
                'calibration_log_period': 5.0,
            }],
        ),
    ])
