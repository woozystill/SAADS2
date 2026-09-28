import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction, LogInfo, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.realpath(__file__))
RUN_SECONDS = 10.0


def setup(context):
    to = LaunchConfiguration('to').perform(context).strip().lower()
    if to not in ('eth', 'uart'):
        raise RuntimeError("Usage: to:=eth  or  to:=uart")
    frm = 'uart' if to == 'eth' else 'eth'
    done_msg = (
        f"\n=====================================================\n"
        f" Mode {to.upper()} sent to the LiDAR. Now:\n"
        f"  1. Unplug LiDAR POWER, wait 5 s, plug it back in\n"
        f"  2. Unplug the {frm.upper()} cable, plug in the {to.upper()} cable\n"
        + f"  Then run:  lidar_{to}\n"
        f"=====================================================\n")
    return [
        LogInfo(msg=f"[switch] connecting over {frm.upper()} to set {to.upper()} mode "
                    f"(stops in {int(RUN_SECONDS)} s)..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, 'lidar.launch.py')),
            launch_arguments={'connection': frm, 'switch_to': to, 'rviz': 'false'}.items(),
        ),
        TimerAction(period=RUN_SECONDS, actions=[
            LogInfo(msg=done_msg),
            Shutdown(reason='mode switch sent'),
        ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('to', description='eth or uart'),
        OpaqueFunction(function=setup),
    ])
