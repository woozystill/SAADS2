from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    yolo_docker = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            '''
            docker start saads-vision-dev >/dev/null 2>&1 || true

            docker exec \
                -e DISPLAY=$DISPLAY \
                saads-vision-dev \
                bash -lc "
                    cd /workspace/saads_ws/src/sensor_drivers/camera_driver/camera_driver &&
                    python3 yolo_camera_test.py
                "
            '''
        ],
        output='screen'
    )

    return LaunchDescription([
        yolo_docker
    ])
