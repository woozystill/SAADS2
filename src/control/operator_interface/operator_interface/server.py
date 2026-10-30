import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template, request
from flask_socketio import SocketIO
from rclpy.executors import SingleThreadedExecutor

from operator_interface.ros_interface import OperatorROSInterface


PACKAGE_SHARE = Path(get_package_share_directory('operator_interface'))

app = Flask(
    __name__,
    template_folder=str(PACKAGE_SHARE / 'templates'),
    static_folder=str(PACKAGE_SHARE / 'static'),
)

socketio = SocketIO(app)

ros_interface = None
ros_executor = None


@app.route('/')
def index():
    """Serve the SAADS operator interface."""
    return render_template('index.html')


@socketio.on('send_mission_destination')
def handle_mission_destination(data):
    """Receive a GPS destination from the Ground Station."""
    try:
        latitude = float(data['latitude'])
        longitude = float(data['longitude'])

        destination = ros_interface.send_mission_destination(
            latitude,
            longitude
        )

        socketio.emit(
            'destination_submitted',
            destination,
            to=request.sid,
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        socketio.emit(
            'mission_message',
            {'message': 'Invalid GPS destination.'},
        )

@socketio.on('confirm_delivery')
def handle_confirm_delivery():
    """Forward operator delivery confirmation to ROS 2."""
    ros_interface.confirm_delivery()

@socketio.on('connect')
def handle_connect():
    """Send the current mission state to a newly connected browser."""
    if ros_interface is not None:
        socketio.emit(
            'mission_status',
            {'state': ros_interface.current_mission_state},
            to=request.sid,
        )

def send_status_to_browser(message):
    """Forward ROS navigation status to connected browsers."""
    socketio.emit(
        'navigation_status',
        {'message': message},
    )


def send_feedback_to_browser(distance, speed):
    """Forward Nav2 controller feedback to connected browsers."""
    socketio.emit(
        'navigation_feedback',
        {
            'distance': float(distance),
            'speed': float(speed),
        },
    )

def send_localization_to_browser(data):
    """Forward global EKF heading telemetry to the browser."""
    socketio.emit('localization_update', data)


def send_gps_to_browser(data):
    """Forward real ROS 2 GPS telemetry to connected browsers."""
    socketio.emit('gps_update', data)


def send_planned_path_to_browser(data):
    """Forward Nav2 planned-path telemetry without issuing commands."""
    socketio.emit('planned_path_update', data)


def send_mission_status_to_browser(state):
    """Forward Mission Supervisor state changes to the browser."""
    socketio.emit(
        'mission_status',
        {'state': state},
    )

def ros_spin():
    """Process ROS 2 callbacks in a background thread."""
    ros_executor.spin()


def main():
    """Start ROS 2 and the SAADS operator web server."""
    global ros_interface
    global ros_executor

    rclpy.init()

    ros_interface = OperatorROSInterface()

    ros_interface.status_callback = send_status_to_browser
    ros_interface.feedback_callback = send_feedback_to_browser
    ros_interface.mission_status_callback = send_mission_status_to_browser
    ros_interface.gps_callback = send_gps_to_browser
    ros_interface.localization_callback = send_localization_to_browser
    ros_interface.path_callback = send_planned_path_to_browser

    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(ros_interface)

    ros_thread = threading.Thread(
        target=ros_spin,
        daemon=True,
    )
    ros_thread.start()

    try:
        socketio.run(
            app,
            host='0.0.0.0',
            port=5000,
        )
    finally:
        ros_executor.shutdown()
        ros_thread.join(timeout=2.0)

        ros_interface.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
