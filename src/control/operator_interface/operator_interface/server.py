import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, render_template
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


@socketio.on('send_goal')
def handle_send_goal(data):
    """Receive a map-frame navigation goal from the browser."""
    try:
        x = float(data['x'])
        y = float(data['y'])
    except (KeyError, TypeError, ValueError):
        socketio.emit(
            'navigation_status',
            {'message': 'Invalid navigation goal.'},
        )
        return

    ros_interface.send_navigation_goal(x, y)


@socketio.on('cancel_goal')
def handle_cancel_goal():
    """Cancel the active navigation goal."""
    ros_interface.cancel_navigation()


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
            allow_unsafe_werkzeug=True,
        )
    finally:
        ros_executor.shutdown()
        ros_thread.join(timeout=2.0)

        ros_interface.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
