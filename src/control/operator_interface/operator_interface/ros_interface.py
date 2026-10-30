import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose, FollowPath
from robot_localization.srv import ToLL
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class OperatorROSInterface(Node):
    """ROS 2 interface between the SAADS operator GUI and Nav2."""

    def __init__(self):
        super().__init__('operator_interface')

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.controller_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        self.controller_goal_handle = None

        # Callbacks assigned by the GUI.
        self.status_callback = None
        self.feedback_callback = None

        # Live GPS telemetry callback assigned by GUI
        self.gps_callback = None

        # Read-only Nav2 planned-path telemetry.
        self.to_ll_client = self.create_client(ToLL, '/toLL')
        self.path_callback = None

        # Track asynchronous geographic conversion requests.
        self.path_conversion_id = 0
        self.path_conversion_points = []
        self.path_geographic_points = []
        self.create_subscription(
            Path,
            '/plan',
            self.on_planned_path,
            10
        )

        # Read-only global localization telemetry.
        self.localization_callback = None
        self.create_subscription(
            Odometry,
            '/odometry/global',
            self.on_global_odometry,
            10
        )

        # Receive actual GPS measurements from the GPS driver
        self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.on_gps_fix,
            10
        )

        # Mission Supervisor communication.
        self.mission_status_callback = None
        self.current_mission_state = 'UNKNOWN'

        self.mission_destination_pub = self.create_publisher(
            NavSatFix,
            '/mission/destination',
            10
        )

        self.mission_cancel_pub = self.create_publisher(
            Bool, '/mission/cancel', 10
        )

        self.delivery_confirmation_pub = self.create_publisher(
            Bool,
            '/mission/delivery_confirmed',
            10
        )

        self.create_subscription(
            String,
            '/mission/status',
            self.on_mission_status,
            10
        )

    def on_planned_path(self, msg):
        """Convert an actual Nav2 planned path for geographic display."""
        import math

        self.path_conversion_id += 1
        conversion_id = self.path_conversion_id

        self.path_conversion_points = []
        self.path_geographic_points = []

        # Clear the previous displayed plan while processing a new one.
        if self.path_callback is not None:
            self.path_callback({
                'frame_id': 'wgs84',
                'coordinates': [],
                'pose_count': 0,
            })

        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Cannot display path in frame {msg.header.frame_id!r}.'
            )
            return

        if not msg.poses:
            if self.path_callback is not None:
                self.path_callback({
                    'frame_id': 'wgs84',
                    'coordinates': [],
                    'pose_count': 0,
                })
            return

        if not self.to_ll_client.service_is_ready():
            self.get_logger().warn(
                '/toLL unavailable; planned path cannot be displayed.'
            )
            return

        for pose in msg.poses:
            point = pose.pose.position
            if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                self.get_logger().warn('Planned path contains invalid coordinates.')
                return
            self.path_conversion_points.append((point.x, point.y, point.z))

        self.convert_next_path_point(conversion_id)

    def convert_next_path_point(self, conversion_id):
        """Convert the next map-frame point using /toLL asynchronously."""
        if conversion_id != self.path_conversion_id:
            return

        index = len(self.path_geographic_points)

        if index == len(self.path_conversion_points):
            if self.path_callback is not None:
                self.path_callback({
                    'frame_id': 'wgs84',
                    'coordinates': self.path_geographic_points.copy(),
                    'pose_count': index,
                })
            return

        x, y, z = self.path_conversion_points[index]
        request = ToLL.Request()
        request.map_point.x = x
        request.map_point.y = y
        request.map_point.z = z

        future = self.to_ll_client.call_async(request)
        future.add_done_callback(
            lambda result: self.on_path_point_converted(result, conversion_id)
        )

    def on_path_point_converted(self, future, conversion_id):
        """Validate one converted coordinate and continue the sequence."""
        import math

        if conversion_id != self.path_conversion_id:
            return

        try:
            response = future.result()
            latitude = float(response.ll_point.latitude)
            longitude = float(response.ll_point.longitude)
        except Exception as exc:
            self.get_logger().warn(f'Path coordinate conversion failed: {exc}')
            return

        if not (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        ):
            self.get_logger().warn('Invalid geographic path coordinate.')
            return

        self.path_geographic_points.append([longitude, latitude])
        self.convert_next_path_point(conversion_id)

    def on_global_odometry(self, msg):
        """Forward estimated vehicle heading from the global EKF."""
        import math

        q = msg.pose.pose.orientation
        components = (q.x, q.y, q.z, q.w)

        if not all(math.isfinite(v) for v in components):
            return

        norm = math.sqrt(sum(v * v for v in components))
        if norm < 1e-8:
            return

        x, y, z, w = (v / norm for v in components)

        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

        # ENU yaw: counterclockwise from east.
        # Map heading: clockwise from north.
        heading = (90.0 - math.degrees(yaw)) % 360.0

        if self.localization_callback is not None:
            self.localization_callback({
                'heading': heading,
                'frame_id': msg.header.frame_id,
            })

    def on_gps_fix(self, msg):
        """Forward GPS measurements to the operator GUI."""
        import math

        valid = (
            msg.status.status >= 0
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )

        if self.gps_callback is not None:
            self.gps_callback({
                'valid': valid,
                'latitude': msg.latitude if valid else None,
                'longitude': msg.longitude if valid else None,
                'covariance': list(msg.position_covariance),
                'covariance_type': int(msg.position_covariance_type),
            })

    def report_status(self, message):
        """Send a status message to the GUI."""
        self.get_logger().info(message)

        if self.status_callback is not None:
            self.status_callback(message)

    def on_mission_status(self, msg):
        """Receive mission-state updates from the supervisor."""
        self.current_mission_state = msg.data

        if self.mission_status_callback is not None:
            self.mission_status_callback(msg.data)

    def send_mission_destination(self, latitude, longitude):
        """Publish a GPS destination to the Mission Supervisor."""
        latitude = float(latitude)
        longitude = float(longitude)

        if not (-90.0 <= latitude <= 90.0):
            raise ValueError('Invalid latitude.')

        if not (-180.0 <= longitude <= 180.0):
            raise ValueError('Invalid longitude.')

        import math

        if not (math.isfinite(latitude) and math.isfinite(longitude)):
            raise ValueError('Coordinates must be finite.')

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = latitude
        msg.longitude = longitude

        self.mission_destination_pub.publish(msg)

        self.report_status(
            f'Mission destination submitted: {latitude:.7f}, {longitude:.7f}'
        )

        return {
            'latitude': latitude,
            'longitude': longitude,
        }

    def cancel_mission(self):
        """Request cancellation through the Mission Supervisor."""
        if self.current_mission_state not in (
            'DELIVERING', 'RETURNING_HOME', 'ARRIVAL_CONFIRMATION'
        ):
            return False
        self.mission_cancel_pub.publish(Bool(data=True))
        self.report_status('Mission cancellation requested; awaiting status.')
        return True

    def confirm_delivery(self):
        """Notify the Mission Supervisor that delivery is confirmed."""
        if self.current_mission_state != 'ARRIVAL_CONFIRMATION':
            self.report_status('Delivery confirmation unavailable in current state.')
            return

        msg = Bool()
        msg.data = True

        self.delivery_confirmation_pub.publish(msg)
        self.report_status('Delivery confirmation submitted.')

    def send_navigation_goal(self, x, y):
        """Request a Nav2 path to the selected map-frame coordinate."""

        if not self.planner_client.server_is_ready():
            self.report_status('Planner is not available.')
            return

        if not self.controller_client.server_is_ready():
            self.report_status('Controller is not available.')
            return

        goal = ComputePathToPose.Goal()

        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()

        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.position.z = 0.0
        goal.goal.pose.orientation.w = 1.0

        goal.planner_id = 'GridBased'
        goal.use_start = False

        self.report_status(
            f'Requesting path to X={x:.2f}, Y={y:.2f}...'
        )

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.planner_goal_response)

    def planner_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.report_status('Planner rejected the goal.')
            return

        self.report_status('Planner accepted the goal.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.planner_result)

    def planner_result(self, future):
        result = future.result().result
        path = result.path

        if not path.poses:
            self.report_status('Planner returned an empty path.')
            return

        self.report_status(
            f'Path generated with {len(path.poses)} poses.'
        )

        self.send_controller_goal(path)

    def send_controller_goal(self, path):
        goal = FollowPath.Goal()

        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'

        self.report_status('Sending path to RPP controller.')

        future = self.controller_client.send_goal_async(
            goal,
            feedback_callback=self.controller_feedback
        )

        future.add_done_callback(self.controller_goal_response)

    def controller_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.report_status('Controller rejected the path.')
            return

        self.controller_goal_handle = goal_handle
        self.report_status('Navigation active.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.controller_result)

    def controller_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback

        if self.feedback_callback is not None:
            self.feedback_callback(
                feedback.distance_to_goal,
                feedback.speed
            )

    def controller_result(self, future):
        wrapped_result = future.result()

        self.controller_goal_handle = None

        if wrapped_result.status == 4:
            self.report_status('Navigation goal reached.')
        else:
            self.report_status(
                f'Navigation ended with status {wrapped_result.status}.'
            )

    def cancel_navigation(self):
        """Cancel the currently active controller goal."""

        if self.controller_goal_handle is None:
            self.report_status('No active navigation goal to cancel.')
            return

        self.report_status('Cancelling navigation...')

        future = self.controller_goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_result)

    def cancel_result(self, future):
        response = future.result()

        if len(response.goals_canceling) > 0:
            self.report_status('Navigation cancelled.')
        else:
            self.report_status('Navigation cancellation failed.')
