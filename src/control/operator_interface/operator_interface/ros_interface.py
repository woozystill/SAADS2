import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose, FollowPath


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

    def report_status(self, message):
        """Send a status message to the GUI."""
        self.get_logger().info(message)

        if self.status_callback is not None:
            self.status_callback(message)

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
