import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose, FollowPath


class NavigationTest(Node):
    """Runs a planner-to-controller closed-loop Navigation CIL test."""

    def __init__(self):
        super().__init__('navigation_test')

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

        self.get_logger().info(
            'Waiting for Nav2 planner and controller action servers...'
        )

        self.planner_client.wait_for_server()
        self.controller_client.wait_for_server()

        self.get_logger().info(
            'Nav2 action servers available. Requesting path.'
        )

        self.send_planner_goal()

    def send_planner_goal(self):
        goal = ComputePathToPose.Goal()

        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()

        goal.goal.pose.position.x = 3.0
        goal.goal.pose.position.y = 0.0
        goal.goal.pose.position.z = 0.0
        goal.goal.pose.orientation.w = 1.0

        goal.planner_id = 'GridBased'
        goal.use_start = False

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.planner_goal_response)

    def planner_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Planner rejected the goal.')
            rclpy.shutdown()
            return

        self.get_logger().info('Planner accepted the goal.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.planner_result)

    def planner_result(self, future):
        result = future.result().result
        path = result.path

        if not path.poses:
            self.get_logger().error('Planner returned an empty path.')
            rclpy.shutdown()
            return

        self.get_logger().info(
            f'Planner generated path with {len(path.poses)} poses.'
        )

        self.send_controller_goal(path)

    def send_controller_goal(self, path):
        goal = FollowPath.Goal()

        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'

        self.get_logger().info(
            'Sending planned path to Regulated Pure Pursuit.'
        )

        future = self.controller_client.send_goal_async(
            goal,
            feedback_callback=self.controller_feedback
        )
        future.add_done_callback(self.controller_goal_response)

    def controller_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Controller rejected the path.')
            rclpy.shutdown()
            return

        self.get_logger().info('Controller accepted the path.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.controller_result)

    def controller_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Distance to goal: {feedback.distance_to_goal:.2f} m, '
            f'Speed: {feedback.speed:.2f} m/s'
        )

    def controller_result(self, future):
        wrapped_result = future.result()

        if wrapped_result.status == 4:
            self.get_logger().info(
                'Navigation CIL test SUCCEEDED.'
            )
        else:
            self.get_logger().error(
                f'Navigation CIL test ended with status '
                f'{wrapped_result.status}.'
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
