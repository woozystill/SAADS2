#!/usr/bin/env python3
"""GPS waypoint -> Nav2 goal bridge, with return-to-base.

--------------------------------------------------------------------------
WHY THIS EXISTS
--------------------------------------------------------------------------
Nav2 on Rolling and Jazzy has a FollowGPSWaypoints action that takes
latitude and longitude directly. ROS 2 Humble does NOT - the maintainers
confirmed it was never branched, and nav2_gps_waypoint_follower_demo does
not exist for Humble.

So the conversion happens here:

    lat/lon --(/fromLL, from navsat_transform)--> map frame --> Nav2 goal

/fromLL uses the same datum the global EKF uses, so the converted point
lands in exactly the frame Nav2 plans in. Projecting by hand with a local
equirectangular approximation would drift relative to navsat_transform and
put the goal somewhere slightly wrong, in a way that gets worse the
further you travel.

--------------------------------------------------------------------------
RETURN TO BASE
--------------------------------------------------------------------------
This is why SAADS does not need a map. "Home" is the GPS coordinate where
the mission started - recorded automatically from the first valid fix.
Going home is navigating to a waypoint you already have. No SLAM, no
occupancy grid, no AMCL.

--------------------------------------------------------------------------
USAGE
--------------------------------------------------------------------------
    # go somewhere
    ros2 topic pub --once /goal_gps sensor_msgs/msg/NavSatFix \\
      "{latitude: 32.2319, longitude: -110.9501}"

    # come back to where the mission started
    ros2 topic pub --once /go_home std_msgs/msg/Empty "{}"

    # re-mark home as wherever the robot is right now
    ros2 topic pub --once /set_home std_msgs/msg/Empty "{}"

    # stop
    ros2 topic pub --once /goal_cancel std_msgs/msg/Empty "{}"
"""

import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String


class GpsWaypointNode(Node):

    def __init__(self):
        super().__init__('gps_waypoint_node')

        # Heading to hold on arrival, radians ENU. Only has an effect if
        # yaw_goal_tolerance is tightened in the Nav2 params; it is 3.14
        # (any heading) by default, which is right for a delivery point.
        self.declare_parameter('goal_yaw', 0.0)

        # Record the first fix as home. Turn off if home should only ever
        # be set explicitly.
        self.declare_parameter('auto_set_home', True)

        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_status_pub = self.create_publisher(
            String, '/navigation/status', 10
        )
        self.mission_result_pub = self.create_publisher(
            String, '/mission/navigation_result', 10
        )

        self.create_subscription(NavSatFix, 'goal_gps', self.on_waypoint, 10)
        self.create_subscription(
            String, '/mission/navigation_command',
            self.on_mission_command, 10
        )
        # Sensor topics are published BEST_EFFORT. A RELIABLE subscriber
        # receives nothing from a BEST_EFFORT publisher, silently.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(NavSatFix, 'gps/fix', self.on_fix, sensor_qos)
        self.create_subscription(Empty, 'go_home', self.on_go_home, 10)
        self.create_subscription(Empty, 'set_home', self.on_set_home, 10)
        self.create_subscription(Empty, 'goal_cancel', self.on_cancel, 10)

        self._goal_handle = None
        self._home = None
        self._last_fix = None

        # Track mission-owned goals separately from ordinary GPS commands.
        # Mission identity is tracked separately from ordinary GPS goals.
        # Mission dispatch remains disabled until result correlation is ready.
        self._active_mission_goal = None
        self.mission_dispatch_enabled = False

        # Tracks requests awaiting GPS conversion or Nav2 acceptance.
        self._goal_pending = False

        # Tracks cancellation while waiting for Nav2 to respond.
        self._cancel_pending = False
        self._cancel_requested = False

        self.get_logger().info(
            'GPS waypoint bridge up. Publish NavSatFix on /goal_gps to '
            'navigate, Empty on /go_home to return to base.')

    # ------------------------------------------------------------------
    def start_mission_goal(self, mission_id, goal_type, latitude, longitude):
        if self._goal_pending or self._goal_handle is not None:
            self.get_logger().warn("Cannot start mission: navigation busy.")
            return False
        if self._active_mission_goal is not None:
            self.get_logger().warn("Cannot start mission: another mission active.")
            return False

        self._active_mission_goal = {
            "mission_id": mission_id,
            "goal_type": goal_type,
            "goal_handle": None,
        }

        fix = NavSatFix()
        fix.latitude = latitude
        fix.longitude = longitude
        self.on_waypoint(fix)
        return True

    def on_mission_command(self, msg):
        try:
            command = json.loads(msg.data)

            if not isinstance(command, dict):
                raise ValueError('Command must be a JSON object.')

            mission_id = command.get('mission_id')
            goal_type = command.get('goal_type')
            latitude = command.get('latitude')
            longitude = command.get('longitude')

            if not isinstance(mission_id, str) or not mission_id.strip():
                raise ValueError('Invalid mission_id.')

            if goal_type not in ('DELIVERY', 'RTB'):
                raise ValueError('Invalid goal_type.')

            if goal_type == 'RTB':
                if self._home is None:
                    raise ValueError('RTB rejected: home is not set.')
                latitude, longitude = self._home

            for name, value, limit in (
                ('latitude', latitude, 90),
                ('longitude', longitude, 180),
            ):
                if (isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(value)
                        or not -limit <= value <= limit):
                    raise ValueError(f'Invalid {name}.')

        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid mission command: {exc}')
            return

        if (self._goal_pending or self._goal_handle is not None
                or self._active_mission_goal is not None):
            self.get_logger().warn(
                'Mission command rejected: navigation is busy.'
            )
            self.publish_mission_result(mission_id, goal_type, 'GOAL_FAILED')
            return

        if not self.mission_dispatch_enabled:
            self.get_logger().warn(
                'Mission command validated, but navigation dispatch is disabled.'
            )
            return

        self.get_logger().info(
            f'Mission command authorized: {mission_id} ({goal_type}).'
        )
        self.start_mission_goal(
            mission_id, goal_type, latitude, longitude
        )

    def on_fix(self, msg):
        if msg.status.status < 0 or math.isnan(msg.latitude):
            return
        self._last_fix = msg
        if self._home is None and self.get_parameter('auto_set_home').value:
            self._home = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Home set to {msg.latitude:.7f}, {msg.longitude:.7f} '
                f'(first valid fix).')
    
    def publish_nav_status(self, status):
        msg = String()
        msg.data = status
        self.nav_status_pub.publish(msg)
        self.get_logger().info(f'Navigation status: {status}')

    def publish_mission_result(self, mission_id, goal_type, status):
        if not isinstance(mission_id, str) or not mission_id.strip():
            raise ValueError('Mission result requires a valid mission_id.')

        if goal_type not in ('DELIVERY', 'RTB'):
            raise ValueError('Mission result has an invalid goal_type.')

        if status not in (
            'GOAL_SUCCEEDED', 'GOAL_CANCELED', 'GOAL_FAILED'
        ):
            raise ValueError('Mission result has an invalid status.')

        msg = String()
        msg.data = json.dumps({
            'mission_id': mission_id,
            'goal_type': goal_type,
            'status': status,
        })
        self.mission_result_pub.publish(msg)
        self.get_logger().info(f'Mission navigation result: {msg.data}')

    def publish_result_for_goal(self, goal_handle, status):
        mission = self._active_mission_goal

        if mission is None:
            return

        if mission.get('goal_handle') is not goal_handle:
            self.get_logger().warn(
                'Ignoring mission result from an unrelated navigation goal.'
            )
            return

        self.publish_mission_result(
            mission['mission_id'],
            mission['goal_type'],
            status
        )

    def fail_pending_mission_goal(self):
        mission = self._active_mission_goal
        if mission is None:
            return

        if mission.get('goal_handle') is not None:
            self.get_logger().warn(
                'Cannot fail a pending mission: goal is already accepted.'
            )
            return

        self.publish_mission_result(
            mission['mission_id'],
            mission['goal_type'],
            'GOAL_FAILED'
        )
        self._active_mission_goal = None

    def cancel_pending_mission_goal(self):
        mission = self._active_mission_goal
        if mission is None:
            return

        if mission.get('goal_handle') is not None:
            self.get_logger().warn(
                'Cannot cancel pending mission: goal is already accepted.'
            )
            return

        self.publish_mission_result(
            mission['mission_id'],
            mission['goal_type'],
            'GOAL_CANCELED'
        )
        self._active_mission_goal = None

    def on_set_home(self, _msg):
        if self._last_fix is None:
            self.get_logger().error('No GPS fix yet - cannot set home.')
            return
        self._home = (self._last_fix.latitude, self._last_fix.longitude)
        self.get_logger().info(
            f'Home re-set to {self._home[0]:.7f}, {self._home[1]:.7f}')

    def on_go_home(self, _msg):
        if self._home is None:
            self.get_logger().error(
                'Home is not set. No valid GPS fix has been received.')
            return
        self.get_logger().info('Returning to base.')
        fix = NavSatFix()
        fix.latitude, fix.longitude = self._home
        self.on_waypoint(fix)

    # ------------------------------------------------------------------
    def on_waypoint(self, msg):
        if self._goal_pending or self._goal_handle is not None:
            self.get_logger().warn(
                'Navigation request rejected: another goal is pending or active.'
            )
            return

        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self.get_logger().error('Waypoint has NaN coordinates, ignoring.')
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().error(
                'Waypoint is 0,0 - that is in the Atlantic. Ignoring.')
            return

        if not self.from_ll.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/fromLL not available. Is navsat_transform running, and has '
                'it had a GPS fix? It cannot convert without a datum.')
            self.fail_pending_mission_goal()
            return

        request = FromLL.Request()
        request.ll_point = GeoPoint()
        request.ll_point.latitude = float(msg.latitude)
        request.ll_point.longitude = float(msg.longitude)
        request.ll_point.altitude = 0.0

        self.get_logger().info(
            f'Converting {msg.latitude:.7f}, {msg.longitude:.7f}')
        self._goal_pending = True
        self.from_ll.call_async(request).add_done_callback(self.on_converted)

    def on_converted(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self._goal_pending = False
            self._cancel_requested = False
            self.get_logger().error(f'/fromLL failed: {exc}')
            self.fail_pending_mission_goal()
            return

        if self._cancel_requested:
            self._cancel_requested = False
            self._goal_pending = False
            self.get_logger().info(
                'Navigation request canceled before Nav2 submission.'
            )
            self.publish_nav_status('GOAL_CANCELED')
            self.cancel_pending_mission_goal()
            return

        x, y = result.map_point.x, result.map_point.y

        # Exactly (0,0) almost always means navsat_transform has no datum
        # rather than a goal genuinely at the origin. Nav2 would happily
        # drive there.
        if abs(x) < 1e-6 and abs(y) < 1e-6:
            self._goal_pending = False
            self._cancel_requested = False
            self.get_logger().error(
                'Converted to (0,0) in map. navsat_transform has no datum or '
                'the TF tree is incomplete. Check /gps/fix is valid and that '
                'map -> odom -> base_link all exist.')
            self.fail_pending_mission_goal()
            return

        yaw = float(self.get_parameter('goal_yaw').value)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Goal at map ({x:.2f}, {y:.2f}). Sending.')
        self.send(goal)

    # ------------------------------------------------------------------
    def send(self, pose):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self._goal_pending = False
            self._cancel_requested = False
            self.get_logger().error(
                'navigate_to_pose server missing. Nav2 is probably not '
                'ACTIVE - check the lifecycle manager brought every server up.')
            self.fail_pending_mission_goal()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.nav.send_goal_async(
            goal, feedback_callback=self.on_feedback
        ).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._goal_pending = False
            self._cancel_requested = False
            self.get_logger().error(f'Goal acceptance error: {exc}')
            self.fail_pending_mission_goal()
            return

        self._goal_pending = False

        if not handle.accepted:
            self._cancel_requested = False
            self.get_logger().error('Nav2 rejected the goal.')
            self.fail_pending_mission_goal()
            return

        self._goal_handle = handle

        if self._active_mission_goal is not None:
            self._active_mission_goal['goal_handle'] = handle
            self.get_logger().info(
                'Mission navigation goal accepted: '
                f"{self._active_mission_goal['mission_id']} "
                f"({self._active_mission_goal['goal_type']})"
            )

        self.get_logger().info('Goal accepted, navigating.')

        if self._cancel_requested:
            self._cancel_requested = False
            self.get_logger().warn(
                'Canceling goal accepted after cancellation was requested.'
            )
            self.on_cancel(None)

        handle.get_result_async().add_done_callback(
            lambda result_future, goal_handle=handle:
                self.on_result(result_future, goal_handle)
        )

    def on_feedback(self, msg):
        self.get_logger().info(
            f'{msg.feedback.distance_remaining:.1f} m remaining',
            throttle_duration_sec=3.0)

    def on_result(self, future, goal_handle):
        if self._goal_handle is not goal_handle:
            self.get_logger().warn(
                'Ignoring result from a goal that is no longer active.'
            )
            return

        try:
            result_status = future.result().status
            if result_status == 4:
                status = 'GOAL_SUCCEEDED'
            elif result_status == 5:
                status = 'GOAL_CANCELED'
            else:
                status = 'GOAL_FAILED'
        except Exception as exc:
            self.get_logger().error(f'Navigation result error: {exc}')
            status = 'GOAL_FAILED'

        self.get_logger().info(f'Navigation finished: {status}')
        self.publish_nav_status(status)
        self.publish_result_for_goal(goal_handle, status)

        mission = self._active_mission_goal
        if mission is not None and mission.get('goal_handle') is goal_handle:
            self._active_mission_goal = None

        self._goal_handle = None


    def on_cancel(self, _msg):
        if self._goal_pending:
            self._cancel_requested = True
            self.get_logger().warn(
                'Cancellation requested while navigation goal is pending.'
            )
            return

        if self._cancel_pending:
            self.get_logger().warn('Cancellation already pending.')
            return
        if self._goal_handle is None:
            self.get_logger().info('No accepted goal to cancel.')
            return
        self._cancel_pending = True
        try:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.on_cancel_response)
        except Exception as exc:
            self._cancel_pending = False
            self.get_logger().error(f'Cancellation request failed: {exc}')

    def on_cancel_response(self, future):
        self._cancel_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Cancellation response error: {exc}')
            return
        if response.goals_canceling:
            self.get_logger().info('Cancellation accepted; awaiting result.')
        else:
            self.get_logger().warn('Nav2 did not accept cancellation.')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GpsWaypointNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
