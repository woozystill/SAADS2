from rclpy.executors import ExternalShutdownException
#!/usr/bin/env python3
"""SAADS autonomous mission supervisor.

Initial implementation: mission-state management only.
Navigation commands will be integrated after goal-result coordination
with gps_waypoint_node has been implemented and tested.
"""

from enum import Enum
import json
import math
import time
import uuid

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class MissionState(Enum):
    IDLE = "IDLE"
    DELIVERING = "DELIVERING"
    ARRIVAL_CONFIRMATION = "ARRIVAL_CONFIRMATION"
    RETURNING_HOME = "RETURNING_HOME"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    FAIL_SAFE = "FAIL_SAFE"


class MissionSupervisor(Node):

    def __init__(self):
        super().__init__("mission_supervisor")

        self.state = MissionState.IDLE
        self.destination = None
        self.mission_id = None
        self.expected_goal_type = None
        self.navigation_dispatch_enabled = False
        self.cancel_pending = False
        self.cancel_timeout_seconds = 10.0
        self.cancel_deadline = None

        self.status_pub = self.create_publisher(
            String, "/mission/status", 10
        )

        self.create_subscription(
            NavSatFix,
            "/mission/destination",
            self.on_destination,
            10,
        )

        self.nav_command_pub = self.create_publisher(String, "/mission/navigation_command", 10)
        self.nav_cancel_pub = self.create_publisher(String, "/mission/navigation_cancel", 10)
        self.create_subscription(String, "/mission/navigation_result", self.on_navigation_result, 10)
        self.create_subscription(Bool, "/mission/delivery_confirmed", self.on_delivery_confirmed, 10)
        self.create_subscription(Bool, "/mission/cancel", self.on_cancel, 10)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            "SAADS mission supervisor initialized in IDLE state."
        )

    def set_state(self, new_state):
        if self.state == new_state:
            return

        self.get_logger().info(
            f"Mission state: {self.state.value} -> {new_state.value}"
        )

        self.state = new_state
        self.publish_status()

    def on_destination(self, msg):
        if not self.navigation_dispatch_enabled:
            self.get_logger().warn(
                "Mission rejected: navigation dispatch is disabled."
            )
            return
        if self.state != MissionState.IDLE:
            self.get_logger().warn(
                "Mission already active. Destination rejected."
            )
            return

        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            self.get_logger().error("Non-finite destination coordinates rejected.")
            return

        if not (-90.0 <= msg.latitude <= 90.0):
            self.get_logger().error("Invalid destination latitude.")
            return

        if not (-180.0 <= msg.longitude <= 180.0):
            self.get_logger().error("Invalid destination longitude.")
            return

        self.destination = (msg.latitude, msg.longitude)
        self.mission_id = uuid.uuid4().hex
        self.expected_goal_type = "DELIVERY"

        self.get_logger().info(
            f"Mission destination received: "
            f"{msg.latitude:.7f}, {msg.longitude:.7f}"
        )

        self.set_state(MissionState.DELIVERING)

        self.dispatch_navigation("DELIVERY", self.destination)

    def on_navigation_result(self, msg):
        try:
            result = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn("Malformed navigation result ignored.")
            return

        if not isinstance(result, dict):
            return
        if result.get("mission_id") != self.mission_id or self.mission_id is None:
            return
        if result.get("goal_type") != self.expected_goal_type:
            return
        if result.get("status") not in ("GOAL_SUCCEEDED", "GOAL_FAILED", "GOAL_CANCELED"):
            return

        status = result["status"]
        if self.cancel_pending:
            self.cancel_pending = False
            self.cancel_deadline = None
            self.expected_goal_type = None
            self.get_logger().warn("Navigation ended after cancellation was requested. Mission entering FAIL_SAFE.")
            self.set_state(MissionState.FAIL_SAFE)
            return


        if self.state == MissionState.DELIVERING:
            self.expected_goal_type = None
            if status == "GOAL_SUCCEEDED":
                self.set_state(MissionState.ARRIVAL_CONFIRMATION)
            else:
                self.set_state(MissionState.FAIL_SAFE)
        elif self.state == MissionState.RETURNING_HOME:
            self.expected_goal_type = None
            if status == "GOAL_SUCCEEDED":
                self.set_state(MissionState.MISSION_COMPLETE)
            else:
                self.set_state(MissionState.FAIL_SAFE)

    def dispatch_navigation(self, goal_type, coordinates):
        if not self.navigation_dispatch_enabled:
            self.get_logger().warn("Navigation dispatch remains disabled.")
            return
        command = String()
        payload = {"mission_id": self.mission_id, "goal_type": goal_type}
        if goal_type == "DELIVERY":
            payload["latitude"], payload["longitude"] = coordinates
        command.data = json.dumps(payload)
        self.nav_command_pub.publish(command)

    def on_cancel(self, msg):
        if not msg.data:
            return

        if self.state == MissionState.ARRIVAL_CONFIRMATION:
            self.get_logger().warn("Mission canceled while awaiting delivery confirmation")
            self.expected_goal_type = None
            self.set_state(MissionState.FAIL_SAFE)
            return

        if self.state not in (MissionState.DELIVERING, MissionState.RETURNING_HOME,):
            self.get_logger().warn("Cancellation ignored: no active navigation mission.")
            return

        if self.cancel_pending:
            self.get_logger().warn("Mission cancellation already pending.")
            return
        self.cancel_pending = True
        self.cancel_deadline = time.monotonic() + self.cancel_timeout_seconds

        cancel_msg = String()
        cancel_msg.data = self.mission_id
        self.nav_cancel_pub.publish(cancel_msg)

        self.get_logger().warn("Mission cancellation requested; awaiting navigation result.")

    def on_delivery_confirmed(self, msg):
        if self.cancel_pending:
            self.get_logger().warn("Delivery confirmation ignored: mission cancellation pending.")
            return

        if self.state != MissionState.ARRIVAL_CONFIRMATION:
            self.get_logger().warn("Delivery confirmation ignored: not awaiting confirmation.")
            return
        if not msg.data:
            return
        self.get_logger().info("Package delivery confirmed.")
        self.expected_goal_type = "RTB"
        self.set_state(MissionState.RETURNING_HOME)
        self.dispatch_navigation("RTB", None)

    def check_cancel_timeout(self):
        if not self.cancel_pending or self.cancel_deadline is None:
            return

        if time.monotonic() < self.cancel_deadline:
            return

        self.cancel_deadline = None
        self.get_logger().error(
            "Mission cancellation timed out. Navigation stop is NOT confirmed."
        )
        self.set_state(MissionState.FAIL_SAFE)

    def publish_status(self):
        self.check_cancel_timeout()
        msg = String()
        msg.data = self.state.value
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
