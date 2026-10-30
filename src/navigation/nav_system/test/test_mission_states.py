from nav_system.mission_supervisor import MissionSupervisor, MissionState
from sensor_msgs.msg import NavSatFix
import rclpy

def test_dispatch_disabled():
    rclpy.init()
    node = MissionSupervisor()
    try:
        goal = NavSatFix()
        goal.latitude = 32.2
        goal.longitude = -110.9
        node.on_destination(goal)
        assert node.state == MissionState.IDLE
        assert node.mission_id is None
        assert not node.navigation_dispatch_enabled
    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_invalid_destination():
    rclpy.init()
    node = MissionSupervisor()
    try:
        node.navigation_dispatch_enabled = True
        goal = NavSatFix()
        goal.latitude = 100.0
        goal.longitude = -110.9
        node.on_destination(goal)
        assert node.state == MissionState.IDLE
        assert node.mission_id is None
        assert node.destination is None
    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_delivery_failure():
    rclpy.init()
    node = MissionSupervisor()
    try:
        node.navigation_dispatch_enabled = True
        node.on_destination(NavSatFix(latitude=32.2, longitude=-110.9))
        assert node.state == MissionState.DELIVERING
        from std_msgs.msg import String
        import json
        result = dict(mission_id=node.mission_id,
                      goal_type="DELIVERY", status="GOAL_FAILED")
        node.on_navigation_result(String(data=json.dumps(result)))
        assert node.state == MissionState.FAIL_SAFE
    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_successful_mission():
    import json
    from std_msgs.msg import String, Bool
    rclpy.init()
    node = MissionSupervisor()
    try:
        node.navigation_dispatch_enabled = True
        node.on_destination(NavSatFix(latitude=32.2, longitude=-110.9))
        mid = node.mission_id
        def result(kind):
            data = dict(mission_id=mid, goal_type=kind,
                        status="GOAL_SUCCEEDED")
            node.on_navigation_result(String(data=json.dumps(data)))
        result("DELIVERY")
        assert node.state == MissionState.ARRIVAL_CONFIRMATION
        node.on_delivery_confirmed(Bool(data=True))
        assert node.state == MissionState.RETURNING_HOME
        result("RTB")
        assert node.state == MissionState.MISSION_COMPLETE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_supervisor_cancellation():
    import json
    from unittest.mock import Mock
    from std_msgs.msg import Bool, String

    for goal_type in ("DELIVERY", "RTB"):
        rclpy.init()
        node = MissionSupervisor()

        try:
            node.navigation_dispatch_enabled = True
            node.on_destination(
                NavSatFix(latitude=32.2, longitude=-110.9)
            )

            if goal_type == "RTB":
                delivery_result = {
                    "mission_id": node.mission_id,
                    "goal_type": "DELIVERY",
                    "status": "GOAL_SUCCEEDED",
                }
                node.on_navigation_result(
                    String(data=json.dumps(delivery_result))
                )
                node.on_delivery_confirmed(Bool(data=True))
                assert node.state == MissionState.RETURNING_HOME

            node.nav_cancel_pub.publish = Mock()

            node.on_cancel(Bool(data=True))

            assert node.cancel_pending
            node.nav_cancel_pub.publish.assert_called_once()
            assert (
                node.nav_cancel_pub.publish.call_args.args[0].data
                == node.mission_id
            )

            # Duplicate cancellation must not publish another request.
            node.on_cancel(Bool(data=True))
            node.nav_cancel_pub.publish.assert_called_once()

            # A late success must not advance a canceled mission.
            result = {
                "mission_id": node.mission_id,
                "goal_type": goal_type,
                "status": "GOAL_SUCCEEDED",
            }
            node.on_navigation_result(
                String(data=json.dumps(result))
            )

            assert node.state == MissionState.FAIL_SAFE
            assert not node.cancel_pending
            assert node.expected_goal_type is None

        finally:
            node.destroy_node()
            rclpy.shutdown()


def test_cancel_at_arrival_confirmation():
    import json
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = MissionSupervisor()

    try:
        node.navigation_dispatch_enabled = True
        node.on_destination(
            NavSatFix(latitude=32.2, longitude=-110.9)
        )

        delivery_result = {
            "mission_id": node.mission_id,
            "goal_type": "DELIVERY",
            "status": "GOAL_SUCCEEDED",
        }
        node.on_navigation_result(
            String(data=json.dumps(delivery_result))
        )

        assert node.state == MissionState.ARRIVAL_CONFIRMATION

        node.on_cancel(Bool(data=True))

        assert node.state == MissionState.FAIL_SAFE
        assert not node.cancel_pending

        # Confirmation afterward must not initiate RTB.
        node.on_delivery_confirmed(Bool(data=True))
        assert node.state == MissionState.FAIL_SAFE

    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cancellation_timeout():
    import json
    import time
    from unittest.mock import Mock
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = MissionSupervisor()

    try:
        node.navigation_dispatch_enabled = True
        node.on_destination(
            NavSatFix(latitude=32.2, longitude=-110.9)
        )

        node.nav_cancel_pub.publish = Mock()
        node.on_cancel(Bool(data=True))

        assert node.cancel_pending
        assert node.cancel_deadline is not None
        assert node.state == MissionState.DELIVERING

        # Simulate the cancellation deadline expiring.
        node.cancel_deadline = time.monotonic() - 1.0
        node.check_cancel_timeout()

        assert node.state == MissionState.FAIL_SAFE
        assert node.cancel_pending
        assert node.cancel_deadline is None

        # A late matching result clears pending cancellation
        # without allowing normal mission progression.
        result = {
            "mission_id": node.mission_id,
            "goal_type": "DELIVERY",
            "status": "GOAL_SUCCEEDED",
        }
        node.on_navigation_result(
            String(data=json.dumps(result))
        )

        assert node.state == MissionState.FAIL_SAFE
        assert not node.cancel_pending
        assert node.cancel_deadline is None
        assert node.expected_goal_type is None

    finally:
        node.destroy_node()
        rclpy.shutdown()
