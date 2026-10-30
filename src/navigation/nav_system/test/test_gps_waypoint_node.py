import json

import rclpy
from std_msgs.msg import String

from nav_system.gps_waypoint_node import GpsWaypointNode


def test_mission_dispatch_disabled():
    rclpy.init()
    node = GpsWaypointNode()

    try:
        command = {
            "mission_id": "test-mission-001",
            "goal_type": "DELIVERY",
            "latitude": 32.2,
            "longitude": -110.9,
        }

        msg = String(data=json.dumps(command))
        node.on_mission_command(msg)

        assert not node.mission_dispatch_enabled
        assert node._active_mission_goal is None
        assert node._goal_handle is None
        assert not node._goal_pending

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_invalid_mission_commands():
    rclpy.init()
    node = GpsWaypointNode()

    try:
        invalid_commands = [
            "not valid json",
            json.dumps({}),
            json.dumps({
                "mission_id": "",
                "goal_type": "DELIVERY",
                "latitude": 32.2,
                "longitude": -110.9,
            }),
            json.dumps({
                "mission_id": "test-002",
                "goal_type": "DELIVERY",
                "latitude": 100.0,
                "longitude": -110.9,
            }),
            json.dumps({
                "mission_id": "test-003",
                "goal_type": "UNKNOWN",
                "latitude": 32.2,
                "longitude": -110.9,
            }),
        ]

        for command in invalid_commands:
            node.on_mission_command(String(data=command))

            assert node._active_mission_goal is None
            assert node._goal_handle is None
            assert not node._goal_pending

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_rtb_without_home():
    rclpy.init()
    node = GpsWaypointNode()

    try:
        assert node._home is None

        command = {
            "mission_id": "test-rtb-001",
            "goal_type": "RTB",
        }

        node.on_mission_command(
            String(data=json.dumps(command))
        )

        assert node._home is None
        assert node._active_mission_goal is None
        assert node._goal_handle is None
        assert not node._goal_pending

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_mission_result_correlation():
    rclpy.init()
    node = GpsWaypointNode()

    try:
        goal_handle = object()
        unrelated_handle = object()

        node._active_mission_goal = {
            "mission_id": "test-mission-004",
            "goal_type": "DELIVERY",
            "goal_handle": goal_handle,
        }

        published = []

        node.publish_mission_result = (
            lambda mission_id, goal_type, status:
            published.append((mission_id, goal_type, status))
        )

        # An unrelated navigation goal must not produce a result.
        node.publish_result_for_goal(
            unrelated_handle, "GOAL_SUCCEEDED"
        )

        assert published == []

        # The matching goal should produce the correct mission result.
        node.publish_result_for_goal(
            goal_handle, "GOAL_SUCCEEDED"
        )

        assert published == [
            ("test-mission-004", "DELIVERY", "GOAL_SUCCEEDED")
        ]

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_gps_conversion_failure():
    from unittest.mock import Mock

    rclpy.init()
    node = GpsWaypointNode()

    try:
        node._active_mission_goal = {
            "mission_id": "test-mission-005",
            "goal_type": "DELIVERY",
            "goal_handle": None,
        }
        node._goal_pending = True

        published = []
        node.publish_mission_result = (
            lambda mission_id, goal_type, status:
            published.append((mission_id, goal_type, status))
        )

        future = Mock()
        future.result.side_effect = RuntimeError(
            "GPS conversion unavailable"
        )

        node.on_converted(future)

        assert not node._goal_pending
        assert node._active_mission_goal is None
        assert published == [
            ("test-mission-005", "DELIVERY", "GOAL_FAILED")
        ]

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_cancel_during_gps_conversion():
    from unittest.mock import Mock
    from std_msgs.msg import Empty

    rclpy.init()
    node = GpsWaypointNode()

    try:
        node._active_mission_goal = {
            "mission_id": "test-mission-006",
            "goal_type": "DELIVERY",
            "goal_handle": None,
        }
        node._goal_pending = True

        published = []
        node.publish_mission_result = (
            lambda mission_id, goal_type, status:
            published.append((mission_id, goal_type, status))
        )

        node.send = Mock()

        # Request cancellation while conversion is pending.
        node.on_cancel(Empty())

        assert node._cancel_requested

        # Simulate successful conversion after cancellation.
        future = Mock()
        future.result.return_value.map_point.x = 10.0
        future.result.return_value.map_point.y = 20.0

        node.on_converted(future)

        assert not node._goal_pending
        assert not node._cancel_requested
        assert node._active_mission_goal is None
        node.send.assert_not_called()

        assert published == [
            ("test-mission-006", "DELIVERY", "GOAL_CANCELED")
        ]

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_cancel_accepted_navigation_goal():
    from unittest.mock import Mock
    from std_msgs.msg import Empty

    rclpy.init()
    node = GpsWaypointNode()

    try:
        goal_handle = Mock()
        cancel_future = Mock()
        goal_handle.cancel_goal_async.return_value = cancel_future

        node._goal_handle = goal_handle

        # Request cancellation of an accepted goal.
        node.on_cancel(Empty())

        assert node._cancel_pending
        goal_handle.cancel_goal_async.assert_called_once()

        # Simulate Nav2 accepting the cancellation request.
        response = Mock()
        response.goals_canceling = [goal_handle]
        cancel_future.result.return_value = response

        node.on_cancel_response(cancel_future)

        assert not node._cancel_pending

        # The goal remains active until Nav2 returns its final result.
        assert node._goal_handle is goal_handle

    finally:
        node.destroy_node()
        rclpy.shutdown()

def test_final_canceled_navigation_result():
    from unittest.mock import Mock

    rclpy.init()
    node = GpsWaypointNode()

    try:
        goal_handle = Mock()
        node._goal_handle = goal_handle
        node._active_mission_goal = {
            "mission_id": "test-mission-007",
            "goal_type": "DELIVERY",
            "goal_handle": goal_handle,
        }

        published = []
        node.publish_mission_result = (
            lambda mission_id, goal_type, status:
            published.append((mission_id, goal_type, status))
        )

        future = Mock()
        future.result.return_value.status = 5  # STATUS_CANCELED

        node.on_result(future, goal_handle)

        assert published == [
            ("test-mission-007", "DELIVERY", "GOAL_CANCELED")
        ]
        assert node._goal_handle is None
        assert node._active_mission_goal is None

    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mission_specific_cancellation():
    from unittest.mock import Mock

    rclpy.init()
    node = GpsWaypointNode()

    try:
        node.on_cancel = Mock()

        # No active mission: cancellation must be ignored.
        node.on_mission_cancel(String(data="mission-001"))
        node.on_cancel.assert_not_called()

        node._active_mission_goal = {
            "mission_id": "mission-001",
            "goal_type": "DELIVERY",
            "goal_handle": None,
        }

        # Incorrect mission ID: cancellation must be rejected.
        node.on_mission_cancel(String(data="wrong-mission"))
        node.on_cancel.assert_not_called()

        # Matching mission ID: cancellation is authorized.
        node.on_mission_cancel(String(data="mission-001"))
        node.on_cancel.assert_called_once_with(None)

    finally:
        node.destroy_node()
        rclpy.shutdown()
