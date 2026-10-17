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
