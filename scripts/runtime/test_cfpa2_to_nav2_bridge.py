import importlib.util
import json
from pathlib import Path

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange


def _load_bridge_module():
    path = Path(__file__).with_name("cfpa2_to_nav2_bridge.py")
    spec = importlib.util.spec_from_file_location("cfpa2_to_nav2_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _ClockNow:
    nanoseconds = 20_000_000_000

    def to_msg(self):
        msg = Time()
        msg.sec = 20
        msg.nanosec = 0
        return msg


class _Clock:
    def now(self):
        return _ClockNow()


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _bridge_for_unit_tests():
    module = _load_bridge_module()
    bridge = module.Cfpa2ToNav2Bridge.__new__(module.Cfpa2ToNav2Bridge)
    bridge.get_clock = lambda: _Clock()
    bridge.get_logger = lambda: _Logger()
    bridge.goal_change_min_m = 0.30
    bridge._last_pose_x = 0.0
    bridge._last_pose_y = 0.0
    bridge._last_goal_x = None
    bridge._last_goal_y = None
    bridge._goal_pub = _Publisher()
    bridge._nav_status_pub = _Publisher()
    bridge._goal_seq = 0
    bridge._active_goal = None
    bridge._planner_failure_count = 0
    bridge._planner_failure_threshold = 3
    bridge._planner_failure_node_names = {"ComputePathToPose"}
    bridge._reported_plan_failure_goal_seq = None
    return bridge


def _waypoint(x=4.5, y=-2.0):
    msg = PointStamped()
    msg.header.frame_id = "map"
    msg.point.x = x
    msg.point.y = y
    return msg


def _bt_log(node_name="ComputePathToPose", status="FAILURE"):
    msg = BehaviorTreeLog()
    change = BehaviorTreeStatusChange()
    change.node_name = node_name
    change.previous_status = "RUNNING"
    change.current_status = status
    msg.event_log.append(change)
    return msg


def _status_payloads(bridge):
    return [json.loads(msg.data) for msg in bridge._nav_status_pub.messages]


def test_new_waypoint_publishes_navigating_status_with_goal_seq():
    bridge = _bridge_for_unit_tests()

    bridge._on_waypoint(_waypoint(4.5, -2.0))

    payloads = _status_payloads(bridge)
    assert payloads[-1]["schema"] == "nav_status/v1"
    assert payloads[-1]["source"] == "cfpa2_to_nav2_bridge"
    assert payloads[-1]["state"] == "navigating"
    assert payloads[-1]["goal_seq"] == 1
    assert payloads[-1]["goal"] == [4.5, -2.0]


def test_bt_compute_path_failures_emit_unreachable_status_for_active_goal():
    bridge = _bridge_for_unit_tests()
    bridge._on_waypoint(_waypoint(4.5, -2.0))

    for _ in range(5):
        bridge._on_behavior_tree_log(_bt_log())

    payloads = _status_payloads(bridge)
    unreachable = [p for p in payloads if p["state"] == "unreachable"]
    assert len(unreachable) == 3
    assert all(p["goal_seq"] == 1 for p in unreachable)
    assert all(p["goal"] == [4.5, -2.0] for p in unreachable)
    assert all(p["reason"] == "bt_compute_path_failure" for p in unreachable)
    assert all(p["bt_node"] == "ComputePathToPose" for p in unreachable)


def test_bt_compute_path_success_resets_failure_counter():
    bridge = _bridge_for_unit_tests()
    bridge._on_waypoint(_waypoint(4.5, -2.0))

    bridge._on_behavior_tree_log(_bt_log(status="FAILURE"))
    bridge._on_behavior_tree_log(_bt_log(status="SUCCESS"))
    bridge._on_behavior_tree_log(_bt_log(status="FAILURE"))
    bridge._on_behavior_tree_log(_bt_log(status="FAILURE"))

    unreachable = [p for p in _status_payloads(bridge) if p["state"] == "unreachable"]
    assert unreachable == []
