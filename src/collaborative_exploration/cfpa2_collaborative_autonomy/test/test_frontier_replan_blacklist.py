from collections import deque
import json

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator
from std_msgs.msg import String


class _ClockMsg:
    nanoseconds = 20_000_000_000


class _Clock:
    def now(self):
        return _ClockMsg()


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def test_frontier_replan_blacklists_current_goal_cluster():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    node.local_nav_stall_blacklist_sec = 45.0
    node.blacklist_key_resolution = 0.5
    node.blacklist_cluster_radius_m = 1.0
    node.last_goal = {"robot_b": (4.75, -5.20)}
    node.goal_blacklist_until_ns = {"robot_b": {}}
    node.goal_blacklist_disks = {"robot_b": []}
    node.goal_fail_counts = {"robot_b": {}}
    node.goal_progress_samples = {"robot_b": deque([(1, 5.0)])}
    node._frontier_replan_last_bl_ns = {"robot_b": 0}

    node._frontier_replan_cb("robot_b")

    assert node.goal_blacklist_until_ns["robot_b"]
    assert node.goal_blacklist_disks["robot_b"] == [
        (4.75, -5.20, 1.0, 65_000_000_000)
    ]
    assert len(node.goal_progress_samples["robot_b"]) == 0


def test_nav_status_unreachable_blacklists_current_goal_after_debounce():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    node.fast_unreachable_enabled = True
    node.fast_unreachable_blacklist_sec = 60.0
    node.fast_unreachable_startup_grace_sec = 0.0
    node.fast_unreachable_consecutive_threshold = 3
    node._node_start_ns = 0
    node.blacklist_key_resolution = 0.5
    node.blacklist_cluster_radius_m = 1.0
    node.last_goal = {"robot_b": (4.75, -5.20)}
    node.nav_status = {}
    node.nav_status_rx_time_ns = {}
    node._last_unreachable_goal_seq = {}
    node._unreachable_consec = {}
    node.goal_blacklist_until_ns = {"robot_b": {}}
    node.goal_blacklist_disks = {"robot_b": []}
    node.goal_fail_counts = {"robot_b": {}}
    node.goal_progress_samples = {"robot_b": deque([(1, 5.0)])}

    msg = String()
    msg.data = json.dumps({
        "schema": "nav_status/v1",
        "source": "cfpa2_to_nav2_bridge",
        "state": "unreachable",
        "reason": "bt_compute_path_failure",
        "goal_seq": 9,
        "goal": [4.75, -5.20],
    })

    node._nav_status_cb(msg, "robot_b")
    node._nav_status_cb(msg, "robot_b")
    assert node.goal_blacklist_until_ns["robot_b"] == {}

    node._nav_status_cb(msg, "robot_b")

    assert node.goal_blacklist_until_ns["robot_b"]
    assert node.goal_blacklist_disks["robot_b"] == [
        (4.75, -5.20, 1.0, 80_000_000_000)
    ]
    assert len(node.goal_progress_samples["robot_b"]) == 0
