from collections import deque

from nav_msgs.msg import OccupancyGrid, Odometry

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _node_in_blocked_pivot_zone():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.get_logger = lambda: _Logger()
    node.pivot_lock_radius_m = 0.45
    node.pivot_lock_max_hold_sec = 15.0
    node.pivot_lock_regress_release_m = 0.50
    node.goal_satisfied_dist = 1.0
    node.goal_satisfied_direct_dist = 0.30
    node.goal_satisfied_requires_los = True
    node.unknown_value = -1
    node.occ_thresh = 50
    node.blacklist_key_resolution = 0.5

    msg = OccupancyGrid()
    msg.info.resolution = 0.1
    msg.info.width = 30
    msg.info.height = 30
    msg.info.origin.position.x = -1.5
    msg.info.origin.position.y = -1.5
    msg.data = [0] * (msg.info.width * msg.info.height)
    msg.data[15 * msg.info.width + 18] = 100

    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0

    node.odoms = {"robot_b": odom}
    node.maps = {"robot_b": msg}
    node._cur_planning_map = msg
    node.last_goal = {"robot_b": (2.0, 0.0)}
    node.last_goal_set_time_ns = {"robot_b": 0}
    node.goal_progress_samples = {"robot_b": deque([(1, 2.0)])}
    node.goal_lock_start_xy = {"robot_b": None}
    node.goal_lock_pose_history = {"robot_b": deque([(1, 0.0, 0.0)])}
    node._last_unreachable_goal_seq = {"robot_b": 7}
    node._unreachable_consec = {"robot_b": {(4, 0): 3}}
    node.goal_blacklist_until_ns = {"robot_b": {}}
    node.goal_blacklist_disks = {"robot_b": []}
    node._pivot_lock_held_since_ns = {}
    node._pivot_lock_start_dist_m = {}
    node.last_policy_reason = {"robot_b": ""}
    return node


def test_pivot_lock_holds_goal_before_max_hold():
    node = _node_in_blocked_pivot_zone()

    goal = node._set_active_goal("robot_b", (3.0, 0.0), 10_000_000_000)

    assert goal == (2.0, 0.0)
    assert node.last_goal["robot_b"] == (2.0, 0.0)
    assert node._pivot_lock_held_since_ns["robot_b"] == 10_000_000_000
    assert node._pivot_lock_start_dist_m["robot_b"] == 2.0
    assert node.last_policy_reason["robot_b"] == "hold/narrow_passage_pivot_lock"


def test_pivot_lock_releases_goal_after_max_hold():
    node = _node_in_blocked_pivot_zone()
    node._set_active_goal("robot_b", (3.0, 0.0), 10_000_000_000)

    goal = node._set_active_goal("robot_b", (3.0, 0.0), 26_000_000_000)

    assert goal == (3.0, 0.0)
    assert node.last_goal["robot_b"] == (3.0, 0.0)
    assert "robot_b" not in node._pivot_lock_held_since_ns
    assert "robot_b" not in node._pivot_lock_start_dist_m
    assert node.last_policy_reason["robot_b"] == "switch/pivot_lock_max_hold"
    assert node.last_goal_set_time_ns["robot_b"] == 26_000_000_000
    assert len(node.goal_progress_samples["robot_b"]) == 0
    assert len(node.goal_lock_pose_history["robot_b"]) == 0
    assert "robot_b" not in node._last_unreachable_goal_seq
    assert "robot_b" not in node._unreachable_consec


def test_pivot_lock_releases_when_held_goal_distance_regresses():
    node = _node_in_blocked_pivot_zone()
    node._pivot_clearance_blocked = lambda _ns: True
    node._set_active_goal("robot_b", (3.0, 0.0), 10_000_000_000)

    node.odoms["robot_b"].pose.pose.position.x = -0.60
    goal = node._set_active_goal("robot_b", (3.0, 0.0), 12_000_000_000)

    assert goal == (3.0, 0.0)
    assert node.last_goal["robot_b"] == (3.0, 0.0)
    assert "robot_b" not in node._pivot_lock_held_since_ns
    assert "robot_b" not in node._pivot_lock_start_dist_m
    assert node.last_policy_reason["robot_b"] == "switch/pivot_lock_regress_release"
