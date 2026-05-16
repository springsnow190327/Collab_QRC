from nav_msgs.msg import Odometry

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def _node():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.cfpa2_max_goal_distance_m = 2.5
    node.min_assign_distance = 0.0
    node.blacklist_key_resolution = 0.5
    node.goal_blacklist_until_ns = {"robot": {}}
    node.goal_blacklist_disks = {"robot": []}
    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    node.odoms = {"robot": odom}
    return node


def test_goal_too_far_rejects_large_elevation_map_jump():
    node = _node()

    assert not node._goal_too_far("robot", (2.0, 0.0))
    assert node._goal_too_far("robot", (3.0, 0.0))


def test_best_available_goal_prefers_local_fallback_over_far_candidate():
    node = _node()
    goal = node._cfpa2_best_available_goal(
        ns="robot",
        now_ns=1,
        utilities={(8.0, 0.0): 100.0},
        fallback_targets=[(8.0, 0.0), (1.2, 0.0)],
    )

    assert goal == (1.2, 0.0)
