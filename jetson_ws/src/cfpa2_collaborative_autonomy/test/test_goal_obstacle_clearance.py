from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def _grid(width=20, height=20, resolution=0.1):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -1.0
    msg.data = [0] * (width * height)
    return msg


def _node():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.unknown_value = -1
    node.occ_thresh = 50
    node.cfpa2_goal_obstacle_clearance_m = 0.25
    node.min_assign_distance = 0.0
    node.blacklist_key_resolution = 0.5
    node.goal_blacklist_until_ns = {"robot_b": {}}
    node.goal_blacklist_disks = {"robot_b": []}
    node.last_goal = {}
    node.odoms = {}
    node.odom_velocity_xy = {}
    node._adaptive_exploration_gain_radius_cells = 4
    node.cfpa2_w_ig = 1.0
    node.cfpa2_w_c = 0.3
    node.cfpa2_w_sw = 0.2
    node.cfpa2_w_momentum = 2.0
    node.cfpa2_momentum_alpha = 1.5
    node.cfpa2_momentum_beta = 2.0
    node.goal_satisfied_dist = 0.0
    node.goal_satisfied_direct_dist = 0.0
    node.goal_satisfied_requires_los = True
    return node


def _set_occ(msg, gx, gy, value=100):
    msg.data[gy * msg.info.width + gx] = value


def test_goal_obstacle_clearance_rejects_newly_mapped_obstacle():
    node = _node()
    msg = _grid()
    # World (0.20, 0.00), 20 cm from the goal. A frontier may have been
    # assigned before this cell was mapped; after it appears, the active goal
    # must be invalidated.
    _set_occ(msg, 12, 10)

    assert not node._goal_has_obstacle_clearance(msg, (0.0, 0.0))


def test_goal_obstacle_clearance_allows_clear_goal():
    node = _node()
    msg = _grid()
    _set_occ(msg, 14, 10)

    assert node._goal_has_obstacle_clearance(msg, (0.0, 0.0))


def test_cfpa2_single_utility_rejects_goal_without_clearance():
    node = _node()
    msg = _grid()
    _set_occ(msg, 12, 10)
    dist_map = {10 * msg.info.width + 10: 5}

    score = node._cfpa2_single_utility(
        ns="robot_b",
        goal=(0.0, 0.0),
        map_msg=msg,
        dist_map=dist_map,
    )

    assert score <= -1e17


def test_cfpa2_single_utility_rejects_goal_already_satisfied():
    node = _node()
    node.goal_satisfied_dist = 0.65
    node.goal_satisfied_direct_dist = 0.30
    msg = _grid()
    goal = (0.5, 0.0)
    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.orientation.w = 1.0
    node.odoms = {"robot_b": odom}

    goal_cell = node._world_to_grid(msg, goal[0], goal[1])
    assert goal_cell is not None
    for gy in range(goal_cell[1] - 2, goal_cell[1] + 3):
        for gx in range(goal_cell[0] + 1, goal_cell[0] + 4):
            if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
                msg.data[gy * msg.info.width + gx] = node.unknown_value
    dist_map = {goal_cell[1] * msg.info.width + goal_cell[0]: 5}

    score = node._cfpa2_single_utility(
        ns="robot_b",
        goal=goal,
        map_msg=msg,
        dist_map=dist_map,
    )

    assert score <= -1e17


def test_held_goal_safety_failure_reports_unsafe_clearance():
    node = _node()
    msg = _grid()
    _set_occ(msg, 12, 10)
    dist_map = {10 * msg.info.width + 10: 5}

    reason = node._held_goal_safety_failure(msg, dist_map, (0.0, 0.0))

    assert reason == "unsafe_clearance"


def test_fallback_goal_selection_skips_unsafe_clearance_target():
    node = _node()
    msg = _grid(width=30)
    _set_occ(msg, 12, 10)
    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    node.odoms = {"robot_b": odom}
    safe_goal = (0.8, 0.0)
    safe_grid = node._world_to_grid(msg, safe_goal[0], safe_goal[1])
    assert safe_grid is not None
    dist_map = {
        10 * msg.info.width + 10: 1,
        safe_grid[1] * msg.info.width + safe_grid[0]: 8,
    }

    goal = node._cfpa2_best_available_goal(
        ns="robot_b",
        now_ns=10_000_000_000,
        utilities={},
        fallback_targets=[(0.0, 0.0), safe_goal],
        map_msg=msg,
        dist_map=dist_map,
    )

    assert goal == safe_goal
