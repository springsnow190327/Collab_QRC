from nav_msgs.msg import OccupancyGrid, Odometry

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def _node_with_map(blocked_cells=None, unknown_cells=None):
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.goal_satisfied_dist = 1.0
    node.goal_satisfied_direct_dist = 0.30
    node.goal_satisfied_requires_los = True
    node.switch_min_dist = 0.80
    node.min_assign_distance = 0.30
    node.unknown_value = -1
    node.occ_thresh = 50

    msg = OccupancyGrid()
    msg.info.resolution = 0.1
    msg.info.width = 20
    msg.info.height = 5
    msg.info.origin.position.x = -0.5
    msg.info.origin.position.y = -0.25
    msg.data = [0] * (msg.info.width * msg.info.height)

    for gx, gy in blocked_cells or []:
        msg.data[gy * msg.info.width + gx] = 100
    for gx, gy in unknown_cells or []:
        msg.data[gy * msg.info.width + gx] = -1

    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    node.odoms = {"robot_b": odom}
    node.maps = {"robot_b": msg}
    node._cur_planning_map = msg
    return node


def test_occluded_frontier_within_sensing_radius_is_not_satisfied():
    blocked_column = [(10, gy) for gy in range(5)]
    node = _node_with_map(blocked_cells=blocked_column)

    assert not node._goal_satisfied("robot_b", (0.9, 0.0))


def test_unknown_between_robot_and_frontier_is_not_satisfied():
    node = _node_with_map(unknown_cells=[(10, 2)])

    assert not node._goal_satisfied("robot_b", (0.9, 0.0))


def test_visible_frontier_within_sensing_radius_is_satisfied():
    node = _node_with_map()

    assert node._goal_satisfied("robot_b", (0.9, 0.0))
