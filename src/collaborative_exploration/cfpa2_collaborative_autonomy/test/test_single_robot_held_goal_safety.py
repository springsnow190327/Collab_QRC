from collections import defaultdict

from nav_msgs.msg import OccupancyGrid, Odometry

from cfpa2_collaborative_autonomy.cfpa2_single_robot_node import CFPA2SingleRobotNode


def _split_map():
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = 5
    msg.info.height = 3
    msg.info.resolution = 1.0
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    for gy in range(msg.info.height):
        msg.data[gy * msg.info.width + 2] = 100
    return msg


def _high_cost_corridor_map():
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = 5
    msg.info.height = 3
    msg.info.resolution = 1.0
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    for gx in range(1, 4):
        for gy in range(msg.info.height):
            msg.data[gy * msg.info.width + gx] = 90
    return msg


def _unknown_corridor_map():
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = 5
    msg.info.height = 3
    msg.info.resolution = 1.0
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    for gx in range(1, 4):
        for gy in range(msg.info.height):
            msg.data[gy * msg.info.width + gx] = -1
    return msg


class _Clock:
    class _Now:
        nanoseconds = int(10e9)

    def now(self):
        return self._Now()


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def debug(self, *_args, **_kwargs):
        pass


def _single_robot_node_for_tick(
    map_msg,
    held_goal=None,
    frontier_goal=None,
    goal_scores=None,
):
    node = CFPA2SingleRobotNode.__new__(CFPA2SingleRobotNode)
    ns = "robot"
    odom = Odometry()
    odom.pose.pose.position.x = 0.5
    odom.pose.pose.position.y = 1.5
    odom.pose.pose.orientation.w = 1.0

    node.namespaces = [ns]
    node.maps = {ns: map_msg}
    node.odoms = {ns: odom}
    node.odom_velocity_xy = {ns: (0.0, 0.0)}
    node.last_goal = {ns: held_goal} if held_goal is not None else {}
    node._paused = False
    node._startup_start_ns = int(1e9)
    node.startup_delay_sec = 0.0
    node.verbose_logs = False
    node.unknown_value = -1
    node.free_value = 0
    node.occ_thresh = 50
    node.min_assign_distance = 0.0
    node.cfpa2_max_goal_distance_m = 0.0
    node.cfpa2_min_utility = 0.0
    node.cfpa2_w_ig = 1.0
    node.cfpa2_w_c = 0.3
    node.cfpa2_w_sw = 0.2
    node.cfpa2_w_momentum = 2.0
    node.cfpa2_momentum_alpha = 1.5
    node.cfpa2_momentum_beta = 2.0
    node.cfpa2_goal_obstacle_clearance_m = 0.0
    node.cfpa2_frontier_obstacle_clearance_m = 0.0
    node.goal_satisfied_dist = 0.65
    node.goal_satisfied_direct_dist = 0.30
    node.goal_satisfied_requires_los = True
    node.blacklist_key_resolution = 0.5
    node.blacklist_cluster_radius_m = 1.0
    node.blacklist_ttl_sec = 30.0
    node.local_nav_stall_blacklist_sec = 45.0
    node.goal_blacklist_until_ns = {ns: {}}
    node.goal_blacklist_disks = {ns: []}
    node.goal_fail_counts = {ns: defaultdict(int)}
    node.goal_progress_samples = {ns: []}
    node.last_policy_reason = {ns: ""}
    node.ramp_ascent_enabled = False
    node.ramp_ascent_exclusive = False
    node._ramp_goal_by_ns = {}
    node._ramp_goal_rx_ns = {}
    node._active_goal_is_ramp = {ns: False}

    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()
    node._publish_coordinator_map = lambda _map: None
    node._publish_robot_markers = lambda _map: None
    node._prune_blacklist = lambda _ns, _now_ns: None
    node._update_reached_goal_blacklist = lambda _ns, _now_ns: None
    node._maybe_force_cfpa2_stuck_recovery = lambda **_kwargs: None
    frontier_goal = held_goal if frontier_goal is None else frontier_goal
    frontier_goals = [] if frontier_goal is None else [frontier_goal]
    node._extract_frontiers_with_scores = (
        lambda _ns, _map, _now_ns: (frontier_goals, goal_scores or {})
    )
    node._apply_goal_policy = lambda **kwargs: kwargs["candidate_goal"]
    node._set_active_goal = lambda _ns, goal, _now_ns: goal

    node.statuses = []
    node.published_goals = []
    node._publish_status = lambda status: node.statuses.append(status)
    node._publish_goal = lambda _ns, _map, goal: node.published_goals.append(goal)
    return node


def test_single_robot_does_not_republish_unreachable_held_goal_when_no_candidate():
    grid = _split_map()
    held_goal = (4.5, 1.5)
    node = _single_robot_node_for_tick(grid, held_goal)

    node._tick_impl()

    assert node.published_goals == [(0.5, 1.5)]
    assert node.last_policy_reason["robot"] == "hold/held_goal_unreachable_stop"
    assert node.goal_blacklist_until_ns["robot"][node._goal_key(held_goal)] > int(10e9)


def test_single_robot_reachability_can_follow_nav2_high_cost_corridor():
    grid = _high_cost_corridor_map()
    frontier_goal = (4.5, 1.5)
    node = _single_robot_node_for_tick(
        grid,
        frontier_goal=frontier_goal,
        goal_scores={frontier_goal: 10.0},
    )
    node.cfpa2_reachability_occ_threshold = 100

    node._tick_impl()

    assert node.published_goals == [frontier_goal]


def test_single_robot_reachability_can_match_nav2_allow_unknown_policy():
    grid = _unknown_corridor_map()
    frontier_goal = (4.5, 1.5)
    node = _single_robot_node_for_tick(
        grid,
        frontier_goal=frontier_goal,
        goal_scores={frontier_goal: 10.0},
    )
    node.cfpa2_reachability_occ_threshold = 100
    node.cfpa2_reachability_allow_unknown = True

    node._tick_impl()

    assert node.published_goals == [frontier_goal]
