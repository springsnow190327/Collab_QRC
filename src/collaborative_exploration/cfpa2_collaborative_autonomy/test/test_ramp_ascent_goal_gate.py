from nav_msgs.msg import OccupancyGrid, Odometry

from cfpa2_collaborative_autonomy.cfpa2_single_robot_node import CFPA2SingleRobotNode


def _free_map(width=80, height=20, resolution=0.1):
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = -1.0
    msg.data = [0] * (width * height)
    return msg


def _node():
    node = CFPA2SingleRobotNode.__new__(CFPA2SingleRobotNode)
    node.ramp_ascent_enabled = True
    node.ramp_ascent_goal_stale_sec = 2.0
    node.ramp_ascent_max_goal_distance_m = 5.0
    node.ramp_ascent_require_grid_reachable = True
    node.ramp_ascent_exclusive = False
    node.ramp_ascent_ignore_blacklist = False
    node.ramp_ascent_corridor_lock_sec = 0.0
    node.ramp_ascent_lock_min_x = -1.0e9
    node.ramp_ascent_lock_max_x = 1.0e9
    node.ramp_ascent_lock_max_abs_y = 1.0e9
    node.cfpa2_frontier_obstacle_clearance_m = 0.0
    node.min_assign_distance = 0.3
    node.switch_min_dist = 0.65
    node.occ_thresh = 50
    node.unknown_value = -1
    node.blacklist_key_resolution = 0.5
    node.goal_blacklist_until_ns = {"robot": {}}
    node.goal_blacklist_disks = {"robot": []}
    node.blacklist_cluster_radius_m = 1.0
    node._ramp_goal_by_ns = {"robot": (4.0, 0.0)}
    node._ramp_goal_rx_ns = {"robot": int(1e9)}
    odom = Odometry()
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    node.odoms = {"robot": odom}
    return node


def test_ramp_ascent_goal_uses_separate_distance_gate_when_reachable():
    node = _node()
    grid = _free_map()
    dist_map = {i: 1 for i in range(len(grid.data))}

    goal = node._ramp_ascent_goal_if_valid(
        ns="robot",
        map_msg=grid,
        dist_map=dist_map,
        now_ns=int(2e9),
    )

    assert goal == (4.0, 0.0)


def test_ramp_ascent_goal_expires_when_stale():
    node = _node()
    grid = _free_map()
    dist_map = {i: 1 for i in range(len(grid.data))}

    goal = node._ramp_ascent_goal_if_valid(
        ns="robot",
        map_msg=grid,
        dist_map=dist_map,
        now_ns=int(5e9) + 1,
    )

    assert goal is None


def test_ramp_ascent_goal_can_bypass_lagging_grid_reachability():
    node = _node()
    node.ramp_ascent_require_grid_reachable = False
    grid = _free_map()

    goal = node._ramp_ascent_goal_if_valid(
        ns="robot",
        map_msg=grid,
        dist_map={0: 1},
        now_ns=int(2e9),
    )

    assert goal == (4.0, 0.0)


def test_ramp_ascent_goal_uses_nonlethal_cost_threshold_for_reachability_and_clearance():
    node = _node()
    node.ramp_ascent_reachability_occ_threshold = 100
    node.cfpa2_frontier_obstacle_clearance_m = 0.1
    grid = _free_map()

    width = int(grid.info.width)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    resolution = float(grid.info.resolution)
    y_cell = int((0.0 - origin_y) / resolution)
    for x_m in (0.5, 1.5, 2.5, 3.5, 4.0):
        x_cell = int((x_m - origin_x) / resolution)
        grid.data[y_cell * width + x_cell] = 80

    goal = node._ramp_ascent_goal_if_valid(
        ns="robot",
        map_msg=grid,
        dist_map={},
        now_ns=int(2e9),
    )

    assert goal == (4.0, 0.0)


def test_ramp_ascent_goal_can_ignore_frontier_blacklist_disk():
    node = _node()
    node.ramp_ascent_ignore_blacklist = True
    node.goal_blacklist_until_ns["robot"][node._goal_key((4.0, 0.0))] = int(9e9)
    grid = _free_map()
    dist_map = {i: 1 for i in range(len(grid.data))}

    goal = node._ramp_ascent_goal_if_valid(
        ns="robot",
        map_msg=grid,
        dist_map=dist_map,
        now_ns=int(2e9),
    )

    assert goal == (4.0, 0.0)


def test_ramp_ascent_candidate_preempts_non_ramp_held_goal_once():
    node = _node()
    node._active_goal_is_ramp = {"robot": False}
    node.last_goal = {"robot": (1.0, 0.0)}

    assert node._ramp_goal_forces_switch(
        ns="robot",
        candidate_goal=(4.0, 0.0),
        ramp_goal_key=node._goal_key((4.0, 0.0)),
    )

    node._active_goal_is_ramp["robot"] = True
    node.last_goal["robot"] = (4.0, 0.0)

    assert not node._ramp_goal_forces_switch(
        ns="robot",
        candidate_goal=(4.2, 0.0),
        ramp_goal_key=node._goal_key((4.2, 0.0)),
    )

    assert node._ramp_goal_forces_switch(
        ns="robot",
        candidate_goal=(5.0, 0.0),
        ramp_goal_key=node._goal_key((5.0, 0.0)),
    )


def test_ramp_ascent_candidate_uses_dedicated_small_switch_distance():
    node = _node()
    node.ramp_ascent_switch_min_dist_m = 0.25
    node._active_goal_is_ramp = {"robot": True}
    node.last_goal = {"robot": (5.60, 0.0)}

    assert node._ramp_goal_forces_switch(
        ns="robot",
        candidate_goal=(5.95, 0.0),
        ramp_goal_key=node._goal_key((5.95, 0.0)),
    )


def test_ramp_ascent_goal_freshness_supports_exclusive_acquisition():
    node = _node()

    assert node._ramp_ascent_goal_is_fresh(ns="robot", now_ns=int(2e9))
    assert not node._ramp_ascent_goal_is_fresh(ns="robot", now_ns=int(5e9) + 1)


def test_active_ramp_goal_does_not_use_frontier_reached_blacklist_radius():
    node = _node()
    goal = (5.60, 0.0)
    node._ramp_goal_by_ns = {"robot": goal}
    node._active_goal_is_ramp = {"robot": True}
    node.last_goal = {"robot": goal}
    node.goal_satisfied_dist = 0.65
    node.goal_satisfied_direct_dist = 0.30
    node.goal_satisfied_requires_los = False
    node.reached_blacklist_dist = 0.65
    node.reached_blacklist_repeat_count = 1
    node.reached_blacklist_ttl_sec = 12.0
    node.reached_goal_last_key = {"robot": node._goal_key(goal)}
    node.reached_goal_repeat_count = {"robot": 0}
    node.goal_blacklist_until_ns = {"robot": {}}
    node.goal_blacklist_disks = {"robot": []}
    node.blacklist_cluster_radius_m = 1.0

    class _Logger:
        def info(self, *_args, **_kwargs):
            pass

        def warn(self, *_args, **_kwargs):
            pass

    class _Tracker:
        _tracked = []

        def record_attempt(self, _goal):
            return None

    node.get_logger = lambda: _Logger()
    node._cluster_tracker = _Tracker()
    node.odoms["robot"].pose.pose.position.x = 5.42
    node.odoms["robot"].pose.pose.position.y = -0.48

    node._update_reached_goal_blacklist("robot", now_ns=int(3e9))

    assert node.goal_blacklist_until_ns["robot"] == {}


def test_startup_delay_blocks_goal_publication_until_sim_time_elapsed():
    node = _node()
    node.startup_delay_sec = 24.0
    node._startup_start_ns = int(2e9)

    assert node._startup_delay_active(now_ns=int(25e9))
    assert not node._startup_delay_active(now_ns=int(27e9))


def test_startup_delay_anchors_on_first_sim_time_tick():
    node = _node()
    node.startup_delay_sec = 24.0
    node._startup_start_ns = 0

    assert node._startup_delay_active(now_ns=int(25e9))
    assert node._startup_start_ns == int(25e9)
    assert node._startup_delay_active(now_ns=int(48e9))
    assert not node._startup_delay_active(now_ns=int(49e9))


def test_ramp_corridor_lock_blocks_non_ramp_frontiers_between_sparse_goal_updates():
    node = _node()
    node.ramp_ascent_exclusive = True
    node.ramp_ascent_goal_stale_sec = 8.0
    node.ramp_ascent_corridor_lock_sec = 20.0
    node.ramp_ascent_lock_min_x = 5.3
    node.ramp_ascent_lock_max_x = 9.8
    node.ramp_ascent_lock_max_abs_y = 0.9
    node._active_goal_is_ramp = {"robot": True}
    node._ramp_goal_rx_ns = {"robot": int(10e9)}
    node.odoms["robot"].pose.pose.position.x = 7.3
    node.odoms["robot"].pose.pose.position.y = 0.05

    assert not node._ramp_ascent_goal_is_fresh(ns="robot", now_ns=int(20e9))
    assert node._ramp_ascent_corridor_lock_active(ns="robot", now_ns=int(20e9))


def test_ramp_corridor_lock_releases_outside_ramp_width():
    node = _node()
    node.ramp_ascent_exclusive = True
    node.ramp_ascent_corridor_lock_sec = 20.0
    node.ramp_ascent_lock_min_x = 5.3
    node.ramp_ascent_lock_max_x = 9.8
    node.ramp_ascent_lock_max_abs_y = 0.9
    node._active_goal_is_ramp = {"robot": True}
    node._ramp_goal_rx_ns = {"robot": int(10e9)}
    node.odoms["robot"].pose.pose.position.x = 7.3
    node.odoms["robot"].pose.pose.position.y = 1.2

    assert not node._ramp_ascent_corridor_lock_active(ns="robot", now_ns=int(20e9))


def test_ramp_corridor_lock_is_pose_based_even_without_recent_callback():
    node = _node()
    node.ramp_ascent_exclusive = True
    node.ramp_ascent_corridor_lock_sec = 20.0
    node.ramp_ascent_lock_min_x = 5.3
    node.ramp_ascent_lock_max_x = 9.8
    node.ramp_ascent_lock_max_abs_y = 0.9
    node._ramp_goal_rx_ns = {}
    node.odoms["robot"].pose.pose.position.x = 6.4
    node.odoms["robot"].pose.pose.position.y = 0.04

    assert node._ramp_ascent_corridor_lock_active(ns="robot", now_ns=int(20e9))
