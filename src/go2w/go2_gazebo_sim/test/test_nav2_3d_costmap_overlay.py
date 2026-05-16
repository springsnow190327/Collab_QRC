from pathlib import Path

import yaml


GO2W_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = GO2W_ROOT.parents[1]


def test_3d_costmap_overlay_uses_traversability_for_global_and_local_costmaps():
    overlay_path = (
        GO2W_ROOT
        / "go2w_config"
        / "config"
        / "nav"
        / "nav2_3d_costmap_overlay.yaml"
    )
    cfg = yaml.safe_load(overlay_path.read_text())

    global_params = cfg["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = cfg["local_costmap"]["local_costmap"]["ros__parameters"]
    controller_params = cfg["controller_server"]["ros__parameters"]

    for params in (global_params, local_params):
        assert params["plugins"] == ["static_layer", "inflation_layer"]
        assert params["static_layer"]["map_topic"] == "/robot/traversability_grid"
    assert controller_params["goal_checker"]["xy_goal_tolerance"] == 0.20
    assert controller_params["goal_checker"]["stateful"] is False


def test_fastlio_launch_aligns_cfpa2_planning_map_with_nav_costmap_mode():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    text = launch_path.read_text()

    assert '"planning_map_topic_suffix": (' in text
    assert 'if nav_costmap_mode == "3d" else "/map"' in text


def test_3d_explore_uses_sensor_verified_ramp_viewpoint_without_cmd_assist():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()
    ramp_block = text.split('name="ramp_ascent_goal"')[1].split(
        "remappings", 1
    )[0]

    assert 'executable="ramp_ascent_goal_node"' in text
    assert 'name="ramp_cmd_vel_assist"' not in text
    assert '"use_pointcloud_ramp_detection": False' in ramp_block
    assert '"traversability_layer": "trav_fused"' in ramp_block
    assert '"wall_cost_layer": "wall_cost"' in ramp_block
    assert '"step_height_layer": "step_height"' in ramp_block
    assert '"max_wall_cost": 0.30' in ramp_block
    assert '"max_step_height_m": 0.25' in ramp_block
    assert '"goal_lookahead_m": 1.0' in ramp_block
    assert '"min_slope_deg": 8.0' in ramp_block
    assert '"min_x"' not in ramp_block
    assert '"max_x"' not in ramp_block
    assert '"goal_center_y"' not in ramp_block


def test_3d_explore_enables_stability_costs_without_scene_coordinate_mask():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()
    occ_block = text.split('name="grid_map_to_occupancy_grid"')[1].split(
        "remappings", 1
    )[0]

    assert '"traversability_layer": "trav_fused"' in occ_block
    assert '"free_threshold":  0.60' in occ_block
    assert '"lethal_threshold": 0.30' in occ_block
    assert '"cliff_proximity_cost_enabled": True' in occ_block
    assert '"cliff_step_layer": "step_height"' in occ_block
    assert '"cliff_proximity_radius_m": 0.25' in occ_block
    assert '"cliff_step_threshold_m": 0.30' in occ_block
    assert '"cliff_step_saturation_m": 0.45' in occ_block
    assert '"ramp_min_slope_rad": 0.20943951023931956' in occ_block
    assert '"workspace_mask_enabled": False' in occ_block


def test_3d_explore_publishes_fixed_world_traversability_grid():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    text = launch_path.read_text()
    occ_block = text.split('name="grid_map_to_occupancy_grid"')[1].split(
        "# 4. ramp_ascent_goal_node", 1
    )[0]

    assert '"fixed_grid_enabled": True' in occ_block
    assert '"fixed_origin_x": -7.0' in occ_block
    assert '"fixed_origin_y": -15.0' in occ_block
    assert '"fixed_width_cells": 300' in occ_block
    assert '"fixed_height_cells": 300' in occ_block
    assert '"unknown_clears_history": False' in occ_block
    assert '"occupied_cost_threshold": 100' in occ_block
    assert '"occupied_confirm_hits": 2' in occ_block
    assert '"workspace_mask_enabled": False' in occ_block


def test_trav_filter_chain_uses_trapezoidal_ramp_safe_without_gain_shortcut():
    cfg_path = (
        REPO_ROOT
        / "src"
        / "collaborative_exploration"
        / "trav_cost_filters"
        / "config"
        / "grid_map_filters.yaml"
    )
    text = cfg_path.read_text()
    cfg = yaml.safe_load(text)
    filters = cfg["/**"]["ros__parameters"]["filters"]

    assert list(filters) == [f"filter{i}" for i in range(1, 31)]
    assert filters["filter17"]["name"] == "slope_floor_margin"
    assert "0.13962634015954636" in filters["filter17"]["params"]["expression"]
    assert filters["filter20"]["name"] == "slope_ceiling_margin"
    assert filters["filter26"]["params"]["expression"] == (
        "slope_floor_margin .* slope_ceiling_margin .* step_margin"
    )
    assert ".* 100.0" not in text


def test_fastlio_launch_passes_ramp_suppression_to_stuck_watchdog_not_path_relay():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    text = launch_path.read_text()
    path_relay_block = text.split('name=f"path_relay_{robot_ns}"')[0].rsplit(
        "path_relay_node = ExecuteProcess", 1
    )[1]
    watchdog_block = text.split('name=f"stuck_watchdog_{robot_ns}"')[0].rsplit(
        "stuck_watchdog_node = ExecuteProcess", 1
    )[1]

    assert "ramp_suppress_enabled" not in path_relay_block
    assert "ramp_suppress_enabled" in watchdog_block
    assert "STUCK_WATCHDOG_WINDOW_SEC" not in path_relay_block
    assert "STUCK_WATCHDOG_WINDOW_SEC" in watchdog_block
    assert "stuck_window_sec" in watchdog_block


def test_3d_explore_uses_long_stuck_window_for_ramp_demo():
    script_path = REPO_ROOT / "scripts" / "launch" / "nav_test_3d_explore.sh"
    text = script_path.read_text()

    assert 'STUCK_WATCHDOG_WINDOW_SEC="${STUCK_WATCHDOG_WINDOW_SEC:-9999.0}"' in text


def test_3d_explore_enables_ramp_ascent_bridge_without_coordinate_gate():
    script_path = REPO_ROOT / "scripts" / "launch" / "nav_test_3d_explore.sh"
    text = script_path.read_text()
    overlay_path = (
        REPO_ROOT
        / "src"
        / "collaborative_exploration"
        / "cfpa2_collaborative_autonomy"
        / "config"
        / "cfpa2_single_robot_demo_ramp.yaml"
    )
    overlay = yaml.safe_load(overlay_path.read_text())["/**"]["ros__parameters"]

    assert "yaml.safe_dump" not in text
    assert "CFPA2_SRC_CONFIG" not in text
    assert "ramp_ascent_lock_min_x" not in text
    assert "ramp_ascent_lock_max_x" not in text
    assert "ramp_ascent_lock_max_abs_y" not in text
    assert overlay["planning_map_topic_suffix"] == "/traversability_grid"
    assert overlay["ig_dimension"] == "2d"
    assert overlay["cfpa2_reachability_occ_threshold"] == 100
    assert overlay["cfpa2_reachability_allow_unknown"] is True
    assert overlay["ramp_ascent_enabled"] is True
    assert overlay["ramp_ascent_require_grid_reachable"] is True
    assert overlay["ramp_ascent_reachability_occ_threshold"] == 100
    assert overlay["ramp_ascent_exclusive"] is True


def test_3d_explore_enables_sensor_verified_ramp_wheel_override_without_coordinate_gate():
    wrapper_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_3d_explore.launch.py"
    )
    fastlio_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    single_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "single_go2w_mujoco_cfpa2.launch.py"
    )
    wrapper_text = wrapper_path.read_text()
    fastlio_text = fastlio_path.read_text()
    single_text = single_path.read_text()
    wrapper_args = wrapper_text.split("launch_arguments={", 1)[1].split(
        "}.items()", 1
    )[0]

    assert '"ramp_force_legged_enabled": "false"' in wrapper_args
    assert '"ramp_force_wheel_enabled": "true"' in wrapper_args
    assert '"ramp_goal_mode_topic": "ramp_ascent_goal_mode"' in wrapper_args
    assert '"ramp_goal_stale_sec": "3.0"' in wrapper_args
    assert '"ramp_force_max_vx_mps": "0.17"' in wrapper_args
    assert '"ramp_force_min_goal_x"' not in wrapper_args
    assert '"ramp_force_max_goal_x"' not in wrapper_args
    assert '"ramp_force_max_abs_goal_y"' not in wrapper_args

    assert '"ramp_force_legged_enabled": LaunchConfiguration("ramp_force_legged_enabled")' in fastlio_text
    assert '"ramp_force_wheel_enabled": LaunchConfiguration("ramp_force_wheel_enabled")' in fastlio_text
    assert '"ramp_goal_mode_topic": LaunchConfiguration("ramp_goal_mode_topic")' in fastlio_text
    assert 'DeclareLaunchArgument("ramp_force_legged_enabled", default_value="false")' in fastlio_text

    assert (
        'ramp_force_legged_enabled = _as_bool(_get(context, '
        '"ramp_force_legged_enabled"))'
    ) in single_text
    assert (
        '"ramp_force_legged_enabled": ramp_force_legged_enabled'
    ) in single_text
    assert (
        '"ramp_force_wheel_enabled": ramp_force_wheel_enabled'
    ) in single_text
    assert '"ramp_goal_mode_topic": ramp_goal_mode_topic' in single_text
    assert 'DeclareLaunchArgument("ramp_force_legged_enabled", default_value="false")' in single_text

    ramp_node_block = wrapper_text.split('name="ramp_ascent_goal"')[1].split(
        "remappings", 1
    )[0]
    assert '"mode_topic": "ramp_ascent_goal_mode"' in ramp_node_block
    assert '"verified_hold_sec": 4.0' in ramp_node_block
    assert '"min_candidate_cells": 30' in ramp_node_block
    assert '"traversability_layer": "trav_fused"' in ramp_node_block
    assert '"max_wall_cost": 0.30' in ramp_node_block
    assert '"max_step_height_m": 0.25' in ramp_node_block
    assert '"min_goal_forward_m"' not in ramp_node_block


def test_base_configs_do_not_hide_demo_ramp_coordinate_gates():
    cfg_path = (
        GO2W_ROOT
        / "go2w_config"
        / "config"
        / "control"
        / "go2w_hybrid_motion.yaml"
    )
    hybrid = yaml.safe_load(cfg_path.read_text())["/**"]["ros__parameters"]
    cfpa2_path = (
        REPO_ROOT
        / "src"
        / "collaborative_exploration"
        / "cfpa2_collaborative_autonomy"
        / "config"
        / "cfpa2_single_robot.yaml"
    )
    cfpa2 = yaml.safe_load(cfpa2_path.read_text())["/**"]["ros__parameters"]
    script_text = (REPO_ROOT / "scripts" / "launch" / "nav_test_3d_explore.sh").read_text()

    assert hybrid["ramp_force_wheel_enabled"] is False
    assert hybrid["ramp_goal_stale_sec"] < 10.0
    assert hybrid["ramp_force_min_goal_x"] < -1.0e8
    assert hybrid["ramp_force_max_goal_x"] > 1.0e8
    assert cfpa2["ramp_ascent_enabled"] is False
    assert cfpa2["ramp_ascent_reachability_occ_threshold"] == 100
    assert cfpa2["ramp_ascent_corridor_lock_sec"] == 0.0
    assert "STUCK_RAMP_MIN_X" not in script_text
    assert "STUCK_RAMP_MAX_X" not in script_text


def test_fastlio_rviz_process_sanitizes_snap_environment():
    launch_path = (
        GO2W_ROOT
        / "go2_gazebo_sim"
        / "launch"
        / "nav_test_mujoco_fastlio.launch.py"
    )
    text = launch_path.read_text()
    rviz_block = text.split('name="rviz2_nav_test"')[1].split("output=", 1)[0]

    assert "additional_env=_rviz_clean_env()" in rviz_block
    assert '"SNAP": ""' in text
    assert '"GTK_PATH": ""' in text
    assert '"XDG_DATA_HOME": ""' in text
