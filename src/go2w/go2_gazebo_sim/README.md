# go2_gazebo_sim Runbook

This package now uses canonical modular launches:

- `go2_gazebo_sim/launch/dual_go2_modular.launch.py`
- `go2_gazebo_sim/launch/dual_go2w_modular.launch.py`

Legacy launch files are still available as wrappers for compatibility.

## 1. Build and source

```bash
cd /path/to/COMP0225_LRC_stack
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_nav_algorithms go2_gazebo_sim --symlink-install
source install/setup.bash
```

## 2. Canonical run commands

Go2 shell entrypoint:

```bash
./run_cfpa2_gazebo.sh
```

Go2W shell entrypoint:

```bash
./run_cfpa2_go2w_gazebo.sh
```

Go2 autonomy profile:

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=autonomy \
  planner_backend:=none
```

Go2 coordinated profile (shared assigner):

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=coordinated \
  planner_backend:=coordinated
```

Go2 M-TARE ROS2 profile:

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=mtare_ros2 \
  planner_backend:=mtare_ros2 \
  enable_frontier_aux:=false
```

M-TARE shared RViz now visualizes:
- coordinator map: `/mtare/coordinator_map`
- coordinator robot overlays: `/mtare/robot_markers`
- per-robot goals: `/robot_a/mtare_goal_marker`, `/robot_b/mtare_goal_marker`
- per-robot planned paths: `/robot_a/planned_path`, `/robot_b/planned_path`

Go2 PointLIO debug profile:

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=pointlio_debug \
  pointlio_autonomous:=true
```

Single-robot CMU-style stack (SLAM + frontier + local planner + sport API publisher):

```bash
ros2 launch go2_gazebo_sim go2_l_corridor_cmu_frontier_local.launch.py
```

Single-Go2W real-robot CFPA2 stack (CMU SLAM backend + single-robot CFPA2 + default_nav + joystick fallback):

```bash
ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py
```

Single-Go2W Gazebo CFPA2 stack (single robot, Go2W assets, simulated odom + point cloud):

```bash
ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py
```

Shell entrypoint for the same stack:

```bash
./src/autonomy_stack_go2/system_real_robot_go2w_cfpa2.sh
```

Go2W M-TARE ROS2 profile:

```bash
ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py \
  profile:=mtare_ros2 \
  planner_backend:=mtare_ros2 \
  enable_frontier_aux:=false
```

## 3. Most useful arguments to change

### Common runtime

- `gui:=true|false` (Gazebo client)
- `rviz:=true|false`
- `cleanup_stale:=true|false`
- `use_sim_time:=true|false`
- `world:=/abs/path/to/world.world`

### Robot spawn

- `robot_a_spawn_x`, `robot_a_spawn_y`, `robot_a_spawn_yaw`
- `robot_b_spawn_x`, `robot_b_spawn_y`, `robot_b_spawn_yaw`

### Domain toggles (modular pipeline)

- `enable_assets:=true|false`
- `enable_perception:=true|false`
- `enable_slam:=true|false`
- `enable_control:=true|false`
- `enable_navigation:=true|false`

### Planner selection

- `planner_backend:=auto|none|coordinated|mtare_ros2|ros1_mtare|far_ros2`
- `coordinated_algorithm_mode:=legacy|committed`

Notes:

- `profile:=autonomy` typically uses `planner_backend:=none`
- `profile:=coordinated` uses `planner_backend:=coordinated`
- `profile:=mtare_ros2` uses `planner_backend:=mtare_ros2`

### Shared map / backend map

- `use_shared_map:=true|false`
- `shared_map_topic:=/disco_slam/global_map`
- `shared_map_wait_sec:=8.0`

### M-TARE tuning

- `mtare_algorithm_mode:=mtare`
- `mtare_goal_publish_rate:=2.0`
- `mtare_overlap_weight:=1.0`
- `mtare_communication_timeout_sec:=6.0`
- `mtare_prediction_horizon_sec:=4.0`
- `mtare_pursuit_weight:=2.0`
- `mtare_pursuit_switch_margin:=0.10`
- `mtare_exploration_gain_radius_cells:=4`
- `mtare_meeting_min_distance:=1.5`
- `mtare_teammate_stale_ttl_sec:=120.0`
- `enable_frontier_aux:=false` (keep false for pure M-TARE behavior)

### FAST-LIO

- `use_fast_lio:=true|false`

## 4. Legacy wrapper commands (still supported)

These still work, but are deprecated:

```bash
ros2 launch go2_gazebo_sim two_go2_t_world_autonomy.launch.py
ros2 launch go2_gazebo_sim two_go2_t_world_coordinated_autonomy.launch.py
ros2 launch go2_gazebo_sim two_go2_t_world_mtare_ros2.launch.py
ros2 launch go2_gazebo_sim test_pointlio.launch.py
```

## 5. Inspect launch args quickly

```bash
ros2 launch go2_gazebo_sim dual_go2_modular.launch.py --show-args
ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py --show-args
```

## 6. Troubleshooting

If launch fails with Python module errors in ROS nodes (for example `numpy` or `yaml` missing), install required packages in the active environment and rebuild/source again.

Typical startup confirmation line:

- `[dual_go2_modular] profile=<...> planner_backend=<...> assets=<...> perception=<...> slam=<...> control=<...> navigation=<...>`
- `[dual_go2w_modular] profile=<...> planner_backend=<...> assets=<...> perception=<...> slam=<...> control=<...> navigation=<...>`
