# Runtime Contracts (go2_gazebo_sim)

## Ownership Table

- `go2_gazebo_sim` owns runtime domains:
  - `scripts/assets/*`: spawn, pose-guard, stand-up, drift checks.
  - `scripts/perception/*`: simulator transport and SIMDATA->PointCloud2 normalization helpers.
  - `scripts/slam/*`: odometry relay/normalization.
  - `scripts/control/*`: local navigation and safety control.
  - `scripts/observability/*`: status + RViz observability tools.
- `go2_nav_algorithms` owns high-level nav algorithms:
  - PointCloud2->LaserScan projection, mapper, frontier, goal assignment.
- `mtare_ros2` owns M-TARE global coordinator behavior.

## Topic Ownership Rules

- Goals into local controller:
  - autonomy profile: `/<ns>/way_point`
  - coordinated/M-TARE profiles: `/<ns>/way_point_coord`
- Frontier candidate output: `/<ns>/way_point_raw` (when frontier is enabled).
- Planner scan output: `/<ns>/scan_3d` (published by planner-owned pointcloud projection).
- Local map output: `/<ns>/map` (published by `go2_nav_algorithms/simple_scan_mapper_cpp`).
- Navigation odometry input: `/<ns>/odom/nav` (published by `go2_gazebo_sim/scripts/slam/slam_odom_relay.py`).
- Velocity source of truth: `/<ns>/cmd_vel_stamped` from `go2_gazebo_sim/scripts/control/default_nav.py`.
- Real-robot velocity arbitration output: root `/cmd_vel` (published by `go2_gazebo_sim/scripts/control/cmd_vel_activity_mux.py`).
- M-TARE marker source: `/<ns>/mtare_goal_marker` from `mtare_ros2/mtare_coordinator.py`.

## `tare_ros2_exact` Backend Notes

- `planner_backend:=tare_ros2_exact` is opt-in and non-default.
- Coordinator publishes split outputs:
  - `/<ns>/way_point_tare`
  - `/<ns>/goal_point`
- FAR planner publishes `/<ns>/way_point_far`.
- `mtare_behavior_executive_cpp` is the sole planner-side publisher of `/<ns>/way_point_coord`.
- Shared graph bus topics:
  - `/mtare/robot_vgraph`
  - `/mtare/decoded_vgraph`
- Launch preflight checks `graph_decoder` + `visibility_graph_msg` unless `require_shared_graph:=false`.

## Compatibility Window

- Legacy top-level script paths in `go2_gazebo_sim/scripts/*.py` are wrappers for one release cycle.
- Planned removal target: next minor release after this refactor cut.
- Canonical Go2 launch is `go2_gazebo_sim/launch/dual_go2_modular.launch.py`.
- Canonical Go2W launch is `go2_gazebo_sim/launch/dual_go2w_modular.launch.py`.
- Canonical single-Go2W Gazebo launch is `go2_gazebo_sim/launch/single_go2w_gazebo_cfpa2.launch.py`.
- Canonical single-Go2W real-robot launch is `go2_real_bringup/launch/single_go2w_real_cfpa2.launch.py`.
- Legacy launch names remain as wrappers:
  - `two_go2_t_world_autonomy.launch.py`
  - `two_go2_t_world_coordinated_autonomy.launch.py`
  - `two_go2_t_world_mtare_ros2.launch.py`
  - `test_pointlio.launch.py`
