# CLAUDE.md

## Project Overview

Multi-robot autonomous exploration with Unitree Go2W wheeled-legged quadrupeds. ROS 2 Humble, Gazebo Classic simulation, real-robot deployment. CFPA2 coordinated frontier exploration with VLM-in-the-loop capabilities.

## Build & Run

```bash
# Environment setup
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Full build
touch src/mtare_ros1_ws/COLCON_IGNORE
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental build (fast iteration)
colcon build --symlink-install --packages-select <package_name>

# Source workspace
source install/setup.bash
```

After build, YAML and Python changes take effect immediately (symlink-install). C++ changes require rebuild.

## Running

```bash
# Single robot (Gazebo)
./demo_v1.sh

# Dual robot CFPA2 (Gazebo)
./scripts/run_cfpa2_go2w_gazebo.sh

# Kill stale processes before re-launch
killall -9 gzserver gzclient rviz2

# Headless for experiments (faster)
# Pass gui:=false rviz:=false to launch files
```

## Repository Layout

```
src/
  go2w/              # Go2W platform packages (control, nav, perception, safety, sim, real bringup)
  exploration/       # Exploration algorithms (CFPA2, TARE, M-TARE, frontier detection)
  vendor/            # Third-party: fast_lio, autonomy_stack_go2, BALM, etc.
  vlm_explorer/      # VLM-in-the-loop exploration
  _archive/          # Deprecated packages
  mtare_ros1_ws/     # ROS 1 workspace (COLCON_IGNORE'd)
scripts/             # Launch & utility scripts
tools/               # Offline SLAM analysis, replay, visualization
hardware/            # Real-robot connectivity, calibration, docker
config/              # Root-level configs (FastDDS, etc.)
baselines/           # Baseline benchmark scripts (M-TARE, GBPlanner2)
docs/                # Architecture, configuration, walkthroughs
```

## Key Packages

| Package | Type | Purpose |
|---|---|---|
| `go2_gazebo_sim` | C++/Launch | Gazebo world + robot spawning |
| `go2w_control` | Python | Default nav + hybrid wheel/leg cmd router |
| `go2_nav_algorithms` | C++/Python | Mapper, frontier explorer, goal assigner |
| `cfpa2_collaborative_autonomy` | Python | Multi-robot CFPA2 coordinator |
| `go2w_perception` | C++ | QoS bridge, odom relay, pointcloud adapter |
| `vlm_explorer` | Python | Vision language model integration |
| `fast_lio` | C++ | Fast-LIO2 SLAM (vendor submodule) |

## Submodules

Managed in `src/vendor/`. Key ones: `fast_lio`, `go2_ros2_sdk`, `BALM`, `sc_pgo`.
Some large vendor packages have been inlined (no longer submodules): `autonomy_stack_go2`, `unitree-go2-ros2`, `unitree_go2w_ros2`.

```bash
git submodule sync --recursive && git submodule update --init --recursive
```

## Communication Style

- Be fast and direct. Short questions expect immediate, precise answers.
- Show work but don't narrate it. Run commands, make changes, report what happened. Don't ask for permission unless the action is destructive.
- When told "still not working", don't repeat the same fix — go deeper.
- Respect the maintainer's hypotheses. When they say "I AM certain it's due to X", treat that as strong signal. Validate or disprove with evidence, not conjecture.

## Codebase Rules

1. **Always `use_sim_time: true`** for all nodes in Gazebo. Mixed time domains corrupt maps.
2. **Never use stale TF fallback.** Drop the scan on TF failure; don't use `tf2::TimePointZero`.
3. **Each scan painted exactly once.** Clear `last_scan_` after processing in mappers.
4. **Dual-robot TF must be namespaced.** Remap `/tf` -> `/{ns}/tf` for all nodes.
5. **DDS config matters.** `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS with unicast.
6. **Verify with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.

## Architecture

```
Gazebo LiDAR → registered_scan → [qos_bridge] → registered_scan_reliable
                                                        │
                                  ┌─────────────────────┤
                                  ▼                     ▼
                          pointcloud_adapter     pointcloud_to_laserscan
                          (for Fast-LIO)         → scan_3d (LaserScan)
                                                        │
                                  ┌─────────────────────┤
                                  ▼                     ▼
                        simple_scan_mapper_cpp    default_nav
                        → /{ns}/map (OccGrid)     ← /{ns}/map (A* global)
                                                  ← scan_3d (local avoid)
                                                  → cmd_vel_stamped

Gazebo p3d → odom/ground_truth → slam_odom_relay → odom/nav
                                                  → TF: world → base_link
```

### Key Nodes

- **`simple_scan_mapper_cpp`**: Builds 2D occupancy grid from laser scan + TF. Owns its own TF broadcaster (world -> base_link from odom). Each scan painted ONCE.
- **`default_nav`**: Layered navigation — A* global planner on map + scan-based local obstacle avoidance. Background thread for A*.
- **`reactive_nav`**: RRT*-based reactive navigation — sampling-based local planner with fast replanning. Use `nav_backend:=rrt_star`.
- **`pointcloud_adapter`**: Adapts Gazebo point clouds for Fast-LIO (adds per-point timestamps, ring field). Only used when `use_fast_lio:=true`.
- **`slam_odom_relay`**: Passes through or aligns odometry. Does NOT broadcast TF.

## Debugging

### Approach

1. **Check prior art first.** Search git history — many issues have been encountered before (Isaac vs Gazebo stacks share patterns).
2. **Trace the full data pipeline:** What generates the data? What timestamp domain? What transforms it? What consumes it? What does the consumer do each tick?
3. **Add diagnostic logging with actual values** (yaw, dt, timestamps). Use `RCLCPP_INFO_THROTTLE` or Python logger with rate limiting.
4. **Measure, don't assume.** Use `ros2 topic hz`, `ros2 topic echo`, and grep logs.
5. **Benchmark before optimizing.** Time the actual code path first.

### Common Root Causes

| Symptom | Likely Cause | Where to Look |
|---|---|---|
| Map flickering / doubled walls | Same scan painted multiple times with different poses | `simple_scan_mapper_cpp.cpp` update timer vs scan arrival rate |
| Map starburst / rotated structures | TF timestamp mismatch or stale TF fallback | TF lookup code, `ExtrapolationException` handlers |
| "No Effective Points!" in Fast-LIO | Wrong timestamp span in pointcloud_adapter | `pointcloud_adapter.py` time_offset calculation |
| Robot walks through walls | Planner doesn't use occupancy grid | `default_nav_core/planner.py` — plans on scan grid, not map |
| Scattered occupied cells in map | Ground/leg hits passing height filter | `pointcloud_to_laserscan` `min_height` parameter |

## Testing

```bash
# pytest is available but test suite is minimal
pytest test/

# Runtime verification
ros2 topic hz /robot1/map
ros2 topic echo /robot1/odom/nav --once
```

## Code Style

- Python: `black` formatter available (`black .`)
- C++: follows ROS 2 / ament conventions
- No enforced pre-commit hooks currently

## Important Config Files

| Config | Purpose |
|---|---|
| `src/go2w_config/` | Shared nav, safety, observability configs |
| `config/nav/simple_scan_mapper_single_go2w.yaml` | Grid scoring (hit_increment, thresholds) |
| `config/nav/geometric_frontier_single.yaml` | Frontier detection params |
| `config/nav/default_nav_single_go2w.yaml` | Local nav speeds, tolerances |
| `config/control/go2w_hybrid_motion.yaml` | Wheel/leg blending |
| `urdf/go2w/go2w_description_3d_lidar.xacro` | Sensor configs (LiDAR, p3d rates) |
| `src/go2w_control/config/` | Default nav params (sim + real), hybrid motion |

## Environment

- **Python**: 3.10 (via micromamba `cmu_env`)
- **ROS 2**: Humble (`/opt/ros/humble/`)
- **Build**: colcon + ament_cmake (C++) / ament_python (Python)
- **DDS**: FastDDS (sim), CycloneDDS (real robot)
