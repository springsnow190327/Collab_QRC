# jetson_ws — Go2 Orin NX onboard autonomy workspace

This is the **deployment snapshot** of the ROS 1 Noetic catkin workspace that
runs the full autonomy stack natively onboard the real Go2's Jetson Orin NX
(16 GB, JetPack 5.1.1 / L4T R35.3.1 / CUDA 11.4 / Python 3.8). It mirrors the
on-robot path `/home/unitree/autonomous_exploration_zhu/`.

Everything runs **onboard, native ROS 1, no ros1_bridge in the data path** — the
laptop is only used for setup (NAT internet + SSH) and, during HIL, RViz2 viz.

## Why a separate workspace (vs `src/vendor/*_ros1`)

The desktop ROS 2 Humble sim lives in `../src/`. This Jetson workspace keeps the
**flat catkin `src/` layout** the robot actually uses, so the deployed set is
unambiguous. The same algorithm source compiles for both ROS distros via:
- CFPA2: hexagonal `core/` + `#ifdef CFPA2_ROS1` adapter selection (`ros1/` vs `ros2/`).
- Nav2 SmacLattice + MPPI: `nav_algo_core/compat.hpp` shim (rclcpp→roscpp).

## Packages (catkin)

| Package | Role | Notes |
|---|---|---|
| `livox_ros_driver2` | Mid-360 driver (CustomMsg) | build first: `./build.sh ROS1` |
| `Point-LIO` | **production SLAM** (iVox) | node `laserMapping`; → `/robot/Odometry` |
| `FAST_LIO` | alt SLAM | `CATKIN_IGNORE`'d (Point-LIO is default) |
| `elevation_mapping_cupy` + `elevation_map_msgs` | 2.5D heightmap + CNN trav (torch) | → `/robot/elevation_map_raw` |
| `trav_pipeline_ros1` | filter chain + OccupancyGrid + goal bridge | → `/robot/traversability_grid` |
| `nav_algo_ros1` | Nav2 SmacLattice (global) + CUDA-MPPI (local) as `move_base` plugins | sm_87 CUDA |
| `cfpa2_collaborative_autonomy` | CFPA2 frontier allocator (C++ Noetic port) | `cfpa2_single_robot_node_cpp` |
| `cfpa2_peer_coordination_msgs` | peer msgs (`builtin_interfaces/Time`→`time` for ROS1) | |
| `cfpa2_peer_coordination` | Python peer pkg | `CATKIN_IGNORE`'d (C++ node doesn't need it) |

## Data flow (verified real-time on NX, 2026-05-20)

```
Mid-360 ─/livox/{lidar,imu}(10/200Hz)→ Point-LIO ─/robot/Odometry(10Hz)─┐
                                          └─/robot/cloud_registered_body(10Hz)→ elevation_mapping_cupy
  static TFs: map→camera_init→body→base_link                              ─/robot/elevation_map_raw→ trav_filter
  /robot/Odometry ─relay→ /robot/odom/nav(10Hz)                            ─/robot/traversability_grid(5Hz)─┐
                                                                                                            ↓
  CFPA2 ─/robot/way_point_coord(2Hz)→ cfpa2_to_movebase_bridge ─/robot/move_base_simple/goal→ move_base
                                                              (SmacLattice + CUDA-MPPI) ─/robot/cmd_vel(20Hz)→ robot
```

Resource use with full stack running (Orin NX): RAM 6.6/15.4 GB, CPU <33 %,
GPU <19 %, 56 °C, 6.6 W. Huge headroom.

## Build (on the NX)

See [`../scripts/real/.orin_nx_cheatsheet.md`](../scripts/real/.orin_nx_cheatsheet.md)
(gitignored — contains the robot password) for the **full one-shot dependency
recipe** (apt deps, xtensor 0.24.7, cupy, torch Jetson wheel, ros_numpy patch,
runtime pip deps). Summary:

```bash
# 1. system deps (apt): grid_map, ompl, move_base/nav_core/costmap, nlohmann-json …
# 2. xtensor 0.24.7 + xtl 0.7.5 vendored into /usr/local (not in focal apt)
# 3. cupy-cuda11x + torch (NVIDIA jp512 wheel) + simple_parsing/ruamel/shapely/sklearn/scipy==1.10
# 4. ros_numpy: patch np.float→float + sort PointFields by offset (Point-LIO cloud is out of order)
export PATH=/usr/local/cuda/bin:$PATH CUDACXX=/usr/local/cuda/bin/nvcc CUDA_HOME=/usr/local/cuda
source /opt/ros/noetic/setup.bash
cd ~/autonomous_exploration_zhu
catkin_make --pkg livox_ros_driver2 cfpa2_peer_coordination_msgs \
  -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc   # messages first
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -j4                                              # full build
```

## Run

```bash
scripts/onboard_autonomy_noetic.sh                 # full stack, explore, ns=robot
scripts/onboard_autonomy_noetic.sh explore=false   # nav only (no CFPA2 auto-goals)
scripts/onboard_autonomy_noetic.sh slam=fastlio    # FAST-LIO instead of Point-LIO
scripts/onboard_autonomy_noetic.sh stop            # tear down
```

> The `scripts/` dir is deployed onto the NX alongside this workspace; the
> source-of-truth for the launcher lives at `../scripts/real/onboard_autonomy_noetic.sh`.

## Key port-specific gotchas (full list in the cheatsheet)

- **nvcc 11.4** mis-parses `rclcpp::Logger logger_{rclcpp::get_logger(...)}`
  (brace-init) → changed to copy-init `= rclcpp::get_logger(...)` in 6 nav_algo headers.
- **sm_89** isn't supported by CUDA 11.4 → `nav_algo_mppi_cuda` gencode is
  CUDA-version-gated (sm_87 always, sm_89 ≥11.8, sm_120 ≥12.8).
- **CFPA2 yamls** are ROS 2 (`/**:ros__parameters:`); flattened for ROS 1
  rosparam via `../scripts/real/generate_cfpa2_ros1_yaml.py`.
- **CFPA2 planning map**: ops2 overlay's `/global_costmap/costmap` is Nav2 naming;
  on ROS 1 move_base nests it under `/move_base/…`, so the onboard launcher
  overrides `planning_map_topic_suffix` → `/traversability_grid` (CFPA2 plans on
  the trav grid directly).
- **odom**: CFPA2 reads `/<ns>/odom/nav` (hardcoded); a `topic_tools relay` maps
  Point-LIO's `/<ns>/Odometry` → `/<ns>/odom/nav`.
- **exec bits**: `trav_pipeline_ros1/scripts/*.py` must be `chmod +x` after deploy.
