# SLAM Backends, LiDAR Hardware & Demo Scenes

Covers Fast-LIO2 integration, Cartographer vs Fast-LIO A/B, LiDAR options, and the demo scene catalog.

## Fast-LIO2 + SC-PGO integration (2026-04-15)

### Why

Cartographer 2D scan matching suffers from scan-odom temporal misalignment — the registered pointcloud and robot odom pose are produced by separate pipelines at different timestamps. When FAR consumes a scan paired with stale odom, terrain voxels are offset from reality → FAR plans paths through walls.

Fast-LIO2 solves it: its tightly-coupled IMU-LiDAR optimization outputs both the registered cloud AND the corrected pose from the same optimization step at the same timestamp. No temporal lag, no ghosting.

### Pipeline

```
MuJoCo physics (500 Hz)
  → mujoco_ros2_control LiDAR plugin → /robot/registered_scan (PointCloud2)
  → Fast-LIO2 (ESKI filter) → /cloud_registered + /Odometry
  → SC-PGO (Scan Context loop closure) → /robot/odom/nav (PoseStamped)
  → octomap_server → /robot/map (OccupancyGrid, TRANSIENT_LOCAL)
  → FAR planner → /robot/way_point
  → localPlanner → pathFollower → /robot/cmd_vel
```

### Key wiring decisions

- `octomap_server` publishes with `latch=True` (TRANSIENT_LOCAL durability) so CFPA2's late-connecting TRANSIENT_LOCAL subscriber gets the initial map. Without this, QoS mismatch silently drops all map messages.
- Two static TFs (`world→map` identity, `map→odom` identity) + `odom_bridge_publish_tf=true` complete the TF tree that `octomap_server` and FAR require.
- SC-PGO sourced from LRC stack: `export AMENT_PREFIX_PATH="/home/hz/COMP0225_LRC_stack/install/sc_pgo:${AMENT_PREFIX_PATH}"` — **only sc_pgo prefix** (whole LRC install would shadow our go2_gazebo_sim).
- `octomap_server` launched OUTSIDE the `nav_backend` conditional so it runs regardless of FAR vs RRT* selection.

### Launch

`src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py` (873 lines). FAR tuning params in `src/go2w/go2w_config/config/nav/far_planner_tuning.yaml` — no rebuild needed for param changes.

## FAR planner tuning YAML

`src/go2w/go2w_config/config/nav/far_planner_tuning.yaml` — extracted from launch for rapid iteration.

| Param | Value | Notes |
|---|---|---|
| `voxel_dim` | 0.2 m | FAR's internal grid resolution |
| `obs_inflate_size` | 1 | Inflate obstacles by 1 cell (0.2 m) per side |
| `dynamic_obs_decay_time` | 3.0 s | Time before dynamic voxels expire |
| `converge_distance` | 0.5 m | Goal convergence |
| `terrain_free_Z` | 0.45 m | Height below which points → ground (not obstacle) |
| `sensor_range` | 6.0 m | Max obstacle registration range |
| `terrain_range` | 4.5 m | Max terrain analysis range |
| `local_planner_range` | 2.0 m | Path library evaluation radius |
| `path_momentum_thred` | 25 | Path commitment (higher = less replanning) |
| `world_frame` | "map" | TF frame for global planning |

### Critical insight on `obs_inflate_size`

Operates on FAR's **voxel grid** (voxel_dim=0.2 m cells), NOT the occupancy grid (0.05 m). `obs_inflate_size=3` → 3 cells × 0.2 m = 0.6 m per side = **1.2 m total corridor narrowing**. Deadlocked the robot in 1 m corridors. `obs_inflate_size=1` = 0.2 m per side = 0.4 m total, leaving ~0.6 m passable in 1 m corridor.

## LiDAR hardware: Livox MID-360 vs Unitree L1

| Spec | Livox MID-360 | Unitree L1 |
|---|---|---|
| Type | Non-repetitive Risley prism | Hemispherical |
| FOV | 360° × 59° (-7° to +52°) | 360° h × 90° v (0°-90°) |
| Point rate | 200k pts/s | ~43k pts/s (360 h × 120 v) |
| Range | 40 m typ | 30 m typ |
| Mount position (sim) | Back-top (0.16, 0, 0.12) | Chin (0.29, 0, -0.04) |
| MJCF site | `livox_mid360` | `unitree_l1` |

### Plugin LiDAR selection

`mujoco_ros2_sensors.cpp` checks sites in order: `unitree_l1` → `livox_mid360`. Switch by commenting/uncommenting `<site>` in MJCF. Both can be active; plugin picks L1 first.

### Sim ray counts (`mujoco_ros2_sensors.cpp`)

- MID-360: 1000 h × 20 v = 20,000 pts/frame @ 10 Hz
- L1: 360 h × 120 v = 43,200 pts/frame
- Both: `range_min=0.1 m`, `range_max=40 m`

### L1 chin-mount gotcha

At (0.29, 0, -0.04) the LiDAR is at z=0.56 m world height. Downward rays hit ground within 1.5 m → terrain_analysis classifies ground returns as obstacles → FAR pathfinding breaks → robot freezes. Fix: z-band filter in `pointcloud_to_laserscan` (`min_height`/`max_height`), already commented in launch. **MID-360 back-top mount avoids this entirely.**

## MuJoCo built-in LiDAR sensor

Moved from external Python node (`mujoco_lidar_node_multiray`) into DFKI `mujoco_ros2_control` C++ plugin to eliminate sensor sync lag.

| File | Purpose |
|---|---|
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_sensors/lidar_sensor.hpp` | LidarSensor + LidarSensorConfig |
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/lidar_sensor.cpp` | Raycast: `mj_multiRay()` on live `mjData*` |
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_sensors.cpp` | Auto-discovers `unitree_l1` or `livox_mid360` MJCF sites |

**How it works:** Sensor shares `mjModel*/mjData*` with physics sim. Wall timer (11 Hz) reads `site_xpos`/`site_xmat` from live sim data, rotates pre-computed ray directions to world frame, calls `mj_multiRay()` for all rays in one C call, transforms hits to LiDAR-local frame, publishes `PointCloud2` on `registered_scan` (BEST_EFFORT QoS).

**Thread safety:** `mj_multiRay` traverses BVH tree and is NOT safe to call concurrently with `mj_step`. A `sim_step_mtx_` mutex is shared between physics loop and lidar sensor. Physics loop locks around `mj_step()`, lidar locks around data reads + `mj_multiRay()`.

## A/B benchmark: Cartographer+L1 vs Fast-LIO+MID-360

3-trial runs on demo1 (12×8 m, 96 m²), 120 s sessions, FAR backend.

### Cartographer + L1 (chin-mounted)

| Trial | Area (m²) | Contacts | PASS |
|---|---|---|---|
| 1 | 0.0 | 0 | FAIL (no motion — L1 chin ground hits) |
| 2 | 0.0 | 0 | FAIL |
| 3 | 0.0 | 0 | FAIL |

L1 chin mount was broken — ground returns flooded terrain_analysis. **0/3 PASS.**

### Fast-LIO + MID-360 (back-top mount)

| Trial | Area (m²) | Contacts | PASS |
|---|---|---|---|
| 1 | 97.9 | 61 | FAIL (contacts) |
| 2 | ~98 | ~60 | FAIL |
| 3 | ~98 | ~60 | FAIL |

Coverage excellent (102% of GT, slight over-count from map inflation). But wall contacts ~60/trial — the fundamental FAR-plans-through-walls problem persists. **0/3 PASS on strict criterion.**

### Key finding

Fast-LIO2 fixes scan-odom temporal alignment (no map ghosting), excellent coverage, but wall contacts remain because obs_inflate_size/stopping margins still need tuning. **SLAM backend is no longer the bottleneck — obstacle avoidance margins are.**

## Cartographer TF configuration (resolved 2026-04-06)

**Problem:** Robot position stuck near origin on Cartographer map. Scans accumulated at one position.

**Root cause:** `cartographer_sim_2d.lua` had `published_frame = "odom"` + `provide_odom_frame = false`, making Cartographer depend on external `odom → base_link` TF. Also, `test.sh` missing `FASTRTPS_DEFAULT_PROFILES_FILE` caused DDS shared memory issues (stale data between runs).

**Fix (odom-free):** Cartographer publishes `map → base_link` directly, no odom frame in TF tree.
- `cartographer_sim_2d.lua`: `published_frame = "base_link"`, `provide_odom_frame = false`, `use_odometry = false` → pure LiDAR+IMU scan matching.
- Launch files: `odom_bridge_publish_tf = false`, removed Cartographer odom topic remap.
- `test.sh`: Added `FASTRTPS_DEFAULT_PROFILES_FILE` export and `/dev/shm/fastrtps_*` cleanup.

**TF tree:** `map → base_link → imu, livox_mid360, ...`. `carto_odom_bridge` converts `map → base_link` TF to `/robot/odom/nav`.

**Note:** `provide_odom_frame = true` does NOT work odom-free — Cartographer's `tf_bridge.cpp` tries to lookup `odom` before it can publish it, deadlock.

## Scene-agnostic wall checker

`scripts/runtime/far_wall_checker.py` refactored from prefix-based (`wall_` / `divider_` geom names) to scene-agnostic:

```
SAFE_GEOMS = {ground, floor, ...}
ROBOT_PREFIXES = {FL_, FR_, RL_, RR_, base_link, hip, thigh, calf, foot, wheel}

for contact in mjData.contact[]:
    geom_a, geom_b = contact pair
    if one is ROBOT_PREFIX and other is NOT in SAFE_GEOMS:
        → WALL CONTACT
```

Works for any scene (demo1, demo2, LRC maze) without knowing geom names in advance.

## Demo scenes

| Scene | File | Size | Area |
|---|---|---|---|
| demo1 | `mujoco/demo1.xml` | 12×8 m | 96 m² |
| demo2 | `mujoco/demo2.xml` | 24×16 m | 384 m² |
| LRC maze | `mujoco/lrc_maze_go2w.xml` | Variable | — |
| door task | `mujoco/two_rooms_door_scene.xml` | 8×4 m | 32 m² |

- **demo1** (renamed from `vlm_exploration_scene_no_artifacts.xml`): single room with interior dividers forming corridors. 19 named collision geoms. MID-360 default.
- **demo2**: 4 quadrants connected by L-corridor, T-junction, dead-end rooms. Interior pillars + crates as clutter. Generated with `scripts/ops/generate_lrc_2025_world.py`.
- **LRC maze**: imported from DARPA-style maze assets, merged with Go2W MJCF.

## Convenience scripts

| Script | Scene | SLAM | Notes |
|---|---|---|---|
| `scripts/launch/nav_test_fastlio.sh` | demo1 | Fast-LIO2 | Sources sc_pgo from LRC stack |
| `scripts/nav_test_demo2.sh` | demo2 | Fast-LIO2 | `scene_area_m2=384` |
| `scripts/launch/nav_test_lrc_maze.sh` | LRC maze | Cartographer | Uses nav_test_mujoco.sh |
| `scripts/benchmark_far_nav.sh` | demo1 | Cartographer | Multi-trial headless |
| `scripts/benchmark_fastlio.sh` | demo1 | Fast-LIO2 | Multi-trial headless |

All support `gui:=false`, `enable_wall_checker:=true`.
