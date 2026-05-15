# 3D-frontier-exploration pipeline — raw cloud → cmd_vel

End-to-end data path for `nav_test_3d_explore.sh` (demo_ramp + Point-LIO/Fast-LIO + nvblox_frontend + CFPA2 3D-IG + Nav2 SE2). Read this BEFORE patching any one stage — most "bugs" turn out to be expectation mismatches between two adjacent stages.

```
 MuJoCo physics ─┬─► Mid-360 cloud  ───┐
                 ├─► IMU 200Hz        ─┤
                 └─► joint_states     ─┘
                          │
                          ▼
                     ┌─────────────┐
                     │   Fast-LIO  │  3D LIO SLAM (HKU)
                     └─────────────┘
                     /robot/Odometry  (10 Hz at IMU rate)
                     /robot/cloud_registered_body  (in lidar/body frame)
                     /robot/cloud_registered       (in map frame)
                          │
              ┌───────────┴────────────┐
              ▼                        ▼
   fast_lio_tf_adapter         nvblox_frontend mapper_node
   /robot/odom/nav             3D occupancy_layer (CUDA)
   TF map→base_link            /robot/traversability_grid (2D, 0.10m)
                               /robot/voxels_3d           (3D sparse)
                                    │           │
                                    │           ▼
                                    │      CFPA2 single_robot_node
                                    │      • frontier_3d.extract_3d_frontiers
                                    │      • ClusterTracker (world-AABB)
                                    │      • _apply_goal_policy
                                    │      /robot/way_point_coord (PointStamped)
                                    │           │
                                    │           ▼
                                    │      cfpa2_to_nav2_bridge
                                    │      /robot/goal_pose (PoseStamped)
                                    │           │
                                    ▼           ▼
                              Nav2 stack (planner_server, controller_server,
                                          behavior_server, bt_navigator)
                              • global_costmap StaticLayer ← traversability_grid
                              • local_costmap  StaticLayer ← traversability_grid
                              • SmacPlannerLattice (SE2 lattice primitives)
                              • MPPIController (no-strafe DiffDrive motion_model)
                              /robot/cmd_vel  (Twist, ~20 Hz)
                                    │
                                    ▼
                              go2w_hybrid_cmd_router
                              • mux CHAMP (legged) vs wheel (skid)
                              /robot/cmd_vel_legged  → CHAMP locomotion
                              /robot/wheel_position_controller/commands
                                    │
                                    ▼
                              CHAMP gait controller + wheel velocity controller
                                    │
                                    ▼
                              mujoco_ros2_control (HW interface) → MuJoCo physics
```

## Per-stage I/O

### 1. MuJoCo (sim source)

- **Process**: `mujoco_ros2_control` + `mujoco_sensor_bridge`.
- **Pubs**:
  - `<lidar>/<topic>` — raw Mid-360 PointCloud2. With `MUJOCO_LIDAR_HZ_SAMPLES=1024`, `VT_SAMPLES=96`: ~23 k pts/scan at 10 Hz, vertical FOV -7°..+52°, asymmetric.
  - `/robot/imu` — 200 Hz IMU.
  - `/robot/joint_states` (under `controller_manager` namespace — for sim usually `/mujoco_sim/joint_states`).
- **Failure modes that break the pipeline**:
  - `libmujoco.so` dlopen failure → no sensor plugins → no cloud + no IMU → everything below is silent.
  - Hardcoded 1000×20 rays → walls only 1-3 voxels tall in nvblox (rule 21).

### 2. Fast-LIO2 (SLAM)

- **Process**: `fast_lio` (use_fast_lio:=true), or Point-LIO on the real robot.
- **Subs**: Mid-360 cloud + IMU.
- **Pubs**:
  - `/robot/Odometry` — IMU-rate pose (10 Hz nominal). Frame: map→lidar (NOT base_link).
  - `/robot/cloud_registered_body` — deskewed cloud in body/lidar frame. **This is what nvblox eats.**
  - `/robot/cloud_registered` — same cloud in map frame.
- **Failure modes**:
  - SLAM divergence → garbage odometry → nvblox carves voxels at wrong world coords → trav_grid is junk.
  - Topic rate drop (e.g., FAST-LIO ikd-tree growth) → mapper subscribes but receives nothing.

### 3. fast_lio_tf_adapter (TF + odom bridge)

- **Process**: `scripts/runtime/fast_lio_tf_adapter.py`.
- **Subs**: `/robot/Odometry`.
- **Pubs**:
  - `/robot/odom/nav` — same odom, but advertised with the topic name CFPA2 + nvblox expect.
  - TF: `map → base_link` (50 Hz). Single owner of this link (rule 16).
- **Failure modes**: missing TF if adapter dies → mapper `lookup_transform(map, base_link)` fails silently → mapper warns but skips frame.

### 4. nvblox_frontend mapper_node (3D mapping + 2D projection)

- **Subs**: `cloud_registered_body` + `odom/nav`.
- **Internal**: nvblox occupancy_layer (Bayesian log_odds 3D raycasting), voxel size 0.10m, transient block allocation.
- **Pubs**:
  - `/robot/traversability_grid` (nav_msgs/OccupancyGrid, **0** = FREE, **100** = OCC, **-1** = UNK):
    1. **Pass 1**: for each allocated block, find highest OCC voxel per column (H[idx]) and bitmap of FREE voxel z-bins (free_bits[idx]).
    2. **Pass 2 (clearance)**: if any OCC voxel exists in z ∈ (H, H + clearance], mark cls=100.
    3. **Classify**: H non-NaN → cls=0 (unless step 2 set 100); H NaN with grounded-FREE run (first_free_z ≤ 0.30m) → cls=0; else cls=-1.
    4. **Slope/step filter**: cls=0 → cls=100 if any 4-neighbour `|hn − h0| > step_max=0.20` (1-cell) OR `|hn − h0| / (5·vs) > tan(30°)` (5-cell baseline) (rule against ramp aliasing).
    5. **3×3 median**: smooths FREE/UNK only; OCC is sacred (sparsely observed walls can't survive vanilla median).
    6. **Blind-disk flood-fill**: within 3m disk around robot, grow FREE into UNK from adjacent FREE (replaces the old "force-FREE within 3m" that leaked through sparse walls).
    7. **Persistence**: merge cls into `cls_persist_` (world-fixed 40×40m grid pinned to first robot pose). Any non-UNK overwrites; UNK keeps history.
  - `/robot/voxels_3d` (custom VoxelGrid3D msg): rolling 20×20×3m robot-centric grid of {-1, 0, 100} sparse OCC for CFPA2 3D-IG.
- **Failure modes**:
  - Mean-centroid of ring-shaped frontier → goal at robot (load-bearing fix: farthest-from-robot voxel, [frontier_3d.py:193-225](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/frontier_3d.py)).
  - voxels_3d AABB drift across frames (robot-centric origin) → ClusterTracker must convert to world coords before matching.
  - Median erodes 2-cell-thick walls → blind disk leaks FREE past wall → cls_persist_ pollution.

### 5. CFPA2 cfpa2_single_robot_node (goal allocator)

- **Subs**:
  - `/robot/traversability_grid` (2D map for reachability check via inflation-blind BFS — known gap, rule pending).
  - `/robot/voxels_3d` (3D map for frontier extraction).
  - `/robot/odom/nav` (robot pose).
- **Internals**:
  - `frontier_3d.extract_3d_frontiers`: voxels with ≥1 UNK 6-neighbour are "frontier"; cluster via 6-connected flood-fill; z-band filter [-0.2, 1.5]m removes air voxels above ramp; **centroid = farthest-from-robot frontier voxel** when robot_xy supplied (NOT mean).
  - `cluster_tracker.update`: world-AABB overlap matches new clusters to tracked IDs; tracks volume history + attempt count; `non_actionable` if attempts≥3 AND no shrink for 30s.
  - `_apply_goal_policy`: utility scoring (IG box + cost + heading), goal_lock + stable-challenger override + reach-blacklist.
- **Pubs**:
  - `/robot/way_point_coord` (PointStamped) — chosen goal in map frame.
  - `/robot/exploration_status` (`searching | executing | no_reachable | no_frontiers | paused`).
- **Failure modes**:
  - Inflation-blind BFS marks frontier reachable that Nav2 can't path through (cfpa2 reachability ≠ Nav2 reachability — rule pending).
  - `switch_min_dist` < Nav2 `xy_goal_tolerance` → CFPA2 keeps re-issuing same goal that Nav2 already SUCCEEDED on (current values 0.45 vs 0.50 align).
  - Cluster ID changes across frames (was AABB-in-voxel-coords bug) → attempt_count resets → `non_actionable` never fires.

### 6. cfpa2_to_nav2_bridge (waypoint → goal)

- **Process**: `scripts/runtime/cfpa2_to_nav2_bridge.py`.
- **Subs**: `/robot/way_point_coord` + `/robot/odom/nav`.
- **Pubs**: `/robot/goal_pose` (PoseStamped). Yaw = robot→goal vector heading; dedupes same-goal republishes.
- **Failure modes**:
  - Same-position re-publish filter discards a goal CFPA2 considers fresh because their distance thresholds disagreed (was a 0.4-0.6m dead band, fixed).

### 7. Nav2 stack (path + control)

- **Lifecycle-managed nodes** (one process each):
  - **planner_server** (`SmacPlannerLattice`): SE2 lattice primitives, motion model = diff-drive (no strafe). Reads global_costmap.
  - **controller_server** (`MPPIController`): no-strafe DiffDrive motion_model, forward + yaw bias, footprint-aware (rule 14). Reads local_costmap, publishes `/robot/cmd_vel`.
  - **behavior_server**: spin / back_up / wait recoveries.
  - **bt_navigator**: BT for `navigate_to_pose`. Subscribes `/robot/goal_pose`.
- **Costmaps** (children of planner_server + controller_server):
  - **global_costmap**: StaticLayer on `/robot/traversability_grid` + InflationLayer.
  - **local_costmap**: same StaticLayer source (3d mode).
  - In 2D mode, StaticLayer reads `/robot/map` from octomap_server instead.
- **Failure modes**:
  - `SmacPlanner2D` is XY-only — for SE2 footprint-fit, must use Lattice/Hybrid (rule 19).
  - `consider_footprint: false` + `robot_radius=0.40 + margin=0.20` rejected all motion in narrow corridors (rule 14).
  - **No action-result listener in CFPA2** → SUCCEEDED/ABORTED is inferred from distance, not the BT — root cause of several "stuck" symptoms.

### 8. go2w_hybrid_cmd_router (cmd_vel mux)

- **Subs**: `/robot/cmd_vel` + `/robot/joint_states` (wheel ω feedback).
- **Pubs**:
  - `/robot/cmd_vel_legged` → CHAMP gait → joint position commands.
  - `/robot/wheel_position_controller/commands` (Float64MultiArray) — wheel ω setpoints with hysteresis to switch between legged and wheeled.
- **Failure modes**:
  - Subscribed to `/mujoco_sim/joint_states` (absolute, dead in single-robot ns) → wheel ω feedback stuck at 0 → kv=5 actuator brake applied 13 N·m → wheel-skid bug (fixed 2026-05-10).
  - Duplicate router started by two launch files → 2× messages on legged_topic (fixed).

### 9. mujoco_ros2_control + CHAMP

- HW interface bridges ROS 2 effort/velocity/position interfaces to MuJoCo actuators. **Bug in VELOCITY branch missing `last_command` update** caused 0-cmd latch (rule 17). Fixed in vendored copy.

## Where bugs actually hide (so we stop fixing the wrong stage)

| Symptom | Most-likely stage | Wrong stage to "fix" |
|---|---|---|
| Robot moves but FREE leaks past walls | **4. mapper_node** (blind-disk fill + median) | CFPA2 reachability, Nav2 costmap |
| Robot doesn't move, no frontier published | **5. CFPA2** (centroid, reachability BFS, z-band) | Nav2 planner, controller |
| Frontier published, no path | **7. Nav2 planner** (footprint, SE2 model, inflation) | CFPA2 utility |
| Path published, no motion | **7. controller / 8. router** (MPPI rejects, wheel mux frozen) | costmap, frontier |
| Wheels skid / robot pulled sideways | **8. router** / vendored mujoco_ros2_control | CHAMP, planner |
| Goal SUCCEEDED but CFPA2 re-issues | **5↔7 threshold gap** (switch_min_dist vs xy_goal_tolerance) | Nav2 BT |

## Diagnostic tools (in repo)

- **[scripts/debug/trav_grid_diag.py](../../scripts/debug/trav_grid_diag.py)** — class counts + named-point probes + outer-wall-box leak + per-wall coverage + radial-FREE histogram + PNG dump. One command for stage 4 health.
- **[scripts/debug/far_monitor.py](../../scripts/debug/far_monitor.py)** — FAR planner observability (legacy).
- **[scripts/debug/failure_decomposer.py](../../scripts/debug/failure_decomposer.py)** — benchmark failure classifier.
- **CFPA2 `verbose_logs: false`** + **`/robot/exploration_status`** — state-change-only INFO, plus structured state machine.
- **[scripts/runtime/stuck_watchdog.py](../../scripts/runtime/stuck_watchdog.py)** — emits `/robot/recovery_event` when (v≈0, ω≈0) + active goal for >10s.
- **[exploration_metrics_logger.py](../../src/go2w/go2w_observability/scripts/exploration_metrics_logger.py)** — central event aggregator + structured event log + 30s rolling summary + stop trigger.

## Open follow-ups (deferred but documented)

- CFPA2 reachability uses `/robot/map` BFS without inflation/footprint → false-positive reachability for cells Nav2 then can't path through. Fix: feed from `/robot/global_costmap/costmap` (already inflated).
- CFPA2 should listen to `navigate_to_pose` action result (SUCCEEDED/ABORTED) instead of inferring from distance.
- Stage 4 blind-disk flood-fill uses 4-conn growth from FREE; takes up to ~30 iterations to saturate (still O(r³), fine at vs=0.10).
