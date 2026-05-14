# CLAUDE.md — Collab_QRC Index

Multi-robot autonomy with Unitree Go2W wheeled-legged quadrupeds + Go2 walking quadrupeds on ROS 2 Humble + MuJoCo (primary) / Gazebo Classic (legacy) / real-robot deployment. Active focus: **Nav2 SE2 holonomic stack tuning** for both single-robot exploration and heterogeneous dual-robot (Go2W + Go2) coordination, with CFPA2 frontier allocation.

Door task (Phase 2 dual-robot VLM coordination) and the legacy A*/default Python nav backends were removed in the 2026-05 cleanup; see [CLAUDE1.md](CLAUDE1.md) for Phase 1 VLM exploration history, Phase 2 FSM archive, archived 2026-04 operational notes, and the deletion log.

## Active state (2026-05-13) — Point-LIO + gbplanner3 onboard, full stack up

- **Point-LIO replacing FAST-LIO2 as production SLAM** ([docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md)).
  Measured 36% rate degradation on FAST-LIO2 over a 3-min ops2 walk (9 Hz start → 4.4 Hz end while inputs steady at 10/200 Hz lidar/IMU). Root cause: ikd-tree grew unbounded → main-loop iterations stalled, lidar frames dropped in callback queue. Switched to Point-LIO (HKU MaRS 2023, [`src/vendor/point_lio_ros1/`](src/vendor/point_lio_ros1/)): iVox replaces ikd-tree (O(1) avg query), decoupled IMU+LiDAR threads, IMU keeps publishing odom when LiDAR stalls. Verified flat 10.001 Hz `/robot/Odometry` (std 1.1 ms) after `jetson_clocks` + MAXN power mode. Patches mirror our FAST-LIO ROS1 set: livox_ros_driver rename → driver2, advertise paths relative, plus a custom NodeHandle split so `/robot/Odometry` doesn't end up under `/robot/laserMapping/Odometry` (Point-LIO uses `nh("~")` for both params + topics; we add `pub_nh` for publishers).
- **gbplanner3 stack built + running on Jetson** (PID 31517 gbplanner_node, PID 31518 pci_general_ros_node, uptime 25 s clean). All 22 UAS vendor repos imported via laptop-side `vcs import` + rsync (Jetson has no GitHub access). 7-step pitfall journey documented in [gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md): src/src/ flattening, `COLCON_IGNORE` blocks catkin_pkg, 5 apt debs scp-installed, gflags ExternalProject pre-stage, `image_transport_plugins` + `manhole_detector_ros` CATKIN_IGNOREs, BT XML path depth (`config/ugv/real/go2/` 4-deep for `../../../bt_xml/` to resolve correctly), v2 → v3 config schema rewrite.
- **gbplanner3 ↔ Point-LIO topic interface**: voxblox baked INSIDE `gbplanner_node` (no separate voxblox_node), subscribes to `/robot/Odometry` + `/robot/cloud_registered_body` natively (no ros1_bridge for SLAM stream — that was the whole point of moving to Noetic). Only `/pci_command_path` (< 1 KB/s) crosses to ROS 2.
- **Updated [scripts/real/gbplanner3/](scripts/real/gbplanner3/)**: `bridge_topics.yaml` trimmed to PoseArray-only (no more 2to1 SLAM topics), `gbplanner_go2.launch` rewritten to match upstream v3 pattern (voxblox-in-gbplanner-node, no robot_config.yaml), `orin_launch_gbplanner.sh` no longer assumes Foxy Fast-LIO; preflights Point-LIO presence.

## Active state (2026-05-11) — Noetic FAST-LIO2 onboard (gbplanner3 prep)

- **Native ROS 1 FAST-LIO2 brought up on the Jetson** in a separate catkin ws (`~/noetic_fastlio_ws/`), parallel to the existing Foxy `~/onboard_ws/`. Eliminates ros1_bridge bandwidth bottleneck for the gbplanner3 voxblox path (heavy `/robot/cloud_registered_body` stream now stays in ROS 1 natively; only the tiny `/pci_command_path` PoseArray still crosses to ROS 2).
- New scripts: [`deploy_noetic_to_jetson.sh`](scripts/real/deploy_noetic_to_jetson.sh) (laptop rsync), [`onboard_fastlio_noetic.sh`](scripts/real/onboard_fastlio_noetic.sh) (Jetson launcher), [`onboard_record_noetic.sh`](scripts/real/onboard_record_noetic.sh) (`rosbag record`, nohup-protected), [`stream_cloud_live.sh`](scripts/real/stream_cloud_live.sh) (Open3D live viewer over ssh binary pipe — replaces X-forwarded RViz which renders black on Jammy + Ogre).
- 11 patches against HKU upstream `FAST_LIO` + `livox_ros_driver2` (livox driver rename, Mid-360s enum, absolute topic paths, launch param-override bug, ...). **Full pitfall list and tuning notes:** [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md).
- Tuning ported from Foxy real-robot yaml: `point_filter_num=1`, `filter_size_*=0.10`, `extrinsic_est_en=true`, `pcd_save_en=false`. Effective rate ~8 Hz on Orin (was 10 Hz with default 0.50 voxel — extra CPU buys ICP correspondence density for sparse outdoor scenes).

## Active state (2026-05-10) — CFPA2 policy + Go2W wheel-skid fix

- **CFPA2 stable-challenger goal override** ([commit 0505ff0](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py)).
  `_apply_goal_policy` previously held a goal as long as the robot was making progress, never re-evaluating utility under the current map → robot kept committing to the older frontier even when a much better one appeared mid-flight. New override (gated by 3-tick streak + 2s lock-age + 1.20× score-improvement vs re-evaluated `utility(last)`) lets a freshly-seen high-IG frontier preempt without re-introducing zigzag from cluster centroid jitter. Bypasses `goal_lock_sec` since the streak gate already provides anti-thrash. Set `cfpa2_challenger_streak_required=0` to disable.

- **CFPA2 multiplicative overlap penalty for joint allocator** (same commit).
  Old `joint = a + b - λ·overlap` with λ=1.0 deducted ≤ 1.0 from IG-dominated sums in the hundreds-thousands → both robots routinely chose the same frontier region. New `joint = (a + b) × (1 − λ·overlap)` makes λ a "max % deduction when fully overlapping" (default 0.5 = up to 50% off). Scale-invariant w.r.t. IG-box / `cfpa2_w_ig` changes. Also tightened `cfpa2_sigma_overlap_m: 0 → 4` (was 2×sensor_range fallback ≈ 7m, too gradual). Live test: same-room (500+500, 2m apart) joint=520 vs different-rooms (400+350, 12m apart) joint=735 → splits decisively.

- **Go2W wheel-skid bug — router subscribed to wrong joint_states topic** ([commit 5d0e01b](src/go2w/go2w_control/scripts/go2w_hybrid_cmd_router.py#L81)).
  In legged mode the router is supposed to mirror actual wheel ω back as setpoint so kv=5 actuator brake torque stays at 0 (true freewheel). Live `ros2 topic echo` showed it publishing `[0,0,0,0]` indefinitely → `kv·(0 − ω_actual)` ≈ 13 N·m brake force per wheel under CHAMP gait → wheels skidded. Two root causes:
  1. Router default `wheel_state_topic = /mujoco_sim/joint_states` (absolute) had publisher_count=0 in single-robot sim (controller_manager runs in `/robot/`). Subscribed to dead topic → `_latest_wheel_vels` stayed `[0,0,0,0]`. Default changed to relative `joint_states` (works for single); mixed/dual launches override back to `/mujoco_sim/joint_states` explicitly.
  2. `nav_test_mujoco_fastlio.launch.py` was spawning a duplicate `go2w_hybrid_cmd_router` (the included `single_go2w_mujoco_cfpa2.launch.py` already starts one). Both shared name+ns → CHAMP saw 2× messages. Removed.

  Bonus dual-Go2W fix: per-namespace `wheel_joint_names` override (b_*-prefixed for robot_b) so robot_b's router doesn't read robot_a's wheel ω from the shared `/mujoco_sim/joint_states`.

## Active state (2026-05-05) — SE2-only direction + observability + lidar_range

- **SE2 holonomic is now the canonical and only navigation profile we tune for.**
  After 2026-05-02 validated `se2_holonomic` (SmacPlannerLattice + diff primitives + no-strafe MPPI) as the operator-preferred behavior, all subsequent tuning effort (frontier reachability, costmap inflation, MPPI critics, exploration metrics, stop conditions) targets THIS profile and no other. The legacy `holonomic_profile=off` (diff-drive Reeds-Shepp) and `omni_2d` (SmacPlanner2D + MPPI Omni) remain in [`real_single.launch.py`](src/go2w/go2w_real_bringup/launch/real_single.launch.py) and [`real_autonomy.sh`](scripts/real/real_autonomy.sh) as **escape hatches only** — not maintained for daily nav and not recipients of new tuning. New operators / sessions should default to SE2.

- **Streamlined entry: [`scripts/real/real_autonomy_se2.sh`](scripts/real/real_autonomy_se2.sh).**
  Thin wrapper over `real_autonomy.sh` that bakes in `holonomic_profile=se2_holonomic` and forwards a curated subset of flags (`robot, slam, oa, execute, lidar_range, manual, record`). Daily ops should call this; profile-selection complexity stays out of operator memory. The full surface (off / omni_2d / etc.) remains accessible via `real_autonomy.sh` for ad-hoc comparison runs.

- **`lidar_range` launch param (default 8.0 m).**
  Threaded through `real_autonomy.sh → real_single.launch.py → real_bringup_core.launch.py`; controls octomap_server raytrace + `pointcloud_to_laserscan` `range_max` simultaneously. Lower (e.g. 4.0 m) for cluttered indoor scenes where far-range obstacles introduce noise; raise for open spaces. Independent of Fast-LIO's `det_range: 100.0` (SLAM odometry quality, not perception).

- **Exploration observability + explicit stop condition shipped.**
  [`exploration_metrics_logger.py`](src/go2w/go2w_observability/scripts/exploration_metrics_logger.py) is now the central event aggregator. Adds:
  1. **Structured event log** to stdout + `$ROS_LOG_SESSION_DIR/exploration_events_*.log` — one line per significant event with absolute timestamp + Δ from previous event. Sources: `/<ns>/exploration_status`, `/<ns>/goal_pose`, `/<ns>/behavior_tree_log` (plan timing via BT-internal timestamps, dedupes recovery-branch `ComputePathToPose` duplicates), `/<ns>/recovery_event`, `/cmd_vel`.
  2. **30 s rolling summary** — `plan_ms[p50/p95/max]`, `plan_ok=N/M (P%)`, `goals[done/abort_p/abort_c]`, `ttm_avg`, `coverage Δ%`, `status`.
  3. **Stop trigger** — publishes latched `/<ns>/exploration_complete` (with reason) + cancels `navigate_to_pose` action + zero-cmd_vel pulse, when either:
     - `consec_no_reachable_threshold=3` ticks of CFPA2's `no_reachable` status, or
     - coverage Δ < `coverage_stagnant_threshold_pct=0.5%` over `coverage_stagnant_window_sec=30s`.

  CFPA2 ([`cfpa2_single_robot_node.py`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_single_robot_node.py)) gained a `_status_pub` on `/<ns>/exploration_status` (states: `searching | executing | no_reachable | no_frontiers | paused`) and a subscriber to `/<ns>/exploration_complete` that latches a `_paused` flag. `verbose_logs: false` (default) suppresses the per-tick `_log_no_goal_debug` / `_maybe_log_summary` spam — only state-change INFO lines remain. [`stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) publishes `/<ns>/recovery_event` (`stuck_detected | backup_started | backup_done | backup_aborted | backup_unavailable`) so recovery activity threads through the same event stream.

- **CFPA2 ↔ Nav2 threshold alignment fix.**
  Discovered a 0.40–0.60 m dead band where Nav2's `xy_goal_tolerance: 0.40` declared SUCCEEDED but CFPA2's `switch_min_dist: 0.60` still considered the goal "in flight" — same goal got republished, bridge filtered it as unchanged, robot sat idle. Fix: [`cfpa2_single_robot.yaml`](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml) `switch_min_dist: 0.60 → 0.45` and added `reached_blacklist_dist: 0.45` so CFPA2's notion of "reached" sits above Nav2's. Long-term fix (action-status listener for ground-truth SUCCEEDED/ABORTED) is documented but not yet implemented.

- **CFPA2 reachability uses an inflation-blind BFS** ([`cfpa2_coordinator_node.py:1208`](src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#L1208)).
  `_distance_transform` walks 4-connected through cells where `_is_free()` is True (i.e. `0 ≤ v < occ_thresh` AND `v != -1`). It does NOT account for inflation, footprint, or kinematics — so it can mark a frontier reachable that Nav2 then cannot path through. For now this is documented as a known gap; the planned fix is to feed CFPA2 from `/{ns}/global_costmap/costmap` (inflated) instead of `/{ns}/map` (raw octomap projection).

## Active state (2026-05-02) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-05-02--go2w-real-nav2-profile-split--no-crab-se2-tuning).
- Headline: 3-profile runtime matrix (`off` / `omni_2d` / `se2_holonomic`) for Go2W real Nav2; finding that `SmacPlanner2D` is XY-only (one angle bin in Node2D); final SE2 tuning forces no-strafe execution (`MPPI motion_model=DiffDrive`, lattice diff primitives, forward+yaw bias). Sim parity added via [`nav2_se2_holonomic_overlay_sim.yaml`](src/go2w/go2w_config/config/nav/nav2_se2_holonomic_overlay_sim.yaml). 2026-05-05 superseded this with "SE2-only" — the older `off` / `omni_2d` profiles are escape hatches now.

## Active state (2026-04-30) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson).
- Headline: onboard SLAM split succeeded (Jetson Foxy + laptop Humble via cross-host DDS); the 13 GB-per-node memory blow-up was resolved with `ulimit -v`; compatibility notes documented for future Humble migration.
- Canonical runbook remains [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson).

## Active state (2026-04-29) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-29--dual-robot-nav2-mppi-migration).
- Headline: dual-robot stack migrated to Nav2 MPPI/SmacHybrid; fixed wheel brake bug in vendored `mujoco_ros2_control`; added `stuck_watchdog`; unified sim/real TF+odom path through `fast_lio_tf_adapter`.
- Deep narrative and rationale remain in [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md).

## Active state (2026-04-26) — archived

- Detailed timeline moved to [`CLAUDE1.md`](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack).
- Headline: A* retired for dual-robot runs in favor of FAR + safety stack (`pivot-lock`, `path_safety_filter`, `cmd_vel_safety_shield`), with documented deadlock tradeoffs and TF remap pitfalls.
- Consolidated context remains in [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) and [docs/claude/debug_notes.md](docs/claude/debug_notes.md).

## Skill-API detail docs

| Topic | Doc |
|---|---|
| Nav stack benchmarking (Phase 5), config A, iteration logs | [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) |
| Fast-LIO2 / Cartographer A/B, LiDAR options, demo scenes | [docs/claude/slam_and_scenes.md](docs/claude/slam_and_scenes.md) |
| Cross-cutting debugging gotchas (QoS, zombies, MuJoCo quirks) | [docs/claude/debug_notes.md](docs/claude/debug_notes.md) |
| **Gazebo vs MuJoCo — why stack works in Gazebo, MuJoCo matches real life** | [docs/claude/sim_comparison.md](docs/claude/sim_comparison.md) |
| **Real Go2W / Go2 — connect modes, SLAM A/B, Mid-360 calib, 10-layer bug chain** | [docs/claude/real_robot.md](docs/claude/real_robot.md) |
| **Go2 (non-W) integration — Menagerie MJCF, CHAMP shipped, real CMU TARE → localPlanner (FAR bypassed), RL scaffold in place** | [docs/claude/go2_integration.md](docs/claude/go2_integration.md) |
| **Dual-robot FAR safety stack (2026-04-26): pivot-lock + path_safety_filter + cmd_vel_safety_shield. Why A\* was retired for dual.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack) |
| **Nav2 MPPI migration journey (2026-04-29): A\* → FAR → MPPI, brake bug, freewheel, stuck_watchdog, TF/SLAM chain. Two real-robot blockers.** | [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md) |
| **Onboard SLAM split (2026-04-30): Fast-LIO + Livox onto Go2 Jetson Foxy. 5 source patches, the 13-GB-per-node bloat, the `ulimit -v` fix.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson) + [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson) |
| **Noetic FAST-LIO2 onboard (2026-05-11): ROS 1 native build of HKU FAST-LIO + Livox driver on Jetson for gbplanner3. 11 patches, conda-poison guard, X11-free Open3D streaming.** | [docs/claude/noetic_fastlio_onboard.md](docs/claude/noetic_fastlio_onboard.md) |
| **Point-LIO + gbplanner3 onboard (2026-05-13): switched SLAM from FAST-LIO2 → Point-LIO (iVox vs ikd-tree fixes 36% rate decay), full gbplanner3 stack built on Jetson, 8 build pitfalls (src/src/, COLCON_IGNORE, gflags ExternalProject, BT XML path depth, v2→v3 config schema, ...).** | [docs/claude/gbplanner3_noetic_onboard.md](docs/claude/gbplanner3_noetic_onboard.md) |
| **nvblox 3D frontier exploration on demo_ramp (2026-05-13): 8 bug fixes wiring nvblox_frontend mapper → Nav2 StaticLayer → CFPA2 3D IG. Includes ground-clutter filter, ramp-vs-stairs discretization fix, RewrittenYaml topic-rewrite gotcha. Open analysis: current "3D" only changes IG, not frontier search; proposes voxel→ground projection.** | [docs/claude/nvblox_3d_frontier.md](docs/claude/nvblox_3d_frontier.md) |
| **GBPlanner3 / OmniPlanner sim-side integration (2026-05-13): ros1_bridge gotchas (latched /tf_static drop), voxblox + elevation_mapping config alignment with NTNU UAS UGV ref, host-pipe spam filter, the unsolved voxblox-doesn't-populate wall, sim-only vs. Jetson-portable artifact split.** | [docs/claude/gbplanner3_integration.md](docs/claude/gbplanner3_integration.md) |
| **Go2W real Nav2 profile split (2026-05-02): `off` / `omni_2d` / `se2_holonomic`, SmacPlanner2D XY-only finding, no-crab final SE2 tuning.** | This file's "Active state (2026-05-02)" entry |

## Scripts layout

```
scripts/
├── launch/    user-invoked entry points (nav_test_*, vlm_demo)
├── bench/     multi-trial PASS-criterion runners + session_reporter
├── runtime/   ROS 2 nodes started by launch files (policy, checkers, supervisors)
├── debug/     observe-a-running-sim tools (far_monitor, vlm_debug_web, …)
├── ops/       one-shot ops & dev utilities (reset_vgraph, test_lidar, sync_to_main)
├── real/      real-robot only (unchanged grouping)
└── common_logging.sh
```

## Quick launch

```bash
# Single-robot Go2W nav smoke test (Nav2 MPPI + SE2 by default)
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=true rviz:=true
./scripts/launch/nav_test_fastlio.sh robot:=go2w gui:=false           # headless

# Nav benchmark (FAR baseline) — 5-trial / 10-trial
./scripts/bench/benchmark_far_nav.sh
NUM_TRIALS=10 DURATION_SEC=120 OUT_DIR=/tmp/cfgA_10 ./scripts/bench/benchmark_far_nav.sh
./scripts/bench/benchmark_fastlio.sh                                  # Fast-LIO + MID-360

# Demo2 / LRC maze
./scripts/launch/nav_test_demo2.sh gui:=false
./scripts/launch/nav_test_lrc_maze.sh

# VLM exploration demo (Phase 1) — uses nav2_mppi
./scripts/launch/vlm_demo_mujoco.sh

# Heterogeneous dual (Go2W + Go2 + demo3_mixed + CFPA2 coord, nav2_mppi)
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far  # both FAR

# Go2 (no-wheel) sim — CHAMP locomotion, demo1 12×8 m / demo3 24×16 m
./scripts/launch/nav_test_go2.sh gui:=true rviz:=true                 # walk + FAR smoke
./scripts/launch/nav_test_go2_demo3.sh gui:=true rviz:=true           # larger scene
./scripts/bench/benchmark_go2.sh                                      # 5-trial PASS check
./scripts/bench/benchmark_go2_demo3.sh                                # same on demo3
./scripts/launch/nav_test_go2_tare_real.sh gui:=true rviz:=true       # CMU TARE → localPlanner
./scripts/bench/benchmark_go2_tare.sh                                 # 10-trial TARE benchmark
./scripts/launch/nav_test_go2.sh rl_policy:=true                      # RL (experimental, see go2_integration.md)

# Real robot (Go2W) — RECOMMENDED entry as of 2026-05-05:
#   SE2 holonomic baked in, oa=false (sport API direct), curated flag surface.
./scripts/real/real_autonomy_se2.sh                                   # default Go2W SE2
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360                # Mid-360 + Fast-LIO
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360 lidar_range=4.0
./scripts/real/real_autonomy_se2.sh stop                              # kill everything

# Legacy multi-profile real launcher (escape hatches for ad-hoc comparison):
./scripts/real/real_autonomy.sh                                       # Cartographer + L1 default
./scripts/real/real_autonomy.sh slam=fastlio_mid360
./scripts/real/real_autonomy.sh oa=false holonomic_profile=off        # diff-drive Reeds-Shepp
./scripts/real/real_autonomy.sh oa=false holonomic_profile=omni_2d    # SmacPlanner2D + MPPI Omni
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far
./scripts/real/real_autonomy_go2.sh                                   # Go2 (no wheel), nav2_mppi
./scripts/real/real_autonomy_go2.sh slam=fastlio_mid360 nav=far
# Real CMU TARE → localPlanner direct (FAR unwired, watchdog armed).
# oa=false is REQUIRED — default (oa=true) routes Move to /api/obstacles_avoid/request
# which needs manual mode pre-arm; oa=false sends to /api/sport/request (api_id=1008).
./scripts/real/real_autonomy.sh robot=go2 slam=fastlio_mid360 nav=tare_real oa=false
./scripts/real/real_autonomy.sh stop                                  # kill everything real-robot
```

Debug dashboard:
- VLM exploration: <http://localhost:8501> (auto-starts)

## Build

```bash
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Full build
touch src/mtare_ros1_ws/COLCON_IGNORE
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Incremental
colcon build --symlink-install --packages-select <pkg>
```

YAML + Python changes are instant via symlink-install; C++ requires rebuild.

## Repo layout

```
src/
  go2w/                             Go2W platform packages
    go2_gazebo_sim/                   MJCF/world + launch files
    mujoco_sensor_bridge/             MuJoCo sensor nodes
    go2w_control/                     Hybrid cmd_vel router (legged + wheel mux)
    go2w_nav/                         Safety utilities (collision_monitor, cmd_vel_safety_shield, …)
    go2w_perception/ go2w_config/     QoS bridge, robot_self_filter, configs, sub-launches
    unitree_go2w_ros2/                Unitree ROS 2 integration
  collaborative_exploration/
    cfpa2_collaborative_autonomy/     CFPA2 frontier allocator (single + dual joint allocator)
    dynamic_scene_filter/             Dynamic obstacle filter (peer body + temporal voxel)
    slam_backend_adapters/            Fast-LIO ↔ Nav2 adapters
  vendor/
    fast_lio/                         Fast-LIO2 SLAM
    autonomy_stack_go2/               CMU stack (FAR, terrain analysis)
    mujoco_ros2_control/              DFKI MuJoCo HW interface
    far_planner/                      FAR global planner
  vlm_explorer/                     VLM-in-the-loop exploration (Phase 1)
scripts/                            Launch scripts, benchmark runners, utilities
config/                             DDS config (fastdds_no_shm.xml)
docs/claude/                        Detailed skill-API docs (this index)
```

## Nav backends

Switchable via `nav_backend:=` at launch time. Two production backends now (legacy `astar` / `default` / `reactive` / `mppi_nav_node` were retired in commit 5c46a51).

| Backend | Planner | Note |
|---|---|---|
| `nav2_mppi` | `nav2_planner` (SmacPlannerHybrid REEDS_SHEPP / SmacPlannerLattice for SE2) + `nav2_controller` (MPPIController) + `nav2_behaviors` + `nav2_bt_navigator` + `nav2_lifecycle_manager` | **Default for sim and real.** Per-platform yaml: [`nav2_go2w_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml) for Go2W, [`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml) for Go2. Real Go2W supports overlay profile selection via `holonomic_profile`: `off` (diff-drive Reeds-Shepp, escape hatch), `omni_2d` (`SmacPlanner2D` + MPPI Omni, escape hatch), `se2_holonomic` (`SmacPlannerLattice` + forward/pivot MPPI, no strafe — **canonical**). Outer-loop `stuck_watchdog` per robot. CFPA2 `way_point` is bridged to `goal_pose` via `cfpa2_to_nav2_bridge`. |
| `far` | CMU autonomy stack | Terrain analysis + FAR V-graph + path follower (see nav_benchmarks.md). Used for benchmarking comparison and Go2 TARE exploration. |

## Golden rules (must-follow across all work)

1. **Always `use_sim_time: true`** for all nodes in MuJoCo or Gazebo. Mixed time domains corrupt maps.
2. **Never use stale TF fallback.** Drop the scan on TF failure; don't use `tf2::TimePointZero`.
3. **Each scan painted exactly once.** Clear `last_scan_` after processing in mappers.
4. **Dual-robot TF must be namespaced.** Remap `/tf` → `/{ns}/tf` for all nodes.
5. **DDS config matters.** `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS.
6. **Verify with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.
7. **Kill zombie MuJoCo before re-launch** (see [debug_notes.md](docs/claude/debug_notes.md)).
8. **Benchmark PASS criterion** = `completed ∧ coverage≥90% ∧ contacts==0 ∧ ¬tipped`. 5 trials is too small for reliability claims; use ≥10.
9. **Supervisor panic = any-button override (real robot).** Any press on the Unitree BT pad latches a 5 s window: auto `cmd_vel` blocked, FAR disarmed, sticks drive directly. See [docs/claude/real_robot.md](docs/claude/real_robot.md#supervisor-panic-override-any-button-emergency).
10. **Any node doing TF lookup in dual-robot setup MUST have `tf_remaps`.** Without `("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")`, the node's TF buffer subscribes the global `/tf` (empty in our namespaced setup) and every lookup silently fails — no error log, just stale data or `passthrough_no_tf` status. Discovered the hard way 2026-04-26: path_safety_filter + cmd_vel_safety_shield were inert for hours because of this. **Verify lookups work via `ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/{ns}/tf -r /tf_static:=/{ns}/tf_static`**.
11. **CMU's `vehicle` frame is NOT in our SLAM tree by default.** CMU only publishes `sensor → vehicle` (static); `sensor` itself isn't connected to `map` (we have `map → sensor_at_scan`, not `sensor`). So `lookup_transform(map, vehicle)` fails. The mixed launch bridges this with a `base_link → vehicle` static publisher per namespace; nodes reading paths in vehicle frame should also accept a `base_frame_fallback` parameter.
12. **Multi-layer safety stacks deadlock easily.** CFPA2 pivot-lock (refuses goal change) + cmd_vel shield (kills ω) + path_safety_filter (rejects path) can all latch a robot in place if held goal demands rotation it can't execute. Always provide a max-hold/timeout escape valve (e.g. `pivot_lock_max_hold_sec`) on each stateful safety layer. Verify each layer's status topic, NOT just the absence of motion — a robot frozen by 3 layers stacked looks identical to "stuck planner" in /nav_status alone.
13. **`peer_pose_stale_sec` must be generous in sim.** sim_time and wall-clock-stamped messages can drift several seconds during startup; a strict 0.3s threshold rejects fresh peer poses → self_filter publishes scans untouched → peer body becomes a permanent imprint in /map. Default 5.0s in dual-robot launches; tighten on real robot only after verifying timestamp alignment.
14. **MPPI's effective rejection radius = `robot_radius` + `collision_margin_distance`.** With `consider_footprint: false` (the default-because-old-comment-said-it-throws), MPPI treats the robot as a circle and adds margin on top. demo3_mixed has 0.425 m corridors; with `robot_radius=0.40 + margin=0.20 = 0.60 m` MPPI permanently rejected forward motion. **Always set `consider_footprint: true` + a polygon `footprint:` on both costmaps**, then drop `collision_margin_distance` to 0.03 m. The "throws at configure" comment was a yaml-parse bug, not a humble-1.1.20 platform bug.
15. **Footprint changes ripple through CFPA2 frontier filters** ([cfpa2_coordinator.yaml](src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_coordinator.yaml)). When you tighten footprint, you typically need to *loosen* `cfpa2_frontier_obstacle_clearance_m`, `cfpa2_frontier_unknown_check_radius_m`, and `cfpa2_frontier_min_cluster_area_m2` proportionally, otherwise late-stage exploration falsely declares "complete" with significant unknown remaining. CFPA2 has **no parameter callback** (`add_on_set_parameters_callback` is not registered), so `ros2 param set` only updates the param store; cached `self.cfpa2_*` values stay frozen. Edit yaml + restart the node.
16. **Sim/real share the same TF + odom data path via `fast_lio_tf_adapter`.** This was *not* the case before 2026-04-29: sim relied on `mujoco_odom_bridge` writing GT directly to TF (bypassing the EKF chain), and real had nothing to take over because CHAMP's `state_estimation_node` outputs `7.8e+34` NaN that starves the EKF. Now [`scripts/runtime/fast_lio_tf_adapter.py`](scripts/runtime/fast_lio_tf_adapter.py) is the single owner of `odom → base_link` TF and `/<ns>/odom/nav` topic, sourced from Fast-LIO's `/<ns>/Odometry`. **`mujoco_odom_bridge.publish_tf` must stay `false`** (or it competes with the adapter on the same TF link). Real-bot drops in unchanged; only `bootstrap_from_gt:=false` differs (no GT to align to).
17. **Don't blame the architecture before grepping the upstream.** Half a day was lost mis-locating the wheel brake bug at the router / MPPI / footprint level when the root cause was a 2-line bug in vendored `mujoco_ros2_control` (the VELOCITY actuator branch never updated `last_command`, so once the controller commanded 0 once, ctrl was never refreshed and stayed at the previous setpoint). When data clearly shows a chain breaking and "everyone says they're publishing the right thing", stop tuning higher layers and read the lowest layer's source.
18. **Outer-loop stuck recovery is needed because MPPI/DWB/FAR rarely *report* failure.** They emit (v ≈ 0, ω ≈ 0) and self-report happy. Nav2's BT recovery only fires on a controller-reported failure → never triggers in this scenario. [`scripts/runtime/stuck_watchdog.py`](scripts/runtime/stuck_watchdog.py) is the watchdog: 10 s no-motion + active goal → Nav2 BackUp action → republish goal. Per-namespace, real-robot-compatible. Caveat: BackUp itself collision-checks `simulate_ahead_time × backup_speed` of clearance (≈ 0.20 m), so a robot wedged between two walls still fails recovery — last-resort raw-cmd_vel pulse not yet wired.
19. **`SmacPlanner2D` is XY-only; do not expect it to solve heading-dependent footprint-fit maneuvers.** If the requirement is "choose body orientation to pass anisotropic narrow geometry", use an SE2 planner (`SmacPlannerHybrid` / `SmacPlannerLattice`) and tune controller execution policy separately.
20. **Missing file errors under `install/.../share/<pkg>/...` usually mean stale install artifacts, not bad launch paths.** New YAML under `src/` is invisible to `get_package_share_directory()` until that package is rebuilt (`colcon build --symlink-install --packages-select <pkg>`). This exact failure happened with `nav2_go2w_real_omni_overlay.yaml` on 2026-05-01.
21. **Don't hardcode absolute topic paths to controller_manager-namespaced state sources.** JointStateBroadcaster (and most `controller_manager`-loaded broadcasters) publish to `<cm_ns>/joint_states`, where `cm_ns` varies per launch: single-robot sim runs `cm_ns=/robot/`, mixed/dual sims run `cm_ns=/mujoco_sim/`, real robots run yet another. A node hardcoding `wheel_state_topic=/mujoco_sim/joint_states` worked silently in mixed/dual but had `publisher_count=0` in single → router published `[0,0,0,0]` wheel commands → kv=5 actuator brake-locked the wheels under CHAMP gait → wheel-skid bug, hidden for months (2026-05-10 fix). Use a relative default that picks up the per-namespace topic, and override in launches whose `cm_ns` is elsewhere. **Verify with `ros2 topic info <topic> -v` that publisher_count > 0 BEFORE assuming the node is wired.**

## Communication style

- Fast and direct. Short questions expect immediate, precise answers.
- Show work but don't narrate it. Run commands, make changes, report what happened.
- When told "still not working", don't repeat the same fix — go deeper.
- Respect the maintainer's hypotheses. When they say "I AM certain it's due to X", treat as strong signal; validate or disprove with evidence, not conjecture.

## Environment

- Python 3.10 (micromamba `cmu_env`)
- ROS 2 Humble (`/opt/ros/humble/`)
- Build: colcon + ament_cmake / ament_python
- DDS: FastDDS (sim), CycloneDDS (real robot)
- Sim: MuJoCo 3.6.0 (pip) + DFKI mujoco_ros2_control
- VLM API: xAI Grok-4-1-fast-non-reasoning, key in `.env.xai`
