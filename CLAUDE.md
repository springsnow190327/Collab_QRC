# CLAUDE.md — Collab_QRC Index

Multi-robot autonomy with Unitree Go2W wheeled-legged quadrupeds on ROS 2 Humble + MuJoCo (primary) / Gazebo Classic (legacy) / real-robot deployment. Two active threads: **dual-robot door task** (VLM-driven coordination) and **single-robot nav benchmarking** (CMU stack tuning).

For Phase 1 (single-robot VLM exploration) background, Phase 2 FSM-era archive, and archived 2026-04 operational notes, see [CLAUDE1.md](CLAUDE1.md).

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

## Active state (2026-05-02)

- **Go2W real Nav2 profile split + SE2 footprint debugging (2026-05-01 → 2026-05-02)** — session started from persistent yaw-hunting / stop-go behavior in narrow passages, moved through "enable holonomic behavior", and converged on a 3-profile runtime matrix with explicit planner/controller semantics.
  1. **Core finding: `SmacPlanner2D` is XY-only** — source inspection confirmed Node2D has one angle bin (`angles == 1`), so it cannot plan heading-dependent entry maneuvers for anisotropic footprints. Local MPPI footprint checks do not replace planner-level SE2 reasoning.
  2. **Profile architecture shipped** — preserved [`nav2_go2w_real.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_real.yaml) as baseline; layered real-time overlays via [`navigation.launch.py`](src/go2w/go2w_config/launch/navigation.launch.py), [`real_single.launch.py`](src/go2w/go2w_real_bringup/launch/real_single.launch.py), and [`real_autonomy.sh`](scripts/real/real_autonomy.sh).
  3. **Three Go2W real Nav2 profiles (`nav=nav2_mppi`)**:
     - `off`: default diff-drive profile (`SmacPlannerHybrid` + MPPI DiffDrive).
     - `omni_2d`: `SmacPlanner2D` + MPPI Omni (legacy `holonomic=true` alias maps here).
     - `se2_holonomic`: `SmacPlannerLattice` with forward/pivot execution policy (final mode from this session).
     **Sim parity (added 2026-05-02 PM)**: same `se2_holonomic` overlay is available in:
     - **Mixed demo** ([`nav_test_demo3_mixed.sh`](scripts/launch/nav_test_demo3_mixed.sh)) via per-robot flags `holonomic_profile_a` / `holonomic_profile_b` (Go2W / Go2).
     - **Single-robot Go2 sim** ([`nav_test_go2.sh`](scripts/launch/nav_test_go2.sh), [`nav_test_go2_demo3.sh`](scripts/launch/nav_test_go2_demo3.sh)) and any `nav_test_fastlio.sh`-based launch via `nav_backend:=nav2_mppi holonomic_profile:=se2_holonomic`. The `nav2_mppi` backend was newly wired into [`nav_test_mujoco_fastlio.launch.py`](src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py) — full Nav2 stack (planner_server + controller_server + behavior_server + bt_navigator + lifecycle_manager) plus `cfpa2_to_nav2_bridge`, `path_relay`, `stuck_watchdog`, and (Go2W only) `go2w_hybrid_cmd_router`.
     - **Sim overlay file** is [`nav2_se2_holonomic_overlay_sim.yaml`](src/go2w/go2w_config/config/nav/nav2_se2_holonomic_overlay_sim.yaml) — same lattice + forward-bias deltas as real, but omits `vx_max`/`wz_max`/`ax_max` so each robot's base sim yaml continues to dictate its speed envelope (Go2W 0.50 m/s, Go2 0.30 m/s). The 0.5 m diff lattice primitives are stock — they're wider than Go2 walking strictly requires (min_turning_radius=0.05) but stay kinematically feasible.
  4. **Important debug incident: missing overlay file was install drift, not launch logic** — launch failed with `No such file ... install/go2w_config/.../nav2_go2w_real_omni_overlay.yaml`; file existed in `src/` but not `install/`. `colcon build --symlink-install --packages-select go2w_config` fixed immediately.
  5. **First SE2 attempt** — `SmacPlannerLattice + omni primitives + MPPI Omni` improved feasibility but enabled crab-walk, which mismatched operator preference.
  6. **Final SE2 tuning (user-confirmed better)** — in [`nav2_go2w_real_se2_holonomic_overlay.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_real_se2_holonomic_overlay.yaml):
     - lattice primitives switched to `.../sample_primitives/.../diff/output.json`
     - MPPI forced to no-strafe execution (`motion_model: DiffDrive`, `vy_std/vy_max/ay_max = 0`)
     - yaw+forward bias increased (`GoalAngleCritic`, `PathAngleCritic`, `PreferForwardCritic`)
  7. **Net behavioral policy** — keep SE2 planner awareness for narrow geometry, but execute as yaw-align + forward motion with pivot turns; no lateral crab-walk.
  8. **Cross-link for operators** — real-robot runbook mirror is in [`docs/claude/real_robot.md`](docs/claude/real_robot.md), with the same 2026-05-02 profile note near the top for launch-time decisions.

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
| Door task current architecture (scene, packages, VLM, perception, barrier) | [docs/claude/door_task.md](docs/claude/door_task.md) |
| Door task evolution, lessons, 5-bug chain | [docs/claude/door_task_history.md](docs/claude/door_task_history.md) |
| Nav stack benchmarking (Phase 5), config A, iteration logs | [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md) |
| **A\* planner (astar_nav_node): Option B footprint, Plan B legged gating, deletion of reactive/MPPI** | [docs/claude/nav_benchmarks.md#a-star-planner-2026-04-24](docs/claude/nav_benchmarks.md#a-star-planner-2026-04-24) |
| Fast-LIO2 / Cartographer A/B, LiDAR options, demo scenes | [docs/claude/slam_and_scenes.md](docs/claude/slam_and_scenes.md) |
| Cross-cutting debugging gotchas (QoS, zombies, MuJoCo quirks) | [docs/claude/debug_notes.md](docs/claude/debug_notes.md) |
| **Gazebo vs MuJoCo — why stack works in Gazebo, MuJoCo matches real life** | [docs/claude/sim_comparison.md](docs/claude/sim_comparison.md) |
| **Real Go2W / Go2 — connect modes, SLAM A/B, Mid-360 calib, 10-layer bug chain** | [docs/claude/real_robot.md](docs/claude/real_robot.md) |
| **Go2 (non-W) integration — Menagerie MJCF, CHAMP shipped, real CMU TARE → localPlanner (FAR bypassed), RL scaffold in place** | [docs/claude/go2_integration.md](docs/claude/go2_integration.md) |
| **Dual-robot FAR safety stack (2026-04-26): pivot-lock + path_safety_filter + cmd_vel_safety_shield. Why A\* was retired for dual.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-26--dual-robot-far--3-tier-safety-stack) |
| **Nav2 MPPI migration journey (2026-04-29): A\* → FAR → MPPI, brake bug, freewheel, stuck_watchdog, TF/SLAM chain. Two real-robot blockers.** | [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md) |
| **Onboard SLAM split (2026-04-30): Fast-LIO + Livox onto Go2 Jetson Foxy. 5 source patches, the 13-GB-per-node bloat, the `ulimit -v` fix.** | [CLAUDE1.md archive](CLAUDE1.md#archive-2026-04-30--onboard-slam-split-fast-lio--livox-on-jetson) + [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson) |
| **Go2W real Nav2 profile split (2026-05-02): `off` / `omni_2d` / `se2_holonomic`, SmacPlanner2D XY-only finding, no-crab final SE2 tuning.** | This file's "Active state (2026-05-02)" entry |

## Scripts layout

```
scripts/
├── launch/    user-invoked entry points (nav_test_*, door_demo, vlm_demo)
├── bench/     multi-trial PASS-criterion runners + session_reporter
├── runtime/   ROS 2 nodes started by launch files (policy, checkers, supervisors)
├── debug/     observe-a-running-sim tools (far_monitor, vlm_debug_web, …)
├── ops/       one-shot ops & dev utilities (reset_vgraph, test_lidar, sync_to_main)
├── real/      real-robot only (unchanged grouping)
└── common_logging.sh
```

## Quick launch

```bash
# Door task (no flags; VLM-only path)
./scripts/launch/door_demo_mujoco.sh

# Nav stack smoke test
NUM_TRIALS=1 DURATION_SEC=30 OUT_DIR=/tmp/far_bench/smoke ./scripts/bench/benchmark_far_nav.sh

# 5-trial / 10-trial nav benchmark
./scripts/bench/benchmark_far_nav.sh                                # 5 trials default
NUM_TRIALS=10 DURATION_SEC=120 OUT_DIR=/tmp/cfgA_10 ./scripts/bench/benchmark_far_nav.sh

# Fast-LIO + MID-360 benchmark
./scripts/bench/benchmark_fastlio.sh

# Demo2 / LRC maze
./scripts/launch/nav_test_demo2.sh gui:=false
./scripts/launch/nav_test_lrc_maze.sh

# VLM exploration demo (Phase 1) — defaults to nav_execution_backend:=far;
# pass nav_execution_backend:=astar to swap in the C++ A* planner.
./scripts/launch/vlm_demo_mujoco.sh

# Single-robot A* smoke test (MuJoCo + CHAMP + astar_nav_node + Option B)
./scripts/launch/single_astar.sh                                 # headless, demo3 default
./scripts/launch/single_astar.sh robot:=go2w scene:=demo3 gui:=true rviz:=true
./scripts/launch/single_astar.sh session_duration_sec:=120       # bounded run + JSON report

# Heterogeneous dual (Go2W + Go2 share demo3_mixed + CFPA2 coord;
# both DEFAULT to nav2_mppi since 2026-04-29 — production stack)
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far  # both FAR
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_b:=astar                   # mixed: A=mppi, B=astar

# Go2 (non-W) sim — CHAMP locomotion, demo1 12×8 m / demo3 24×16 m
./scripts/launch/nav_test_go2.sh gui:=true rviz:=true          # walk + FAR smoke
./scripts/launch/nav_test_go2_demo3.sh gui:=true rviz:=true    # larger scene
./scripts/bench/benchmark_go2.sh                                # 5-trial PASS check
./scripts/bench/benchmark_go2_demo3.sh                          # same on demo3
# Go2 + real CMU TARE exploration (FAR bypassed, TARE→localPlanner direct)
./scripts/launch/nav_test_go2_tare_real.sh gui:=true rviz:=true
# 10-trial TARE benchmark (10 min each, demo3, ~3.3 h wall-clock)
./scripts/bench/benchmark_go2_tare.sh
# RL policy (experimental, robot saturates — see go2_integration.md)
./scripts/launch/nav_test_go2.sh gui:=true rviz:=true rl_policy:=true

# Real robot (Go2W) — RECOMMENDED entry as of 2026-05-05:
#   SE2 holonomic baked in, oa=false (sport API direct), curated flag surface.
./scripts/real/real_autonomy_se2.sh                                  # default Go2W SE2
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360               # Mid-360 + Fast-LIO
./scripts/real/real_autonomy_se2.sh slam=fastlio_mid360 lidar_range=4.0
./scripts/real/real_autonomy_se2.sh stop                              # kill everything

# Legacy multi-profile launcher (still supported for ad-hoc comparison runs):
./scripts/real/real_autonomy.sh                                       # Cartographer + L1 default
./scripts/real/real_autonomy.sh slam=fastlio_mid360
./scripts/real/real_autonomy.sh oa=false holonomic_profile=off        # diff-drive Reeds-Shepp
./scripts/real/real_autonomy.sh oa=false holonomic_profile=omni_2d    # SmacPlanner2D + MPPI Omni
./scripts/real/real_autonomy.sh oa=false holonomic=true               # legacy alias for omni_2d
./scripts/real/real_autonomy.sh nav=cfpa2                             # Python default_nav.py
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far
# Real robot (Go2, no-wheel) — same stack, walking-gait nav tuning
./scripts/real/real_autonomy_go2.sh                       # nav2_mppi default
./scripts/real/real_autonomy_go2.sh slam=fastlio_mid360 nav=far
# TARE exploration on either robot — stub-based (go2_tare_planner_ros2 over CFPA2+mux)
./scripts/real/real_autonomy.sh robot=go2 nav=tare
# **Real CMU TARE** → localPlanner direct (FAR unwired, watchdog armed).
# oa=false is REQUIRED — default (oa=true) routes Move to /api/obstacles_avoid/request
# which needs manual mode pre-arm; oa=false sends to /api/sport/request (api_id=1008).
./scripts/real/real_autonomy.sh robot=go2 slam=fastlio_mid360 nav=tare_real oa=false
./scripts/real/real_autonomy.sh stop           # kill everything real-robot
```

Debug dashboards:
- Door task: <http://127.0.0.1:8080> (auto-starts)
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
    go2w_control/ go2w_nav/           Locomotion + nav (C++ astar_nav + Python default_nav)
    go2w_perception/ go2w_config/     QoS bridge, configs, sub-launches
    unitree_go2w_ros2/                Unitree ROS 2 integration
  exploration/
    cfpa2_collaborative_autonomy/     CFPA2 frontier allocator (single-robot)
    go2_nav_algorithms/               simple_scan_mapper, frontier detection
  collaborative_exploration/
    door_task/                        Door task package (see door_task.md)
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

Switchable via `nav_backend:=` / `nav_execution_backend:=` at launch time.

| Backend | Planner | Note |
|---|---|---|
| `nav2_mppi` | `nav2_planner` (SmacPlannerHybrid REEDS_SHEPP) + `nav2_controller` (MPPIController) + `nav2_behaviors` + `nav2_bt_navigator` + `nav2_lifecycle_manager` | Production stack for both robots (2026-04-29). Per-platform yaml: [`nav2_go2w_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml) for Go2W, [`nav2_go2_full_stack.yaml`](src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml) for Go2. **Real Go2W now supports overlay profile selection** via `holonomic_profile`: `off` (default), `omni_2d` (`SmacPlanner2D` + MPPI Omni), `se2_holonomic` (`SmacPlannerLattice` + forward/pivot MPPI, no strafe). Outer-loop `stuck_watchdog` per robot. CFPA2 `way_point` is bridged to `goal_pose` via `cfpa2_to_nav2_bridge`. |
| `astar` | `astar_nav_node` | C++ A* + pure-pursuit + Stanley + curvature speed shaping + oriented footprint validation (Option B). **Retired for dual-robot 2026-04-26**, fully superseded by nav2_mppi 2026-04-29; left in place for door-task baseline. |
| `default` | `default_nav.py` | Python A* grid + D* Lite + recovery; legacy stable for real robot + door task |
| `far` | CMU autonomy stack | Terrain analysis + FAR V-graph + path follower (see nav_benchmarks.md) |

Legacy aliases silently upgrade: `reactive` → `default`, `rrt_star` / `far_rrt_star` / `mppi` → `astar`. The reactive RRT* planner (`reactive_nav_node`) and MPPI (`mppi_nav_node`) were deleted 2026-04-24 once A* had matched their capabilities. Door task uses `astar` with `astar_nav_door.yaml` (aggressive obstacle thresholds for bumper contact).

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
