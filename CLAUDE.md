# CLAUDE.md — Collab_QRC Index

Multi-robot autonomy with Unitree Go2W wheeled-legged quadrupeds on ROS 2 Humble + MuJoCo (primary) / Gazebo Classic (legacy) / real-robot deployment. Two active threads: **dual-robot door task** (VLM-driven coordination) and **single-robot nav benchmarking** (CMU stack tuning).

For Phase 1 (single-robot VLM exploration) background and Phase 2 FSM-era archive, see [CLAUDE1.md](CLAUDE1.md).

## Active state (2026-04-24)

- **Nav planner cleanup + A\* planner shipped (2026-04-24)** — deleted `reactive_nav_node` (RRT\*) and `mppi_nav_node` after A* matched their capability on demo3. Three nav backends remain: **`astar`** (new C++ A* + pure-pursuit + Stanley + curvature speed shaping + Option B oriented footprint validation), `default` (Python A*/D* Lite, real-robot + door task baseline), `far` (CMU stack). Door task migrated reactive_nav_door.yaml → astar_nav_door.yaml. Heterogeneous dual launch now supports `nav_backend_a:=astar nav_backend_b:=far` — hybrid_cmd_router's `wheel_command_topic` is absolute `/mujoco_sim/{ns}_wheel_velocity_controller/commands` (latent mixed-launch bug: relative path under `/{ns}` never reached the controller under `/mujoco_sim/controller_manager` — FAR masked it because cmd_vel was too smooth to trigger wheel mode; A* exposed it). Back-compat aliases in every launch: `reactive→default`, `rrt_star/far_rrt_star/mppi→astar`. → [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md#a-star-planner-2026-04-24) | [docs/claude/door_task.md](docs/claude/door_task.md)
- **Door task** — Phase 3 VLM controller + Phase 0 refactor shipped. Analytical door-lock barrier verified end-to-end 2026-04-14. Button-gated collaborative protocol working. Next: re-capture 5-trial benchmark with full fix stack (now on astar backend). → [docs/claude/door_task.md](docs/claude/door_task.md)
- **Nav benchmarking** — Config A = 7/10 FULL PASS on demo1 (12×8 m). Fast-LIO2 + SC-PGO integrated, fixes scan-odom temporal lag; contact margins now the bottleneck. Next: stuck detector for corner-wedge 1/10 failure mode. → [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md)
- **Go2 (non-W) integration** — **shipped** under CHAMP on `demo1_go2_real.xml` / `demo3_go2_real.xml` (Menagerie body). Exploration planner swapped from CFPA2 → **real CMU TARE** (vendored from `caochao39/tare_planner` humble-jazzy at `src/vendor/tare_planner/`). TARE feeds `localPlanner` **directly, bypassing FAR** — FAR's V-graph can't route to exploration frontiers; CMU's own TARE pipeline also skips FAR. Sensor-derived **waypoint watchdog** (2026-04-21) publishes to `/{ns}/nogo_boundary` as a persistent blacklist whenever TARE picks a goal the stack can't reach (4 fault modes: terrain-cluster, occgrid-occupied, out-of-grid, progress-stall); also republishes RViz markers the FAR-branch rviz expects. **Real-robot port shipped** as [`real_single_tare_real.launch.py`](src/go2w/go2w_real_bringup/launch/real_single_tare_real.launch.py) (`nav=tare_real`), plus `obstacle_avoidance:=false` gotcha: default `true` routes Move to `/api/obstacles_avoid/request` which requires pre-arming; `oa=false` routes to `/api/sport/request` (api_id=1008), no manual mode switch needed. **10-trial × 10-min demo3 bench** (2026-04-21): 10/10 completed, 10/10 zero contacts, 71 % avg coverage (σ = 5.4 %), 0/10 passed the 90 % bar — stack is robust; 10 min is a tight budget for 384 m² given CHAMP's 0.3 m/s cap × MuJoCo RTF ≈ 0.5. → [docs/claude/go2_integration.md](docs/claude/go2_integration.md)
- **Real-robot Fast-LIO path — shipped after 10-layer bug hunt (2026-04-17)**. Livox Mid-360 auto-detect at `192.168.123.20`, `livox_ros_driver2` + `Livox-SDK2` vendored (workspace-local install, no `/usr/local` pollution). Red TRIANGLE_LIST robot-pose marker, dual RViz (2D top-down + 3D voxel orbit), supervisor-panic any-button override, dry-run mode. Map **expands correctly on flat and ramped terrain**; `/robot/map` via octomap_server RANSAC ground filter, 3D voxel grid via `/robot/octomap_point_cloud_centers`. Mid-360 mount tilt (measured +15.1° pitch / -2.1° roll) compensated via two static TFs; gravity-aligned map frame. → [docs/claude/real_robot.md](docs/claude/real_robot.md#bug-chain-2026-04-17-map-doesnt-expand)

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

# Heterogeneous dual (Go2W + Go2 share demo3_mixed + CFPA2 coord; both default A*)
./scripts/launch/nav_test_demo3_mixed.sh gui:=true rviz:=true
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_a:=far nav_backend_b:=far  # both FAR
./scripts/launch/nav_test_demo3_mixed.sh nav_backend_b:=far                     # mixed: A=astar, B=FAR

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

# Real robot (Go2W) — Ethernet, Cartographer + L1 LiDAR, CFPA2 (defaults)
./scripts/real/real_autonomy.sh
# Real robot (Go2W) — Livox Mid-360 + Fast-LIO2, FAR nav
./scripts/real/real_autonomy.sh slam=fastlio_mid360 nav=far
# Real robot (Go2, no-wheel) — same stack, walking-gait nav tuning
./scripts/real/real_autonomy_go2.sh
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
| `astar` | `astar_nav_node` | C++ A* + pure-pursuit + Stanley + curvature speed shaping + oriented footprint validation (Option B) |
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
