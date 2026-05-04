# CLAUDE1.md

## Project Overview

Multi-robot autonomous exploration with Unitree Go2W wheeled-legged quadrupeds. Single-robot configuration. ROS 2 Humble, **MuJoCo** simulation (primary) + Gazebo Classic (legacy), real-robot deployment.

**Default demo stack:** xAI Grok-4-1-fast-non-reasoning VLM, Cartographer 2D SLAM, RRT*-based local planner, CFPA2 frontier explorer, CHAMP locomotion controller, DFKI mujoco_ros2_control.

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

YAML and Python changes take effect immediately (symlink-install). C++ changes require rebuild.

## Running

```bash
# VLM demo (MuJoCo) — primary entry point
./scripts/vlm_demo_mujoco.sh

# Override nav backend
./scripts/vlm_demo_mujoco.sh nav_execution_backend:=far

# VLM demo (Gazebo Classic, legacy)
./scripts/vlm_demo.sh

# LRC course variant
./scripts/vlm_demo_lrc.sh

# Kill stale processes before re-launch
pkill -9 -f mujoco_ros2_control; pkill -9 -f mujoco_sensor; killall -9 rviz2

# Headless (faster)
# Pass gui:=false rviz:=false to launch files
```

VLM history logs: `~/.ros/log/vlm_history/<run_timestamp>/`
Live debug viewer: http://localhost:8501 (starts automatically)

## Repository Layout

```
src/
  go2w/                # Go2W platform packages
    go2_gazebo_sim/      # Gazebo/MuJoCo worlds, robot spawning, CHAMP configs
      mujoco/              # MJCF models, STL meshes, MuJoCo controller YAML
    mujoco_sensor_bridge/  # MuJoCo sensor nodes (LiDAR raycast, contact, odom)
    go2w_control/        # Startup gates, hybrid motion cmd router
    go2w_nav/            # reactive_nav (RRT*) + default_nav (A*)
    go2w_perception/     # QoS bridge, carto_odom_bridge, pointcloud adapter
    go2w_safety/         # Safety monitors
    go2w_config/         # Shared YAML configs + sub-launch files
    go2w_spawn/          # Robot spawning utilities
    go2w_observability/  # Diagnostics
    unitree_go2w_ros2/   # Unitree ROS 2 integration (description, driver, bringup)
  exploration/         # Exploration algorithms
    cfpa2_collaborative_autonomy/  # CFPA2 coordinator (single-robot)
    go2_nav_algorithms/            # Scan mapper, frontier detection
  vendor/              # Third-party
    fast_lio/            # Fast-LIO2 SLAM
    autonomy_stack_go2/  # CMU autonomy stack (FAR planner, terrain analysis)
    mujoco_ros2_control/ # DFKI MuJoCo ros2_control hardware interface
  vlm_explorer/        # VLM-in-the-loop exploration
scripts/               # Launch scripts, logging utilities, dev tools
config/                # Workspace-level DDS config (fastdds_no_shm.xml)
docs/                  # Research proposals, architecture diagrams
```

## Key Packages

| Package | Type | Purpose |
|---|---|---|
| `vlm_explorer` | Python | VLM coordinator, artifact detectors (YOLO, Florence2, red block), map renderer, skeleton extractor |
| `cfpa2_collaborative_autonomy` | Python | CFPA2 frontier exploration (single-robot) |
| `go2_nav_algorithms` | C++/Python | `simple_scan_mapper_cpp` (2D occupancy grid), frontier detection |
| `go2w_nav` | C++/Python | `reactive_nav_node` (RRT*), `default_nav.py` (A* + local avoidance) |
| `go2_gazebo_sim` | C++/Launch | Gazebo/MuJoCo worlds, robot spawning, CHAMP gait/joint/link configs, MJCF models |
| `mujoco_sensor_bridge` | Python | MuJoCo sensor nodes: LiDAR raycasting, foot contact, ground-truth odometry |
| `mujoco_ros2_control` | C++ | DFKI MuJoCo ros2_control hardware interface (vendor) |
| `go2w_perception` | C++/Python | QoS bridge, Cartographer odom bridge, pointcloud adapter |
| `go2w_control` | Python | SLAM startup gate, SLAM health guard, waypoint readiness gate |
| `fast_lio` | C++ | Fast-LIO2 SLAM (vendor submodule) |

## Navigation Backends

Switchable via `nav_backend:=` or `nav_execution_backend:=` at launch time:

| Backend | Planner | Description |
|---|---|---|
| `rrt_star` / `reactive` | `reactive_nav_node` | **Default.** RRT*-based sampling planner with fast replanning |
| `default` | `default_nav.py` | A* global on occupancy grid + scan-based local avoidance |
| `far` | CMU autonomy stack | Terrain analysis + FAR global planner + path follower |
| `far_rrt_star` | FAR global + RRT* local | FAR route waypoints fed to RRT* local planner |

## Architecture

### MuJoCo Backend (primary)

```
mujoco_ros2_control (DFKI)    ← MJCF scene (robot + world)
  ├── /clock                   (sim time)
  ├── joint_states             (ros2_control)
  ├── ros2_control HW iface   (MujocoSystem)
  └── built-in LiDAR sensor   → /{ns}/registered_scan (PointCloud2, 11 Hz, mj_multiRay × 21600)

mujoco_sensor_bridge:
  mujoco_contact_node → /{ns}/foot_contacts   (ContactsStamped, 50 Hz)
  mujoco_odom_bridge  → /{ns}/odom/ground_truth (50 Hz, TF disabled when Cartographer active)
```

### Shared Perception/Nav Pipeline (same for MuJoCo and Gazebo)

```
registered_scan → [qos_bridge] → registered_scan_reliable
                                          │
                    ┌─────────────────────┤
                    ▼                     ▼
            pointcloud_adapter     pointcloud_to_laserscan
            (for Fast-LIO)         → scan_3d (LaserScan)
                                          │
                      ┌───────────────────┤
                      ▼                   ▼
            simple_scan_mapper_cpp  reactive_nav / default_nav
            → /{ns}/map (OccGrid)  ← /{ns}/map (planning)
                                   ← scan_3d (local avoid)
                                   → cmd_vel_stamped
                                          │
                                          ▼
                                    CHAMP controller
                                    → joint commands

Cartographer 2D: scan_3d → cartographer_node → TF (map→base_link)
                                              → submap_list
                 carto_odom_bridge → odom/nav (from Cartographer TF)
                 probability_grid_binarizer → binary occupancy grid

CFPA2: /{ns}/map → frontier detection → goal assignment → nav waypoint
VLM:   map + camera → vlm_coordinator → exploration goal override
```

### Key Nodes

- **`mujoco_ros2_control`**: DFKI MuJoCo physics sim with ros2_control hardware interface. Publishes `/clock`. Includes built-in LiDAR sensor (C++ `mj_multiRay`, 360x60=21600 rays at 11 Hz).
- **`mujoco_lidar_node`**: **Legacy/unused.** External Python raycaster, superseded by built-in C++ LiDAR in `mujoco_ros2_control`.
- **`mujoco_contact_node`**: Reads MJCF touch sensors for foot contacts at 50 Hz.
- **`mujoco_odom_bridge`**: Ground-truth odometry from MuJoCo body state at 50 Hz.
- **`simple_scan_mapper_cpp`**: Builds 2D occupancy grid from laser scan + TF. Each scan painted ONCE.
- **`reactive_nav_node`**: RRT*-based local planner with fast replanning. Default nav backend.
- **`default_nav`**: A* global planner on map + scan-based local obstacle avoidance. Background A* thread.
- **`vlm_coordinator_node`**: Queries VLM (Grok) with rendered map + camera images, decides exploration goals.
- **`cfpa2_single_robot_node`**: Single-robot CFPA2 frontier exploration.
- **`carto_odom_bridge`**: Converts Cartographer TF output to Odometry messages for nav.

## Communication Style

- Be fast and direct. Short questions expect immediate, precise answers.
- Show work but don't narrate it. Run commands, make changes, report what happened. Don't ask for permission unless the action is destructive.
- When told "still not working", don't repeat the same fix — go deeper.
- Respect the maintainer's hypotheses. When they say "I AM certain it's due to X", treat that as strong signal. Validate or disprove with evidence, not conjecture.

## Codebase Rules

1. **Always `use_sim_time: true`** for all nodes in simulation (MuJoCo or Gazebo). Mixed time domains corrupt maps.
2. **Never use stale TF fallback.** Drop the scan on TF failure; don't use `tf2::TimePointZero`.
3. **Each scan painted exactly once.** Clear `last_scan_` after processing in mappers.
4. **Dual-robot TF must be namespaced.** Remap `/tf` -> `/{ns}/tf` for all nodes.
5. **DDS config matters.** `config/fastdds_no_shm.xml` disables shared memory for reliability. Real robot uses CycloneDDS with unicast.
6. **Verify with `ros2 topic hz`** after changing sensor rates. Xacro changes require model re-spawn.

## Debugging

1. **Check prior art first.** Search git history.
2. **Trace the full data pipeline:** What generates the data? What timestamp domain? What transforms it? What consumes it?
3. **Add diagnostic logging with actual values** (yaw, dt, timestamps). Use `RCLCPP_INFO_THROTTLE` or Python logger with rate limiting.
4. **Measure, don't assume.** Use `ros2 topic hz`, `ros2 topic echo`, and grep logs.

### Common Root Causes

| Symptom | Likely Cause | Where to Look |
|---|---|---|
| Map flickering / doubled walls | Same scan painted multiple times | `simple_scan_mapper_cpp.cpp` update timer vs scan arrival rate |
| Map starburst / rotated structures | TF timestamp mismatch or stale TF | TF lookup code, `ExtrapolationException` handlers |
| "No Effective Points!" in Fast-LIO | Wrong timestamp span | `pointcloud_adapter.py` time_offset calculation |
| Robot walks through walls | Planner not using occupancy grid | Check planner is reading `/{ns}/map` |
| Scattered occupied cells | Ground/leg hits passing height filter | `pointcloud_to_laserscan` `min_height` parameter |
| Robot position stuck on map (scans pile at origin) | Cartographer scan matcher weights too high for IMU-only prediction | `cartographer_sim_2d.lua` `translation_delta_cost_weight`, `ceres translation_weight` |
| `malloc_consolidate` crash in mujoco_ros2_control | Concurrent `mj_step` + `mj_multiRay` on different threads | `sim_step_mtx_` must protect both calls; see lidar_sensor.cpp |

## MuJoCo Built-in LiDAR Sensor

The LiDAR was moved from an external Python node (`mujoco_lidar_node_multiray`) into the DFKI `mujoco_ros2_control` C++ plugin to eliminate sensor sync lag. Key files:

| File | Purpose |
|---|---|
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_sensors/lidar_sensor.hpp` | LidarSensor class + LidarSensorConfig struct |
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/lidar_sensor.cpp` | Raycast implementation: `mj_multiRay()` on live `mjData*` |
| `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_sensors.cpp` | Auto-discovers `unitree_l1` or `livox_mid360` MJCF sites |
| `src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml` | MJCF scene with `unitree_l1` site definition |

**How it works:** The sensor shares `mjModel*/mjData*` with the physics sim. On a wall timer (11 Hz), it reads `site_xpos`/`site_xmat` from live sim data, rotates pre-computed ray directions to world frame, calls `mj_multiRay()` for all 21,600 rays in one C call, transforms hits to LiDAR-local frame, and publishes `PointCloud2` on `registered_scan` (BEST_EFFORT QoS).

**Thread safety:** `mj_multiRay` traverses the BVH tree and is NOT safe to call concurrently with `mj_step`. A `sim_step_mtx_` mutex is shared between the physics loop and the lidar sensor. The physics loop locks it around `mj_step()`, the lidar locks it around data reads + `mj_multiRay()`.

**Site auto-discovery:** Checks for `unitree_l1` first (360x60 rays, 0-90 vertical, 11 Hz), then `livox_mid360` (720x16, -7 to +52 vertical, 10 Hz). Only one LiDAR is registered.

## Cartographer TF Configuration (resolved 2026-04-06)

**Problem:** Robot position stuck near origin on Cartographer map. Scans accumulated at one position.

**Root cause:** `cartographer_sim_2d.lua` had `published_frame = "odom"` + `provide_odom_frame = false`, making Cartographer depend on an external `odom → base_link` TF. Additionally, `test.sh` was missing `FASTRTPS_DEFAULT_PROFILES_FILE`, causing DDS shared memory issues (stale data between runs, broken inter-node communication).

**Fix (odom-free):** Cartographer publishes `map → base_link` directly, no odom frame in the TF tree.
- `cartographer_sim_2d.lua`: `published_frame = "base_link"`, `provide_odom_frame = false`, `use_odometry = false` → pure LiDAR+IMU scan matching.
- `nav_test_mujoco.launch.py` and `single_vlm_mujoco_far.launch.py`: `odom_bridge_publish_tf = false`, removed Cartographer odom topic remap.
- `test.sh`: Added `FASTRTPS_DEFAULT_PROFILES_FILE` export and `/dev/shm/fastrtps_*` cleanup.

**TF tree:** `map → base_link → imu, livox_mid360, ...` (Cartographer + robot_state_publisher). `carto_odom_bridge` converts `map → base_link` TF to `/robot/odom/nav` Odometry topic for the nav stack.

**Note:** `provide_odom_frame = true` does NOT work odom-free — Cartographer's `tf_bridge.cpp` tries to look up the `odom` frame before it can publish it, causing a deadlock. Use `provide_odom_frame = false` with `published_frame = "base_link"` instead.

## Important Config Files

| Config | Purpose |
|---|---|
| `src/vlm_explorer/config/vlm_explorer.yaml` | VLM node params (model, prompts, thresholds) |
| `src/vlm_explorer/config/cartographer_sim_2d.lua` | Cartographer 2D SLAM tuning |
| `src/go2w/go2w_config/config/nav/reactive_nav_vlm.yaml` | RRT* nav params for VLM demo |
| `src/go2w/go2w_config/config/nav/simple_scan_mapper_single_go2w.yaml` | Occupancy grid scoring |
| `src/exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml` | Single-robot CFPA2 params |
| `src/exploration/cfpa2_collaborative_autonomy/config/cfpa2_coordinator.yaml` | Dual-robot CFPA2 coordinator |
| `src/go2w/go2_gazebo_sim/config/champ/go2w/` | CHAMP gait, joints, links |
| `src/go2w/go2_gazebo_sim/mujoco/go2w.xml` | Go2W MJCF robot model (standalone) |
| `src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml` | Combined VLM world + robot MJCF scene |
| `src/go2w/go2_gazebo_sim/mujoco/go2w_mujoco_controllers.yaml` | ros2_control config for MuJoCo |
| `config/fastdds_no_shm.xml` | FastDDS shared memory disabled |

## Environment

- **Python**: 3.10 (via micromamba `cmu_env`)
- **ROS 2**: Humble (`/opt/ros/humble/`)
- **Build**: colcon + ament_cmake (C++) / ament_python (Python)
- **DDS**: FastDDS (sim), CycloneDDS (real robot)
- **Sim**: MuJoCo 3.6.0 (pip, `~/.local/lib/python3.10/site-packages/mujoco/`) + DFKI mujoco_ros2_control
- **VLM API**: xAI (Grok-4-1-fast-non-reasoning), key in `.env.xai`

## Phase 3 Archive — Notes moved from CLAUDE.md (2026-05-02)

Detailed 2026-04 operational narratives were moved here to keep `CLAUDE.md`
as a short index/current-state file while preserving the technical memory.

### Archive 2026-04-30 — Onboard SLAM split (Fast-LIO + Livox on Jetson)

- Objective: offload SLAM from laptop to Jetson (`192.168.123.18`) to reduce
  MPPI planning latency and runtime contention.
- Cross-host Humble↔Foxy DDS worked, but required strict config discipline
  (`ROS_DOMAIN_ID`, XML schema differences, interface binding).
- Foxy compatibility tax encountered in practice:
  - source-level patches in Livox/Fast-LIO codepaths,
  - `static_transform_publisher` CLI syntax differences,
  - Foxy parameter parsing differences (empty-string overrides),
  - broken Foxy `rclpy` on this target (worked around via C++ remaps).
- Main blocker was extreme per-process memory growth (~13 GB RSS / node);
  root cause traced to speculative large virtual mapping.
- Session-saving fix: `ulimit -v 1500000` forced bounded fallback allocator
  behavior; stack dropped to low hundreds of MB total and became stable.
- Canonical runbook: [docs/claude/real_robot.md](docs/claude/real_robot.md#onboard-slam-split-shipped-2026-04-30--fast-lio--livox-on-jetson).

### Archive 2026-04-29 — Dual-robot Nav2 MPPI migration

- A* and stacked FAR safety patches were no longer sufficient for the narrow
  dual-robot geometry; migrated to Nav2 MPPI + SmacHybrid as production path.
- Key correctness fixes:
  - MPPI footprint trap resolved (`consider_footprint: true` + polygon footprint),
  - vendored `mujoco_ros2_control` brake caching bug fixed,
  - legged-mode freewheel policy added to avoid wheel drag torque.
- Added outer-loop recovery (`stuck_watchdog`) for planner/controller states
  that report success while outputting near-zero motion.
- Unified odom/TF data path with `fast_lio_tf_adapter`; removed sim-only TF
  privilege from `mujoco_odom_bridge` to align sim and real behavior.
- Residual strategic gap remained loop-closure correction (SC-PGO ROS1→ROS2
  porting), tracked separately.
- Deep technical chronology: [docs/claude/nav2_mppi_journey.md](docs/claude/nav2_mppi_journey.md).

### Archive 2026-04-26 — Dual-robot FAR + 3-tier safety stack

- A* retired for dual-robot because 2D footprint reasoning could not reliably
  capture real contact envelopes in constrained mixed geometry.
- FAR became base planner with three runtime safety layers:
  `CFPA2 pivot-lock`, `path_safety_filter`, `cmd_vel_safety_shield`.
- Critical lesson: TF remaps were mandatory; missing remaps can silently make
  safety layers inert (appearing healthy but effectively passthrough).
- Tuning stabilized behavior but introduced deadlock risks across layers;
  timeout/escape valves were identified as required design elements.
- This phase documented the transition rationale that directly enabled the
  later Nav2 MPPI migration.
- Related docs: [docs/claude/nav_benchmarks.md](docs/claude/nav_benchmarks.md), [docs/claude/debug_notes.md](docs/claude/debug_notes.md).

---

# Phase 2 Archive — Door Task FSM Era (deleted 2026-04-14)

Historical record of the FSM-based dual-robot door task that preceded the current VLM controller. Code removed; kept here for archaeology. See `docs/claude/door_task_history.md` for the lessons that carried forward.

## FSM package (deleted)

`src/collaborative_exploration/door_task/` — the package still exists but `legacy_fsm/` subtree was removed. The FSM files were:

| File | Purpose |
|---|---|
| `door_monitor_node.py` | Extracted door hinge angle from MuJoCo pose sensor → `/door_task/door_state` (kept — still used by VLM path) |
| `fsm_executor_node.py` | Per-robot FSM executor, subscribed to `/door_task/fsm_plan`, executed skills |
| `skill_primitives.py` | MOVE_TO, PUSH, HOLD_POSITION, WAIT_UNTIL, SIGNAL, RETREAT, PUSH_TO, DRIVE_THROUGH |
| `door_task_coordinator.py` | Generated FSM plans (VLM or static fallback), monitored completion |
| `prompting_door.py` | Door-task VLM system/user prompts |
| `fsm_validator.py` | Syntactic FSM validation |

Launch was `./scripts/door_demo_mujoco.sh` with `use_static_fsm:=true/false`. Nav config was `src/go2w/go2w_config/config/nav/reactive_nav_door.yaml` — aggressive obstacle thresholds so robot drove close for bumper contact.

## FSM sensor-to-actuator pipeline

```
MuJoCo physics (500 Hz)
  → mujoco_ros2_control (ros2_control HW interface)
  → mujoco_odom_bridge (50 Hz) → /{ns}/odom/ground_truth
  → mujoco_contact_node (50 Hz) → /{ns}/foot_contacts
  → built-in LiDAR (11 Hz) → /{ns}/registered_scan
     → qos_bridge → pointcloud_to_laserscan → scan_3d
        → simple_scan_mapper_cpp (4 Hz) → /{ns}/map
        → reactive_nav_node (12 Hz, RRT*) → cmd_vel_stamped
           → twist_bridge → cmd_vel
              → go2w_hybrid_cmd_router → cmd_vel_legged
                 → CHAMP quadruped_controller (200 Hz IK)
                    → ros2_control effort controller
                       → MuJoCo joint actuators
```

FSM executor published `PointStamped` on `/{ns}/way_point` which reactive_nav used as goal.

## PUSH skill evolution

1. **v1 (open-loop waypoint)**: Published fixed waypoints. Worked for lightweight door, no feedback.
2. **v2 (direct cmd_vel)**: Disabled reactive_nav, sent raw cmd_vel. Robot barely pushed — cmd_vel at 12 Hz from reactive_nav overwrote direct commands even when "stopped".
3. **v3 (reactive_nav + contact feedback)**: Kept reactive_nav running. Phase 1 approached via waypoint; on contact detection (near door + low actual_vx, or door starts moving), advanced waypoint past door. reactive_nav drove robot through. Monitored bounce-back via peak door angle tracking.

**Key insight (preserved)**: reactive_nav IS the force generator. Disabling it and sending raw cmd_vel produces less force because CHAMP's IK loop works best with the full nav→twist pipeline.

## HOLD_POSITION — dual mode

- Waypoint mode (hold_x > 0): reactive_nav running, waypoint past hold position for continuous drive
- Direct mode (no hold_x): reactive_nav disabled, raw cmd_vel forward pressure (for swing-side hold where no obstacle in the way)

## Contact detection

Inferred from odom twist: if commanding forward but actual_vx ~ 0, robot is in contact with something. No force sensor needed.

## FSM fundamental problems

The FSM abstraction was too coarse for physical coordination:

1. **Not Markovian** — occupancy map carries history (stale cells from closed door); door momentum matters; robot-door contact state not captured.
2. **Not deterministic** — door bounces back after threshold; nav planner may/may-not find path through opening; push force varies with contact angle.
3. **Recovery too slow** — FSM's only recovery was timeout → fail → replan (10 s VLM call); door closes in 3 s.
4. **reactive_nav contradiction** — obstacle avoidance planner asked to push INTO an obstacle. Aggressive door config (`obstacle_stop_dist: 0.05`) was a hack.
5. **Occupancy map persistence** — `simple_scan_mapper` marked cells occupied with no decay; when door moved, old cells remained blocking path planning through opening.

## FSM redesign — collision-safe strategy (2026-04-07)

Original 8-state FSM (A pushes → B holds → A retreats → A passes through) had a fundamental collision problem. Tried y-lane offsets (0.5 m, 0.7 m) but RRT* steered off intended lane.

Pivoted to 7-state "obstacle scenario":

1. `S0_B_POSITION`: B navigates above door-blocking crate to (4.5, 2.8)
2. `S1_B_PUSH_CRATE`: B uses PUSH_TO to drive crate from (4.5, 2.0) to (4.5, 0.5), clearing door swing arc
3. `S2_B_CLEAR`: B retreats to far corner (7.0, 2.0) — >2 m from doorway
4. `S3_A_APPROACH`: A approaches door via MOVE_TO
5. `S4_A_PUSH`: A pushes with `push_through_x=6.0`, `door_open_threshold=1.22` (70°)
6. `S5_A_ENTER`: A continues into Room B
7. `S_DONE`: Both in Room B

B's role shifted from "hold door" (collision-prone) to "clear obstacles" (spatially separated). MuJoCo scene added a 5 kg `door_blocker` body (free-body crate) blocking the door's swing arc at ~26°. Required B to push clear before A could open.

## Phase 2 verified results (2026-04-07)

All 3 criteria PASS with obstacle scenario:
- CRIT 1: A at (4.62, 2.01), B at (6.89, 1.99) — both Room B
- CRIT 2: Door peak 81.3°
- CRIT 3: 0 inter-robot contacts

## Phase 2 occupancy map fixes (carried forward)

These fixes are still in `simple_scan_mapper_cpp` although the VLM path no longer uses it for the door task:

1. **Score decay**: timer every 2 s decrements positive scores (`decay_interval_sec: 2.0`, `decay_amount: 1`)
2. **Raytrace threshold raised**: `raytrace_free()` wall-stop changed from `occupied_score_threshold_` to `score_max_` so free-space rays punch through partially-occupied cells
3. **Door corridor exemption zone**: rectangular area (x=[3.5,4.8], y=[1.4,2.6]) forced free in `publish_map()`. Prevents RRT* sideways routing around stale door cells (`exempt_x_min/max`, `exempt_y_min/max` params)

## Phase 2 skill primitive improvements

1. **PUSH angular velocity tracking**: sliding window of 5 samples for dθ/dt; if door closing (ω < -0.1), advance waypoint +0.8 instead of +0.5 — reacted in ~100 ms instead of waiting 0.15 rad decline from peak.
2. **B door-angle triggers over peer signals**: B reacted to `door_open(0.15)` directly instead of waiting for `signal(door_open_come_hold)`, gaining 1-2 s.
3. **PUSH_TO**: heading-controlled drive to any (x, y). Phase 1 aligned heading (proportional), phase 2 drove forward with heading correction. Bypassed reactive_nav. Used for obstacle clearing.
4. **DRIVE_THROUGH**: forward drive with PD y-correction. Kept robot locked on target y-lane, bypassing reactive_nav's RRT* which steered toward doorway center.

## Deleted launch flags

`use_static_fsm:=true/false` — controlled whether FSM plans came from a hardcoded template or from a VLM call. Removed when FSM was deleted; VLM path is now unconditional.

## Architectural lessons that carried forward

- **Fast/slow decoupling wins** — 500 Hz physics / 10 Hz control / 1 Hz executer / 6 s planner.
- **Perception ≠ planning** — LLM should not be sole perception; hallucinates, too slow.
- **Dataclasses at interface boundaries** — typed Action, Plan, Observation, WorldMemory.
- **Prompts are code** — version-controllable markdown, loaded at startup.
- **Config single-source-of-truth** — still TODO (`button_xy` in 3 files).
- **Physics-based success checker** (`door_task_checker.py`) — loads MJCF independently, syncs state via ROS, calls `mj_forward()`. Still shipped alongside VLM path.
