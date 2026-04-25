# Door Task — Current Architecture

Dual-robot (Go2W) collaborative door task, Phase 3 VLM controller with Phase 0 refactor applied. Target: ICRA 2027 (T1: Door Wedge & Pass-Through).

## Scenario

Two robots in separate rooms divided by a UK FD30 fire door. A button pressure pad in Room B unlocks the door. Goal: both robots end up in the same room. A single Go2W can push the FD30 door once unlocked (the original "B holds, A pushes" plan was abandoned — see `door_task_history.md`).

## MuJoCo scene

**File:** `src/go2w/go2_gazebo_sim/mujoco/two_rooms_door_scene.xml`

- 8 m × 4 m space, wall at x=4 with 1 m door opening (y ∈ [1.5, 2.5])
- Room A (push side): x ∈ [0, 4]; Room B (swing side): x ∈ [4, 8]
- Robot A spawns (2.0, 2.0, 0.6); Robot B spawns (6.0, 2.0, 0.6)
- Door hinge at (4.0, 2.5), swings into Room B, range [0, π/2], FD30 physical params (30 kg, k=4.5 Nm/rad, c=3.0 Nm·s/rad)
- Button pressure pad: red cylinder site in Room B (see `button_monitor_node` for location)
- **Door barrier body** (`door_barrier`) — analytical hard lock, see below

## Packages

**Primary:** `src/collaborative_exploration/door_task/` — Python pkg, 14-line shim entry point.

Current tree after Phase 0 refactor:
```
door_task/
  core/          — pure logic: actions, memory, rendering, control math, geometry
  perception/    — YOLO detector, IoU tracker, CLIP inspector, WorldDict
  llm/           — backend protocol, xAI + mock impl, JSON parser
  prompts/       — planner.md, executer.md, loader, user_prompt builder
  ros/           — controller.py (was 1449-line god file, now ~560 lines),
                   perception_node.py, button_monitor_node, door_lock_from_button_node
```

`legacy_fsm/` was deleted 2026-04-14 (archived in CLAUDE1.md Phase 2 archive).

## Active nodes

| Node | Purpose |
|---|---|
| `vlm_controller_node` | Multi-agent brain: 6 s planner + 1 Hz executer + 10 Hz heading loop |
| `perception_node` | 5 Hz YOLO + CLIP inspector → `/perception/world_dict` |
| `door_monitor_node` | MuJoCo pose sensor → door angle on `/door_task/door_state` |
| `button_monitor_node` | Odom-based proximity check → `/door_task/button_pressed` |
| `door_lock_from_button_node` | Toggles door barrier on button press (BEST_EFFORT QoS) |

## Launch

```bash
cd ~/Collab_QRC
./scripts/launch/door_demo_mujoco.sh        # no flags needed, VLM path is the only path
```

Debug dashboard auto-starts at <http://127.0.0.1:8080> (subscribes to `/vlm_debug/state`, `/perception/world_dict`, `/perception/debug_image`; 2x2 camera composite + planner/executer panels + world_dict table).

## Analytical door lock

`door_hinge` is pure FD30 spring (no position actuator). Locking comes from a second body `door_barrier` with a slide joint on z, driven by a position actuator `door_barrier_lock` (kp=20000, kv=0). `qpos=0` parks a 0.08×1.0×2.0 m box filling the doorway; `qpos=3` lifts it above the wall.

`door_lock_from_button_node` publishes `0.0` (locked) or `3.0` (retracted) on `/door_assist_controller/commands` with `QoSProfile(reliability=BEST_EFFORT)` to match the controller's subscriber QoS.

**Five-bug chain history** lives in `door_task_history.md` — critical reading if touching the lock mechanism.

## VLM architecture (Method B)

VLM outputs strategy-level actions, not raw velocities:
- `{"mode": "drive", "tx": ..., "ty": ..., "vx_max": ...}`
- `{"mode": "drive_relative", "forward_m": ..., "heading_deg": ...}`

A local 10 Hz heading P-controller in `controller.py` converts those into `cmd_vel_legged`. Killed yaw oscillation. Went from 0/5 → 3/5 PASS overnight after the switch.

Two agents with separate system prompts (in `prompts/planner.md` + `prompts/executer.md`):
- **Planner** (6 s period) — emits strategy + `world_memory` pillar/door/other-robot entries
- **Executer** (1 Hz) — emits per-tick `cmd`, tracks the plan

## Perception stack (Phase 1a + 1b)

```
Camera @ 5 Hz → YOLOv8n (class-agnostic) → IoU tracker (Phase 1a)
            → crops → CLIP ViT-B/32 scores open-vocab queries
            → temporal softmax pool over last 5 frames per track (Phase 1b)
            → depth back-projection (5×5 median patch, NaN/0/>20 m filtered)
            → WorldDict (nearest-neighbor association + EMA + decay)
            → /perception/world_dict (JSON dict)
```

Settings:
- `min_confidence=0.55` CLIP score threshold (tuned after wall-hit ghost at (7.3, 4.5))
- **Scene-bounds drop**: any world_xy outside [0,8]×[0,4] silently discarded in perception_node
- Planner rule: `pillar.known=True` ⟺ `semantic_label ∈ {"red button", "red pressure pad"} ∧ conf ≥ 0.55 ∧ hits ≥ 3`
- CLIP latency: 0.3 s first call, 0.016 s cached (CUDA)

## Testing

```bash
PYTHONPATH=. /usr/bin/python3 -m pytest tests/ -q
```

44 unit tests covering actions, control, memory, parser, rendering.

## Physics-based success checker

**File:** `scripts/debug/door_task_checker.py`

Loads the MJCF independently, syncs full sim state via ROS topics (odom → free joints, joint_states → actuated, door_state → hinge), calls `mj_forward()` at 20 Hz, reads `mjData.contact` for geom-level inter-robot collision detection.

Three PASS criteria:
1. Both robots in same room for 3+ s (both x<3.5 or both x>4.5)
2. Door hinge exceeded 70° (1.2217 rad) at any point
3. Zero inter-robot body collisions (geom ownership map built from MJCF kinematic tree)

Run alongside demo via `scripts/launch/run_door_with_checker.sh`.

## Benchmark status (pre-barrier pivot)

Before the analytical barrier landed (2026-04-14): 3/5 trials reached `button_ever_pressed=True`, A pinned at x≈3.62 against the unbroken lock. With barrier verified working end-to-end, A's ~7.5 Nm push should now exceed the spring torque for the first ~1.5 rad. Full 5-trial re-benchmark is the next session's first task.

## Config single-source-of-truth (TODO)

`button_xy` currently lives in 3 files (MJCF, door_task.yaml, checker). Plan: introduce `config/scene.yaml` and load from all three.

## Key config files

| File | Purpose |
|---|---|
| `src/go2w/go2_gazebo_sim/mujoco/two_rooms_door_scene.xml` | MJCF scene (door + barrier + button + robots) |
| `src/go2w/go2_gazebo_sim/launch/dual_go2w_mujoco_door.launch.py` | Dual-robot door launch |
| `src/go2w/go2w_config/config/nav/astar_nav_door.yaml` | Aggressive A* nav for door approach — `obstacle_stop_dist=0.05 m`, `inflation=0.05 m`, `startup_delay=0`, `footprint_buffer=0`. Ported 2026-04-24 from the now-deleted `reactive_nav_door.yaml` (RRT\*). VLM path normally skips the nav sub-launch; this config is loaded by the FSM path and by any external `/{ns}/way_point` publisher. |
| `src/go2w/go2w_config/ros_control_dual_mujoco_door.yaml` | ros2_control controllers (incl. `door_assist_controller`) |
| `scripts/launch/door_demo_mujoco.sh` | Launch entry point |

## Debug topics

```bash
ros2 topic echo /door_task/fsm_status --once          # (legacy, may be empty)
ros2 topic echo /door_task/door_state --once
ros2 topic echo /door_task/button_pressed
ros2 topic echo /robot_a/odom/nav --once --field pose.pose.position
ros2 topic echo /robot_b/odom/nav --once --field pose.pose.position
ros2 topic echo /vlm_debug/state --once
ros2 topic echo /perception/world_dict --once
```

## Build

```bash
colcon build --symlink-install --packages-select door_task
# Full door-task stack:
colcon build --symlink-install --packages-select door_task go2_gazebo_sim mujoco_ros2_control
```
