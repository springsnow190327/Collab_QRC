# Door Task — Evolution & Lessons

History of how the door task got to its current architecture. See `door_task.md` for current state; see CLAUDE1.md Phase 2 archive for the deleted FSM-era package structure.

## Phase 2 → 3 timeline

| Phase | Approach | Outcome |
|---|---|---|
| Phase 2 (FSM) | 8-state then 7-state FSM, skill primitives (MOVE_TO, PUSH, HOLD_POSITION, etc.) | Too coarse for physics coordination, deleted 2026-04-14 |
| Phase 2 fixes | Occupancy map decay, PUSH_TO/DRIVE_THROUGH skills, crate obstacle + physics checker | 3/3 criteria PASS on obstacle scenario |
| Phase 3 (VLM) | `vlm_controller_node` replaces FSM+reactive_nav entirely | 2-4/5 PASS depending on config |
| Phase 0 refactor | 1449-line god file → package with core/, llm/, prompts/, ros/ | 44 unit tests, 14-line shim |
| Phase 1a | YOLOv8n + IoU tracker + WorldDict | — |
| Phase 1b | CLIP inspector + temporal softmax pool + depth upgrade | Eliminated OOB hallucinations |
| Barrier pivot | Replaced position-actuator door lock with analytical barrier body | Hard lock verified end-to-end |

## Why FSM was abandoned

The original 8-state FSM (A pushes → B holds → A retreats → A passes through) had a **fundamental collision problem**: both robots in the 1 m doorway simultaneously always caused body collisions (Go2 legs + hips extend ~0.55 m laterally; doorway is 1 m wide). Tried y-lane offsets (0.5 m, 0.7 m gap) but reactive_nav's RRT* steered A off the intended lane.

**Key insight (2026-04-07)**: A single Go2W CAN push the 30 kg FD30 door to 88° peak with aggressive progressive waypoints (`push_through_x=6.0`). B's help was never needed for the door itself.

FSM redesign shifted B's role to "clear obstacles" (spatially separated) — the crate scenario. Worked (3/3 PASS) but still brittle:

1. **Not Markovian** — occupancy map carries history; door momentum matters; contact state not captured in FSM state
2. **Not deterministic** — door bounce-back; nav may/may-not find path; push force varies with angle
3. **Recovery too slow** — only recovery was timeout → fail → replan (10 s VLM call, door closes in 3 s)
4. **reactive_nav contradiction** — it's an obstacle avoidance planner being asked to push INTO an obstacle
5. **Occupancy map persistence** — simple_scan_mapper marked cells occupied with no decay; when door moved, old cells remained blocking path planning

## Phase 2 fixes worth remembering

### Occupancy map

- **Score decay** in `simple_scan_mapper_cpp`: timer every 2 s decrements positive scores (`decay_interval_sec: 2.0`, `decay_amount: 1`)
- **Raytrace threshold raised**: `raytrace_free()` wall-stop from `occupied_score_threshold_` to `score_max_` so free-space rays punch through partially-occupied cells
- **Door corridor exemption zone**: rectangular area (x=[3.5,4.8], y=[1.4,2.6]) forced free in `publish_map()` to prevent RRT* sideways routing around stale door cells

### New skills added (since deleted)

- **PUSH v3** — keep reactive_nav running, detect contact from odom twist (commanded vx vs actual vx), advance waypoint past door on contact
- **PUSH angular velocity tracking** — sliding window of 5 samples for dθ/dt; advance waypoint +0.8 m if ω < -0.1 (reacts in ~100 ms instead of waiting 0.15 rad decline from peak)
- **PUSH_TO** — heading-controlled drive to (x, y) bypassing reactive_nav; phase 1 align heading, phase 2 drive forward with correction
- **DRIVE_THROUGH** — forward drive with PD y-correction, keeps robot on target y-lane (bypasses reactive_nav RRT* doorway-centering)

## Phase 3 VLM controller — what worked

1. **Method B (waypoint actions)** — VLM outputs `drive(tx, ty)` targets; 10 Hz heading P-controller in the ROS node turns those into cmd_vel. Killed yaw oscillation. 0/5 → 3/5 PASS overnight.
2. **Ground-truth scene rendering** — hardcoded walls + MuJoCo framepos for robots/door/crate → 480×240 PNG → VLM. Beat cartographer (drifts ~10° in 30 s in the long symmetric scene).
3. **`push_door` skill with curved Stage-2 target (5.5, 2.0)** — slightly north of door center tracks the door tip as it arcs north, keeping contact at leverage edge.
4. **Button-gated collaboration task** — pressure pad is the only door unlock. When perception works, the protocol emerges cleanly.
5. **MuJoCo position actuator hard lock** (pre-barrier): `<position joint="door_hinge" kp="20000"/>` + a small node driving ctrl. Ran at 500 Hz physics step — no feedback lag.
6. **Bypass reactive_nav entirely in VLM path** — its zero-cmd_vel on startup branches kept overwriting direct commands via twist_bridge. Skipping the whole nav sub-launch closed subtle bugs.
7. **button_monitor_node as proximity sensor** — tiny node watches odom, decouples "button pressed" from physical contact geometry.
8. **MuJoCo framepos auto-published** — `<framepos name="X_pos" objtype="body" objname="X"/>` → `/mujoco_sim/X_pose_sensor/pose`. One MJCF line per semantic object.
9. **Slow planner (6 s) + fast executer (1 s) split** — separate VLM calls with different system prompts, configurable model per agent.
10. **World memory across ticks** — planner outputs `world_memory` with `pillar`/`door` entries (known/confidence/evidence); next call receives previous memory back.
11. **Debug web dashboard** — single `/vlm_debug/state` JSON topic + stdlib http.server at :8080. Debug loop dropped from "tail 3 logs" to "refresh a tab".
12. **`door_task_checker.py` loads MJCF independently** — runs as separate process for per-trial PASS/FAIL.

## Phase 3 VLM controller — what didn't work

1. **VLM as raw `wz` at 1 Hz** — wz_max=1.2 × 1 s tick = 69°/tick; always overshoots, next tick picks opposite, chaotic. Moral: VLM picks goals, not velocities.
2. **Pure cartographer SLAM without odometry** — long symmetric two-room scene drifts ~10° in 30 s. Fix: GT rendering or `use_odometry=true`.
3. **`simple_scan_mapper_cpp` alone** — ghost cells linger without decay hack. Per-task hack.
4. **Effort-based virtual spring lock** — 50 Hz Python node with `effort = -kp*angle` had feedback lag, ~5° steady-state oscillation. Replaced with MuJoCo position actuator at 500 Hz.
5. **Opportunistic override rules in executer prompt** — "if red object in B's camera, bypass planner". Worked but violated clean planner-authority model. Removed.
6. **Camera-only bearing from VLM** — `grok-4-1-fast-non-reasoning` can't convert "red blob on right side" into numeric bearing reliably. Led to B driving wrong direction. Fix: dedicated perception stack.
7. **VLM as perception layer** — hallucinated `pillar_known=True` with wrong `world_xy` that didn't match `button_pressed`. Added "sensor validation" guard, helped but didn't cure. Correct fix: YOLO/CLIP outputs structured dict.
8. **Scanning by spinning at `wz_max=0.6` with `heading_deg=60°`** — VLM inference 1-2 s, robot rotates past target in that window. Mitigation: lower wz_max to 0.25 + prompt heading_deg ≤ 30. Real fix: fast detector at 25 Hz.
9. **`drive_relative(forward_m=0, heading_deg=X)` naively → `drive(tx=current_xy)`** — arrived immediately (dist < 0.2 m), emitted (0,0). Robot never rotated. Fix: phantom 0.5 m target in new heading direction + `vx_max=0`.
10. **`<equality><joint polycoef=...>` joint-lock** — the "right" way in MuJoCo with `mjData.eq_active` toggle, BUT `mujoco_ros2_control` does not expose `eq_active` to ROS (needs C++ patch). Position actuator pragmatic alternative.
11. **Prompt hints for pillar location** (e.g. "(7.5, 3.5)") — user rejected leaking GT into prompt. Pillar must be camera+SLAM discoverable.
12. **Thin 5-cm-radius pillar** — Cartographer 0.05 m resolution maps it as 1-2 cells, noise-indistinguishable. Thickened to 0.12 m. Then geom contact trapped robot on outer edge of 0.6 m press ring. Balance: small button, larger press radius (0.8 m).
13. **`mujoco_depth_camera` plugin encoding bug** — publishes `"8UC3"` with BGR memory. Plugin source does extra `BGR2RGB` on already-RGB buffer. Decoder treated `"8UC3"` as RGB → red/blue swapped → red pillar looked blue. Fix in `_image_msg_to_np`: swap channels when encoding is `"8uc3"` or empty.
14. **VLM scan spinning with single snap-and-reason frame** — VLM at 1 Hz can't keep up with moving FOV target. Motivated Phase 4 perception stack.

## Phase 1b perception tuning

- `min_confidence=0.55` tuned after false-positive (weak red cue + depth error) put "red pressure pad" ghost at (7.3, 4.5) outside room → B drove into north wall
- **Scene-bounds drop** in perception_node: world_xy outside [0,8]×[0,4] silently discarded
- Depth upgrade: replaced `assumed_depth_m=2.5` with real depth from `/{cam}/depth/image_rect_raw` (32FC1); 5×5 median patch, NaN/0/>20 m filtered; projection uses `range = depth / cos(bearing)` for oblique-pixel correction

## Analytical door-lock barrier — the 5-bug chain

Five compounding bugs while getting the barrier to respond end-to-end (2026-04-14). All fixed; preserved because any one is a good hazard to avoid in future ros2_control + MuJoCo work.

### Bug 1: `mujoco_ros2_control` POSITION actuator detection requires `kv=0`

The plugin's `mujoco_system.cpp` pattern-matches raw `dynprm`/`gainprm`/`biasprm` arrays to decide actuator type. For `<position>`:
```
gainprm[0] == -1 * biasprm[1]  &&  biasprm[2] == 0
```
where `biasprm[2] == -kv`. Any non-zero kv → actuator silently rejected. No INFO log, no warning, `ctrl[i]` never touched.

**Fix:** `kv="0"` on `<position>`; move velocity damping to `<joint damping="2000">`. Startup log prints `"added position actuator for joint: door_barrier_slide"` — grep for it to confirm.

### Bug 2: `<contact>` must be direct child of `<mujoco>`, not `<worldbody>`

Putting it in `<worldbody>` raises `XML Error: Schema violation: unrecognized element 'contact'` at load time, killing the entire mujoco_ros2_control process mid-startup, taking all controller spawners down with it. **Fix:** move `<contact>` to top-level next to `<worldbody>` and `<actuator>`.

### Bug 3: `forward_command_controller` uses BEST_EFFORT QoS

`door_assist_controller` subscribes to `/commands` with `Reliability: BEST_EFFORT, Durability: VOLATILE`. Default Python publisher uses `RELIABLE` → **DDS silently drops every message**. No error, no warning, subscription count of 1 looks fine. Only `ros2 topic info -v` reveals the mismatch.

**Fix:** `QoSProfile(depth=10, reliability=BEST_EFFORT)` on any publisher to a ros2_control command topic.

### Bug 4: URDF joint limit clamps position commands

Plugin clamps `joint.position_command` to `[lower_limit, upper_limit]` from urdfdom at init. After flipping retract direction from qpos=-3 to qpos=+3, MJCF `<joint range>` and `ctrlrange` were updated but `dual_urdf.py` stub still said `lower="-3.1" upper="0.1"` → `std::clamp(3.0, -3.1, 0.1) = 0.1` → every retract silently truncated to 0.1 m, barrier parked ~8 cm above home.

**Fix:** keep URDF stub `<limit lower/upper>` in sync with MJCF range. `build_dual_mujoco_urdf` now has a comment spelling this out.

### Bug 5: Ground-plane contact pins a downward-retract barrier

Original design retracted DOWN into floor. With `contype=1` on both ground plane and barrier geom, the contact solver pinned the barrier against z=0 the moment the actuator pushed below home position. No force could drive the geom through the floor.

**Fix:** retract UP. Body pos `z=1.01` so home barrier spans `z∈[0.01, 2.01]` (1 cm floor clearance); retract target `qpos=+3.0` places geom at `z∈[3.01, 5.01]`, clear of everything.

### End-to-end verification

Fresh launch 2026-04-14: `added position actuator` in startup log; `/mujoco_sim/door_assist_controller/commands` has `Publisher count: 1 (BEST_EFFORT) / Subscription count: 1`; barrier qpos snaps ~0 → ~3 within ~150 ms when `button_pressed` flips True, back to ~0 when False.

## Phase 4 plan — generalizable perception stack

**Mostly implemented** in Phase 1a+1b; remaining: L3 semantic inspector temporal-pooling polish, OWL-ViT swap for CLIP-only.

```
L0  Camera @ 25 Hz (raw RGB + depth)  [TODO: currently 5 Hz]
L1  YOLOv8n class-agnostic → top-K boxes per frame     [DONE]
L2  Depth back-projection → world (x,y,z)              [DONE]
L3  CLIP scores crops against open-vocab queries       [DONE, using CLIP ViT-B/32]
L4  Rolling world_dict keyed by semantic label         [DONE]
L5  VLM planner calls world_dict.query("red button")   [DONE, injected into prompt]
```

## Phase 0 refactor outcome

Target vs actual sizes:

| Module | Target | Actual |
|---|---|---|
| `ros/vlm_controller_node.py` | ~250 | 14 (shim) + 560 in `ros/controller.py` |
| `core/*.py` total | ~800 | ~matched |
| `llm/*.py` total | ~150 | ~matched |
| `ros/*.py` total | ~500 | ~matched |

44 pytest tests covering actions, control, memory, parser, rendering. Run: `PYTHONPATH=. /usr/bin/python3 -m pytest tests/ -q`.

Legacy `skill_primitives.py`, `fsm_executor_node.py`, `door_task_coordinator.py`, `fsm_validator.py`, `prompting_door.py` (~1700 LOC combined) — DELETED, not archived.

## Benchmark history

| Config | PASS | Notes |
|---|---|---|
| Crate task + GT info in prompt + simple_scan_mapper | 3/5 | Baseline |
| Pure cam+SLAM, VLM outputs raw wz | 0/5 | Yaw chaos |
| `wz_max=0.5` + prompt fix | 2/5 | Still shaky |
| **Method B** (drive(tx,ty) + 10 Hz heading loop) | **3-4/5** | Yaw chaos solved |
| Method B + push_door + curved Stage-2 (5.5, 2.0) | **4/5** | Best pre-button |
| Button-gated task (new collaborative forcing function) | 2/5 | A physics brittleness bottleneck |
| kp=20000 kv=5 door lock | 3/5 button_pressed | A pinned at x≈3.62 |
| Barrier pivot (2026-04-14) | TBD | Next benchmark |

## Architectural lessons (carry forward)

- **Fast/slow decoupling wins.** 500 Hz physics / 10 Hz control / 1 Hz executer / 6 s planner. Each tier runs at its natural rate.
- **Perception ≠ planning.** LLM should not be sole perception — hallucinates, too slow. Dedicated fast detector + world dict for LLM queries.
- **Dataclasses at interface boundaries.** Typed Action, Plan, Observation, WorldMemory. (Partially done in Phase 0.)
- **Prompts are code.** Version-controllable markdown, loaded at startup, testable via mock LLM.
- **Config single-source-of-truth.** `button_xy` in 3 places still. TODO.
