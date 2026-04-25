# Debugging Notes & Gotchas

Cross-cutting hazards that have cost real time. Not per-task architecture — things to remember when anything breaks.

## Zombie MuJoCo process

Stale `mujoco_ros2_control` processes (165% CPU) block physics for new launches — MuJoCo model file is locked or GPU context held. **Symptoms:** new launch starts, nodes appear, but robot never moves (physics frozen). `ps aux | grep mujoco` reveals the zombie.

**Fix:** kill all sim processes before each benchmark trial:
```bash
pkill -9 -f 'mujoco_ros2_control/mujoco'
pkill -9 -f 'fastlio_mapping|far_planner|cartographer_node'
pkill -9 -f 'localPlanner|pathFollower|terrain_analysis'
sleep 4  # wait for sockets to release
```

Both benchmark scripts do this automatically between trials.

## `hybrid_cmd_router` wheel topic under mixed launch: absolute, not relative

**Symptom:** Robot A (Go2W on A* backend) picks up CFPA2 goals, astar plans valid paths (`valid=1 fp_ok=1`), hybrid_cmd_router switches modes between `legged` and `wheel` at the expected times, but the robot physically sits still for long stretches. Tiny movements only during the short legged windows.

**Cause:** `go2w_hybrid_cmd_router.py` defaults `wheel_command_topic` to the relative string `"wheel_velocity_controller/commands"`. Under `namespace=robot_a` that resolves to `/robot_a/wheel_velocity_controller/commands`. In the *mixed* launch the ros2_control controller_manager lives at `/mujoco_sim/controller_manager`, so the wheel controller actually listens at `/mujoco_sim/robot_a_wheel_velocity_controller/commands`. Nobody subscribes to the `/{ns}/...` version → wheel commands go nowhere while the router thinks it's driving.

FAR masked this bug because FAR's cmd_vel is too smooth to trigger the router's "wheel" mode for more than a blip. A*'s heading/curvature dispatch commits to wheel mode aggressively — exposing the silent wheel-topic miss.

**Fix:** pass the absolute topic as a parameter, in both the A* and FAR branches of `_build_fastlio_nav_stack`:

```python
"wheel_command_topic": f"/mujoco_sim/{ns}_wheel_velocity_controller/commands",
```

The leading `/` makes the string absolute — the node's `robot_a` namespace does not prepend.

Single-robot launches (`single_astar_mujoco.launch.py`) aren't affected because their CM lives at `/{ns}/controller_manager`, so the relative default happens to resolve correctly.

## QoS mismatches that fail silently

**`forward_command_controller` uses BEST_EFFORT.** Default Python publisher uses RELIABLE. DDS silently drops every message; subscription count of 1 looks fine. Only `ros2 topic info -v` reveals it.

Fix: `QoSProfile(depth=10, reliability=BEST_EFFORT)` on any publisher to a ros2_control command topic.

**`octomap_server` with TRANSIENT_LOCAL** — CFPA2's late-connecting TRANSIENT_LOCAL subscriber gets the initial map only if octomap publishes with `latch=True`.

## `mujoco_ros2_control` POSITION actuator requires `kv=0`

Plugin pattern-matches raw `dynprm`/`gainprm`/`biasprm` to decide actuator type. For `<position>`: `gainprm[0] == -1 * biasprm[1] && biasprm[2] == 0` where `biasprm[2] == -kv`. Any non-zero `kv` → actuator silently rejected, `ctrl[i]` never touched. No warning.

Fix: `kv="0"` on `<position>`; move velocity damping to `<joint damping="...">`. Startup log prints `"added position actuator for joint: X"` — grep for it.

## MuJoCo `<contact>` must be direct child of `<mujoco>`

NOT inside `<worldbody>`. Putting it there raises `XML Error: Schema violation: unrecognized element 'contact'` at load, killing `mujoco_ros2_control` mid-startup, taking all controller spawners down.

## URDF joint limits clamp position commands

`mujoco_ros2_control` clamps `joint.position_command` to `[lower_limit, upper_limit]` from urdfdom at init. If MJCF `<joint range>` or `<actuator ctrlrange>` is updated but URDF stub in `dual_urdf.py` (or equivalent) isn't, commands are silently truncated.

Fix: keep URDF `<limit lower/upper>` in sync with MJCF range whenever moving a joint.

## DFKI mujoco_ros2_control concurrency crash

`malloc_consolidate` crash = concurrent `mj_step` + `mj_multiRay` on different threads. `sim_step_mtx_` must protect both calls — see `lidar_sensor.cpp`.

## Cartographer deadlock

`provide_odom_frame = true` does NOT work odom-free — `tf_bridge.cpp` tries to look up `odom` frame before it can publish it, causing a deadlock. Use `provide_odom_frame = false` with `published_frame = "base_link"` instead.

## FastDDS shared memory issues

`test.sh` was missing `FASTRTPS_DEFAULT_PROFILES_FILE`. Stale data between runs, broken inter-node communication. Fix: export the var + clean `/dev/shm/fastrtps_*` between runs.

## LRC stack prefix shadowing

When sourcing SC-PGO from `/home/hz/COMP0225_LRC_stack/install`, add ONLY the sc_pgo prefix to `AMENT_PREFIX_PATH`, NOT the whole LRC install (would shadow our go2_gazebo_sim).

## `mujoco_depth_camera` channel-order bug

Plugin publishes `encoding="8UC3"` with BGR-ordered memory, then does an extra `cvtColor(BGR2RGB)` on what is already RGB. Our decoder treating `"8UC3"` as RGB got red/blue swapped — red pillar looked blue to VLM.

Fix in consumer (`_image_msg_to_np`): always swap channels when encoding is `"8uc3"` or empty.

## xAI API key whitespace bug

Line-wrapped paste from docs introduces whitespace inside API key string. `vlm_backends.py` `_clean()` now strips any whitespace before sending.

`preflight_vlm_key()` pings provider before sim startup and aborts on 401/429 with `prefix=.. suffix=.. len=..` dump.

## Common nav/map symptoms

| Symptom | Likely Cause | Where to Look |
|---|---|---|
| Map flickering / doubled walls | Same scan painted multiple times | `simple_scan_mapper_cpp.cpp` update timer vs scan arrival rate |
| Map starburst / rotated structures | TF timestamp mismatch or stale TF | TF lookup code, `ExtrapolationException` handlers |
| "No Effective Points!" in Fast-LIO | Wrong timestamp span | `pointcloud_adapter.py` time_offset calculation |
| Robot walks through walls | Planner not using occupancy grid | Check planner is reading `/{ns}/map` |
| Scattered occupied cells | Ground/leg hits passing height filter | `pointcloud_to_laserscan` `min_height` |
| Robot stuck on map, scans pile at origin | Cartographer scan matcher weights too high for IMU-only prediction | `cartographer_sim_2d.lua` `translation_delta_cost_weight`, Ceres `translation_weight` |
| `malloc_consolidate` crash in mujoco_ros2_control | Concurrent `mj_step` + `mj_multiRay` | `sim_step_mtx_` in `lidar_sensor.cpp` |

## Debugging workflow

1. **Check prior art first.** Search git history.
2. **Trace the full data pipeline:** What generates it? What timestamp domain? What transforms it? What consumes it?
3. **Add diagnostic logging with actual values** (yaw, dt, timestamps). Use `RCLCPP_INFO_THROTTLE` or Python logger with rate limiting.
4. **Measure, don't assume.** `ros2 topic hz`, `ros2 topic echo`, grep logs.
5. When told "still not working", don't repeat the same fix — go deeper.
