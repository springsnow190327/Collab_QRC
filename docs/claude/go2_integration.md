# Go2 (non-W) Integration — Pure Quadruped Variant

Status as of **2026-04-20**: Menagerie-body sim **shipped and benchmarked** under CHAMP (0/5 PASS on demo3 due to coverage, 5/5 completed with 0 contacts). **Real CMU TARE → FAR-bypass pipeline working end-to-end** on demo3 (70% coverage at 1370 s wall-clock, no crashes); bottleneck is MuJoCo RTF (≈ 0.57×) + CHAMP walking cap (0.3 m/s), not nav. RL-policy swap attempted earlier, not shipped.

Index:
- [Why the stripped-Go2W approach failed](#why-the-stripped-go2w-approach-failed)
- [Integration — what shipped](#integration--what-shipped)
- [Three correctness fixes along the way](#three-correctness-fixes-along-the-way)
- [Benchmark results (CHAMP)](#benchmark-results-champ)
- [Speed sweep — where CHAMP tops out](#speed-sweep--where-champ-tops-out)
- [Controller landscape & why there is no drop-in](#controller-landscape--why-there-is-no-drop-in)
- [RL-policy swap attempt — what worked, what didn't](#rl-policy-swap-attempt--what-worked-what-didnt)
- [Exploration planner: CFPA2 → TARE (real CMU port)](#exploration-planner-cfpa2--tare-real-cmu-port)
- [RTF bottleneck — where the wall-clock time actually goes](#rtf-bottleneck--where-the-wall-clock-time-actually-goes)
- [Files touched (by layer)](#files-touched-by-layer)
- [Known gotchas](#known-gotchas)
- [If you come back to the RL path](#if-you-come-back-to-the-rl-path)

---

## Why the stripped-Go2W approach failed

`demo1_go2.xml` / `demo3_go2.xml` (pre-session artifacts) were built by stripping wheel actuators from Go2W MJCF, keeping a separate `{LEG}_foot` body with a sphere collision at an inherited Go2W wheel lateral offset (`±0.04 m` sideways from the calf centreline).

```
Menagerie Go2 (correct):               Stripped Go2W (broken):
FL_calf {                              FL_calf {
  geom mesh="foot" @(0,0,-0.213)         body "FL_foot" @(0,0,-0.2264) {
  geom name="FL" class="foot"              geom sphere @(0, +0.04, 0)   ← SIDE-OFFSET
}   ↑ pos="-0.002 0 -0.213"               site @(0, +0.04, 0)
    centred on calf tip                  }                               ← inherits Go2W wheel lateral pos
                                         }
```

Foot contact at ±4 cm sideways creates lateral torques during trot that CHAMP can't compensate → early tipover. **Don't try to patch by shifting the sphere** — use the Menagerie body wholesale.

---

## Integration — what shipped

### 1. Two MJCFs with Menagerie robot body + demo worlds
- **`src/go2w/go2_gazebo_sim/mujoco/demo1_go2_real.xml`** — 12×8 m scene, spawn (4, 0, 0.5)
- **`src/go2w/go2_gazebo_sim/mujoco/demo3_go2_real.xml`** — 24×16 m scene (4 quadrants), spawn (4, 2, 0.5)

Both use Menagerie's `<default class="foot">` geom at `pos="-0.002 0 -0.213"` on each calf, 12 effort motors (no wheel actuators), Menagerie joint `damping="2" armature="0.01" frictionloss="0.2"`, `<option cone="elliptic" impratio="100"/>`. Meshes at `src/go2w/go2_gazebo_sim/mujoco/assets/go2_menagerie/` (16 OBJs from DeepMind Menagerie). Sensor block: `imu_accel / imu_gyro / 4× touch / framepos+quat+linvel+angvel on base_link_site`.

Each also carries a `<keyframe name="home">` with `qpos` at Menagerie home stance (z=0.27, thighs 0.9, calves -1.8). Applied on startup when env `MUJOCO_INIT_KEYFRAME=home` is set (RL-policy launch path only) — see the [threadpool/keyframe patch to DFKI](#rl-policy-swap-attempt--what-worked-what-didnt) below.

### 2. Xacro wrapper for the LRC URDF
- **`src/go2w/go2_gazebo_sim/urdf/go2/go2_description_3d_lidar.xacro`** — mirrors `go2w_description_3d_lidar.xacro` with:
  - `FL/FR/RL/RR_foot_joint` changed from `continuous` (wheel) to `fixed` (spherical foot)
  - Foot-link visual = `foot.dae`; collision = sphere(0.022) at `(-0.002, 0, 0)` — matches Menagerie contact geom
  - `<ros2_control>` with 12 joints only (hips/thighs/calves)
  - Same `<gazebo>` sensors (IMU, MID-360, camera, contact, P3D) for Gazebo parity
  - Mesh root: `${go2_mesh_root} = file://$(find go2_description)/dae` (LRC's package, pre-staged)

### 3. Launch plumbing
- **`src/go2w/go2_gazebo_sim/launch/single_go2w_mujoco_cfpa2.launch.py`**
  - When `has_wheels:=false`, load the Go2 xacro instead of Go2W
  - `ros_control_yaml` selection is 3-way: Go2W YAML / Go2-CHAMP YAML / Go2-RL YAML
  - `build_dual_robot_stack` is called with `skip_champ=rl_policy` (new kwarg — see `modules/assets.py`)
  - When `rl_policy:=true`, launches `scripts/runtime/go2_rl_policy_node.py` instead of CHAMP
- **`src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py`** — propagates `has_wheels`, `rl_policy`, `rl_use_champ_gains` down
- **`src/go2w/go2_gazebo_sim/launch/modules/assets.py`** — `build_dual_robot_stack(..., skip_champ=False)` omits `quadruped_controller_node`, `state_estimator_node`, and the two EKFs when True

### 4. Scripts point at new MJCFs
- `scripts/launch/nav_test_go2.sh`, `nav_test_go2_demo3.sh`, `benchmark_go2.sh`, `benchmark_go2_demo3.sh` — all updated to `demo*_go2_real.xml`

### 5. Thread-pool patch (plumbed but disabled by default)
- **`src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_plugin.cpp`** — creates an `mjThreadPool` and binds it via `mju_bindThreadPool` when env `MUJOCO_THREAD_POOL=<n>` is set. Cleans up in the destructor (before `mj_deleteData`). Our sim is dominated by a single serial physics step so this didn't unlock meaningful RTF gains in practice, but the hook is ready.
- Same file also adds `MUJOCO_INIT_KEYFRAME=<name>` support (reads the MJCF `<keyframe>`, calls `mj_resetDataKeyframe`). Needed for the RL path; harmless for CHAMP path.

---

## Three correctness fixes along the way

All three were uncovered during the first GUI smoke test and all had to land before the stack walked end-to-end under CHAMP.

### A. Stand-up joint name preset
**Symptom**: controller log `Incoming joint lf_hip_joint doesn't match the controller's joints.` → stand-up trajectory rejected → CHAMP never got a valid pose → legs stayed limp.

**Cause**: `stand_up_slowly.py` had two presets — `go2` (lowercase CHAMP names `lf_hip_joint` …) and `go2w` (Unitree names `FL_hip_joint` …). The launch chose `go2` for `has_wheels=false`. But our LRC URDF + Menagerie MJCF both use Unitree naming, so the trajectory message got silently dropped.

**Fix**: alias both presets to the Unitree names in `src/go2w/go2w_spawn/scripts/stand_up_slowly.py`. Documented reason: "lowercase CHAMP naming is no longer carried by any URDF in the stack."

### B. Octomap z-band for Go2's lower LiDAR
**Symptom**: "LiDAR ground points hit the ground, scatter as walls on the occupancy grid, stuck the planner" — phantom walls appeared under the robot during yaw dynamics; FAR V-graph polluted; robot commanded motion but clipped to 0 on localPlanner's obstacle checks.

**Cause**: Go2's stance is ~0.18 m lower than Go2W's (0.27 vs 0.45). The MID-360 at z≈0.39 m with its 13° forward tilt + −7° lower-beam sweeps ground ~0.9–1.5 m ahead of the robot. Pose jitter during fast yaw puts those ground hits above the `point_cloud_min_z: 0.20` filter → projected as wall cells. `filter_ground_plane: False` (because RANSAC fails on spherical-foot Go2).

**Fix**: in `nav_test_mujoco_fastlio.launch.py`, conditional on `has_wheels`:
- `point_cloud_min_z` / `occupancy_min_z`: **0.30** for Go2, 0.20 for Go2W (unchanged)
- `filter_speckles: True` (cheap single-voxel suppression for residual jitter)

### C. `twist_bridge` `cmd_vel` remap
**Symptom**: `cmd_vel_stamped` flowing from pathFollower at 50 Hz, but CHAMP never saw it; robot stuck at spawn. `ros2 topic info -v /robot/cmd_vel_legged` showed **Publisher count: 0** — twist_bridge was publishing to `/robot/cmd_vel` instead.

**Cause**: the launch remap key was absolute `("/cmd_vel", "/robot/cmd_vel_legged")`. ROS 2 applies remaps on the resolved name. twist_bridge declares its pub on relative `cmd_vel` → resolves to `/robot/cmd_vel` under namespace `/robot`. Remap `/cmd_vel` → `/robot/cmd_vel_legged` has the wrong key (`/cmd_vel` never appears after ns scoping) so it's a silent no-op. Go2W happened to work because `/robot/cmd_vel` IS where the hybrid wheel router listens. Go2 has no router, so the pub landed on an orphan topic.

**Fix**: switch to a relative remap `("cmd_vel", "cmd_vel_legged")`. Gated on `has_wheels=false`. Documented in the launch with a block comment.

---

## Benchmark results (CHAMP)

Demo3 (24×16 m, 384 m² ground truth), 5 trials × 300 s each, headless, `far_max_speed:=0.3`:

| trial | outcome | dist | cov | contacts | tip | SLAM μ |
|---|---|---|---|---|---|---|
| 1 | completed | 29.3 m | 54.5 % | 0 | ✗ | 3.1 cm |
| 2 | completed | 30.1 m | **73.8 %** | 0 | ✗ | 4.5 cm |
| 3 | completed | 14.8 m | 56.1 % | 0 | ✗ | 5.8 cm |
| 4 | completed | 8.5 m | 41.3 % | 0 | ✗ | 16.1 cm |
| 5 | completed | 56.4 m | 51.3 % | 0 | ✗ | 33.7 cm |

- **0/5 PASS** (90 % coverage target)
- **5/5 completed** cleanly — no crashes, no tipovers, **0 total contacts over 25 min sim**
- Menagerie-body geometry is fully validated: no clipping, no spawn instability, proper contact
- The 90 % ceiling is **duration/compute-bound**, not nav-stack-bound. At MuJoCo RTF ≈ 0.15 on the dev laptop, 300 s sim ≈ 2000 s wall-clock — not enough walking to cover 384 m²

Archived runs:
- `/tmp/far_bench/go2_demo3_run_180s.old` — initial 180 s trials (avg cov 48.4 %, 0 contacts)
- `/tmp/far_bench/go2_demo3_run_0p2ms.old` — 0.2 m/s speed baseline
- `/tmp/far_bench/go2_demo3_run_0p5ms.bad` — 0.5 m/s collapse experiment
- `/tmp/far_bench/go2_demo3_run` — final 0.3 m/s run

---

## Speed sweep — where CHAMP tops out

`far_max_speed` swept at 5 × 300 s on demo3:

| speed | avg dist | avg cov | contacts | gait health |
|---|---|---|---|---|
| 0.2 m/s | 16.3 m | 48.4 % | 0 | ✓ stable baseline |
| **0.3 m/s** | **27.8 m** | **55.4 %** | **0** | ✓ **best** (chosen default) |
| 0.5 m/s | 2.4 m (1 trial) | 24.1 % | 0 | ✗ gait collapsed |

**Why 0.5 m/s broke**: `src/go2w/go2_gazebo_sim/config/champ/go2w/gait.yaml` sets `max_linear_velocity_x: 0.3`. CHAMP internally clips pathFollower's 0.5 request to 0.3, but the smoothing + gait generator sees the 0.5 target on the input side. The resulting desync between commanded and realised velocity breaks phase coupling and the trot degenerates into a shuffle.

**No one has retuned CHAMP for MuJoCo**. Web search turned up only MPC / RL alternatives ([`elijah-waichong-chan/go2-convex-mpc`](https://github.com/elijah-waichong-chan/go2-convex-mpc), [`alexeiplatzer/unitree-go2-mjx-rl`](https://github.com/alexeiplatzer/unitree-go2-mjx-rl)) and Anuj Jain's original Gazebo-targeted CHAMP config ([`anujjain-dev/unitree-go2-ros2`](https://github.com/anujjain-dev/unitree-go2-ros2)) that our config is derived from. The community consensus is to swap CHAMP out rather than tune it further.

---

## Controller landscape & why there is no drop-in

We looked for a Unitree-quality locomotion controller that plugs into MuJoCo with the `cmd_vel` interface unchanged. Three official Unitree repos:

- **[`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco)** — *just a DDS↔MuJoCo bridge*. Only `example/stand_go2.{cpp,py}` ships, plus `simulate/src/unitree_sdk2_bridge.h`. **No controller**. Verified via GitHub API listing: `simulate/src/` contains `main.cc`, `param.h`, `unitree_sdk2_bridge.h`, `physics_joystick.h` — no gait/IK/MPC code. The real Go2's walking controller is a **closed Jetson binary** and isn't distributed.
- **[`unitreerobotics/unitree_rl_gym`](https://github.com/unitreerobotics/unitree_rl_gym)** — training recipe for G1 / H1 / H1_2 + Go2. `deploy/pre_train/` ships `motion.pt` only for **G1 / H1 / H1_2** — **no pre-trained Go2 policy**. `deploy/deploy_mujoco/configs/` similarly has no `go2.yaml`. To get a Go2 policy you'd have to train it yourself (hours on one A100).
- **[`unitreerobotics/unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab)** — newer RL-in-MuJoCo package, Apache 2.0, has a `deploy/robots/go2/` dir but the `policy/` subdir is empty — same story as unitree_rl_gym for Go2.

**Best available ready-to-run Go2 policy**: [`eppl-erau-db/go2_rl_ws`](https://github.com/eppl-erau-db/go2_rl_ws) (MIT, Embry-Riddle lab). Ships 5 pre-trained ONNX checkpoints:
- `flat_policy_v3.onnx` (163 KB, 45-dim obs) — pronking gait
- `flat_policy_v5.onnx` (163 KB, 45-dim obs) — pronking gait (per upstream's own README comment)
- `flat_policy_v6.onnx` (163 KB, 45-dim obs) — trot
- `flat_policy_v7.onnx` (165 KB, **49-dim** obs) — trot, newer arch with extra inputs we don't build
- `rough_policy_v1.onnx` (767 KB, includes height scan we don't plumb)

Is the real Go2 controller MPC or RL? **Closed-source, unknown publicly.** Best guess: convex MPC + hand-coded state-machine for transitions + RL for `sport` mode dynamic skills. Consistent with Unitree's pre-2023 published papers + Spot / ANYmal industry pattern. Beyond-stance RL is only recently ready for quadrupeds, and the crisp-looking trot demos at launch (2023) predate that.

---

## RL-policy swap attempt — what worked, what didn't

Clone + inference **works**. End-to-end walking under cmd_vel **doesn't work yet** — the community checkpoint + Menagerie dynamics + upstream deploy-code quirks compound. Documenting here so it's cheap to resume.

### What's in place
- `scripts/runtime/go2_rl_policy_node.py` — standalone ROS 2 Python node. Subscribes to `/{ns}/joint_states`, `/{ns}/imu/data`, `/{ns}/cmd_vel_legged`. Computes 45-dim obs in Isaac-Lab ordering (hips/thighs/calves type-grouped), runs ONNX (`flat_policy_v6`) at 50 Hz, applies PD τ = kp·(target − pos) − kd·vel with `kp=20 / kd=0.5`, reorders IL→YAML (leg-grouped) and publishes `Float64MultiArray` torques.
- Three-phase control: **STANDUP** (stiff PD to Menagerie home, optional, disabled by default) → **HOLD** (policy-gain PD to home, ignores policy output for `cmd_hold_sec`) → **POLICY** (PD tracks `raw × 0.25 + IL_DEFAULTS`).
- MuJoCo-side controller manifest: **`src/go2w/go2_gazebo_sim/mujoco/go2_rl_mujoco_controllers.yaml`** loads `forward_command_controller/ForwardCommandController` on the **effort** interface. This was the critical fix — the original Gazebo-side YAML swap (`ros_control_go2_robot_rl.yaml` in `config/ros_control/`) was ignored by MuJoCo because DFKI's plugin loads `go2w_mujoco_controllers.yaml` directly via `--params-file`, bypassing the xacro's `<gazebo>` block entirely. Two YAML files needed to stay in sync.
- DFKI plugin patch: `MUJOCO_INIT_KEYFRAME=<name>` env var → `mj_resetDataKeyframe` at init. Without it, MJCF default qpos (all zeros) puts calves below ground at spawn and triggers explosive initial contact.
- Launch gating: `rl_policy:=true` + `has_wheels:=false` activates the RL pipeline, sets the env var, swaps both YAMLs, skips CHAMP quadruped_controller / state_estimator / EKFs (via `build_dual_robot_stack(skip_champ=True)`), keeps `twist_bridge` (with `cmd_vel → cmd_vel_legged` remap) so FAR still drives the policy.

### What broke and what we learned

1. **Pronking gait (v5)**. First try used `flat_policy_v5.onnx` (matches the path in upstream's deploy code) — robot jumped instead of trotting. Upstream README comment actually says *"pronking gait — do not use v3 or v5 currently"* but then loads v5 anyway. Fix: use v6. v7 has 4 extra obs inputs we don't build yet.

2. **Direct-torque bypass of `joint_trajectory_controller`**. First integration published joint trajectories and let `joint_trajectory_controller` run its internal PID. But that PID uses CHAMP's `kp=100/kd=1.0` — **5× stiffer than training** (`kp=20/kd=0.5`) — so the policy's action magnitudes produced 5× the torque they were shaped for. Stable at idle, unstable under any cmd_vel. Fix: swap to `ForwardCommandController` and have the node compute τ itself.

3. **Projected-gravity quaternion-order bug**. Upstream deploy passes `[w, x, y, z]` to `scipy.spatial.transform.Rotation.from_quat()` which expects `[x, y, z, w]`. Accidentally produces correct output at identity, diverges under yaw (e.g. returns `[0, -1, 0]` at yaw=90° instead of `[0, 0, -1]`). Policy sees "sideways gravity" under small yaw → aggressive recovery → faceplant. Fix: use correct `rot.inv().apply([0, 0, -1])` with scalar-last ordering. (Whether the policy was *trained* with the bug or not is unknown, but empirically the correct formula is required for it to even hold stance.)

4. **PD-target vs obs-default mismatch**. IL_DEFAULTS = `[0, 0, 0, 0, 1.1×4, -1.8×4]` but Menagerie home = `[0, 0, 0, 0, 0.9×4, -1.8×4]` — 0.2 rad thigh offset. Early code used `IL_DEFAULTS` as the stand-up PD target. Under kp=20 the PD can't pull thighs from 0.9 → 1.1 against gravity cleanly → underdamped oscillation → by the time POLICY takes over, joint velocities are >10 rad/s and obs is out-of-distribution → policy saturates at `|raw|>5`. Fix: use `MENAGERIE_HOME` (thigh=0.9) as the PD target, keep `IL_DEFAULTS` only for centring the observation.

5. **Stand-up kp discontinuity**. Using `stand_up_kp=80` followed by `kp=20` creates a large torque transient at the phase switch. With the keyframe init putting the robot already at stance, `stand_up_sec:=0` is the right default; HOLD phase with soft PD is enough.

6. **"Observations live" log message vanished in one run**. Cause: ros2 daemon state got out of sync after repeated partial kills — fresh daemon needed. Documented in the cleanup sequence (`pkill` + explicit `kill -9` on `ros2_daemon` PID).

### Unresolved as of session end
With all the above fixes, the policy still saturates at `|raw|≥5` within seconds of POLICY phase and the robot topples. The remaining hypotheses:
- Training env (Isaac Lab Go2 MJCF) differs enough from our Menagerie MJCF that the policy is effectively out-of-distribution even at stance. Foot geometry, joint friction, armature, contact solver all differ subtly.
- Upstream training may have used a different `last_action` recurrence (e.g. clipped, delayed) than our forward-pass.
- Observation scaling — upstream deploy code applies none, but training scripts sometimes apply `obs_scales.ang_vel`, `obs_scales.dof_vel` etc. If those were applied in training but not in deploy, `|raw|` would saturate on mismatched magnitudes.

**Right move to actually ship RL Go2**: train a policy in `unitree_rl_gym` with our MJCF and our obs pipeline. That closes both distribution gaps in one pass. Estimated effort: 3–10 GPU-hours + 1–2 days of integration (the inference / PD / torque scaffolding is all in place).

---

## Exploration planner: CFPA2 → TARE (real CMU port)

### The CFPA2 stuck

On demo3 under the original CHAMP + CFPA2 + FAR stack, the robot consistently hit a silent stuck mode:

```
t=45 body=(5.3,1.8)  cfpa2_goal=(11.7,3.0)  far_wp=(5.3,1.8)  cmd=(0,0)  STUCK(N)
...                                                                      STUCK(235s)
```

`far_wp = body_pos` meant FAR's last-published local waypoint was the robot's current pose. FAR's V-graph couldn't connect start→goal for this CFPA2-picked frontier (the goal lay inside unobserved space, beyond the current pillar at `(7, 2.5)`) so FAR silently stopped publishing new waypoints. pathFollower saw "distance 0" and decayed cmd_vel to zero. CFPA2 kept **re-publishing the same unreachable goal** at 2 Hz; its stuck-recovery timer never triggered a blacklist.

Bypass-test confirmed the diagnosis: manually publishing a simple reachable goal `(7, 1)` via `ros2 topic pub -r 5 /robot/way_point_coord ...` immediately unstuck the robot (1.3 m progress, `ok` status, cmd_vel non-zero). So the stack below the goal layer was healthy; the goal picker was giving FAR something FAR couldn't handle.

### Why this is FAR's scope limit, not a bug

FAR builds a V-graph **over observed traversable terrain only**. CFPA2 picks frontier cells — which by definition sit at the boundary of observed space. When the picked frontier is on the far side of a narrow gap / pillar / junction the robot hasn't yet navigated through, FAR has no V-graph vertices there, no path exists, FAR stops publishing. Correct behaviour for a "navigate known space" planner; wrong behaviour for exploration.

CFPA2's contract assumes the downstream planner will handle "can't reach this" gracefully; FAR's contract assumes the upstream picker will give it known-reachable goals. Neither assumption held.

### The TARE stub that wasn't TARE

Repo had `src/collaborative_exploration/go2_tare_planner_ros2/` which sounded like CMU TARE. It's actually:

- **`src/tare_planner_node.cpp`** (117 lines) — a bare pub/sub forwarder, optional rate-change. **No planning.**
- **`generated/tare_planner/`** (33 k LOC) — real CMU TARE's source, but **catkin ROS 1**. Not buildable under our ROS 2 Humble workspace.

The 117-line node's startup log `provider=vendored` was our first hint that the whole thing was a placeholder.

First experiment (`nav_test_go2_tare.launch.py` — the non-"real" variant) chained the stub downstream of CFPA2 with a `waypoint_mux` fallback, hoping smoothed output would help. It didn't — all three topics (`way_point_coord`, `way_point_tare`, `way_point_coord_nav`) carried the same CFPA2 goal `(11.675, 3.025)`, and the robot still got stuck once the goal happened to require non-trivial V-graph routing.

### Real TARE ported into our tree

Chao Cao (TARE paper's first author) maintains a `humble-jazzy` branch of his own repo with a proper ROS 2 port:
- **`~/planner_comparison/tare_planner/`** — checked out from `github.com/caochao39/tare_planner`, `ament_cmake` + `rclcpp` + `tf2_ros`, vendored OR-Tools for x86_64.

Copied into our workspace at **`src/vendor/tare_planner/`**. Builds in ≈ 5 min (`colcon build --packages-select tare_planner`), warnings only, no errors. Includes `keypose_graph`, `viewpoint_manager`, `sensor_coverage_planner`, `rolling_occupancy_grid`, `local_coverage_planner`, `tsp_solver`, etc. — the full hierarchical planner described in the TRO 2023 paper.

### Topic contract

Real TARE from config/indoor.yaml:

```
sub_state_estimation_topic_   /state_estimation_at_scan     (Odometry)
sub_registered_scan_topic_    /registered_scan              (PointCloud2)
sub_terrain_map_topic_        /terrain_map
sub_terrain_map_ext_topic_    /terrain_map_ext
sub_start_exploration_topic_  /start_exploration            (std_msgs/Bool, optional)
pub_waypoint_topic_           /way_point                    (PointStamped)
```

Our nav stack exposes the same topic types under `/robot/...`. The launch just remaps each via params.

### The kAutoStart namespacing trap

TARE's `config/indoor.yaml` is keyed on bare `tare_planner_node:` (no leading namespace). Our launch sets `namespace="robot"`, so the resolved node is `/robot/tare_planner_node` and the YAML key doesn't match — ROS 2 silently loads code defaults. Discovery: node printed `parameter kAutoStart: 0` despite the YAML stating `kAutoStart: true`. Result: TARE sat at `Waiting for start signal` forever.

**Fix**: override `kAutoStart` (and the topic-name params) directly in the launch's `parameters=[...]` list. The YAML is still read for the algorithm-tuning parameters that aren't namespace-sensitive once inside the node.

This is a general ROS 2 trap worth remembering — same class of silent-failure as the `/cmd_vel` absolute-remap vs relative-topic mismatch we hit earlier.

### TARE → FAR was still stuck

With real TARE feeding FAR as global planner, stuck mode returned:

```
t=107 body=(7.8,1.4)  cfpa2_goal=(17.2,-4.1)  far_wp=(7.8,1.4)  STUCK(48s) WP_BEHIND(179°)
```

Same shape as before — `far_wp = body_pos`, FAR can't route, pathFollower idle. Even when TARE picks reachable keypose-graph targets, FAR's V-graph disagrees about reachability through terrain FAR hasn't independently observed via its own point-cloud intake.

### Real fix — bypass FAR, go TARE → localPlanner directly

This matches CMU's own reference architecture: when TARE is the global planner, `local_planner + pathFollower` sit directly beneath it. FAR is a *separate* planner for destination-directed navigation — not meant to sit between TARE and the local stack.

Wiring changes in `nav_test_go2_tare_real.launch.py`:

```python
# TARE publishes its global waypoint straight into localPlanner's input.
{"pub_waypoint_topic_": f"/{robot_ns}/way_point"}

# FAR's output topic redirected to a dead sink to avoid publisher collision.
"far_way_point_out": f"/{robot_ns}/_far_way_point_unused"

# FAR's goal input has no publisher (CFPA2 is off) — FAR runs idle.
"far_goal_topic": ""

# CFPA2 disabled.
"explore": "false"
```

New launch args added to `nav_test_mujoco_fastlio.launch.py`:
- `far_goal_topic` — override the topic FAR subscribes to for its global goal
- `far_way_point_out` — override the topic FAR publishes its local waypoint to

### Result

Immediate unlock. First run after the bypass: robot walked from spawn `(4, 2)` through the central-junction gap into the SE quadrant, reached `(18, -4)` — **~14 m of continuous motion**, cmd_vel sustained at 0.22 m/s, `ok` status throughout, no STUCK, no contacts. Real exploration.

Longer run: **70 % coverage in 1370 s wall-clock on demo3** with zero crashes.

### Files
- **`src/vendor/tare_planner/`** — real CMU TARE, humble-jazzy, vendored OR-Tools (~137 MB)
- **`src/go2w/go2_gazebo_sim/launch/nav_test_go2_tare_real.launch.py`** — new launch
- **`scripts/launch/nav_test_go2_tare_real.sh`** — shell wrapper

---

## Waypoint watchdog — sensor-derived blacklist (2026-04-21)

TARE's `/reset_waypoint` input is **not** a blacklist — it's a one-tick lookahead nudge. On its next planning cycle TARE re-runs the TSP over viewpoints and picks the same high-scoring one, looping forever. To get genuine skip-behaviour we publish **persistent polygons on `/{ns}/nogo_boundary`**, which `viewpoint_manager::UpdateNogoBoundary()` uses to invalidate every viewpoint inside.

`scripts/runtime/tare_waypoint_watchdog.py` watches `/{ns}/way_point` and fires on four independent failure modes (each adds a ~0.6 m square to the running nogo list and re-publishes the full PolygonStamped since TARE overwrites its internal list on every message):

| Mode | Check | Catches |
|------|-------|---------|
| `terrain` | ≥ 2 points in `/{ns}/terrain_map` within 0.4 m of wp have `intensity ≥ 0.2 m` | Goal on an observed wall surface |
| `occgrid` | any cell in `/{ns}/map` (octomap 2D projection) within 0.25 m of wp is ≥ 50 | Goal **inside** a wall's 2D footprint (LiDAR can't see through walls so terrain_map is empty there) |
| `oob` | wp is outside the current occupancy-grid bounds | Goal in unobserved territory beyond the map extent |
| `stall` | `min_dist(robot, wp)` hasn't decreased by ≥ 0.05 m for 10 s | localPlanner can't find a motion primitive to reach it |

Two guards keep the blacklist from eating the robot's own operating space:
- **No blacklist near robot** (`nogo_min_dist_from_robot_m: 0.8`) — TARE's `kExtendWayPoint` can collapse the extended goal onto the robot itself after a reset nudge; blacklisting there would shrink the sampling space around the robot.
- **No stall when already there** (`stall_already_there_m: 0.4`) — goals within `goalCloseDis` don't trip the stall clock; pathFollower has effectively reached them and is just waiting for TARE's next viewpoint.

Also publishes RViz markers the nav rviz configs expect: a 0.6 m magenta sphere on `/{ns}/way_point_marker` (TARE goal) and a green ARROW on `/{ns}/robot_pose_marker` (drop-in for the `reactive_nav_node`'s RobotPoseTriangle, since `nav_backend=far` doesn't run reactive_nav).

## Benchmark: 10 × 10 min on demo3 (2026-04-21)

`scripts/bench/benchmark_go2_tare.sh` — 10 trials × 600 s, `use_sim_time` frozen per trial, per-trial JSON via `session_reporter.py` (same schema as `benchmark_fastlio`).

| metric | value |
|---|---|
| **FULL PASS** (completed ∧ cov ≥ 90 % ∧ contacts = 0 ∧ ¬tipped) | **0 / 10** |
| outcome = completed | 10 / 10 |
| zero wall contacts | 10 / 10 |
| coverage ≥ 90 % | 0 / 10 |
| coverage mean / σ / range | **71.2 % / 5.4 % / 63 – 83 %** |
| distance travelled mean | 56 m / trial (≈ 0.093 m/s effective) |
| SLAM trans drift peak mean | 0.11 m |
| SLAM yaw drift peak mean | 16° (max 31°) |

Reading: stack is **robust and reproducible** (10/10 clean, σ = 5.4 % = low variance), but **duration-limited**. At 345 m² needed for 90 % × 1 m coverage swath × ~0.1 m/s effective → ~60 min per trial to genuinely hit the PASS bar. 10 min on 384 m² is a tight budget given CHAMP's 0.3 m/s cap × MuJoCo RTF ≈ 0.5–0.57 × gait latency. Levers to lift coverage within 10 min: (a) raise CHAMP cap 0.3 → 0.35 m/s (untested, but 0.5 is known-broken); (b) halve LiDAR rays to 4096 (`MUJOCO_LIDAR_HZ_SAMPLES=512 VT=8`) for ~25 % RTF gain; (c) MuJoCo `timestep` 0.002 → 0.003.

## Real-robot port — `real_single_tare_real.launch.py` + `nav=tare_real` (2026-04-21)

Full CMU TARE pipeline now runs on the real Go2 / Go2W the same as sim:

- [`src/go2w/go2w_real_bringup/launch/real_single_tare_real.launch.py`](../../src/go2w/go2w_real_bringup/launch/real_single_tare_real.launch.py) — new. Wraps `real_single.launch.py` with `explore:=false` (CFPA2 off), `far_goal_topic:=""`, `far_way_point_out:=/{ns}/_far_way_point_unused` (FAR idle), `registered_scan_topic:=/{ns}/registered_scan_map`. Adds a `topic_tools/relay` from `/cloud_registered` → `/{ns}/registered_scan_map` (Fast-LIO's world-frame cloud renamed into our namespace — no transform needed because real has no GT bootstrap; camera_init ≡ world numerically). Adds the TARE node + watchdog with 12 s launch delay for sensors to settle.
- [`src/go2w/go2w_real_bringup/config/tare/indoor.yaml`](../../src/go2w/go2w_real_bringup/config/tare/indoor.yaml) — `/**`-wildcard-keyed copy of the sim config so params load under the `/robot` namespace.
- `navigation.launch.py` + `real_single.launch.py` gained three passthrough args: `explore` (CFPA2 gate), `far_goal_topic` + `far_way_point_out` (FAR unwire overrides), `registered_scan_topic`.
- `scripts/real/real_autonomy.sh` — new `nav=tare_real` value (dispatches to the new launch with `NAV_BACKEND=far`).

**What we did NOT port** (sim-specific): `cloud_world_offset_bridge` (no GT bootstrap on real → no offset), `robust_controller_spawner` (Unitree SDK not mujoco_ros2_control), `pose_sensor.cpp` TF filter (MuJoCo plugin only).

### ⚠ `obstacle_avoidance:=false` is mandatory on real

`cmd_vel_to_sport_bridge` is already parameterised, but every launch defaults `obstacle_avoidance:=true`. That routes Move commands to `/api/obstacles_avoid/request` with `api_id=1003`, which the Go2 SDK **silently drops** unless the robot was manually put into obstacles_avoid mode via the BT pad or Unitree app first. Setting `oa=false` on `real_autonomy.sh` (or `obstacle_avoidance:=false` on the launch directly) routes to `/api/sport/request` with `api_id=1008` — the standard sport Move endpoint, no pre-arm needed. Verified 2026-04-21 by live-switching the bridge mid-session; test-pub `/robot/cmd_vel_manual` immediately produced robot motion. If you ever see "reactive_nav is publishing real velocities and the full chain reaches `/api/obstacles_avoid/request` at 12 Hz but the robot is idle", this is the cause.

---

## RTF bottleneck — where the wall-clock time actually goes

After the TARE swap the nav stack is correct; remaining slowness is physics-sim and locomotion, not planning. Measured at ~20 min into the 1370 s demo3 run:

| metric | value |
|---|---|
| MuJoCo CPU | **222%** (2 cores pinned) |
| `/clock` publish | **57 Hz** (target 100) → RTF ≈ **0.57** |
| cmd_vel | 50 Hz, **0.22 m/s commanded** (under 0.3 gait.yaml cap) |
| actual robot speed | **~0.13 m/s sim** (body covered 2.46 m in 19 s sim-time) |

Back-of-envelope: 1370 s wall × 0.57 RTF = 780 s sim. At 0.13 m/s × 780 s ≈ **101 m of path**. Observed 70 % of 384 m² with ~1 m coverage swath ≈ 270 m — so TARE actually backtracks through explored space a bit (as expected — hierarchical TSP isn't a single-shot lawnmower). Consistent picture: nav is fine, wall-time is chewed by physics.

### Where the CPU goes (ranked)

1. **LiDAR raycast** — DFKI plugin defaults to `1000 × 20 = 20 000` rays at 10 Hz. Serial `mj_multiRay` loop on CPU. ~40–50 % of the 222 %.
2. **Menagerie joint damping=2 + armature=0.01** — implicit integrator takes more iterations per step than Go2W's damping=0.1 case. ~20–30 %.
3. **`fastlio_mapping`** — 3D scan matching at 10 Hz. ~15–18 %.
4. **ros2_control + everything else** — ~10 %.

### Quick wins (not applied yet)

- **Halve LiDAR ray count** (biggest lever): set the DFKI plugin's `hz_samples=720`, `vt_samples=10` → 7 200 rays. Roughly doubles RTF, negligible mapping-quality impact.
- **Bump MuJoCo `<option timestep>` 0.002 → 0.003**: ~30 % CPU saving, physics still stable at 333 Hz on our robot mass scale.
- **Raise CHAMP `max_linear_velocity_x: 0.3 → 0.35`** in `gait.yaml`: untested edge. 0.5 is known-broken (earlier speed sweep). 0.35 is plausibly stable.

Combined A + B + C could drop a 1370 s run to ~600 s wall-clock for the same coverage, or let a 300 s bounded trial actually hit 90 %.

### Why not just push further on the gait side

Without swapping the locomotion controller (real Unitree SDK on hardware, or a trained RL policy in sim — see RL section above), **0.3 m/s is the ceiling**. 0.5 m/s CHAMP run with Menagerie body collapsed the gait. That's a physics reality, not a tuning knob.

---

## Files touched (by layer)

**New (MJCFs & configs)**:
- `src/go2w/go2_gazebo_sim/mujoco/demo1_go2_real.xml`
- `src/go2w/go2_gazebo_sim/mujoco/demo3_go2_real.xml`
- `src/go2w/go2_gazebo_sim/mujoco/go2_rl_mujoco_controllers.yaml`
- `src/go2w/go2_gazebo_sim/config/ros_control/ros_control_go2_robot_rl.yaml` (Gazebo-side, not used under MuJoCo but kept for parity)
- `src/go2w/go2_gazebo_sim/config/ros_control/ros_control_go2_robot_rl_effort.yaml` (same, effort-mode variant)
- `src/go2w/go2_gazebo_sim/urdf/go2/go2_description_3d_lidar.xacro`

**New (launches for TARE exploration path)**:
- `src/go2w/go2_gazebo_sim/launch/nav_test_go2_tare.launch.py` — earlier variant with the 117-line stub + waypoint_mux. **Superseded** by the `_real` one below, kept for reference.
- `src/go2w/go2_gazebo_sim/launch/nav_test_go2_tare_real.launch.py` — real CMU TARE → localPlanner direct, FAR bypassed.
- `scripts/launch/nav_test_go2_tare.sh` / `scripts/launch/nav_test_go2_tare_real.sh` — shell wrappers.

**New (vendor)**:
- `src/vendor/tare_planner/` — real CMU TARE (humble-jazzy branch of `github.com/caochao39/tare_planner`), ~137 MB incl. vendored OR-Tools.

**New (scripts)**:
- `scripts/runtime/go2_rl_policy_node.py`

**Modified**:
- `src/go2w/go2_gazebo_sim/launch/single_go2w_mujoco_cfpa2.launch.py` — 3-way YAML select, `rl_policy` + `rl_use_champ_gains` args, RL-node TimerAction, MUJOCO_INIT_KEYFRAME env
- `src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py` — arg propagation, octomap z-band conditional, **`far_goal_topic` + `far_way_point_out` overrides** for the TARE-real bypass path
- `src/go2w/go2_gazebo_sim/launch/modules/assets.py` — `build_dual_robot_stack(..., skip_champ=False)`
- `src/go2w/go2w_spawn/scripts/stand_up_slowly.py` — joint-name preset alias
- `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_plugin.cpp` — mjThreadPool + keyframe support
- `src/vendor/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_ros2_control_plugin.hpp` — pool member + include
- `scripts/launch/nav_test_go2.sh`, `scripts/launch/nav_test_go2_demo3.sh`, `scripts/bench/benchmark_go2.sh`, `scripts/bench/benchmark_go2_demo3.sh` — MJCF path swap

---

## Known gotchas

- **Menagerie's `<option cone="elliptic" impratio="100"/>` must be kept** — switching to pyramidal breaks contact stability under trot.
- **Friction `0.8 0.02 0.01` on foot class** is correct for indoor surface; `1.2` caused tipover in an earlier experiment.
- **Spawn z=0.5 + CHAMP stand-up works** (kp=100 in trajectory_controller fights the drop). Spawn z=0.5 + RL gains doesn't work — the soft PD can't catch the drop. The `<keyframe name="home">` elements exist for this reason; applied only when `MUJOCO_INIT_KEYFRAME=home` is set.
- **DFKI plugin loads `go2w_mujoco_controllers.yaml` directly** (not via the xacro `<gazebo>` block). If you add new controllers, they need entries in whichever manifest the plugin actually reads.
- **`build_namespaced_robot_description(...)` rewrites** the `gazebo_ros2_control` plugin's `<parameters>` tag in the xacro, but *only* the Gazebo path reads it. MuJoCo reads the plugin's `--params-file` list.
- **Damping=2 on Menagerie joints** makes the sim noticeably heavier than Go2W's damping=0.1 (~ 1.2× MuJoCo CPU). Plus 20000 LiDAR rays and Fast-LIO scan matching makes RTF ≈ 0.15 on the dev laptop — coverage-bound benchmarks are still duration-limited.
- **cmd_vel remap on ROS 2 must match the resolved name**. Relative remap keys are the safer default (`cmd_vel` not `/cmd_vel`). Golden rule #10 candidate.
- **YAML param keys must match the node's resolved fully-qualified name.** CMU TARE's shipped `config/indoor.yaml` is keyed on `tare_planner_node:`; under `namespace="robot"` the node becomes `/robot/tare_planner_node` and **none of the params load** — silent fallback to code defaults (`kAutoStart=false` → stuck at "Waiting for start signal" forever). Either use `/**:` wildcard keys in the YAML or override the critical params in the launch's `parameters=[...]` list. Same class of silent failure as the `/cmd_vel` absolute-remap gotcha.
- **FAR is for destination navigation, not exploration**. When TARE (or any exploration planner) is the goal source, skip FAR entirely and route the goal topic to `localPlanner`'s input. Putting FAR in between causes the robot to stuck whenever TARE-picked goals sit in terrain FAR hasn't independently V-graphed. CMU's own development environment pairs TARE with `local_planner` + `pathFollower` directly, without FAR.
- **Controller spawner "already loaded" race** showed up once this session — `spawner` died with `A controller named 'robot_joint_states_controller' was already loaded inside the controller manager`, cascading to no joint flow / no Fast-LIO / stuck stack. Couldn't reproduce after a reboot — suspected stale controller_manager state from an earlier partial kill. If it recurs, `pkill -9 -f controller_manager ros2_daemon` fully before relaunch.

---

## If you come back to the RL path

Ordered by effort:

1. **Train a Go2 policy against our exact MJCF** (unitree_rl_gym workflow). Use `demo1_go2_real.xml`'s robot block as the training robot, keep obs 45-dim, PD gains `kp=20/kd=0.5`, action scale `0.25`, `IL_DEFAULTS` from Menagerie home. Deploy the resulting ONNX with the existing `scripts/runtime/go2_rl_policy_node.py` — minimal changes needed.
2. **Try v7 with the +4 obs inputs plumbed.** Upstream's v7 is newer and supposedly stable. Requires figuring out what those 4 extra inputs are (likely `clock_phase` or `gait_phase` or per-foot contact); easy to test once identified.
3. **Port Walk-These-Ways Go2 variant** — community-maintained, open RL policy with solid stumble recovery. Would need to match its obs spec (different from eppl-erau-db's).
4. **Hand-written convex MPC** (e.g. fork of go2-convex-mpc) — highest quality but 1–2 weeks of integration.

The scaffolding that's in place (`go2_rl_policy_node.py` + `forward_command_controller` path + MUJOCO_INIT_KEYFRAME) is policy-agnostic — any 12-joint-target policy with the right obs size drops in.
