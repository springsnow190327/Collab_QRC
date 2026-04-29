# Nav2 MPPI migration journey (2026-04-27 → 2026-04-29)

## Context — why a third migration

Two prior nav backends had been ruled inadequate for dual-robot demo3_mixed:

1. **A\* (`astar_nav_node`)** — 2D footprint check on costmap. Accumulated band-aids (pivot-relief, body-clip escape, head-tip extension, brake-priority, fast-BL debounce). Could not see pillars at z < 0.20 m, head extension, or the leg-swing envelope. Retired for dual-robot 2026-04-26 (see CLAUDE.md "Active state (2026-04-26)").
2. **FAR (CMU stack)** — 3D voxel terrain analysis fixed the 2D blindness, but FAR's V-graph occasionally connects two contour vertices through walls (sparse contour sampling, `terrain_free_Z` thresholds, sensor-range edge effects), and pathFollower can rotate body-into-wall when held goal demands an unsafe pivot. The 3-layer safety stack added 2026-04-26 (CFPA2 pivot-lock + path_safety_filter + cmd_vel_safety_shield) stabilised the run but didn't eliminate scuffs and was prone to multi-layer deadlock.

This doc captures the third migration to **Nav2 MPPI + SmacPlannerHybrid**, the bugs found along the way, and the two real-robot blockers that remain after sim verification.

## Final architecture (per namespace)

```
Sim (MuJoCo)
  ├─ Fast-LIO 2 (slam_node)            — /robot_a/Odometry  (10 m drift, no LC)
  ├─ mujoco_odom_bridge                 — writes odom→base_link TF directly from GT
  ├─ slam_odom_relay                    — /robot_a/odom/nav (frame=world)  ← consumed by 6 nodes
  ├─ octomap_server                     — /robot_a/map  (uses Fast-LIO TF, self-consistent)
  ├─ map_augmenter                      — /robot_a/map  (merged with peers)
  ├─ Nav2 stack (PushRosNamespace=robot_a)
  │    ├─ planner_server  (SmacPlannerHybrid REEDS_SHEPP)
  │    ├─ controller_server  (MPPIController, polygon footprint, consider_footprint=true)
  │    ├─ behavior_server  (Spin / BackUp / Wait)
  │    ├─ bt_navigator
  │    └─ lifecycle_manager_navigation
  ├─ cfpa2_to_nav2_bridge               — way_point_coord (PointStamped) → goal_pose (PoseStamped)
  ├─ path_relay                         — /plan → /planned_path  (RViz back-compat)
  ├─ stuck_watchdog                     — 10s no-motion w/ active goal → Nav2 BackUp + republish goal
  ├─ go2w_hybrid_cmd_router  (Go2W only, skipped for Go2)
  └─ ros2 bag record                    — cmd_vel chain + plan + waypoint + goal + collisions

Shared
  └─ cfpa2_coordinator                  — frontier extraction + MDVRP allocation + pivot-lock
```

Two yamls, parameterised by `has_wheels`:
- [`nav2_go2w_full_stack.yaml`](../../src/go2w/go2w_config/config/nav/nav2_go2w_full_stack.yaml) — Go2W (wheels), 0.70 × 0.40 m footprint, vx_max 0.50, min_turning_radius 0.30
- [`nav2_go2_full_stack.yaml`](../../src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml) — Go2 (legged-only), 0.65 × 0.30 m footprint, vx_max 0.30, min_turning_radius 0.05

## Bugs found (in chronological order of discovery)

### 1. MPPI footprint trap

**Symptom**: Robot stamps in place at the entrance to demo3 0.425 m narrow corridor. Plan exists, but MPPI emits (v ≈ 0, ω ≈ low) for minutes.

**Diagnosis**: With the original yaml (`robot_radius: 0.40 m`, `consider_footprint: false`, `collision_margin_distance: 0.20 m`), MPPI's effective rejection envelope was `0.40 + 0.20 = 0.60 m` from any obstacle centre. The corridor is 0.425 m wide; from centerline to wall is 0.213 m on each side. Every forward-motion sample had at least one predicted pose within 0.60 m of a wall → `collision_cost: 10000.0` penalty → MPPI selected v ≈ 0.

**Fix**: Switch to polygon footprint + per-pose rotated-rectangle collision check.

```yaml
local_costmap:
  ros__parameters:
    footprint: "[[0.35, 0.20], [0.35, -0.20], [-0.35, -0.20], [-0.35, 0.20]]"
    footprint_padding: 0.03
    inflation_radius: 0.45  # for cost gradient, not collision

controller_server:
  FollowPath:
    ObstaclesCritic:
      consider_footprint: true
      collision_cost: 10000.0
      collision_margin_distance: 0.03  # tiny safety pad on top of polygon check
```

The yaml comment claiming `consider_footprint: true` *throws at controller-configure-time on humble 1.1.20* turned out to be **wrong** — re-reading the source ([nav2_costmap_2d/costmap_2d_ros.hpp](file:///opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp)), `padded_footprint_` is filled at `Costmap2DROS::on_configure()` from either `robot_radius` or `footprint:` parameter. The throw observed earlier was a polygon-string parse error from a malformed yaml; with a clean polygon string, MPPI configures fine.

**Verification log line**: `ObstaclesCritic instantiated with 1 power and 20.000000 / 1.500000 weights. Critic will collision check based on footprint cost.`

### 2. Wheel brake bug — ctrl-cache miss in vendored mujoco_ros2_control

**Symptom**: `wheel_velocity_controller/commands` carries `[0, 0, 0, 0]`, but `joint_states.velocity[FL_foot_joint] ≈ 4–5 rad/s` for several seconds. Wheels coast despite cmd=0.

**Misdiagnosis path** (half a day lost):
1. Suspected MPPI sending non-zero velocity → bag analysis disproves
2. Suspected hybrid_cmd_router holding stale wheel_cmd → router publishes 0 cleanly at 20 Hz
3. Suspected `<velocity kv=5>` actuator gain too low → bumped to kv=50, no effect
4. Suspected actuator type mismatch with ros2_control velocity HW interface → switched MJCF wheel actuators from `<velocity>` to `<motor>` (effort), added per-joint PID gains in xacro `<param>` tags. **Side effect**: sim RT factor crashed from 0.95× to 0.035× (25× slower); xacro+MJCF reverted.

**Real cause**: [`src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp:466-470`](../../src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp#L466-L470).

```cpp
// VELOCITY actuator branch (BEFORE)
if (actuators.find(VELOCITY) != actuators.end()) {
    if (velocity != joint.last_command) {            // last_command never updated!
        mujoco_data_->ctrl[actuators[VELOCITY]] = velocity;
    }
}
```

`last_command` is declared `double last_command;` in `JointData` struct ([mujoco_system.hpp:270](../../src/vendor/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp#L270)) with no default initialiser. POSITION branch updates it (line 441), EFFORT branch is a no-op (it always writes ctrl). VELOCITY branch checks but **never updates**.

Sequence:
1. `last_command` starts at ~0 (uninitialised double, often 0 from zeroed memory)
2. Controller commands 5 rad/s: `velocity (5) != last_command (0)` → write ctrl=5. `last_command` stays 0.
3. Subsequent commands of 5: redundant writes (still ≠ 0).
4. Controller commands 0: `velocity (0) == last_command (0)` → **skip write**. ctrl stays at 5. Wheels keep spinning.

**2-line fix**:
```cpp
// VELOCITY actuator branch (AFTER)
if (actuators.find(VELOCITY) != actuators.end()) {
    if (velocity != joint.last_command) {
        joint.last_command = velocity;               // ← update the cache
        mujoco_data_->ctrl[actuators[VELOCITY]] = velocity;
    }
}
```

Plus NaN-init in struct so first write always fires:
```cpp
double last_command{std::numeric_limits<double>::quiet_NaN()};  // first cmd != NaN → always write
```

Verification (post-fix): cmd 5 rad/s → wheels reach +5; cmd 0 → wheels decelerate to ~0.7 rad/s in 3 s (residual decel from ground friction + body drag), RT factor 0.99 ×.

**Lesson**: when the data clearly shows a chain breaking and every node reports it's publishing the right thing, stop tuning higher layers and grep the lowest layer's source. The lowest-level vendored code can have real bugs.

### 3. Freewheel-in-legged

**Symptom** (after fix #2): legged-mode walking is *worse* than before. Wheels skid on the ground while CHAMP walks; observed by user as "leg-mode skidding".

**Cause**: With the brake fixed, the velocity actuator now correctly applies up to ±15 N·m brake torque (kv=50 × clamped error → ctrlrange ±15). When the router publishes wheel_cmd=0 in legged mode and CHAMP's leg gait carries the body forward at ~0.3 m/s, ground friction tries to roll the wheels. The actuator's brake torque opposes the roll → wheel skids against the ground.

**Fix**: In legged/idle mode, publish `wheel_cmd = current actual ω` (mirror) instead of 0. Error → 0, no brake torque, wheels passively roll. [`go2w_hybrid_cmd_router.py`](../../src/go2w/go2w_control/scripts/go2w_hybrid_cmd_router.py) added a `JointState` subscription to `/mujoco_sim/joint_states`, caches the four foot_joint velocities, and uses them as the freewheel setpoint.

```python
if self.freewheel_in_legged:
    wheel_cmd.data = list(self._latest_wheel_vels)  # mirror, no brake
else:
    wheel_cmd.data = [0.0, 0.0, 0.0, 0.0]
```

Verification: in legged mode under CHAMP gait, FL wheel actual ω = +0.39 rad/s, FL cmd = +0.44 rad/s, residual brake = `kv × |diff| = 50 × 0.05 = 2.5 N·m` (down from ~15 N·m).

### 4. CFPA2 frontier filters need to mirror the footprint

**Symptom**: After fix #1+#2 robot_a navigates demo3 successfully, but CFPA2 reports `fronts=0 → exploration_complete` at 79.5 % cov with 153 k unknown cells still on the map.

**Diagnosis**: Probed `/merged_map` directly:
- 167 valid frontier cells (free cell adjacent to unknown) exist
- After clustering: 55 connected components, largest area = 0.15 m²
- Default `cfpa2_frontier_min_cluster_area_m2 = 0.20 m²` → all clusters rejected → `fronts=0`

The frontier filters were calibrated to the old 0.40 m circle approximation. With the new tighter polygon footprint (effective half-width 0.20 m), the robot can navigate into narrower geometry where frontiers are inherently smaller and closer to walls. Three coupled thresholds need to drop together:

| param | old | new | reason |
|---|---|---|---|
| `cfpa2_frontier_min_cluster_area_m2` | 0.20 m² | **0.05 m²** | Late-stage frontiers are fragmented; 0.05 (= 20 cells at 0.05 m res) still filters speckle |
| `cfpa2_frontier_obstacle_clearance_m` | 0.40 m | **0.25 m** | Was 0.40 to mirror old robot_radius=0.40; new half-width 0.15 + 0.10 buffer = 0.25 |
| `cfpa2_frontier_unknown_check_radius_m` | 0.40 m | **0.30 m** | Same logic; smaller radius fits in narrow geometry |

**Live-tuning gotcha**: CFPA2 has **no `add_on_set_parameters_callback`** registered. `ros2 param set` only updates the parameter store; cached `self.cfpa2_*` values are frozen at init. Verified by setting the params live and watching `fronts=0` persist — only after editing yaml + restarting `cfpa2_coordinator` did frontier extraction recover.

**Lesson**: footprint changes ripple through every distance-style threshold in CFPA2. Treat them as a coupled set.

### 5. SLAM drift mystery — properly resolved

**Observation**: GT vs `/odom/nav` shows ~10 m offset on x. RViz trajectory drifted out of the arena, but the planned path on the map starts from the robot's actual position.

**False starts**:
1. **"SC-PGO false-positive loop closure"** — I proposed it; user agreed. Both wrong.
   - Verification: `ros2 topic info /corrected_odom --verbose` → `Publisher count: 0`.
   - SC-PGO never ran. The launch ([nav_test_mujoco_fastlio.launch.py:249-264](../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio.launch.py#L249-L264)) hardcoded paths to `/home/hz/COMP0225_LRC_stack/...` (a previous developer's machine), and silently skipped if not found. Has been silent-skipping for months.
2. **"Map should be ghosted if SLAM drifted"** — the user observed correctly: *the map is clean, only the trajectory is messy*. This is the **co-drift property of LIO**: octomap projects lidar scans through Fast-LIO's TF; if Fast-LIO drifts, both the robot pose marker AND the wall observations drift the same way → relative geometry preserved → map looks self-consistent. The trajectory is messy because it's drawn from `/odom/nav` (slam_odom_relay output), which has different drift dynamics than the TF chain that the planner uses.

**Real cause**: pure Fast-LIO 2 ICP scan-matching drift, accumulated open-loop. ~10 m on x over ~7 minutes. Yaw stays aligned (-126° both sources). Not a bug — just SLAM doing what unbounded SLAM does.

**Why nav still works**: SmacPlannerHybrid reads pose from TF (`map → base_link`), which in sim is anchored to GT via `mujoco_odom_bridge`. So plans start from the actual robot position. MPPI also tracks the actual pose via TF. Only nodes that subscribe `/odom/nav` directly (cfpa2_coordinator, bt_navigator, cfpa2_to_nav2_bridge, stuck_watchdog) see the drifted pose — they make decisions on a 6 m offset frame, but their decisions are fed downstream where TF takes over.

**`loop_closure:=true|false` toggle scaffolded**:
- Vendored `engcang/FAST-LIO-SAM` to [`src/vendor/sc_pgo/`](../../src/vendor/sc_pgo/) (ROS 1 source, `COLCON_IGNORE` placed).
- Launch arg `loop_closure:=true` attempts to spawn `sc_pgo_node` per-namespace; `slam_odom_relay` already prefers `/corrected_odom` when fresh.
- Currently warn-skips silently because the source needs ROS 2 humble porting (catkin → ament_cmake + API translation). See [`src/vendor/sc_pgo/PORT_TO_ROS2.md`](../../src/vendor/sc_pgo/PORT_TO_ROS2.md) for the porting checklist.

### 6. Stuck recovery — outer-loop watchdog

**Symptom**: Robot wedges with active goal, sits idle for minutes. CFPA2's pivot-lock refuses to change goal (clearance < 0.45 m); MPPI emits (v=0, ω=0) without reporting failure → BT recovery never fires.

**Why Nav2 BT recovery doesn't help**: Nav2's default BT runs `Backup → Spin → Wait` only when the controller reports failure to the action client. MPPI in narrow-pivot scenarios is "happy" — it *did* find a sample (just one with v ≈ 0) → reports success per cycle → BT never enters recovery branch.

**Why CFPA2's `pivot_lock_max_hold_sec` doesn't help**: declared in source ([cfpa2_coordinator_node.py:465](../../src/collaborative_exploration/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py#L465)) but **never used** anywhere in the lock-decision code path (line 1514-1528). Dead parameter. Robot_a was wedged for 220 s before user noticed; no escape valve fired.

**Fix**: External watchdog — [`scripts/runtime/stuck_watchdog.py`](../../scripts/runtime/stuck_watchdog.py).

```python
# Subscribes:
#   /<ns>/odom/nav        — pose for stuck detection
#   /<ns>/goal_pose       — only act when goal is active
# Action client:
#   /<ns>/backup          — Nav2 BackUp (already in behavior_server)
# Publishes:
#   /<ns>/goal_pose       — re-issue cached goal after BackUp finishes
#                          (forces SmacHybrid to replan from new pose)
```

Logic: 10 s rolling window; if `|displacement| < 0.20 m` AND active goal AND > 0.5 m from goal AND cooldown elapsed → fire BackUp action with `target.x = -0.40 m, speed = 0.10 m/s, time_allowance = 8 s`. On completion (success or abort), republish the cached goal with a fresh timestamp; bt_navigator treats a re-published goal_pose as a preempt → SmacHybrid replans from the post-backup pose.

Per-namespace, real-robot-compatible (uses Nav2's existing behavior_server / BackUp action that's already in our yamls).

**Verified working**:
```
[stuck_watchdog ns=robot_b] STUCK detected: moved 15.3 cm in 10 s, goal at (+8.26,+7.65)
                                d2g=14.07 m — triggering BackUp + replan
[stuck_watchdog ns=robot_b] BackUp accepted, awaiting result.
[stuck_watchdog ns=robot_b] BackUp finished (status=6); republished goal (+8.26,+7.65)
                                to force replan.
```

**Caveat**: BackUp itself collision-checks `simulate_ahead_time × backup_speed = 0.20 m` of clearance behind the robot. If the robot is genuinely wedged with walls on both sides (`Collision Ahead - Exiting DriveOnHeading`), BackUp aborts and watchdog still fires the goal republish, but no actual motion happens. Last-resort raw-cmd_vel-pulse mode not yet wired.

## Aside: TF chain primer (REP-105) — what `map → odom → base_link` means

Required reading for understanding both blockers below. The full chain is `map → odom → base_link`, and **each segment has a distinct semantic role**:

```
map           ← world-fixed, SLAM-corrected, drift-stable
 │
 │  map → odom: SLAM's "drift correction" transform.
 │            Static identity when no LC; jumps when SLAM closes a loop.
 │
odom          ← continuous, smooth, dead-reckoning frame. Only ever
 │              accumulates; long-term drifts but never jumps.
 │
 │  odom → base_link: robot body's current pose in the odom frame.
 │            High-frequency (100-500 Hz), smooth integration of
 │            wheel/leg encoders + IMU. The control loop reads this.
 │
base_link     ← robot body (rigid)
```

**Worked example**. Robot starts at (0, 0), drives 10 m straight, actually ends at (10, 0):

- After 10 m, no drift yet:
  - `odom → base_link = (10, 0)`
  - `map → odom = identity`
  - Robot in map = identity × (10, 0) = **(10, 0)** ✓

- After 100 m, IMU + leg integration has drifted by 1 m:
  - `odom → base_link = (101, 0)` (odometry thinks it travelled 101 m)
  - SLAM observes via lidar: "I'm actually at (100, 0)"
  - SLAM updates `map → odom = (-1, 0)` (the correction shift)
  - Robot in map = (-1, 0) + (101, 0) = **(100, 0)** ✓
  - **odom didn't jump** — only `map → odom` did. Control loop reading `odom → base_link` sees a continuous trajectory.

**Why two layers and not just `map → base_link` directly?**

- Control loops (PID, MPPI, pathFollower) need a **high-frequency, smooth** pose. If pose jumps 1 m on a loop closure, the controller's derivative-of-error explodes → spurious commands. So the controller reads `odom → base_link`, which is guaranteed smooth.
- SLAM is **low-frequency + accurate**. Loop closures *do* jump (that's the whole point — they correct drift in one shot).
- The two-layer design **absorbs SLAM jumps in the `map → odom` segment**, leaving the control loop's view of `odom → base_link` continuous. That's REP-105's central insight.

**Our setup** ([nav_test_mujoco_fastlio_mixed.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py)):

| segment | how it's published | drifts? |
|---|---|---|
| `world → map` | static identity (multi-robot namespace book-keeping; for single-robot map ≡ world) | no |
| `map → odom` | static identity ← **no SLAM correction layer**. SC-PGO would update this if it ran | accumulates with no LC |
| `odom → base_link` | sim: `mujoco_odom_bridge` (writes GT). Real: was supposed to be EKF, but EKF starves on the NaN | sim no, real broken |

So robot's pose in map = `map → odom (identity)` × `odom → base_link (mujoco_odom_bridge in sim)` = MuJoCo GT. That's why everything looks correct in sim — the GT is being injected at the only place it can be.

The sim privilege ends at the moment we deploy real. There's no `mujoco_odom_bridge`. The EKF needs `/odom/raw` from CHAMP to function, and CHAMP is broken (Blocker A). The `map → odom` layer is identity (Blocker B's pose-source mismatch becomes the only drift correction we get). Both blockers exist *only because* sim's GT injection masks them.

## Real-robot blockers — resolved (2026-04-29 PM)

Both blockers below were originally identified mid-journey and are
**now bypassed** by the `fast_lio_tf_adapter` shipping in the same
session. Section retained for historical reference + as a worked
example of what "sim-only privilege" looks like in this codebase.

### Blocker A: CHAMP state_estimation NaN

**Symptom**: `/robot_a/odom/raw` (CHAMP's leg-odometry estimate) publishes `pose.position.x: 7.802912350277346e+34, y: 1.49e-19`. Bonkers.

**Why sim doesn't notice**: The TF chain `odom → base_link` is written by `mujoco_odom_bridge` directly from MuJoCo's pose sensor (sim privilege), bypassing the EKF chain entirely. EKF (`footprint_to_odom_ekf`) subscribes to `/odom/raw` + `/imu/data` and *would* normally publish the canonical TF; with garbage on `/odom/raw` the EKF emits no `/odom`, but the sim's mujoco_odom_bridge fills in.

**Why real-robot can't**: No mujoco_odom_bridge. EKF starves on the same NaN. `odom → base_link` TF never gets written. Nav2 has no robot pose → broken.

**Pragmatic fix** (not yet done): drop the EKF + leg-odom chain entirely. Have Fast-LIO publish the TF `map → base_link` directly via a small adapter:
1. Write `scripts/runtime/fast_lio_tf_adapter.py` (~40 lines): subscribe `/robot_a/Odometry`, republish as TF with frame names `map → base_link`.
2. Remove EKFs from launch.
3. Stop sinking Fast-LIO's TF (`/tf:=/{ns}/fastlio_tf_sink`); let the adapter handle it.
4. Keep `mujoco_odom_bridge` as a **GT publisher only** for collision_monitor / session_reporter; don't put it on /tf.

On real, the adapter chain becomes the only TF source. Drift inherits from Fast-LIO (10 m/run open-loop), but every consumer sees the same drifted pose → co-drift saves us. SC-PGO (when ported) eliminates the drift.

### Blocker B: `/odom/nav` consumed by 6 nodes, drifts vs TF

```
/odom/nav subscribers:
  - cfpa2_coordinator        (frontier utility distance — 6 m off skews allocation)
  - bt_navigator [robot_a]   (decision pose — but the *plan* uses TF, so impact subtle)
  - bt_navigator [robot_b]
  - cfpa2_to_nav2_bridge     (orientation synthesis)
  - stuck_watchdog           (the new one — uses pose for stuck detection)
```

None match the TF that the planner uses. In sim they're all consistently 6 m off; in real, the offset depends on Fast-LIO drift dynamics.

**Pragmatic fix** (originally proposed): add a `pose_from_tf: bool` mode to `slam_odom_relay`. When true, the relay's *pose* comes from TF lookup; *twist* from the input topic.

**Actually shipped** (cleaner than the originally-proposed patch): wrote a brand-new node [`scripts/runtime/fast_lio_tf_adapter.py`](../../scripts/runtime/fast_lio_tf_adapter.py) that **owns both the TF and the topic**, replacing slam_odom_relay (and the EKF/mujoco_odom_bridge TF roles) entirely. See "TF adapter — both blockers resolved" below.

## TF adapter — both blockers resolved (2026-04-29 PM)

Wrote [`scripts/runtime/fast_lio_tf_adapter.py`](../../scripts/runtime/fast_lio_tf_adapter.py) (~250 lines). Single node per namespace that:

- subscribes `/<ns>/Odometry` (Fast-LIO raw), `/<ns>/odom/ground_truth` (sim GT for one-shot bootstrap), `/<ns>/corrected_odom` (SC-PGO, prefer when fresh)
- on first matched (raw, GT) pair: computes static alignment offset (Δx, Δy, yaw) so the published pose's frame origin coincides with world origin
- per Fast-LIO message: applies offset, publishes Odometry on `/<ns>/odom/nav` AND broadcasts `odom → base_link` TF (with `/tf`/`/tf_static` remapped to per-namespace topics)
- on real robot: `bootstrap_from_gt:=false` (no GT to align to), or use a one-shot known-spawn-pose param

Launch changes ([nav_test_mujoco_fastlio_mixed.launch.py](../../src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py)):

| change | why |
|---|---|
| Spawn `fast_lio_tf_adapter` per ns (replaces `slam_odom_relay`) | Single source of truth for robot pose: TF + topic come from one place |
| `mujoco_odom_bridge` `publish_tf: True → False` | Sim-only privilege; bridge keeps publishing the *topic* `/<ns>/odom/ground_truth` for collision_monitor / GT bootstrap, no longer writes TF |
| Adapter cmd has `-r /tf:=/<ns>/tf -r /tf_static:=/<ns>/tf_static` | TransformBroadcaster defaults to global `/tf`; without this remap, namespaced consumers don't see the adapter's TF (`Could not find a connection between 'odom' and 'base_link'`) |
| Adapter `output_frame_id:=odom` (not `map`) | Matches local_costmap's `global_frame: odom`; static `map → odom = identity` connects to map frame; alignment offset is baked into the published pose so `odom` frame's origin = world origin |
| EKF nodes (`base_to_footprint_ekf`, `footprint_to_odom_ekf`) | Still spawn (left in `build_dual_robot_stack` for legacy reasons), but starve on `/odom/raw` NaN. No output → harmless. Could be removed in a later cleanup. |

**Verification post-fix** (live data, robot_a after ~6m of nav):

```
GT (MuJoCo truth)                : (+10.517, +1.952)
/robot_a/odom/nav (adapter out)  : (+10.503, +2.027)
TF odom → base_link              : (+10.503, +2.027)
```

All three within 8 cm — the residual is Fast-LIO's instantaneous tracking lag, not accumulated drift. Six topic consumers (cfpa2_coordinator, both bt_navigators, cfpa2_to_nav2_bridge, stuck_watchdog) now see the same pose Nav2's planner sees via TF.

### Frequency

Adapter is 1:1 relay → output rate = Fast-LIO rate = lidar rate = **10 Hz** (MuJoCo lidar 10 Hz; real Mid-360 also 10 Hz).

| consumer | period | 10 Hz adequate? |
|---|---|---|
| Nav2 SmacPlannerHybrid (planner) | 1 Hz replan | ✓ huge headroom |
| Nav2 MPPIController | 20 Hz inner, `transform_tolerance: 0.2 s` | ✓ 4-frame TF buffer |
| Nav2 BT navigator (progress check) | 1–2 Hz | ✓ |
| CFPA2 frontier utility | 2 Hz tick | ✓ |
| stuck_watchdog (10 s window) | needs ≥ 10 samples in window | ✓ (100 samples) |
| Tight PID 100+ Hz | needs 100+ Hz pose | ✗ — but we don't run such loops |

**If real-robot ever shows pose jitter / "Control loop missed rate" spam**, two upgrade paths:

- **Option 1**: modify Fast-LIO source to publish odometry at IMU rate (~200 Hz between scans, using its internal IMU forward-propagation that already exists). One C++ change to `laserMapping.cpp`'s publish loop.
- **Option 2**: REP-105 two-layer split. Adapter publishes `map → odom` (10 Hz, SLAM correction). Add an IMU-only EKF for `odom → base_link` (200 Hz, smooth dead-reckoning). Bigger refactor, fully canonical.

Current 10 Hz hasn't shown problems in sim or simulated real-bot stress tests; we keep it simple.

### What this replaces / makes obsolete

- **slam_odom_relay** — superseded. Both relayed the topic, but only the adapter publishes TF. Could be deleted in a cleanup pass.
- **EKF chain** (`base_to_footprint_ekf` + `footprint_to_odom_ekf`) — no longer in the pose-publishing path. Still spawn (idle). Removing them requires modifying the shared `build_dual_robot_stack` helper which is used by 4 other launches; deferred.
- **`mujoco_odom_bridge.publish_tf: true`** — flipped to false. Bridge keeps publishing the topic for sim-only consumers; TF role moved to adapter.

### Real-robot path forward

With this fix shipped, the only remaining real-robot gap is **drift correction** (loop closure). That's what the `loop_closure:=true` toggle + vendored `src/vendor/sc_pgo/` are scaffolded for. Once SC-PGO ports to ROS 2 humble (see [`PORT_TO_ROS2.md`](../../src/vendor/sc_pgo/PORT_TO_ROS2.md)), the adapter automatically picks up `/<ns>/corrected_odom` when fresh, and the long-trajectory drift becomes bounded.

## End-to-end verification (sim)

Both robots on `nav2_mppi`:

```bash
ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  nav_backend_a:=nav2_mppi nav_backend_b:=nav2_mppi \
  debug:=true gui:=true rviz:=true
```

Expected:
- Both `lifecycle_manager_navigation` log "Managed nodes are active" within ~25 s
- Both robots get goals from CFPA2 within ~1 min, start moving
- RT factor 0.95–0.99 ×
- `dual_robot_collision_monitor` reports `c=0`, `stuck=N` for the bulk of a 10-min run
- Late game: ~0–2 contacts per run from leg-swing edge cases (B玄关-corner geometry); no wall-climbs, no tip-overs
- Coverage 70–80 % typical (hits CFPA2 frontier-extraction floor; demo3 is 384 m²; 10 min × 0.3 m/s × ~50 % effective = ~90 m driven)

Real-robot deployment: **TF / odom path is now sim-and-real symmetric** (Blockers A + B above are bypassed by the TF adapter). The only remaining gap is **drift correction**: Fast-LIO accumulates ~10 m / 7 min open-loop. For runs that stay self-consistent (single robot, return-to-start tasks), co-drift saves us. For absolute-coordinate tasks (multi-robot map fusion, GPS-tagged goals, return-after-restart), need SC-PGO ported and `loop_closure:=true` engaged.

## Files touched (this journey)

```
src/vendor/mujoco_ros2_control/mujoco_ros2_control/
  src/mujoco_system.cpp                                   ← 2-line brake bug fix
  include/mujoco_ros2_control/mujoco_system.hpp           ← NaN-init last_command
src/go2w/go2w_control/scripts/
  go2w_hybrid_cmd_router.py                                ← freewheel-in-legged
src/go2w/go2w_config/config/nav/
  nav2_go2w_full_stack.yaml                                ← polygon footprint + consider_footprint=true
  nav2_go2_full_stack.yaml                                 ← new (Go2 legged-only)
src/go2w/go2_gazebo_sim/launch/
  nav_test_mujoco_fastlio_mixed.launch.py                  ← yaml-by-has_wheels, skip router for legged,
                                                              add stuck_watchdog, loop_closure toggle
src/collaborative_exploration/cfpa2_collaborative_autonomy/
  config/cfpa2_coordinator.yaml                            ← 3 frontier-filter values loosened
  cfpa2_collaborative_autonomy/cfpa2_coordinator_node.py   ← (started: pivot-lock max-hold tracker)
src/vendor/sc_pgo/                                          ← new vendor (ROS 1 source, COLCON_IGNORE)
  PORT_TO_ROS2.md                                          ← porting checklist
scripts/runtime/
  stuck_watchdog.py                                        ← new outer-loop self-recovery node
  cfpa2_to_nav2_bridge.py                                  ← way_point → goal_pose
  path_relay.py                                            ← /plan → /planned_path
  fast_lio_tf_adapter.py                                   ← new TF + /odom/nav owner
                                                              (replaces slam_odom_relay,
                                                               disables mujoco_odom_bridge TF)
```

Commit `a88818f` covers (1)–(7); stuck_watchdog, fast_lio_tf_adapter, and this journey-doc
are post-commit.
