# Swarm-LIO2 + Dynamic-LIO + ERASOR Integration Plan

## Goal

Replace the current Fast-LIO / SC-PGO primary SLAM backend with a more advanced combined SLAM architecture while preserving the existing cross-robot loop closure safety layer.

Final target:

```text
Primary SLAM:
  Swarm-LIO2

Online dynamic filtering:
  Dynamic-LIO filtering / label-consistency logic integrated into the Swarm-LIO2 scan-map pipeline

Long-term static map cleanup:
  ERASOR asynchronous static map cleanup

Safety and cross-robot validation:
  existing ROS2 team_loop_closure + robust_loop_selector + map merge gate
```

Do **not** run Dynamic-LIO and Swarm-LIO2 as two competing primary odometry sources. Swarm-LIO2 is the primary SLAM candidate. Dynamic-LIO contributes dynamic filtering. ERASOR cleans accumulated maps. The existing `team_loop_closure` remains the independent verification layer.

---

## Current Validated System

Current validated backend:

```text
Fast-LIO / SC-PGO
+ team_loop_closure
+ robust_loop_selector
+ g2o_export_only team graph
+ safety-gated /merged_map
```

Already validated behavior:

```text
overlap aligns
no-overlap rejects
GT is not used at runtime
/merged_map opens only after robust alignment
team graph g2o export works
```

The migration must not break this.

---

## Reference Project Roles

### Swarm-LIO2

Repository:

```text
https://github.com/hku-mars/Swarm-LIO2
```

Role:

```text
Primary decentralized multi-robot LiDAR-inertial SLAM backend.
```

Use for:

```text
ego-state estimation
mutual state estimation
global extrinsic calibration
decentralized swarm communication
peer state exchange
multi-robot relative state estimation
```

Important compatibility note:

```text
Swarm-LIO2 is ROS Noetic / UAV / Livox oriented.
Current project is ROS2 Humble / quadruped / Nav2.
Therefore use adapter + shadow mode first.
```

### Dynamic-LIO

Repository:

```text
https://github.com/ZikangYuan/dynamic_lio
```

Role:

```text
Online dynamic-scene filtering and label-consistency logic.
```

Use for:

```text
dynamic/static point discrimination
moving-object filtering
preventing dynamic points from entering long-term map updates
```

Do not use Dynamic-LIO as a second primary odometry source.

### ERASOR

Repository:

```text
https://github.com/LimHyungTae/ERASOR
```

Role:

```text
Offline/asynchronous static 3D map cleanup backend.
```

Use for:

```text
removing dynamic-object traces from accumulated point cloud maps
generating cleaned static map
evaluating dynamic object removal quality
```

Do not use ERASOR as a real-time odometry frontend.

---

## Target Architecture

```text
Robot A                                                        Robot B
────────────────────────────────────────────────────────────────────────────

LiDAR + IMU                                                    LiDAR + IMU
   │                                                              │
   ▼                                                              ▼
Dynamic-LIO-style filtering                                 Dynamic-LIO-style filtering
   │                                                              │
   ├── static scan / cloud                                       ├── static scan / cloud
   └── dynamic scan / cloud                                      └── dynamic scan / cloud
   │                                                              │
   ▼                                                              ▼
Swarm-LIO2 primary decentralized LIO  <── peer/mutual state ──>  Swarm-LIO2 primary decentralized LIO
   │                                                              │
   ├── ego-state estimate                                        ├── ego-state estimate
   ├── mutual-state estimate                                     ├── mutual-state estimate
   ├── local/static map                                          ├── local/static map
   └── relative extrinsic estimate                               └── relative extrinsic estimate
   │                                                              │
   ▼                                                              ▼
ROS2 Swarm-LIO2 adapter                                    ROS2 Swarm-LIO2 adapter
   │                                                              │
   ├── /robot_a/Odometry                                         ├── /robot_b/Odometry
   ├── /robot_a/corrected_odom                                   ├── /robot_b/corrected_odom
   ├── /robot_a/cloud_static                                     ├── /robot_b/cloud_static
   └── /team_slam/swarm_lio2_relative_transform                  └── /team_slam/swarm_lio2_relative_transform
   │                                                              │
   └──────────────────────┬───────────────────────────────────────┘
                          ▼
                  Existing ROS2 team_loop_closure
                          │
                          ├── Scan Context from static cloud
                          ├── ICP verification
                          ├── robust_loop_selector
                          ├── team_pose_graph_node / GTSAM or g2o export
                          └── relative_transform_manager
                          │
                          ▼
              /merged_map gate + inter_robot_rendezvous gate
                          │
                          ▼
                         Nav2

Asynchronous map cleanup:

Swarm-LIO2 accumulated map + keyframe poses
          │
          ▼
ERASOR wrapper / map cleanup backend
          │
          ├── cleaned_static_map.pcd
          ├── removed_dynamic_points.pcd
          └── ERASOR metrics
          │
          ▼
ROS2 cleaned static map publisher
```

---

## Backend Modes

### 1. Baseline Mode

```yaml
slam_backend: fast_lio_scpgo
```

Behavior:

```text
Fast-LIO / SC-PGO remains primary.
Swarm-LIO2 may be disabled.
Dynamic-LIO filtering may be disabled or shadow-only.
ERASOR may be disabled.
Existing v2 cross-loop validation must still pass.
```

### 2. Swarm-LIO2 Shadow Mode

```yaml
slam_backend: swarm_lio2_shadow
```

Behavior:

```text
Fast-LIO / SC-PGO still drives Nav2 and the existing team_loop_closure.
Swarm-LIO2 runs in parallel.
Swarm-LIO2 outputs are published under /<ns>/swarm_lio2/*.
No downstream production module depends on Swarm-LIO2 yet.
```

Expected outputs:

```text
/<ns>/swarm_lio2/Odometry
/<ns>/swarm_lio2/cloud_static
/<ns>/swarm_lio2/cloud_map
/<ns>/swarm_lio2/mutual_state
/<ns>/swarm_lio2/relative_transform
/team_slam/swarm_lio2_metrics
```

Purpose:

```text
Compare Swarm-LIO2 against Fast-LIO without risking the already validated runtime.
```

### 3. Swarm-LIO2 Primary Mode

```yaml
slam_backend: swarm_lio2_primary
```

Behavior:

```text
Swarm-LIO2 becomes primary SLAM.
Fast-LIO / SC-PGO is disabled or fallback only.
Swarm-LIO2 adapter publishes the same topic contract previously provided by Fast-LIO.
Nav2 and team_loop_closure do not need to understand Swarm-LIO2 internals.
```

Adapter outputs:

```text
/<ns>/Odometry
/<ns>/corrected_odom
/<ns>/cloud_registered_body
/<ns>/cloud_static
/<ns>/cloud_dynamic
/team_slam/swarm_lio2_relative_transform
/tf
```

---

## Dynamic-LIO Filtering Integration

### Correct Integration Point

Dynamic-LIO should be integrated before Swarm-LIO2 map update:

```text
raw LiDAR scan
  -> Dynamic-LIO-style dynamic/static filtering
  -> static scan
  -> Swarm-LIO2 scan-to-map update
```

Alternative wrapper mode:

```text
raw LiDAR / IMU
  -> Dynamic-LIO wrapper
  -> /<ns>/dynamic_lio/cloud_static
  -> /<ns>/dynamic_lio/cloud_dynamic
```

Then:

```text
cloud_static -> Swarm-LIO2 input or map export
cloud_static -> team_loop_closure keyframes
cloud_dynamic -> Nav2 dynamic obstacle TTL layer
```

### Implementation Options

#### Option A — Port Dynamic-LIO Filtering Logic into Swarm-LIO2

Preferred final route if practical.

Actions:

```text
Identify Dynamic-LIO label consistency / dynamic point filtering logic.
Extract filtering module.
Add it before Swarm-LIO2 map update.
Expose static/dynamic point outputs.
```

Pros:

```text
best runtime performance
lowest bridge overhead
cleaner Jetson deployment
one primary SLAM process
```

#### Option B — Dynamic-LIO ROS1 Docker Wrapper

Fallback if source port is too risky.

Actions:

```text
Build Dynamic-LIO in ROS1 container.
Bridge input LiDAR/IMU to container.
Bridge cloud_static/cloud_dynamic back to ROS2.
Use static cloud in Swarm-LIO2 and team_loop_closure.
```

#### Option C — Existing Temporal Voxel Filter Fallback

Use only if Dynamic-LIO port/wrapper is blocked.

### Required Outputs

Expose:

```text
/<ns>/cloud_static
/<ns>/cloud_dynamic
/<ns>/dynamic_filter_metrics
```

Metrics:

```json
{
  "dynamic_filter_backend": "dynamic_lio_port|dynamic_lio_wrapper|temporal_voxel_fallback",
  "dynamic_points_filtered": 0,
  "static_points_kept": 0,
  "dynamic_filter_ratio": 0.0,
  "stale_obstacle_decay_time_sec": null,
  "fallback_used": false,
  "blocker": ""
}
```

---

## ERASOR Integration

### Role

ERASOR cleans accumulated maps after dynamic contamination has already happened.

It complements Dynamic-LIO:

```text
Dynamic-LIO:
  online prevention

ERASOR:
  offline / asynchronous cleanup
```

### Correct Integration Point

```text
Swarm-LIO2 keyframes + map + poses
    -> export ERASOR input format
    -> ERASOR cleanup
    -> cleaned static map
    -> ROS2 cleaned map publisher
```

ERASOR should not run inside the real-time odometry update loop.

### Required Data Export

Export from Swarm-LIO2 / adapter:

```text
pcds/
dense_global_map.pcd
poses_lidar2body.csv
initial_naive_map.pcd
```

ERASOR output:

```text
cleaned_static_map.pcd
removed_dynamic_points.pcd
erasor_metrics.json
```

ROS2 topics:

```text
/team_slam/cleaned_static_map
/team_slam/erasor_removed_dynamic_cloud
/team_slam/erasor_metrics
```

### Runtime Modes

```yaml
static_map_cleanup_backend:
  none
  erasor_wrapper
  temporal_voxel_fallback
```

Trigger modes:

```yaml
erasor_trigger_mode:
  manual
  periodic
  benchmark
```

Recommended:

```text
manual or benchmark first
periodic only after runtime cost is measured
```

### Validation

Pass criteria:

```text
naive accumulated map contains moving object trace
ERASOR removes trace from cleaned map
static walls are preserved
cleaned map can be published to ROS2
Nav2 is not blocked while ERASOR runs
```

---

## Integration With Existing Cross-Robot Loop Closure

This is mandatory. The existing `team_loop_closure` system should remain as the safety verification layer.

### Why Keep Existing Loop Closure

Swarm-LIO2 may provide mutual state estimation and global extrinsic calibration, but the current system already has:

```text
Scan Context retrieval
icp_2d verification
robust_loop_selector
false-positive rejection
/team_slam/relative_transform
/merged_map safety gate
team pose graph export
```

Role:

```text
Swarm-LIO2 estimates relative state.
team_loop_closure verifies relative state using place recognition and geometry.
Only then does map merge open.
```

### Input Contract to team_loop_closure

In `swarm_lio2_primary` mode, the adapter must provide the same inputs that Fast-LIO used to provide:

```text
/<ns>/Odometry
/<ns>/corrected_odom
/<ns>/cloud_registered_body
/<ns>/cloud_static
```

The keyframe exporter should prefer:

```text
/<ns>/cloud_static
```

and fallback to:

```text
/<ns>/cloud_registered_body
```

if static cloud is unavailable.

### Cross-Robot Loop Closure Flow

```text
Swarm-LIO2 primary odometry
  -> adapter publishes corrected_odom

Dynamic-LIO static cloud
  -> team_loop_closure keyframe exporter

keyframes
  -> Scan Context descriptor

peer descriptors / compact clouds
  -> cross_robot_loop_matcher

matches
  -> icp_2d verification

verified matches
  -> robust_loop_selector

robust inliers
  -> team_pose_graph_node / GTSAM or g2o export

relative_transform_manager
  -> alignment_status

alignment_status=aligned
  -> /merged_map enabled
  -> inter_robot_rendezvous enabled
```

### Agreement Gate Between Swarm-LIO2 and team_loop_closure

Add a consistency check:

```text
Swarm-LIO2 mutual transform: T_swarm_a_b
team_loop_closure robust transform: T_loop_a_b
```

Compute:

```text
translation_error = || translation(T_swarm_a_b^-1 * T_loop_a_b) ||
yaw_error = yaw(T_swarm_a_b^-1 * T_loop_a_b)
```

Acceptance thresholds:

```yaml
swarm_loop_agreement_max_translation: 0.5
swarm_loop_agreement_max_yaw_deg: 5.0
```

Map merge only opens if:

```text
robust_loop_selector accepted
AND team pose graph has accepted inter-robot factors
AND Swarm-LIO2 mutual transform agrees with loop closure transform
```

Default:

```yaml
require_swarm_loop_agreement: true
```

in `swarm_lio2_primary` mode.

### Safety Rules

Never allow:

```text
descriptor-only match -> map merge
single weak match -> map merge
Swarm-LIO2 mutual state alone -> map merge
ERASOR cleaned map alone -> map merge
GT -> runtime alignment
```

Require:

```text
robust inter-robot inliers
geometric verification
agreement with Swarm-LIO2 mutual estimate if primary mode
no-overlap rejection
```

---

## GTSAM Backend

If GTSAM is available, the pose graph should optimize accepted factors.

Modes:

```yaml
team_pose_graph_backend:
  auto
  gtsam_cpp
  gtsam_python
  g2o_export_only
```

Pass criteria:

```text
optimization_backend=gtsam_cpp or gtsam_python
optimization_success=true
pose_graph_error_after <= pose_graph_error_before
pose_graph_inter_robot_factors > 0
```

If GTSAM is unavailable:

```text
fallback to g2o_export_only
do not claim optimized PGO
record blocker
```

---

## Launch and Config Changes

Add or update launch args:

```yaml
slam_backend: fast_lio_scpgo|swarm_lio2_shadow|swarm_lio2_primary
dynamic_filter_backend: dynamic_lio_port|dynamic_lio_wrapper|temporal_voxel_fallback|none
static_map_cleanup_backend: erasor_wrapper|temporal_voxel_fallback|none
team_pose_graph_backend: auto|gtsam_cpp|gtsam_python|g2o_export_only
require_swarm_loop_agreement: true
swarm_loop_agreement_max_translation: 0.5
swarm_loop_agreement_max_yaw_deg: 5.0
```

Add launch files:

```text
launch/swarm_lio2_shadow.launch.py
launch/swarm_lio2_primary.launch.py
launch/swarm_lio2_ros2_adapter.launch.py
launch/dynamic_lio_filtering.launch.py
launch/erasor_map_cleanup.launch.py
```

Add configs:

```text
config/slam_backend/fast_lio_scpgo.yaml
config/slam_backend/swarm_lio2_shadow.yaml
config/slam_backend/swarm_lio2_primary.yaml
config/dynamic_filter/dynamic_lio.yaml
config/map_cleanup/erasor.yaml
```

---

## Validation Plan

### 1. Regression: Existing Fast-LIO Backend

```bash
START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh
```

Pass:

```text
overlap_pass=true
no_overlap_pass=true
gt_used_runtime=false
```

### 2. Swarm-LIO2 Shadow Validation

Run:

```text
slam_backend:=swarm_lio2_shadow
```

Pass:

```text
Swarm-LIO2 starts
Swarm-LIO2 publishes odometry
Swarm-LIO2 publishes mutual state or relative transform
Fast-LIO-driven baseline still runs
No downstream component depends on Swarm-LIO2 yet
metrics recorded
```

### 3. Swarm-LIO2 Primary Validation

Run:

```text
slam_backend:=swarm_lio2_primary
```

Pass:

```text
/<ns>/Odometry valid
/<ns>/corrected_odom valid
/<ns>/cloud_static or /<ns>/cloud_registered_body valid
Nav2 receives valid odometry
team_loop_closure receives keyframes
overlap aligns
no-overlap rejects
GT is not used
/merged_map opens only after robust alignment
```

### 4. Dynamic Object Validation

Pass:

```text
moving object does not remain as long-term obstacle
cloud_static excludes dynamic trace
cloud_dynamic includes moving object
Nav2 dynamic layer clears object after TTL
team_loop_closure uses static cloud
```

### 5. ERASOR Validation

Pass:

```text
naive map contains dynamic trace
ERASOR cleaned map removes dynamic trace
static walls preserved
cleaned map published to ROS2
ERASOR does not block control loop
```

### 6. Swarm-LIO2 + Loop Closure Agreement Validation

Pass:

```text
Swarm-LIO2 mutual transform exists
team_loop_closure robust transform exists
translation agreement < 0.5 m
yaw agreement < 5 deg
map merge opens only after agreement
```

---

## Required Logs

Generate:

```text
logs/slam_backend_comparison.json
logs/slam_backend_comparison.md
logs/swarm_lio2_shadow_validation.json
logs/swarm_lio2_primary_validation.json
logs/dynamic_lio_filter_integration.json
logs/erasor_map_cleanup_validation.json
logs/cross_loop_closure_final_eval.json
logs/cross_loop_closure_final_eval.md
logs/team_pose_graph_metrics.json
```

---

## Required Docs

Add:

```text
docs/slam_backend_migration_to_swarm_lio2.md
docs/dynamic_lio_swarm_lio2_integration.md
docs/erasor_static_map_cleanup.md
docs/swarm_lio2_loop_closure_agreement_gate.md
```

---

## Final Status Labels

### Status A — Full Replacement Passed

```text
Swarm-LIO2 primary backend passed.
Dynamic-LIO filtering integrated or wrapper active.
ERASOR static map cleanup passed.
Existing loop closure safety gate passed.
GTSAM optimized backend passed if available.
```

Valid claim:

```text
Swarm-LIO2-based primary decentralized SLAM with Dynamic-LIO dynamic filtering, ERASOR static map cleanup, and independent robust cross-robot loop closure safety verification.
```

### Status B — Shadow Passed, Primary Blocked

```text
Swarm-LIO2 shadow mode passed.
Swarm-LIO2 primary mode blocked.
Fast-LIO remains production backend.
Dynamic-LIO / ERASOR integration may still be active.
Exact blocker recorded.
```

### Status C — External Blocker

Examples:

```text
Swarm-LIO2 build failure
ROS Noetic dependency conflict
Livox driver unavailable
Jetson resource limit
Dynamic-LIO filtering integration failure
ERASOR wrapper failure
GTSAM unavailable
```

Action:

```text
Do not fake replacement.
Keep validated Fast-LIO backend.
Record exact blocker.
```

---

# Prompt to Coding Agent

```text
We are changing the SLAM migration strategy.

Target:
Replace Fast-LIO / SC-PGO with a combined advanced SLAM backend:
- Swarm-LIO2 as primary decentralized multi-robot LIO.
- Dynamic-LIO filtering ported into the Swarm-LIO2 scan/map pipeline.
- ERASOR as asynchronous static map cleanup backend.
- Existing ROS2 team_loop_closure remains as independent cross-robot loop closure verification and safety-gating layer.

Do not run Dynamic-LIO and Swarm-LIO2 as two competing primary odometry sources.
Swarm-LIO2 is the primary SLAM candidate.
Dynamic-LIO provides dynamic/static filtering.
ERASOR provides asynchronous static map cleanup.

Implement backend modes:
slam_backend:
  fast_lio_scpgo
  swarm_lio2_shadow
  swarm_lio2_primary

Default must remain:
  fast_lio_scpgo

Implement:
1. Swarm-LIO2 shadow mode.
2. Swarm-LIO2 primary mode.
3. ROS2 adapter preserving existing topic contract:
   /<ns>/Odometry
   /<ns>/corrected_odom
   /<ns>/cloud_registered_body
   /<ns>/cloud_static
   /team_slam/swarm_lio2_relative_transform
   /tf
4. Dynamic-LIO filtering integration:
   preferred: port filtering into Swarm-LIO2 preprocessing or map update
   fallback: Dynamic-LIO wrapper outputs cloud_static/cloud_dynamic
   final fallback: existing temporal voxel filter
5. ERASOR wrapper:
   export Swarm-LIO2 map/keyframes/poses to ERASOR format
   run ERASOR asynchronously
   import cleaned_static_map and removed_dynamic_points to ROS2
6. Loop closure integration:
   team_loop_closure must consume Swarm-LIO2 adapter outputs
   keyframe exporter should prefer cloud_static
   robust_loop_selector remains required
   no-overlap rejection remains required
7. Agreement gate:
   compare Swarm-LIO2 mutual transform and team_loop_closure robust transform
   require translation error < 0.5 m
   require yaw error < 5 deg
   /merged_map opens only after robust loop closure and agreement gate pass
8. GTSAM backend:
   use optimized backend if available
   fallback to g2o_export_only if blocked

Validation:
- fast_lio_scpgo regression
- swarm_lio2_shadow
- swarm_lio2_primary
- dynamic moving object scene
- ERASOR map cleanup
- no-overlap rejection
- Swarm-LIO2 / loop closure agreement
- no GT runtime

Do not fake replacement.
Do not claim Fast-LIO replaced until swarm_lio2_primary passes overlap, no-overlap, dynamic-object, ERASOR cleanup, Nav2 runtime, and loop-closure agreement validation.
```
