# gbplanner3 / OmniPlanner integration — Collab_QRC ↔ NTNU UAS

Status as of **2026-05-13**. Infra fixes shipped, dog still doesn't move; root cause localized to voxblox cloud-integration layer but not yet patched.

---

## 1. What we're trying to do

Plug the **NTNU gbplanner3** exploration planner (a.k.a. "OmniPlanner", [arxiv 2603.04284](https://arxiv.org/abs/2603.04284)) into the Collab_QRC stack so the Go2 quadruped explores indoor environments under it. Two deployment targets:

| Target | OS | Compute | Why we need this |
|---|---|---|---|
| **Sim** (laptop) | Humble + Noetic-in-Docker | Bridge via `ros1_bridge` | Iterate on configs, benchmark, demo |
| **Real** (Go2 / Mid-360) | ROS 1 Noetic native | Jetson Orin Nano | Field deployment, no bridge |

Architecture sketch (see [`scripts/sim/gbplanner3_mujoco/README.md`](../../scripts/sim/gbplanner3_mujoco/README.md) for the full version):

```
┌─────────────────── SIM ───────────────────┐    ┌─────── REAL (Jetson Orin Nano) ───────┐
│                                            │    │                                       │
│  Humble (laptop host)                      │    │  Noetic (Jetson native)               │
│  ┌──────────┐    /cloud   ┌─────────────┐  │    │  ┌──────────────┐                     │
│  │ MuJoCo   │   /odom →   │ ros1_bridge │  │    │  │ Mid-360 +    │                     │
│  │ Fast-LIO │   /tf       │     ↓       │  │    │  │ livox_driver │                     │
│  │ CHAMP    │  ←────────  │  gbplanner3 │  │    │  │     ↓        │                     │
│  │ Nav2     │  /cmd_traj  │   voxblox   │  │    │  │ Fast-LIO2    │                     │
│  └──────────┘             │   elev_map  │  │    │  │     ↓        │                     │
│        ↑                  └─────────────┘  │    │  │ gbplanner3   │                     │
│        ↑   adapter (Humble): /command/      │    │  │  voxblox     │                     │
│        └── trajectory → /goal_pose → Nav2  │    │  │  elev_map    │                     │
│                                            │    │  └──────┬───────┘                     │
│                                            │    │         ↓ /cmd_traj                   │
│                                            │    │  adapter (Noetic) → Unitree SDK       │
└────────────────────────────────────────────┘    └───────────────────────────────────────┘
```

**Crucial point**: the bridge only exists in sim. On Jetson everything is native ROS 1.

---

## 2. Final pipeline state (sim)

Goal: trigger `automatic_planning` service → gbplanner emits `/command/trajectory` → adapter converts to `/robot/way_point_coord` → existing `cfpa2_to_nav2_bridge` converts to `/robot/goal_pose` → Nav2 plans → CHAMP walks.

Actual state at end of 2026-05-13 session:

| Stage | Status |
|---|---|
| MuJoCo + Fast-LIO + CHAMP on Humble | ✅ Healthy |
| `ros1_bridge dynamic_bridge` forwarding /tf, /Odometry, /cloud_registered_body | ✅ Healthy, ~10 Hz cloud, ~100 Hz /tf |
| gbplanner3 container starts, gbplanner_node registers automatic_planning service | ✅ |
| Cloud reaches voxblox subscriber inside gbplanner_node (4976 pts × 8.9 Hz) | ✅ Verified |
| TF chain world→body resolves inside container | ✅ After our fix |
| elevation_mapping_lidar logs "First corresponding point cloud and pose found, elevation mapping started" | ✅ |
| Voxblox `/gbplanner_node/tsdf_pointcloud` width | ❌ 24 (essentially empty) |
| Voxblox `/gbplanner_node/surface_pointcloud` width | ❌ 0 |
| gbplanner produces `/command/trajectory` | ❌ Never |
| Robot moves | ❌ Stationary at spawn |

---

## 3. Fixes that landed (worth committing)

### 3.1 Disk-fill protection (sim + real both benefit)

Without this, when gbplanner can't find the elevation layer it logs at ~90 kHz. Docker's json.log fills `/home/docker` partition in minutes; on Jetson it would similarly OOM the eMMC.

- **`compose/docker-compose.collab_qrc.yml`**: per-container `logging: { driver: json-file, options: { max-size: 20m, max-file: 3 } }`. Caps each container's json log at 60 MB.
- **`scripts/launch/nav_test_gbplanner_demo3.sh`**: pipes the `make launch` stdout through `grep --line-buffered -v "No 'elevation' layer in map"` before writing to `/tmp/gbplanner3_demo3_uas.log`. The container json.log is what `docker logs` sees, but the host-side make pipe was a parallel sink that was also filling disk.

### 3.2 ros1_bridge does not preserve latched /tf_static

**The non-obvious infra bug we hit**: `dynamic_bridge --bridge-all-topics` drops the `TRANSIENT_LOCAL` (latched) durability of Humble's `/robot/tf_static`. By the time the Noetic-side relay or any TF subscriber connects, the latched message has already been delivered to the bridge once and is gone. Result: Noetic-side `/tf_static` shows "Publishers: None" even though Humble is faithfully publishing it.

Symptom inside gbplanner: every cloud lookup fails with `"Could not find a connection between 'world' and 'body' because they are not part of the same tree. Tf has two or more unconnected trees."` at multi-kHz.

We tried Noetic-side `rosrun tf static_transform_publisher` and `rosrun tf2_ros static_transform_publisher` (both flavors); both publish to `/tf` periodically at ~9 Hz, not to a truly latched `/tf_static`. That gave enough samples for ad-hoc `tf_echo` to work but voxblox's per-cloud lookup at specific timestamps still failed.

**Fix that worked**: hand-rolled rospy node that publishes a single latched `TFMessage` on `/tf_static` and `rospy.spin()`-s to stay the connected publisher. See [`scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/publish_static_tfs.py`](../../scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/publish_static_tfs.py). Bundled into compose via volume mount.

After this fix, `tf_echo world body` resolves cleanly and "two unconnected trees" errors stop.

**Real-robot equivalent**: not needed. On Jetson, Fast-LIO + `robot_state_publisher` are native ROS 1 — `/tf_static` is properly latched in-process.

### 3.3 Voxblox config aligned to NTNU UGV reference

Diffed our [`config/collab_qrc_go2/voxblox_config.yaml`](../../scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/voxblox_config.yaml) against the canonical reference at `~/Research/uas_deploy/unified_autonomy_stack/workspaces/robot_bringup/config/ros1/gbplanner/ugv_sim/voxblox_config.yaml`:

| Param | Was | Now | Why it mattered |
|---|---|---|---|
| `max_ray_length_m` | 8.0 | 20 | gbplanner's Local bounded space is 40 m — at 8 m most samples landed in unknown voxels and got rejected |
| `tsdf_voxel_size` | 0.15 | 0.20 | Mid-360 noise is bigger than 15 cm of body-frame jitter |
| `truncation_distance` | 0.4 | 0.6 | Below 3× voxel size is too thin |
| `min_ray_length_m` | 0.5 | 0.1 | Fast-LIO's `body` frame origin sits ≈ 20 cm BEHIND the Mid-360 sensor; 0.5 m clipped most close-range floor returns |
| `use_freespace_pointcloud` | True | False | Nobody publishes `/freespace_pointcloud` on Collab_QRC — keeping it True meant voxblox waited on a pipeline that never arrived |
| `allow_clear`, `max_weight`, `clearing_ray_weight_factor`, ESDF block | (missing) | (added per NTNU ref) | Various integration knobs from the production UGV config |

### 3.4 elevation_mapping config

Same diff against NTNU UGV ref:

| Param | Was | Now |
|---|---|---|
| `sensor_processor.type` | `perfect` → `laser` (during diag) → `perfect` (reverted) | `perfect` |
| `min_variance` | 0.0001 | 1e-5 |
| `mahalanobis_distance_threshold` | 2.5 | 5.0 |

The laser detour assumed PerfectSensorProcessor's near-zero variance was causing degenerate Kalman updates. Empirically: switching to laser changed nothing. Reverted to match upstream.

### 3.5 Cloud timestamp rewriting (sim-only experiment)

Reading voxblox source (`voxblox_ros/src/transformer.cc`), confirmed voxblox calls `tf_listener_.canTransform(to_frame, from_frame, cloud_stamp)` per cloud and **silently drops the cloud if false**. Bridge jitter means cloud_stamp regularly lands in a window where the Noetic-side TF buffer hasn't caught up.

[`scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/cloud_stamp_rewriter.py`](../../scripts/sim/gbplanner3_mujoco/config/collab_qrc_go2/cloud_stamp_rewriter.py) is a 30-line ROS 1 node that re-stamps `/robot/cloud_registered_body` with `rospy.Time.now()` and republishes to `/rmf/lidar/points_downsampled`. Replaces the previous `topic_tools relay` in compose.

**Effect**: the canonical voxblox failure log `"Input pointcloud queue getting too long! Dropping some pointclouds. Either unable to look up transform timestamps or the processing is taking too long."` stops firing — voxblox is no longer rejecting clouds for TF reasons.

**Side effect**: TSDF / surface widths **still don't grow**. So TF stamping wasn't the only blocker.

**Real-robot equivalent**: not needed. Cloud and TF originate from the same Fast-LIO process, no bridge jitter.

### 3.6 Launch script + adapter wiring

- Stop subcommand had a relative-path bug that broke once the script `cd`-ed to UAS_REPO_ROOT — fixed with absolute `SCRIPT_DIR` derivation.
- Adapter `gbplanner_to_waypoint_adapter.py` was never wired into any launch. Now spawned as `nohup python3 ... &` from `nav_test_gbplanner_demo3.sh` between the static-TF setup and `exec`-ing `nav_test_fastlio.sh`, with `publish_goal_pose:=false` (the existing cfpa2_to_nav2_bridge handles that conversion).

---

## 4. The wall we hit: voxblox doesn't populate even after all fixes

Despite **every** infra-layer fix above:

- Cloud arrives at voxblox subscriber: **verified, 8.9 Hz × 4976 pts**
- TF chain resolves: **verified, `tf_echo world body` succeeds**
- "queue getting too long" voxblox warning: **stops firing after stamp rewrite**
- TSDF `/gbplanner_node/tsdf_pointcloud` width: **stays at 24**
- surface `/gbplanner_node/surface_pointcloud` width: **stays at 0**
- gbplanner emits `/command/trajectory`: **never**

So something inside voxblox's `processPointCloudMessageAndInsert` is silently rejecting points. We could not verify with `rosservice call .../publish_pointclouds` because the gbplanner_node's single-thread spinner is saturated by the ~90 kHz "No elevation" warning logging — service calls hang.

**Why we can't introspect further today**: voxblox's `verbose: True` param doesn't propagate to per-cloud integration logging (gbplanner3 wraps it via MapManager, which has its own log control). Would need source patches to add prints.

---

## 5. Mental model of what's gating the dog

Two independent map sources, each with its own gate:

```
                Mid-360 cloud (10 Hz)
                       │
       ┌───────────────┼───────────────┐
       ↓                               ↓
   voxblox                       elevation_mapping_lidar
   (TSDF/ESDF, 3D)               (GridMap "elevation" layer, 2.5D)
       │                               │
       ↓                               ↓
   Used by:                        Used by:
   - RRG edge feasibility          - kGroundRobot traversability gate
     ("can I walk from A to B        ("is this sample point on
      without hitting an              walkable ground?")
      obstacle?")
   - Volumetric gain
     ("how many unknown voxels     Only relevant when
      can I see from here?")        RobotParams.type = kGroundRobot
       │
       └──────────┐
                  ↓
            gbplanner RRG
                  ↓
          /command/trajectory
                  ↓
          adapter (Humble)
                  ↓
              /goal_pose
                  ↓
                Nav2
                  ↓
              cmd_vel_legged
                  ↓
                CHAMP
                  ↓
              MuJoCo (sim) / Unitree SDK (real)
```

Symptoms in our session:

- spam **`No 'elevation' layer in map`** at ~90 kHz → elevation_mapping not producing useful data (or `elevation` layer is all NaN at sample positions).
- **RRG "1 vertex 0 edges, 2000 loops"** → voxblox doesn't have enough free-space voxels for edge connection.

Even bypassing one (we tried `kAerialRobot`, which skips the elevation gate) the other still fails — the planner can't connect ANY edge because voxblox has effectively nothing.

**So 3D planning (voxblox-driven) is broken, and we can't see why.**

---

## 6. Recommended next paths

### Plan A — kAerialRobot + small Local bound + clear_sphere (~30 min, low confidence)

Force voxblox to seed a free sphere around the robot at startup so RRG has somewhere to expand from.

```yaml
# voxblox_config.yaml
clear_sphere_for_planning: True   # was False
clear_sphere_radius: 2.0          # 2 m sphere around init pose

# gbplanner_config.yaml
RobotParams:
  type: kAerialRobot              # skip elevation gate

PlanningParams:
  Local:
    min_val: [-3, -3, -1]
    max_val: [3, 3, 1]            # was [-20, -20, -2] to [20, 20, 2]
```

Risk: voxblox might still not populate beyond the seeded sphere if integration is genuinely broken.

### Plan B — Fake the elevation_map from Humble (~30-60 min, moderate confidence)

Bypass elevation_mapping_lidar entirely. Publish a flat `grid_map_msgs/GridMap` (Z=0 everywhere) from Humble side directly to `/elevation_mapping_lidar/elevation_map_raw` (bridge forwards it). Stops the spam, unblocks `rosservice` calls, lets us probe voxblox internals.

**Does NOT lose 3D planning** — voxblox handles 3D, elevation_map is only a 2.5D ground-walkability lookup. For an indoor flat scene the assumption "ground is at Z=0" is trivially true.

But: if voxblox is also broken, B alone doesn't make the dog move. B+probe gives us info to decide next step.

### Plan C — Patch voxblox with per-cloud diagnostic prints, rebuild container (~2-4 hr)

Add `ROS_INFO` lines to `voxblox_ros/src/tsdf_server.cc:processPointCloudMessageAndInsert` and `voxblox_ros/src/transformer.cc:lookupTransformTf` to log exactly which check rejects clouds. Rebuild the `unified_autonomy:ros1_gbplanner` image with this patch.

Definitive but expensive. Saves itself if we end up needing it on Jetson too.

### Plan D — Skip sim, deploy to Jetson real robot (~1-2 days)

The bridge complexity is sim-only. On Jetson Fast-LIO + gbplanner are both ROS 1 native — no `dynamic_bridge`, no `/tf_static` latching gap, no clock-source mismatch. If gbplanner is otherwise sound, it should "just work" on the real robot. Validates the real path while sidestepping bridge issues. Requires:

- Build `unified_autonomy:ros1_gbplanner` for ARM64 (or run native compilation on Jetson with `~/Research/uas_deploy/unified_autonomy_stack/workspaces/ws_gbplanner/`)
- Reuse [`scripts/real/onboard_fastlio_noetic.sh`](../../scripts/real/onboard_fastlio_noetic.sh) for Fast-LIO bringup, add a parallel `gbplanner_real.launch` (template `gbplanner_sim.launch`, remove `use_sim_time`, rewire topics to Fast-LIO's native names)
- Write a ROS 1 `gbplanner_to_unitree_adapter.py` that converts `/command/trajectory` → Unitree SportClient `Move(vx, vy, vyaw)` calls.

The yaml configs (`voxblox_config.yaml`, `elevation_mapping_config.yaml`, `gbplanner_config.yaml`) **transfer verbatim** between sim and real.

---

## 7. What sim-only vs. real-portable matters here

| Artifact | Sim-only | Portable to Jetson |
|---|---|---|
| `voxblox_config.yaml` | | ✅ all values |
| `elevation_mapping_config.yaml` | | ✅ all values |
| `gbplanner_config.yaml` (RobotParams, SensorParams, BoundedSpace, Planning) | | ✅ all values |
| `publish_static_tfs.py` (latched /tf_static aliases) | ✅ (bridge artifact) | ❌ |
| `cloud_stamp_rewriter.py` | ✅ (bridge jitter mitigation) | ❌ |
| `docker-compose.collab_qrc.yml` (logging caps, relays) | ✅ | ❌ (Jetson runs native) |
| `nav_test_gbplanner_demo3.sh` (stop fix, adapter spawn, spam filter) | ✅ | ❌ |
| `gbplanner_to_waypoint_adapter.py` | partial — ROS 2 version for Nav2 | Need ROS 1 sibling that talks Unitree SDK |

---

## 8. References

- Paper: [arxiv 2603.04284 — OmniPlanner](https://arxiv.org/abs/2603.04284)
- Upstream: <https://github.com/ntnu-arl/gbplanner_ros> branch `gbplanner3`
- Wiki: <https://github.com/ntnu-arl/gbplanner3_wiki/wiki>
- Local NTNU UGV reference configs: `~/Research/uas_deploy/unified_autonomy_stack/workspaces/robot_bringup/config/ros1/gbplanner/ugv_sim/`
- Voxblox source: `~/Research/uas_deploy/unified_autonomy_stack/workspaces/ws_gbplanner/src/mapping/voxblox/voxblox_ros/`
- Memory notes (cross-session): [reference_gbplanner3_upstream.md](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/reference_gbplanner3_upstream.md), [feedback_ros1_bridge_latching.md](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/feedback_ros1_bridge_latching.md), [project_gbplanner3_integration.md](../../../.claude/projects/-home-hanszhu-Research-Collab-QRC/memory/project_gbplanner3_integration.md)
