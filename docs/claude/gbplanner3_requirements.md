# GBPlanner3 + Point-LIO onboard — open requirements (2026-05-13)

Companion to [`gbplanner3_noetic_onboard.md`](gbplanner3_noetic_onboard.md). This file is **requirements only — no proposed solutions, no recommendations**. Each item below states what the current deployment cannot do that we need it to do, with enough specificity to verify when it's done.

## R1. GBPlanner3 must produce a meaningful path

**Currently**: `gbplanner_node` is alive, voxblox integrates Mid-360 scans (verified `/gbplanner_node/surface_pointcloud` @ 2.94 Hz), the `/planner_control_interface/std_srvs/automatic_planning` Trigger service returns `success: True`, but every plan attempt logs `[GlobalGraph] Graph is empty, nothing to search` → `Planner returned an empty path`. No samples land on `/gbplanner_path` or `/pci_command_path`.

**Required**:

- `/pci_command_path` MUST publish a non-empty `trajectory_msgs/MultiDOFJointTrajectory` within 5 s of `automatic_planning` being triggered.
- Path waypoints MUST be inside the voxblox-known free space (no collisions against the latest `/gbplanner_node/tsdf_map_out`).
- Path MUST end at a frontier or specified goal point — i.e. produces forward progress, not zero-length loop at the robot position.
- Path waypoints MUST be in the `map` (or `camera_init`) world frame, NOT in robot-local frame.
- Path MUST refresh at ≥ 0.5 Hz during continuous exploration.

**Currently-known blockers** (state only, no fix):

- `kGroundRobot` mode requires an `/elevation_map` layer which we are not publishing.
- `kAerialRobot` mode (the hack we temporarily used) bypasses elevation but `BoundedSpaceParams` / `RandomSamplerParams` defaults were copied from the gzc/urban_exploration template — never tuned for an indoor Mid-360 scene at robot z ≈ 0.3 m.

## R2. Live remote visualization over the dongle Ethernet

**Currently**: X11-forwarded RViz over `ssh -X` is broken (verified: Mesa indirect GLX disabled on Jammy, swrast renders empty viewport, Grid display crashes Ogre). Foxglove-bridge installed but WebSocket connection from laptop Foxglove Studio was not validated end-to-end. The non-X path that works (`stream_cloud_live.sh` Open3D over ssh-stdin) only streams one PointCloud2 topic and no other map state.

**Required**:

The laptop side MUST display, simultaneously and live (≥ 5 Hz refresh on each), at least these layers, sourced from the Jetson running Point-LIO + gbplanner3:

| Layer | Topic | Type | Rate | Why |
|---|---|---|---|---|
| Live LiDAR cloud (sensor frame) | `/robot/cloud_registered_body` | sensor_msgs/PointCloud2 | 10 Hz | Verify Point-LIO is feeding gbplanner |
| Accumulated map cloud (world frame) | `/robot/cloud_registered` | sensor_msgs/PointCloud2 | 10 Hz | See map growth as dog walks |
| Voxblox 3D occupancy voxels | `/gbplanner_node/occupied_nodes` | visualization_msgs/MarkerArray | low-rate | See what gbplanner thinks is occupied |
| Voxblox surface points | `/gbplanner_node/surface_pointcloud` | sensor_msgs/PointCloud2 | ~3 Hz | TSDF surface inspection |
| Robot pose / odom trail | `/robot/Odometry` | nav_msgs/Odometry | 10 Hz | Drift / coverage diagnostic |
| Planner output path | `/pci_command_path` | trajectory_msgs/MultiDOFJointTrajectory | event-driven | The thing we're debugging |
| TF tree | `/tf`, `/tf_static` | tf2_msgs/TFMessage | 10 Hz | Frame consistency |

Additional requirements:

- The viewer MUST tolerate the USB-C dongle's intermittent NO_CARRIER events — reconnect cleanly without manual restart.
- The viewer MUST NOT require apt-installing ROS 1 on the laptop (laptop is Jammy + Humble; Noetic doesn't install cleanly here).
- The viewer MUST NOT require X11 forwarding of GL-heavy GUIs from Jetson.
- The viewer MUST allow toggling individual layers on/off at runtime (so an operator can drop the heavy point cloud when only voxels matter).
- The viewer SHOULD let an operator click a point and read its (x, y, z, frame) for debugging (sensor offsets, frame mismatches).
- The end-to-end bandwidth at full layer set MUST fit the USB-C Ethernet's sustained ≤ ~50 Mbit/s.
- Frame ID interpretation MUST correctly handle our TF chain `map → camera_init → body → base_link`; a fixed frame of `camera_init` should align all per-frame visualisations.

## R3. Diagnostic loop — what "meaningful path" means

We need a deterministic answer to "is gbplanner doing something useful right now?" without leaving the laptop terminal. Specifically:

- Reading the path output through the visualizer of R2 MUST be sufficient to tell, in one glance, whether:
  1. The planner is just idle (no Trigger called yet).
  2. The planner is running but graph is empty (no free space found).
  3. The planner returned a path but the path is degenerate (zero length, loops, jumps).
  4. The planner returned a healthy path.
- The visualizer MUST show the path overlaid on the voxblox voxel grid so collisions / pass-throughs are visible.

## R4. Path → cmd_vel translation is out of scope here

Path output (R1) and visualization (R2) are the immediate scope. Path-following — converting `/pci_command_path` into `/cmd_vel` for the Go2 — is **deferred** until R1 produces real paths. No path follower will be designed, written, or installed until R1 is verified.

## R5. Operational guarantees

When R1 + R2 are met:

- The full stack (Point-LIO + gbplanner3 + visualizer) MUST survive a 5-minute walk without manual intervention.
- The Jetson MUST recover automatically after the dongle's NO_CARRIER bounces (network drop during a walk MUST NOT kill any onboard process).
- A clean `Ctrl+C` from the laptop MUST stop the entire Jetson stack within 5 s (laptop quickstart trap behaviour, already implemented for Point-LIO must extend to gbplanner3 + visualizer).
