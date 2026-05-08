# SLAM Backend Migration to Swarm-LIO2

## Implemented Scaffolding

`slam_backend` supports `fast_lio_scpgo`, `swarm_lio2_shadow`, and `swarm_lio2_primary`. `fast_lio_scpgo` remains the default production backend.

`deployment_mode` supports `sim_ros2`, `sim_hybrid_ros1_slam_ros2_nav`, `real_hybrid_ros1_slam_ros2_nav`, and `real_ros1_only_experimental`. `sim_ros2` remains the default deployment path.

The ROS2 adapter, launch args, configs, Docker/catkin bridge scaffolding, backend availability scripts, and validation summaries are implemented. Swarm-LIO2 primary is not a replacement claim until all required runtime validations pass.

## Topic Contract

Shadow mode publishes only isolated Swarm-LIO2 outputs:

- `/<ns>/swarm_lio2/Odometry`
- `/<ns>/swarm_lio2/cloud_static`
- `/<ns>/swarm_lio2/cloud_map`
- `/<ns>/swarm_lio2/mutual_state`
- `/<ns>/swarm_lio2/relative_transform`
- `/team_slam/swarm_lio2_metrics`

Primary candidate mode maps Swarm-LIO2 outputs into the existing ROS2 contract:

- `/<ns>/Odometry`
- `/<ns>/corrected_odom`
- `/<ns>/odom/nav`
- `/<ns>/cloud_registered_body`
- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/team_slam/swarm_lio2_relative_transform`
- `/tf`

## Mock / Synthetic Validation

Synthetic tests validate the ROS2-side adapter contract, Dynamic-LIO cloud forwarding contract, ERASOR cleanup topic/metrics contract, and Swarm-loop agreement gate math. These tests do not prove ROS1 backend runtime or real robot readiness.

## Docker Runtime Validation

Current Docker/catkin recovery state:

- `ros1_hybrid_slam` Docker image builds with the required ROS1 dependencies.
- ROS1 catkin workspace builds from writable staged backend copies.
- Livox-SDK is built/installed into the container from `external/Livox-SDK` when needed.
- Swarm-LIO2 ROS1 wrapper launch starts and stays alive for the 45 s smoke window.
- The wrapper launch relays `robot_a` / `robot_b` ROS2-sim-compatible LiDAR/IMU topics into the Swarm-LIO2 `quad1` / `quad2` input contract and relays raw Swarm-LIO2 odom/cloud topics back under `/<ns>/swarm_lio2_raw/*`.
- The ROS1 bridge entrypoint now tolerates ROS setup files under `set -u`, and the hybrid launch exports the local FastDDS no-SHM profile when present.
- Dynamic-LIO and ERASOR source targets build in the ROS1 catkin workspace.
- Fresh Fast-LIO / SC-PGO baseline regression still passes with `gtsam_cpp`, overlap accepted, no-overlap rejected, and no runtime GT.

Manual rerun command:

```bash
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

Primary replacement still requires overlap, no-overlap, dynamic-object, ERASOR cleanup, Nav2 runtime, and loop-closure agreement validation.

## Real Robot Validation

Run on the robot/Jetson or validated field computer:

```bash
bash scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh --host
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Backend Docker/catkin recovery is complete, but full hybrid runtime validation is not complete.
- Swarm-LIO2 shadow mode has not passed ROS2 odometry reception: ROS2 shadow topics are visible, but message rates remain below `0.1 Hz`, odometry frames are empty, and ROS1 `/<ns>/swarm_lio2_raw/*` relay output topics were missing in the bounded runtime check.
- Swarm-LIO2 primary mode has not passed ROS2 odometry/corrected odom/cloud/Nav2/tf/keyframe runtime validation.
- Dynamic-LIO and ERASOR are buildable wrappers, but their runtime outputs are not validated.
- Real robot LiDAR/IMU/Unitree topics and peer network are unavailable here.
- Fast-LIO remains production backend.

Status labels:

- Status A: sim and real hybrid passed.
- Status B: sim hybrid passed, real hybrid blocked.
- Status C: Swarm-LIO2 shadow passed, primary blocked.
- Status D: external/runtime blocker.
