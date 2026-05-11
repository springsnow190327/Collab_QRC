# Sim Hybrid ROS1 SLAM / ROS2 Nav

`sim_hybrid_ros1_slam_ros2_nav` keeps MuJoCo, Nav2, exploration, and `team_loop_closure` in ROS2/Humble while Swarm-LIO2, Dynamic-LIO, and ERASOR run in a Dockerized ROS1/Noetic catkin side.

## Implemented Scaffolding

- Default remains `deployment_mode:=sim_ros2` and `slam_backend:=fast_lio_scpgo`.
- ROS1 hybrid Docker files, bridge config, launch config, backend check scripts, and ROS2 adapter nodes are present.
- Swarm-LIO2 shadow mode remains isolated under `/<ns>/swarm_lio2/*`.
- Swarm-LIO2 primary mode is only a configured candidate topic contract, not a replacement claim.
- The ROS1 hybrid wrapper launch now stages Swarm-LIO2 in a writable catkin workspace and adds `topic_tools relay` entries for ROS2-sim-compatible LiDAR/IMU inputs plus Swarm-LIO2 raw odom/cloud outputs.
- The ROS1 bridge entrypoint is guarded against `set -u` failures while sourcing ROS1/ROS2 setup files, and the hybrid launch exports `config/fastdds_no_shm.xml` when present so host ROS2 CLI/topic checks can see container topics on this machine.

## Mock / Synthetic Validation

- ROS2-side adapter tests cover synthetic Swarm-LIO2 odometry forwarding into `/<ns>/Odometry`, `/<ns>/corrected_odom`, and `/<ns>/odom/nav`.
- Dynamic-LIO wrapper tests cover static/dynamic cloud forwarding and metrics schema.
- ERASOR adapter tests cover synthetic PCD export, metrics JSON, and ROS2 cleanup topics.
- Swarm-loop agreement tests cover accept/reject threshold behavior and prevent Swarm mutual state alone from opening the merge gate.

## Docker Runtime Validation

The latest recovery pass on this branch produced this result:

- Swarm-LIO2 Docker image build passed.
- ROS1 catkin workspace build passed, including Swarm-LIO2, `udp_bridge`, Livox driver, Dynamic-LIO `sr_lio`, and ERASOR targets.
- Swarm-LIO2 ROS1 launch smoke passed: the wrapper launch stayed alive for the 45 s smoke window and included `topic_tools relay` nodes for `robot_a` / `robot_b` raw input and output topics.
- Dynamic-LIO and ERASOR Docker/catkin wrapper builds passed.
- Fresh Fast-LIO / SC-PGO baseline regression passed with `START_BRIDGE=true`, `gtsam_cpp`, overlap accepted, no-overlap rejected, and `gt_used_runtime=false`.
- Full ROS1/ROS2 bridge runtime did not pass: ROS2 topics were visible after FastDDS no-SHM setup, but Swarm-LIO2 odometry/cloud message rates were zero and odometry frame fields were empty.
- Swarm-LIO2 primary Nav2/tf ownership, Dynamic-LIO runtime cloud output, and ERASOR cleanup output are not validated.

Manual rerun commands:

```bash
bash scripts/manual/run_swarm_lio2_docker_build_and_test.sh
bash scripts/manual/run_dynamic_lio_docker_build_and_test.sh
bash scripts/manual/run_erasor_docker_build_and_test.sh
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

For direct launch inspection:

```bash
ros2 launch go2_gazebo_sim sim_hybrid_ros1_slam_ros2_nav.launch.py --show-args
bash scripts/launch/ros1_hybrid_slam_bridge.sh mode=sim
bash scripts/bench/run_sim_hybrid_ros1_slam_ros2_nav_validation.sh
```

## Real Robot Validation

This mode is simulation-only. Real robot validation must use `real_hybrid_ros1_slam_ros2_nav` and must run on the robot/Jetson or validated field computer.

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Backend Docker/catkin recovery passed, but full `sim_hybrid_ros1_slam_ros2_nav` did not pass.
- Fresh `START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh` passed the existing Fast-LIO / SC-PGO baseline regression with GTSAM C++ optimization.
- The current ROS1/ROS2 topic contract blocker is: missing ROS1 raw relay output topics for `/robot_a` and `/robot_b`, ROS2 `/<ns>/swarm_lio2/Odometry` and cloud topics present but below `0.1 Hz`, and empty odometry `header.frame_id` / `child_frame_id`.
- Swarm-LIO2 primary `/<ns>/Odometry` / `/<ns>/corrected_odom`, Nav2 odom/tf, and `team_loop_closure` keyframes remain runtime-unvalidated.
- Dynamic-LIO runtime static/dynamic cloud separation and ERASOR cleaned map output remain unvalidated; fallbacks stay active.
- Fast-LIO remains the production backend until sim and real primary validations pass.
- Do not claim Swarm-LIO2 replacement from this runbook alone.
