# Real Hybrid ROS1 SLAM / ROS2 Nav

`real_hybrid_ros1_slam_ros2_nav` keeps the ROS2/Humble high-level stack on the laptop or team computer while each robot runs the ROS1/Noetic SLAM layer onboard.

## Implemented Scaffolding

- Onboard ROS1 side is expected to provide LiDAR/IMU interfaces, Swarm-LIO2, Dynamic-LIO filtering outputs, and ERASOR async cleanup.
- ROS2 side keeps Nav2, exploration, `team_loop_closure`, `robust_loop_selector`, `team_pose_graph_node`, `relative_transform_manager`, and `/merged_map` gating.
- `swarm_lio2_shadow` must pass before `swarm_lio2_primary`.
- In primary mode, the ROS2 adapter republishes Swarm-LIO2 outputs into the existing odom/cloud contract so downstream ROS2 nodes do not consume Swarm-LIO2 internals.

## Mock / Synthetic Validation

- ROS2 adapter and safety-gate synthetic tests are available on the development host.
- These tests validate message contracts and gate math only. They do not validate real LiDAR/IMU timing, Jetson load, DDS peer communication, Nav2 runtime, or Go2 motion safety.

## Docker Runtime Validation

Docker is only for simulation hybrid backend checks. It does not prove real robot readiness. The latest Docker/catkin run did build the shared ROS1 hybrid backend image and kept the Swarm-LIO2 wrapper launch alive for a 45 s smoke window, but that does not validate robot LiDAR/IMU timing, Jetson load, DDS peer communication, or Nav2 motion safety.

```bash
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

## Real Robot Validation

Run preflight first on the robot/Jetson or validated field computer:

```bash
bash scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh --host
```

Run shadow before primary:

```bash
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh
```

Direct bringup entrypoints remain:

```bash
scripts/real/real_autonomy.sh deployment_mode=real_hybrid_ros1_slam_ros2_nav slam_backend=swarm_lio2_shadow
scripts/real/onboard_ros1_slam.sh slam_backend=swarm_lio2_shadow
ros2 launch go2_gazebo_sim real_hybrid_ros1_slam_ros2_nav.launch.py
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- This host lacks native ROS1 Noetic/catkin/rospack for real backend build checks.
- Live `/livox/lidar`, `/livox/imu`, `/sportmodestate`, DDS/bridge observation, and peer robot network are unavailable here.
- Real preflight result on this host: ROS2 Humble is available, but ROS1 Noetic, `catkin_make` / `catkin build`, `rospack`, LiDAR topic, IMU topic, Unitree topic, peer robot IP/reachability, and DDS/bridge observation are missing.
- Docker/catkin simulation recovery does not satisfy real robot validation; real shadow and primary scripts must run on the robot/Jetson or validated field computer.
- Real deployment cannot be marked passed without robot/Jetson/Go2 validation logs.
- Fast-LIO remains production for real deployment until real shadow and real primary Swarm-LIO2 validations pass.
