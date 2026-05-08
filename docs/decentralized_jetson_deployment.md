# Decentralized Two-Jetson Deployment

## Goal

Each robot runs a replicated team-SLAM stack onboard. No central laptop is
required for descriptors, robust loop selection, factor export, or alignment
status.

## Launch

Robot A:

```bash
ros2 launch go2_gazebo_sim decentralized_robot.launch.py \
  robot_id:=robot_a \
  peer_robot_id:=robot_b \
  decentralized_mode:=true \
  use_dynamic_filter:=true \
  team_pose_graph_backend:=auto \
  team_comm_mode:=dds
```

Robot B:

```bash
ros2 launch go2_gazebo_sim decentralized_robot.launch.py \
  robot_id:=robot_b \
  peer_robot_id:=robot_a \
  decentralized_mode:=true \
  use_dynamic_filter:=true \
  team_pose_graph_backend:=auto \
  team_comm_mode:=dds
```

## Communication Policy

DDS is the default transport. Descriptors and metadata are always exchanged.
Compact static keyframe clouds are forwarded only after a descriptor candidate
references that keyframe.

Defaults:

- `peer_keyframe_rate_hz: 0.5`
- `peer_cloud_voxel_size: 0.4`
- `peer_cloud_max_points: 2000`
- `peer_descriptor_only_until_candidate: true`
- `send_cloud_only_on_candidate: true`

## Readiness

Run:

```bash
bash scripts/deploy/check_jetson_readiness.sh
```

The script checks CPU architecture, RAM, ROS 2 visibility, Livox package
availability, optional peer ping, and records external blockers in
`logs/jetson_readiness_report.md`.
