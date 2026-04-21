# Runtime Contracts (go2_nav_algorithms)

This package owns shared high-level navigation algorithms used by Gazebo and Isaac runtimes.

It also owns reusable launch builders in `launch/pipeline_components.py` for planner-stage
composition (scan projection, mapper, goal passthrough, waypoint mux wiring).

## Node Ownership

- `pointcloud_to_laserscan_node` (launched via `launch/pipeline_components.py`)
  - Consumes normalized planner PointCloud2 input (`/<ns>/registered_scan_reliable`).
  - Publishes planner LaserScan (`/<ns>/scan_3d`).

- `simple_scan_mapper_cpp`
  - Default runtime mapper for all stacks.
  - Consumes LaserScan + Odometry.
  - Publishes occupancy map (`nav_msgs/OccupancyGrid`).

- `simple_scan_mapper.py`
  - Consumes LaserScan + Odometry.
  - Publishes occupancy map (`nav_msgs/OccupancyGrid`).
  - Does not publish goals.

- `simple_frontier_explorer.py`
  - Consumes occupancy map + odometry.
  - Publishes frontier goal candidates and frontier markers.
  - Does not command velocity.

- `multi_robot_goal_assigner.py`
  - Consumes per-robot maps + odometry.
  - Publishes assigned coordinated goals.
  - Supports `legacy`, `committed`, and `mtare` coordination modes.

- `geometric_frontier.py`
  - Compatibility alias executable to `simple_frontier_explorer.py`.

## Stable Topic Contract

- Planner scan pipeline remains:
  - `/<ns>/registered_scan_reliable -> /<ns>/scan_3d -> /<ns>/map`.
- Goal output to controllers remains `/<ns>/way_point_coord`.
- Map output remains `/<ns>/map`.
- Marker contract remains compatible with existing RViz configs.
- Legacy compatibility wrappers remain callable from `go2_gazebo_sim` for one release cycle.

## Out of Scope

- Local control loops (`default_nav`).
- Sensor transport bridges (`qos_bridge`, `twist_bridge`).
- Gazebo/Isaac simulation bringup.
