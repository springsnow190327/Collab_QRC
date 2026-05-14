"""3D-frontier-exploration launch wrapper.

Composes the canonical nav_test_mujoco_fastlio.launch.py with:
  - demo_ramp.xml scene (override mujoco_model_path)
  - spawn at (2, 0) — west end of ramp scene
  - nvblox_frontend mapper_node (CUDA 3D mapping)

For CFPA2 ig_dimension=3d, edit
src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml
(or pass a yaml overlay via --ros-args -p) — the base launch hardcodes the
config path. The cfpa2_single_robot_3d.yaml overlay sitting next to it
contains the 3-line diff (planning_map_topic_suffix + ig_dimension +
voxels_3d_topic_suffix) ready to copy in.

A future tidy-up is to wire `cfpa2_config_path` as a LaunchArgument in the
base launch; deferred so this wrapper stays purely additive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Resolve the workspace root to point at the demo_ramp scene by default.
    # (This launch file lives in <ws>/src/go2w/go2_gazebo_sim/launch/.)
    here = os.path.dirname(os.path.realpath(__file__))
    ws_root = os.path.realpath(os.path.join(here, "..", "..", "..", ".."))
    default_scene = os.path.join(
        ws_root, "src", "go2w", "go2_gazebo_sim", "mujoco", "demo_ramp.xml")

    args = [
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="2.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        # nvblox_frontend knobs
        DeclareLaunchArgument("nvblox_voxel_size_m", default_value="0.10"),
        # Costmap source: '3d' swaps both global+local StaticLayers to read
        # /robot/traversability_grid so planner and MPPI treat ramps as free.
        # Pass nav_costmap_mode:=2d to revert to the octomap-based baseline.
        DeclareLaunchArgument("nav_costmap_mode", default_value="3d",
            description="'3d': both costmaps use traversability_grid (nvblox). "
                        "'2d': default octomap /robot/map (baseline)."),
        DeclareLaunchArgument(
            "enable_legacy_2d_proj", default_value="false",
            description="Re-enable mapper_node's legacy 2D traversability "
                        "projection. Default false; the planned "
                        "elevation_mapping_cupy + grid_map filter pipeline "
                        "(docs/claude/plans/2026-05-14-trav-grid-rewrite.md) "
                        "owns /<ns>/traversability_grid when this is false. "
                        "Set true for A/B comparison or fallback."),
    ]

    # Reuse the full fastlio launch — it handles MuJoCo, Point-LIO/Fast-LIO,
    # CFPA2, Nav2, RViz, and all the supporting plumbing.
    base_launch_path = os.path.join(
        get_package_share_directory("go2_gazebo_sim"),
        "launch",
        "nav_test_mujoco_fastlio.launch.py")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
            "spawn_x":   LaunchConfiguration("spawn_x"),
            "spawn_y":   LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "rviz": LaunchConfiguration("rviz"),
            "gui":  LaunchConfiguration("gui"),
            "nav_costmap_mode": LaunchConfiguration("nav_costmap_mode"),
        }.items(),
    )

    # nvblox_frontend mapper node — delayed 5 s so any residual preflight
    # activity from a previous session finishes before the mapper starts.
    # respawn=True guards against SIGTERM from a concurrent preflight.
    mapper = Node(
        package="nvblox_frontend",
        executable="mapper_node",
        name="nvblox_frontend_mapper",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            # cloud_topic and odom_topic intentionally not set here.
            # The node's namespace (robot) prefixes relative defaults
            # cloud_registered_body → /robot/cloud_registered_body and
            # odom/nav → /robot/odom/nav, matching Fast-LIO + slam_odom_relay.
            "use_sim_time":      LaunchConfiguration("use_sim_time"),
            "world_frame":       "map",
            "voxel_size_m":      LaunchConfiguration("nvblox_voxel_size_m"),
            "publish_period_s":  0.5,
            # 40m × 40m world-fixed grid: covers demo_ramp (24×16m) plus
            # margin, so historical observations persist as the robot moves.
            "trav_xy_extent_m":  40.0,
            "voxel_xy_extent_m": 20.0,
            "voxel_z_extent_m":  3.0,
            "voxel_z_origin_m":  -0.5,
            "slope_max_deg":     30.0,
            "step_max_m":        0.20,
            "robot_clearance_m": 0.50,
            "enable_legacy_2d_proj": LaunchConfiguration("enable_legacy_2d_proj"),
        }],
    )

    # frontier_3d_test_node — subscribes voxels_3d + traversability_grid + goal_pose;
    # publishes /robot/frontier_3d_markers (top-5 spheres, red = current goal).
    frontier_viz = Node(
        package="cfpa2_collaborative_autonomy",
        executable="frontier_3d_test_node",
        name="frontier_3d_test_node",
        namespace=LaunchConfiguration("robot_namespace"),
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "top_n_clusters":  5,
            "publish_period_s": 1.0,
        }],
        output="screen",
        respawn=True,
        respawn_delay=3.0,
    )

    # Delay mapper + frontier viz by 5 s. The preflight kill script targets
    # "mapper_node" by name; if a second launch attempt runs ≤5 s into the
    # first, the mapper would be killed before MuJoCo even starts. A 5 s
    # delay means the mapper starts after MuJoCo+SLAM are up and any
    # concurrent preflight has already finished. respawn=True above gives a
    # second layer of protection if it's still killed.
    deferred = TimerAction(period=5.0, actions=[mapper, frontier_viz])

    return LaunchDescription([*args, base_launch, deferred])
