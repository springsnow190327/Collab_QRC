#!/usr/bin/env python3
"""Real-robot SLAM launcher — selects between Cartographer+L1 and Fast-LIO+Mid360.

Contract (both backends must satisfy):
  - Publish TF: map → base_link (directly or via odom).
  - Provide a scan-compatible topic that pointcloud_to_laserscan can consume.

Cartographer (L1):
  - Subscribes to /utlidar/transformed_cloud (from transform_everything) + transformed_imu.
  - Publishes /<ns>/map_prob via cartographer_occupancy_grid_node.
  - Publishes TF map → body; a downstream static TF maps body → base_link.

Fast-LIO (Mid360):
  - Subscribes to /livox/lidar + /livox/imu.
  - Publishes TF camera_init → body (hardcoded frame names in laserMapping.cpp).
    This launch adds two static TFs so the nav stack sees map → base_link:
      map → camera_init     (identity rename)
      body → base_link      (identity — attach base_link as child of body)
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


# Hardcoded fallback if URDF parse fails. Matches the values that were
# baked into slam.launch.py before 2026-05-09 (mount tilt calibrated
# 2026-04-17 via tools/measure_mid360_tilt.py).
_MOUNT_FALLBACK_RPY_INV = ("0.036809", "-0.263591", "0")


def _read_mount_inverse_rpy() -> tuple[str, str, str]:
    """Read the Mid-360 mount tilt from livox_mid360.xacro and return its
    inverse rpy (component-wise negation, OK for small angles & axis-aligned
    rotations) as strings — for the body→base_link static publisher.

    URDF is the source of truth; this avoids duplicating the calibration
    constant between livox_mid360.xacro and slam.launch.py. If parsing
    fails (file missing / malformed), fall back to the hardcoded values.
    """
    import xml.etree.ElementTree as ET
    try:
        pkg = get_package_share_directory("go2_description")
    except Exception:
        return _MOUNT_FALLBACK_RPY_INV
    xacro_path = os.path.join(pkg, "xacro", "livox_mid360.xacro")
    if not os.path.isfile(xacro_path):
        return _MOUNT_FALLBACK_RPY_INV
    try:
        tree = ET.parse(xacro_path)
    except ET.ParseError:
        return _MOUNT_FALLBACK_RPY_INV
    for joint in tree.iter():
        # ET stores tag with namespace prefix in some XMLs; match suffix.
        tag = joint.tag.rsplit("}", 1)[-1]
        if tag != "joint" or joint.get("name") != "livox_mid360_mount_joint":
            continue
        for child in joint:
            if child.tag.rsplit("}", 1)[-1] != "origin":
                continue
            rpy_str = child.get("rpy", "0 0 0")
            try:
                r, p, y = (float(v) for v in rpy_str.split())
            except ValueError:
                return _MOUNT_FALLBACK_RPY_INV
            return (f"{-r:.6f}", f"{-p:.6f}", f"{-y:.6f}")
    return _MOUNT_FALLBACK_RPY_INV


def _launch_setup(context):
    slam = _get(context, "slam").strip().lower() or "carto_l1"
    mode_2d = _get(context, "carto_mode").strip().lower() == "2d"
    bringup_share = get_package_share_directory("go2w_real_bringup")

    actions = []

    if slam == "carto_l1":
        lua = "cartographer_l1_2d.lua" if mode_2d else "cartographer_l1_3d.lua"
        config_dir = os.path.join(bringup_share, "config", "slam")
        actions.append(
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                arguments=[
                    "-configuration_directory", config_dir,
                    "-configuration_basename", lua,
                ],
                remappings=[
                    ("points2", "/utlidar/transformed_cloud"),
                    # Cartographer 3D needs real gravity — use the raw (non-zeroed)
                    # transformed IMU. /utlidar/transformed_imu has orientation and
                    # linear_acceleration zeroed and will trigger an imu_tracker
                    # gravity-vector CHECK failure.
                    ("imu", "/utlidar/transformed_raw_imu"),
                ],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
                remappings=[("map", "/robot/map_prob")],
                parameters=[{"use_sim_time": False}],
                output="screen",
            )
        )
        # Attach `body` beneath base_link so Cartographer's tracking_frame has
        # a TF path to the nav stack's base_link. Direction matters: Cartographer
        # publishes map → base_link (published_frame=base_link), so base_link
        # must remain root of the robot subtree. body hangs off as a child.
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_body_carto",
                arguments=[
                    "--frame-id", "base_link", "--child-frame-id", "body",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )
        # map → odom (identity) — Nav2's local_costmap is configured with
        # `global_frame: odom` (REP-105 convention), but Cartographer doesn't
        # publish an `odom` frame: it produces `map → base_link` directly,
        # then `base_link → body` is added above. controller_server's TF
        # lookup `odom → base_link` therefore fails with `Invalid frame ID
        # "odom"`. Same fix pattern as the fastlio_mid360 branch below
        # (added 2026-04-30): a static identity makes odom resolvable via
        # tree-walk through map. No multi-parenting risk — base_link stays
        # parented to `map` via Cartographer's dynamic; odom is a separate
        # child of map. The fast_lio_tf_adapter is not running in carto mode
        # so CLAUDE.md golden rule 16 ("adapter is the single owner of
        # odom→base_link TF") is preserved by scope: only relevant when
        # slam=fastlio_mid360. Independent of CHAMP's broken state_estimation
        # (golden rule "bypassed, not fixed" still applies — this fix
        # doesn't touch the EKF/leg-odom path).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity_carto",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "odom",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )

    elif slam == "fastlio_mid360":
        config_path = os.path.join(bringup_share, "config", "slam")
        mid360_json = os.path.join(config_path, "MID360_config.json")

        # ── livox_ros_driver2: reads UDP from Mid-360 @ 192.168.123.20,
        #    publishes /livox/lidar (Livox CustomMsg, xfer_format=1) + /livox/imu.
        #    Must start BEFORE fastlio_mapping so topics exist when it subscribes.
        #    frame_id="body" matches Fast-LIO's tracking frame (hardcoded in
        #    laserMapping.cpp: trans.child_frame_id="body"). The Mid-360's
        #    IMU↔LiDAR extrinsic is handled by fastlio_mid360.yaml extrinsic_T,
        #    so the ~4 cm offset doesn't need a separate TF frame.
        actions.append(
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                parameters=[
                    {"xfer_format": 1},                # Livox CustomMsg (what Fast-LIO expects)
                    {"multi_topic": 0},                # single /livox/lidar
                    {"data_src": 0},                   # 0=lidar, 1=lvx file
                    {"publish_freq": 10.0},
                    {"output_data_type": 0},
                    {"frame_id": "body"},              # match Fast-LIO's child frame
                    {"lvx_file_path": ""},
                    {"user_config_path": mid360_json},
                    {"cmdline_input_bd_code": "livox0000000001"},
                ],
                output="screen",
            )
        )

        actions.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                parameters=[
                    os.path.join(config_path, "fastlio_mid360.yaml"),
                    {"use_sim_time": False},
                ],
                output="screen",
            )
        )
        # Fast-LIO's actual TF output is hardcoded as `camera_init → body`
        # (see vendor/fast_lio/src/laserMapping.cpp: trans.child_frame_id = "body").
        # Note: earlier versions of this file added a body_lidar_to_base_link
        # static TF, which was a dangling dead-frame — `body_lidar` is never
        # part of Fast-LIO's chain. Removed 2026-04-17.
        #
        # Bridge Fast-LIO's naming to the nav stack's map→base_link expectation:
        #   map → camera_init    (RUNTIME-MEASURED via gravity_align_at_init.py)
        #   (Fast-LIO dynamic)   camera_init → body
        #   body → base_link     (static, inverse of mount tilt — see below)
        #
        # Fast-LIO2 does NOT gravity-align its world frame (see
        # vendor/fast_lio/src/IMU_Processing.hpp:195 — the rot-to-gravity
        # init line is commented out). So `camera_init` = body orientation
        # at t=0 in body's own (tilted) frame. To get a gravity-aligned
        # `map`, we measure the actual body tilt at startup from the IMU
        # itself and publish the matching static.
        #
        # 2026-05-09 refactor: the previous map→camera_init static was
        # hardcoded to (-0.0368, +0.2636) — the mount tilt measured on
        # FLAT ground. That worked indoors / on level startup but tilted
        # the whole map by the ground angle whenever the robot started on
        # a slope. Replaced with gravity_align_at_init.py which buffers
        # ~1 s of static IMU and publishes the matching latched static.
        # If the IMU window is non-static or missing, the node falls back
        # to the hardcoded mount-tilt values (preserves old behavior).
        # Locate the runtime script via REPO_ROOT (set by real_autonomy.sh)
        # or fall back to a relative search from the bringup share dir.
        _repo_root = os.environ.get("REPO_ROOT") or os.path.abspath(
            os.path.join(bringup_share, "..", "..", "..", "..")
        )
        _gravity_script = os.path.join(
            _repo_root, "scripts", "runtime", "gravity_align_at_init.py"
        )
        # Fallback values for the gravity-align node — also pulled from the
        # URDF mount calibration so all three TF inputs (URDF chain,
        # body→base_link static, gravity-align fallback) stay in sync.
        # `_read_mount_inverse_rpy()` returns the BODY→BASE inverse; the
        # gravity-align fallback expects the FORWARD MAP→CAMERA_INIT
        # rotation (= the original mount tilt without negation), so we
        # negate the inverse back.
        _ri, _pi, _yi = _read_mount_inverse_rpy()
        _fb_roll = f"{-float(_ri):.6f}"
        _fb_pitch = f"{-float(_pi):.6f}"
        actions.append(
            ExecuteProcess(
                cmd=["python3", "-u", _gravity_script,
                     "--ros-args",
                     "-p", "imu_topic:=/livox/imu",
                     "-p", "samples_required:=200",
                     "-p", "max_wait_sec:=5.0",
                     "-p", "static_thresh_g:=0.05",
                     "-p", "parent_frame:=map",
                     "-p", "child_frame:=camera_init",
                     "-p", f"fallback_roll:={_fb_roll}",
                     "-p", f"fallback_pitch:={_fb_pitch}"],
                name="gravity_align_at_init",
                output="screen",
            )
        )
        # Mid-360 mount calibration. Source of truth is
        # go2_description/xacro/livox_mid360.xacro (mount joint origin rpy).
        # We read those values at launch time and emit the inverse here for
        # the body→base_link static. Single-source-of-truth: re-calibrating
        # the mount only requires editing the xacro file.
        # Translation is left at (0, 0, 0) — the URDF mount has translation
        # (0.08, 0, 0.12) but slam.launch.py treats body as colocated with
        # base_link (raycast/octomap error at 5 cm resolution is small).
        _r_inv, _p_inv, _y_inv = _read_mount_inverse_rpy()
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link_fastlio",
                arguments=[
                    "--frame-id", "body", "--child-frame-id", "base_link",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll",  _r_inv,
                    "--pitch", _p_inv,
                    "--yaw",   _y_inv,
                ],
                output="log",
            )
        )
        # map → odom (identity) — phantom frame so Nav2's local_costmap can
        # resolve TF(odom → base_link) via tree walk through the existing
        # chain (odom → map (inv) → camera_init → body → base_link). On real
        # there is no loop-closure correction yet, so map and odom are
        # functionally identical; once SC-PGO ports cleanly, this static
        # gets replaced by SC-PGO's dynamic map→odom + the adapter takes
        # over odom→base_link as the canonical REP-105 split.
        # The fast_lio_tf_adapter on real is launched with publish_tf=false
        # (see real_single.launch.py) precisely to avoid double-parenting
        # base_link (body→base_link static above is already its parent).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_identity",
                arguments=[
                    "--frame-id", "map", "--child-frame-id", "odom",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )

    elif slam == "swarm_lio2_hybrid":
        # ── Phase B: ROS1 Swarm-LIO2 in Docker, bridged to ROS2.
        # Prerequisite (manual): from docker/ros1_hybrid_slam/, run
        #   `docker compose up -d`
        # to start ros1_master + ros1_hybrid_slam + ros1_bridge. This launch
        # only spawns the ROS2-side livox driver and the topic relays that
        # bridge to/from the docker stack's `/robot_a/...` contract.
        #
        # Topic graph (single-robot path uses robot_a in the docker stack):
        #   ROS2  /livox/lidar (PointCloud2, xfer_format=0)
        #         → relay → /robot_a/velodyne_points
        #         → ros1_bridge → ROS1 /robot_a/velodyne_points
        #         → docker-internal relay → /quad1_pcl_render_node/sensor_cloud
        #         → swarm_lio (drone_id=1)
        #   ROS1  /quad1/lidar_slam/odom
        #         → docker-internal relay → /robot_a/swarm_lio2_raw/Odometry
        #         → ros1_bridge → ROS2 /robot_a/swarm_lio2_raw/Odometry
        #         → relay → /Odometry  (consumed by fast_lio_tf_adapter)
        #
        # Frame chain: swarm_lio publishes Odometry with frame_id=quad1/world,
        # child_frame_id=quad1_aft_mapped. fast_lio_tf_adapter rewrites those
        # to map / base_link respectively when it republishes the topic +
        # publishes TF. So we set fast_lio_publish_tf=true on real for this
        # SLAM (vs. false for fastlio_mid360 which uses static-chain TF).
        # The cloud topic frame_id stays quad1_aft_mapped, so we add a
        # static TF aliasing it to base_link for octomap's TF lookup.
        config_path = os.path.join(bringup_share, "config", "slam")
        mid360_json = os.path.join(config_path, "MID360_config.json")

        from launch.actions import LogInfo
        actions.append(LogInfo(msg=(
            "[slam] swarm_lio2_hybrid → expecting docker compose stack from "
            "docker/ros1_hybrid_slam/ to be running. If `ros2 topic list` "
            "doesn't show /robot_a/swarm_lio2_raw/Odometry within ~10s, "
            "start it with `docker compose up -d` and relaunch."
        )))

        # Livox driver — PointCloud2 output (xfer_format=0) for swarm_lio's
        # PCL-based input contract. Note: differs from fastlio_mid360 above
        # which uses xfer_format=1 (Livox CustomMsg).
        actions.append(
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                parameters=[
                    {"xfer_format": 0},
                    {"multi_topic": 0},
                    {"data_src": 0},
                    {"publish_freq": 10.0},
                    {"output_data_type": 0},
                    {"frame_id": "body"},
                    {"lvx_file_path": ""},
                    {"user_config_path": mid360_json},
                    {"cmdline_input_bd_code": "livox0000000001"},
                ],
                output="screen",
            )
        )

        # ROS2 input relays — feed docker stack via ros1_bridge.
        actions += [
            Node(
                package="topic_tools", executable="relay",
                name="livox_lidar_to_swarm_lio2_in",
                arguments=["/livox/lidar", "/robot_a/velodyne_points"],
                output="log",
            ),
            Node(
                package="topic_tools", executable="relay",
                name="livox_imu_to_swarm_lio2_in",
                arguments=["/livox/imu", "/robot_a/imu/data"],
                output="log",
            ),
        ]

        # ROS2 output relays — pull docker stack outputs into the names the
        # rest of the bringup expects (/Odometry → fast_lio_tf_adapter,
        # /cloud_registered_body → octomap_server).
        actions += [
            Node(
                package="topic_tools", executable="relay",
                name="swarm_lio2_odom_out",
                arguments=["/robot_a/swarm_lio2_raw/Odometry", "/Odometry"],
                output="log",
            ),
            Node(
                package="topic_tools", executable="relay",
                name="swarm_lio2_cloud_body_out",
                arguments=["/robot_a/swarm_lio2_raw/cloud_static", "/cloud_registered_body"],
                output="log",
            ),
        ]

        # Static TF: octomap consumes /cloud_registered_body whose frame_id
        # is "quad1_aft_mapped" (set by swarm_lio's laserMapping.cpp). Alias
        # it to base_link so the tf2 lookup `map → quad1_aft_mapped` resolves
        # via `map → base_link → quad1_aft_mapped` (fast_lio_tf_adapter
        # publishes the map→base_link dynamic).
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="quad1_aft_mapped_to_base_link",
                arguments=[
                    "--frame-id", "base_link",
                    "--child-frame-id", "quad1_aft_mapped",
                    "--x", "0", "--y", "0", "--z", "0",
                    "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                ],
                output="log",
            )
        )

    else:
        raise ValueError(
            f"Unknown slam backend '{slam}' "
            "(expected carto_l1 | fastlio_mid360 | swarm_lio2_hybrid)"
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("slam", default_value="carto_l1"),
            DeclareLaunchArgument("carto_mode", default_value="3d", description="2d or 3d"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
