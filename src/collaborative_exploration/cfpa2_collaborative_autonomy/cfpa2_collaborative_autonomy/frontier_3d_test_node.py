#!/usr/bin/env python3
"""
Standalone validator for the 3D frontier extractor in [[frontier_3d]].

Subscribes:
  /robot/voxels_3d           (nvblox_frontend_msgs/VoxelGrid3D)
  /robot/traversability_grid (nav_msgs/OccupancyGrid)   — for ground projection
  /robot/goal_pose           (geometry_msgs/PoseStamped) — current Nav2 goal

Publishes:
  /robot/frontier_3d_markers (visualization_msgs/MarkerArray)
    - top-N spheres per cluster centroid (red = selected/current goal, cyan = others)
    - ground-projection lines + cylinders at Nav2 waypoint positions
    - text labels with volume and rank

Logs at 1 Hz a short summary per cluster.

Run:
  ros2 run cfpa2_collaborative_autonomy frontier_3d_test_node \\
      --ros-args -p robot_namespace:=robot
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA

try:
    from nvblox_frontend_msgs.msg import VoxelGrid3D
except ImportError as e:
    raise SystemExit(
        f"nvblox_frontend_msgs not importable: {e}\n"
        "Build with: colcon build --packages-select nvblox_frontend_msgs"
    )

from cfpa2_collaborative_autonomy.frontier_3d import (
    extract_3d_frontiers,
    project_to_traversability_goal,
)


# Cyan for unselected top-N frontiers (matches real-robot viz script).
_COLOR_UNSELECTED = (0.0, 0.8, 1.0)
# Red for the frontier closest to the current Nav2 goal.
_COLOR_SELECTED = (1.0, 0.1, 0.1)


class Frontier3DTestNode(Node):
    def __init__(self) -> None:
        super().__init__("frontier_3d_test_node")
        self.declare_parameter("robot_namespace", "robot")
        self.declare_parameter("min_unknown_volume_m3", 1.0)
        self.declare_parameter("min_frontier_voxels", 50)
        self.declare_parameter("border_margin_cells", 3)
        self.declare_parameter("geodesic_voronoi", False)
        self.declare_parameter("publish_period_s", 1.0)
        self.declare_parameter("top_n_clusters", 5)
        self.declare_parameter("selected_match_radius_m", 1.5)
        ns = str(self.get_parameter("robot_namespace").value).strip("/")
        self.min_vol    = float(self.get_parameter("min_unknown_volume_m3").value)
        self.min_vox    = int(self.get_parameter("min_frontier_voxels").value)
        self.border_m   = int(self.get_parameter("border_margin_cells").value)
        self.geodesic   = bool(self.get_parameter("geodesic_voronoi").value)
        self.top_n      = int(self.get_parameter("top_n_clusters").value)
        self.sel_radius = float(self.get_parameter("selected_match_radius_m").value)

        # Subscribe with BEST_EFFORT to match the mapper's voxels_3d publisher.
        qos_voxels = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Trav grid uses TRANSIENT_LOCAL since fix #7.
        qos_trav = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_goal = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._latest_voxels: VoxelGrid3D | None = None
        self._latest_trav: OccupancyGrid | None = None
        self._latest_goal: PoseStamped | None = None

        self.create_subscription(
            VoxelGrid3D, f"/{ns}/voxels_3d", self._voxels_cb, qos_voxels)
        self.create_subscription(
            OccupancyGrid, f"/{ns}/traversability_grid", self._trav_cb, qos_trav)
        self.create_subscription(
            PoseStamped, f"/{ns}/goal_pose", self._goal_cb, qos_goal)

        self.markers_pub = self.create_publisher(
            MarkerArray, f"/{ns}/frontier_3d_markers", 1)

        self.create_timer(
            float(self.get_parameter("publish_period_s").value), self._tick)

        self.get_logger().info(
            f"frontier_3d_test_node started. ns=/{ns} "
            f"min_vol={self.min_vol:.2f} m³ "
            f"min_vox={self.min_vox} border={self.border_m} "
            f"geodesic={self.geodesic} top_n={self.top_n}")

    # ------------------------------------------------------------------
    def _voxels_cb(self, msg: VoxelGrid3D) -> None:
        self._latest_voxels = msg

    def _trav_cb(self, msg: OccupancyGrid) -> None:
        self._latest_trav = msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._latest_goal = msg

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        v = self._latest_voxels
        if v is None:
            return
        nx, ny, nz = int(v.size_x), int(v.size_y), int(v.size_z)
        if nx * ny * nz == 0 or len(v.data) != nx * ny * nz:
            return
        # VoxelGrid3D layout: row-major (z, y, x). See mapper_node.cpp.
        try:
            data = np.frombuffer(bytes(v.data), dtype=np.int8).reshape(nz, ny, nx)
        except ValueError:
            self.get_logger().warn(
                f"voxels_3d data length {len(v.data)} ≠ nx*ny*nz {nx*ny*nz}")
            return

        all_clusters = extract_3d_frontiers(
            voxel_data=data,
            voxel_size_m=float(v.voxel_size),
            origin_xyz=(float(v.origin.x), float(v.origin.y), float(v.origin.z)),
            min_unknown_volume_m3=self.min_vol,
            min_frontier_voxels=self.min_vox,
            border_margin_cells=self.border_m,
            geodesic_voronoi=self.geodesic,
        )

        # Keep only top-N by unknown volume.
        clusters = sorted(all_clusters, key=lambda c: c.unknown_volume_m3, reverse=True)[: self.top_n]

        # Optional ground projection (if traversability_grid is up).
        ground_goals: list[tuple[float, float] | None] = []
        if self._latest_trav is not None:
            t = self._latest_trav
            tw, th = int(t.info.width), int(t.info.height)
            tarr = np.array(t.data, dtype=np.int8).reshape(th, tw)
            for c in clusters:
                ground_goals.append(project_to_traversability_goal(
                    centroid_xyz=c.centroid_world,
                    trav_grid=tarr,
                    trav_resolution_m=float(t.info.resolution),
                    trav_origin_xy=(float(t.info.origin.position.x),
                                    float(t.info.origin.position.y)),
                ))
        else:
            ground_goals = [None] * len(clusters)

        # Determine which cluster is currently targeted by Nav2.
        goal_xy: tuple[float, float] | None = None
        if self._latest_goal is not None:
            goal_xy = (self._latest_goal.pose.position.x,
                       self._latest_goal.pose.position.y)

        self._publish_markers(v.header.frame_id, v.header.stamp,
                              clusters, ground_goals, goal_xy)
        self._log_summary(clusters, ground_goals, goal_xy, len(all_clusters))

    # ------------------------------------------------------------------
    def _is_selected(self, ground_goal: tuple[float, float] | None,
                     centroid_xyz: tuple[float, float, float],
                     goal_xy: tuple[float, float] | None) -> bool:
        """True if this cluster is the one currently targeted (goal is nearby)."""
        if goal_xy is None:
            return False
        # Match against ground projection first; fall back to XY of centroid.
        ref = ground_goal if ground_goal is not None else (centroid_xyz[0], centroid_xyz[1])
        dist = math.hypot(ref[0] - goal_xy[0], ref[1] - goal_xy[1])
        return dist <= self.sel_radius

    def _publish_markers(self, frame_id, stamp, clusters, ground_goals, goal_xy):
        ma = MarkerArray()
        # Always emit a DELETEALL first so old clusters disappear next tick.
        m_clear = Marker()
        m_clear.header.frame_id = frame_id
        m_clear.header.stamp = stamp
        m_clear.action = Marker.DELETEALL
        ma.markers.append(m_clear)

        for rank, (c, gg) in enumerate(zip(clusters, ground_goals)):
            selected = self._is_selected(gg, c.centroid_world, goal_xy)
            cr, cg, cb = _COLOR_SELECTED if selected else _COLOR_UNSELECTED
            alpha_sphere = 0.95 if selected else 0.65

            # Centroid sphere — larger + brighter for selected.
            sph = Marker()
            sph.header.frame_id = frame_id
            sph.header.stamp = stamp
            sph.ns = "frontier_3d_centroid"
            sph.id = rank
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = c.centroid_world[0]
            sph.pose.position.y = c.centroid_world[1]
            sph.pose.position.z = c.centroid_world[2]
            sph.pose.orientation.w = 1.0
            radius = max(0.15, min(0.8, (3 * c.unknown_volume_m3 / (4 * np.pi)) ** (1 / 3)))
            scale = 2.0 * radius * (1.3 if selected else 1.0)
            sph.scale.x = sph.scale.y = sph.scale.z = scale
            sph.color = ColorRGBA(r=cr, g=cg, b=cb, a=alpha_sphere)
            ma.markers.append(sph)

            # Volume + rank label.
            sel_tag = " ◀GOAL" if selected else ""
            txt = Marker()
            txt.header.frame_id = frame_id
            txt.header.stamp = stamp
            txt.ns = "frontier_3d_label"
            txt.id = rank
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = c.centroid_world[0]
            txt.pose.position.y = c.centroid_world[1]
            txt.pose.position.z = c.centroid_world[2] + radius + 0.2
            txt.scale.z = 0.28 if selected else 0.22
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = (f"#{rank+1} {c.unknown_volume_m3:.1f}m³ "
                        f"A={c.frontier_area_m2:.1f}m²{sel_tag}")
            ma.markers.append(txt)

            # Ground-projection line + disk.
            if gg is not None:
                line = Marker()
                line.header.frame_id = frame_id
                line.header.stamp = stamp
                line.ns = "frontier_3d_groundlink"
                line.id = rank
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.06 if selected else 0.03
                line.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.6)
                p_top = Point()
                p_top.x, p_top.y, p_top.z = c.centroid_world
                p_grd = Point()
                p_grd.x, p_grd.y, p_grd.z = gg[0], gg[1], 0.05
                line.points = [p_top, p_grd]
                ma.markers.append(line)

                disk = Marker()
                disk.header.frame_id = frame_id
                disk.header.stamp = stamp
                disk.ns = "frontier_3d_ground"
                disk.id = rank
                disk.type = Marker.CYLINDER
                disk.action = Marker.ADD
                disk.pose.position.x = gg[0]
                disk.pose.position.y = gg[1]
                disk.pose.position.z = 0.05
                disk.pose.orientation.w = 1.0
                diam = 0.55 if selected else 0.35
                disk.scale.x = disk.scale.y = diam
                disk.scale.z = 0.05
                disk.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.9)
                ma.markers.append(disk)

        self.markers_pub.publish(ma)

    # ------------------------------------------------------------------
    def _log_summary(self, clusters, ground_goals, goal_xy, total_count):
        if not clusters:
            self.get_logger().info("3D frontier: 0 clusters above volume threshold")
            return
        total_v = sum(c.unknown_volume_m3 for c in clusters)
        msg = (f"3D frontier: {total_count} total → showing top {len(clusters)} "
               f"(Σvol={total_v:.1f} m³). Top:")
        for rank, (c, gg) in enumerate(zip(clusters, ground_goals)):
            sel = self._is_selected(gg, c.centroid_world, goal_xy)
            gg_s = f"→ground({gg[0]:+.2f},{gg[1]:+.2f})" if gg else "→ground:NONE"
            sel_s = " ◀GOAL" if sel else ""
            msg += (f"\n  #{rank+1} center=({c.centroid_world[0]:+.2f},"
                    f"{c.centroid_world[1]:+.2f},{c.centroid_world[2]:+.2f}) "
                    f"V={c.unknown_volume_m3:.1f}m³ A={c.frontier_area_m2:.1f}m² "
                    f"N={c.frontier_voxel_count} {gg_s}{sel_s}")
        self.get_logger().info(msg)


def main() -> None:
    rclpy.init()
    node = Frontier3DTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
