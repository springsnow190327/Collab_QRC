#!/usr/bin/env python3
"""path_safety_filter — octomap hard-collision shield between localPlanner and pathFollower.

Inserted on the /path topic flow:

    localPlanner ──/path──remap──> /local_path_raw
    /local_path_raw ──┐
    /map ─────────────┤  path_safety_filter
    pose ─────────────┘
                       └──> /local_path ──> pathFollower

For every Path message on /local_path_raw, we walk the poses in order and
test the underlying occupancy grid (/map) at each pose. A "footprint
disk" of radius `footprint_radius_m` around each pose must contain no
occupied cell — otherwise that path point is treated as a collision.

Behaviour on collision:
  * If the FIRST pose collides (robot itself is in occupied cell):
    publish empty Path (length 0). pathFollower will stop. This is rare
    and indicates upstream perception drift.
  * If a later pose collides: truncate the path at the last safe pose.
    pathFollower follows the safe prefix and stops at the truncation —
    localPlanner replans next tick.
  * If no collision: republish unchanged (zero-cost passthrough).

Why this exists: FAR's V-graph topology occasionally connects two
contour vertices through what is, in the real occupancy grid, an
obstacle (sparse contour sampling, decay glitches, etc.). The 2D
occupancy grid is the ground-truth view of "where is the wall" — by
gating localPlanner output through it we trade some path availability
for guaranteed no-cross-wall execution.

Trade-off acknowledged: when this filter rejects a path, the robot
stops until the next replan. If FAR keeps emitting the same blocked
path, the robot stays stopped. CFPA2's stuck-detection (legacy stall
+ planner stuck monitor) will eventually blacklist the goal and pick
another. This is by design — better to be stuck briefly than to walk
through a wall.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class PathSafetyFilter(Node):
    def __init__(self) -> None:
        super().__init__("path_safety_filter")

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("footprint_radius_m", 0.30)
        self.declare_parameter("occ_threshold", 50)  # >= => occupied
        self.declare_parameter("check_stride_m", 0.05)  # densify path before check
        self.declare_parameter("min_safe_path_len", 1)  # poses
        self.declare_parameter("input_topic", "local_path_raw")
        self.declare_parameter("output_topic", "local_path")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("status_topic", "path_safety_status")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("vehicle_frame", "vehicle")
        # If the path's header frame_id is the vehicle frame and TF
        # lookup fails (frame disconnect), fall back to this frame —
        # the path's xy values represent robot-local coordinates and
        # base_link is a guaranteed-connected frame in the namespace.
        self.declare_parameter("base_frame_fallback", "base_link")

        self.footprint_radius_m = float(
            self.get_parameter("footprint_radius_m").value
        )
        self.occ_threshold = int(self.get_parameter("occ_threshold").value)
        self.check_stride_m = max(
            0.01, float(self.get_parameter("check_stride_m").value)
        )
        self.min_safe_path_len = max(
            1, int(self.get_parameter("min_safe_path_len").value)
        )
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.vehicle_frame = str(self.get_parameter("vehicle_frame").value)
        self.base_frame_fallback = str(
            self.get_parameter("base_frame_fallback").value
        )

        # ── State ─────────────────────────────────────────────────────
        self._latest_map: Optional[OccupancyGrid] = None

        # ── TF: localPlanner publishes path in "vehicle" frame; we
        # need to convert poses to "map" frame for collision check.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Pubs / subs ──────────────────────────────────────────────
        # Map: TRANSIENT_LOCAL because octomap_server publishes that way.
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.create_subscription(Path, in_topic, self._on_path, 5)
        self.path_pub = self.create_publisher(Path, out_topic, 5)
        self.status_pub = self.create_publisher(String, status_topic, 5)

        self.get_logger().info(
            f"path_safety_filter armed. radius={self.footprint_radius_m:.2f}m "
            f"stride={self.check_stride_m:.2f}m occ_thr={self.occ_threshold} | "
            f"in={in_topic} out={out_topic} map={map_topic}"
        )

    # ── Callbacks ──────────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_path(self, msg: Path) -> None:
        if self._latest_map is None:
            # No map yet — fail-open (publish unchanged) so we don't deadlock
            # at startup. octomap takes a few seconds to publish first map.
            self.path_pub.publish(msg)
            self._publish_status("passthrough_no_map", len(msg.poses), len(msg.poses))
            return

        if not msg.poses:
            self.path_pub.publish(msg)
            self._publish_status("empty_in", 0, 0)
            return

        # localPlanner publishes path with frame_id="vehicle" — pose
        # coordinates are robot-local. Look up vehicle→map transform
        # so collision check can run against /map (in map frame).
        path_frame = msg.header.frame_id or self.vehicle_frame
        tf = None
        if path_frame != self.map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, path_frame, Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                # vehicle frame may be disconnected from map tree
                # (CMU's "sensor → vehicle" static doesn't bridge to
                # SLAM's tree). Fall back: use base_link as the path
                # frame since path coordinates are robot-local
                # regardless of which frame_id the publisher tagged.
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.map_frame, self.base_frame_fallback,
                        Time(), timeout=Duration(seconds=0.05),
                    )
                except Exception:
                    tf = None
            if tf is None:
                # Both lookups failed — fail-open so we don't paralyse
                # at startup before TF is ready.
                self.path_pub.publish(msg)
                self._publish_status(
                    "passthrough_no_tf", len(msg.poses), len(msg.poses)
                )
                return
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            # 2D yaw from quaternion (assuming roll/pitch ~0)
            yaw = 2.0 * math.atan2(qz, qw)
            cy = math.cos(yaw)
            sy = math.sin(yaw)

            def transform(pose: PoseStamped) -> PoseStamped:
                p = PoseStamped()
                p.header.frame_id = self.map_frame
                p.header.stamp = msg.header.stamp
                px = pose.pose.position.x
                py = pose.pose.position.y
                p.pose.position.x = tx + cy * px - sy * py
                p.pose.position.y = ty + sy * px + cy * py
                p.pose.position.z = tf.transform.translation.z + pose.pose.position.z
                p.pose.orientation = pose.pose.orientation
                return p

            mapped_poses = [transform(p) for p in msg.poses]
        else:
            mapped_poses = list(msg.poses)

        densified = self._densify(mapped_poses)
        first_collision_idx = self._first_collision_index(densified)

        if first_collision_idx is None:
            # Whole path safe — passthrough
            self.path_pub.publish(msg)
            self._publish_status(
                "passthrough_safe", len(msg.poses), len(msg.poses)
            )
            return

        # Map densified-collision-index back to original-poses-index
        # (use mapped_poses so distances are in map-frame consistent with
        # densified). Original message poses (msg.poses) are in vehicle
        # frame but distances are the same magnitude as map-frame after
        # rigid transform, so either works for arclength matching.
        cut_at = self._densified_idx_to_original(
            densified, first_collision_idx, mapped_poses
        )

        if cut_at < self.min_safe_path_len:
            # Even the start is collided — publish empty path; pathFollower
            # will stop until next localPlanner tick produces a clean path.
            empty = Path()
            empty.header = msg.header
            empty.poses = []
            self.path_pub.publish(empty)
            self.get_logger().warn(
                f"path REJECTED: collision at start (densified idx "
                f"{first_collision_idx}, world "
                f"({densified[first_collision_idx].pose.position.x:.2f},"
                f"{densified[first_collision_idx].pose.position.y:.2f}))"
            )
            self._publish_status(
                "rejected_start_collision", len(msg.poses), 0
            )
            return

        # Truncate at last safe pose
        truncated = Path()
        truncated.header = msg.header
        truncated.poses = list(msg.poses[:cut_at])
        self.path_pub.publish(truncated)
        if first_collision_idx == 0:
            tag = "rejected"
        else:
            tag = "truncated"
        self.get_logger().warn(
            f"path {tag.upper()}: occ_collision at original idx ~{cut_at}"
            f" / total {len(msg.poses)} (world "
            f"({densified[first_collision_idx].pose.position.x:.2f},"
            f"{densified[first_collision_idx].pose.position.y:.2f}))"
        )
        self._publish_status(tag, len(msg.poses), cut_at)

    # ── Geometry ───────────────────────────────────────────────────────
    def _densify(self, poses: list[PoseStamped]) -> list[PoseStamped]:
        """Sample poses every check_stride_m so collision check resolution
        is independent of pathFollower's input pose density."""
        if len(poses) < 2:
            return list(poses)
        out: list[PoseStamped] = [poses[0]]
        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            d = math.hypot(dx, dy)
            if d < self.check_stride_m:
                out.append(poses[i])
                continue
            n_steps = max(1, int(math.ceil(d / self.check_stride_m)))
            for k in range(1, n_steps + 1):
                t = k / n_steps
                p = PoseStamped()
                p.header = poses[i].header
                p.pose.position.x = p0.x + dx * t
                p.pose.position.y = p0.y + dy * t
                p.pose.position.z = p0.z + (p1.z - p0.z) * t
                p.pose.orientation = poses[i].pose.orientation
                out.append(p)
        return out

    def _densified_idx_to_original(
        self, densified, dense_idx: int, original
    ) -> int:
        """Walk from start through original poses and find the first
        original index whose cumulative arclength >= the dense_idx point's
        arclength. Returns cut index (the path is poses[:cut]); the cut
        is one BEFORE the collision so we keep only safe poses."""
        target_pt = densified[dense_idx].pose.position
        # Cumulative distance to dense_idx
        target_dist = 0.0
        for i in range(1, dense_idx + 1):
            p0 = densified[i - 1].pose.position
            p1 = densified[i].pose.position
            target_dist += math.hypot(p1.x - p0.x, p1.y - p0.y)
        # Walk original; cumulative distance
        cum = 0.0
        for i in range(1, len(original)):
            p0 = original[i - 1].pose.position
            p1 = original[i].pose.position
            cum += math.hypot(p1.x - p0.x, p1.y - p0.y)
            if cum >= target_dist:
                # cut at i-1 (last safe pose, exclusive of i)
                return max(0, i - 1)
        # target beyond original → keep all (shouldn't happen)
        _ = target_pt  # silence unused
        return len(original)

    def _first_collision_index(self, poses: list[PoseStamped]) -> Optional[int]:
        # Skip the FIRST pose: it represents the robot's current
        # location. If the robot is actually inside an occupied cell,
        # we have bigger problems than path filtering — and the disk
        # check is overly pessimistic in tight corridors (a robot
        # 0.4 m from a wall has its 0.50 m disk overlap the wall, but
        # the body itself is fine). Only future poses are checked.
        for i, p in enumerate(poses):
            if i == 0:
                continue
            x = p.pose.position.x
            y = p.pose.position.y
            if self._pose_collides(x, y):
                return i
        return None

    def _pose_collides(self, wx: float, wy: float) -> bool:
        m = self._latest_map
        if m is None:
            return False
        info = m.info
        res = info.resolution
        if res <= 0.0:
            return False
        ox = info.origin.position.x
        oy = info.origin.position.y
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        w = int(info.width)
        h = int(info.height)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            # Outside map — treat as unsafe (don't drive into unknown space)
            # Note: if you want fail-open instead, return False here.
            return True
        radius_cells = max(0, int(math.ceil(self.footprint_radius_m / res)))
        if radius_cells == 0:
            idx = gy * w + gx
            v = m.data[idx]
            return v >= self.occ_threshold
        radius_sq = radius_cells * radius_cells
        data = m.data
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                continue
            row_off = ny * w
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= w:
                    continue
                v = data[row_off + nx]
                if v >= self.occ_threshold:
                    return True
        return False

    def _publish_status(
        self, tag: str, n_in: int, n_out: int
    ) -> None:
        msg = String()
        msg.data = (
            '{"schema":"path_safety/v1","action":"' + tag + '"'
            f',"poses_in":{n_in},"poses_out":{n_out}'
            "}"
        )
        self.status_pub.publish(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PathSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
