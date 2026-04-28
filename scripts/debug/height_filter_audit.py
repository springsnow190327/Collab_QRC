#!/usr/bin/env python3
"""height_filter_audit.py — sanity-check the height filtering chain.

Runs against any namespace (sim or real). Audits:
  1. TF z-values: map→base_link, base_link→sensor (lidar)
  2. Raw registered_scan: z-histogram (in publisher frame)
  3. octomap point cloud centers: z-histogram (in map frame)
  4. octomap projected /map: cells in 1m disk around robot (occupied/free/unknown)
  5. /scan_3d (laserscan): just sample range/count to confirm it's alive
  6. terrain_map: z-histogram (FAR's view; sim only / when stack running)

For each cloud / map, also notes the publisher's frame_id so we can
correlate against the height-filter parameters that target that frame.

Usage:
  ./scripts/debug/height_filter_audit.py --ns robot_a
  ./scripts/debug/height_filter_audit.py --ns robot         # real-robot single
  ./scripts/debug/height_filter_audit.py --ns robot_a --duration 5
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from collections import Counter, defaultdict
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid


def _bin_z(z: float, bin_size: float = 0.05) -> float:
    return round(z / bin_size) * bin_size


def _z_histogram(zs: list[float]) -> str:
    if not zs:
        return "(empty)"
    c = Counter(_bin_z(z) for z in zs)
    lines = []
    bins = sorted(c.keys())
    z_min = min(bins)
    z_max = max(bins)
    total = sum(c.values())
    max_count = max(c.values())
    for z in bins:
        count = c[z]
        pct = 100 * count / total
        bar_len = int(40 * count / max_count)
        bar = "█" * bar_len
        lines.append(f"  z={z:+.2f}m: {count:6d} ({pct:5.1f}%) {bar}")
    return (
        f"  range: {z_min:+.2f} … {z_max:+.2f} m, total {total} points\n"
        + "\n".join(lines)
    )


def _read_pc2_xyz(msg: PointCloud2, max_points: int = 50000) -> list[tuple[float, float, float]]:
    """Iterate PointCloud2 yielding (x, y, z). Capped at max_points."""
    fields = {f.name: f for f in msg.fields}
    if "x" not in fields or "y" not in fields or "z" not in fields:
        return []
    x_off = fields["x"].offset
    y_off = fields["y"].offset
    z_off = fields["z"].offset
    step = msg.point_step
    n = msg.width * msg.height
    n = min(n, max_points)
    out = []
    data = msg.data
    for i in range(n):
        base = i * step
        x = struct.unpack_from("<f", data, base + x_off)[0]
        y = struct.unpack_from("<f", data, base + y_off)[0]
        z = struct.unpack_from("<f", data, base + z_off)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        out.append((x, y, z))
    return out


class HeightAudit(Node):
    def __init__(self, ns: str, duration_sec: float):
        super().__init__("height_filter_audit")
        self.ns = ns.strip("/")
        self.duration_sec = duration_sec

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest message slots for each topic
        self.latest: dict[str, object] = {}

        # QoS profiles to match common publishers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Topic candidates → (msg_type, qos, slot_name)
        topics: list[tuple[str, type, QoSProfile, str]] = [
            (f"/{self.ns}/registered_scan_reliable", PointCloud2, reliable_volatile, "raw_cloud"),
            (f"/{self.ns}/registered_scan_octomap", PointCloud2, reliable_volatile, "filtered_cloud"),
            (f"/{self.ns}/registered_scan_classified", PointCloud2, reliable_volatile, "classified_cloud"),
            (f"/{self.ns}/octomap_point_cloud_centers", PointCloud2, reliable_volatile, "octomap_voxels"),
            (f"/{self.ns}/map", OccupancyGrid, reliable_latched, "map_2d"),
            (f"/{self.ns}/map_raw", OccupancyGrid, reliable_latched, "map_raw_2d"),
            (f"/{self.ns}/scan_3d", LaserScan, sensor_qos, "scan_3d"),
            (f"/{self.ns}/terrain_map", PointCloud2, sensor_qos, "terrain_map"),
        ]
        for topic, msg_t, qos, slot in topics:
            self.create_subscription(
                msg_t,
                topic,
                lambda msg, s=slot, t=topic: self._on_msg(s, t, msg),
                qos,
            )
        self._topic_list = topics

    def _on_msg(self, slot: str, topic: str, msg) -> None:
        self.latest[slot] = (topic, msg)

    def run(self) -> int:
        start = time.time()
        deadline = start + self.duration_sec
        print(f"\n=== height_filter_audit  ns=/{self.ns}  collecting {self.duration_sec:.0f}s ===\n")
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._report()

    # ──────────────────────────────────────────────────────────────────
    def _report(self) -> int:
        print("=" * 70)
        print("TF lookups (frame heights are critical for understanding filters)")
        print("=" * 70)
        self._tf_report("map", "base_link", f"{self.ns}/base_link")
        self._tf_report("map", f"b_base_link", "b_base_link")
        for sensor_frame in [
            "livox_mid360", "b_livox_mid360", "lidar_link",
            "sensor", "vehicle",
        ]:
            self._tf_report("map", sensor_frame, sensor_frame)
        # base_link → sensor (offset of lidar above body)
        for sensor_frame in ["livox_mid360", "b_livox_mid360", "lidar_link"]:
            self._tf_report("base_link", sensor_frame, f"base_link → {sensor_frame}")

        print()
        print("=" * 70)
        print("Cloud z-histograms (5 cm bins)")
        print("=" * 70)
        for slot in ["raw_cloud", "filtered_cloud", "classified_cloud", "octomap_voxels", "terrain_map"]:
            entry = self.latest.get(slot)
            if entry is None:
                print(f"\n[{slot}]  no message received during audit window")
                continue
            topic, msg = entry
            xyz = _read_pc2_xyz(msg, max_points=20000)
            zs = [p[2] for p in xyz]
            print(f"\n[{slot}]  {topic}")
            print(f"  frame_id: '{msg.header.frame_id}'  (point coords interpreted in this frame)")
            print(f"  point count: {msg.width * msg.height}")
            print(_z_histogram(zs))

        print()
        print("=" * 70)
        print("LaserScan stats")
        print("=" * 70)
        entry = self.latest.get("scan_3d")
        if entry is None:
            print("\n[scan_3d]  no message")
        else:
            topic, msg = entry
            valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
            print(f"\n[scan_3d]  {topic}")
            print(f"  frame_id: '{msg.header.frame_id}'")
            print(f"  rays: {len(msg.ranges)}, valid: {len(valid)}")
            print(f"  range_min={msg.range_min:.2f}  range_max={msg.range_max:.2f}")
            if valid:
                print(f"  hits: min={min(valid):.2f}  max={max(valid):.2f}  mean={sum(valid)/len(valid):.2f}")

        print()
        print("=" * 70)
        print("OccupancyGrid (2D /map) — cells around robot")
        print("=" * 70)
        for slot in ["map_2d", "map_raw_2d"]:
            entry = self.latest.get(slot)
            if entry is None:
                print(f"\n[{slot}]  no message")
                continue
            topic, msg = entry
            self._occupancy_report(slot, topic, msg)

        print()
        print("=" * 70)
        print("Frame-reference cheat sheet for height filters")
        print("=" * 70)
        print("""
  pointcloud_to_laserscan min/max_height : target_frame (typically base_link)
  octomap point_cloud_min/max_z          : map (frame_id of octree)
  octomap occupancy_min/max_z            : map (frame_id of octree)
  octomap filter_ground_plane (RANSAC)   : base_frame_id (typically base_link)
  terrain_analysis maxRelZ / minRelZ     : vehicle z (≈ base_link)
  localPlanner maxRelZ / minRelZ         : vehicle z (≈ base_link)
  terrain_free_Z (FAR)                   : LOCAL ground z (per-voxel estimate)
  obstacleHeightThre (localPlanner)      : LOCAL ground z (terrain_analysis intensity)
""")
        return 0

    def _tf_report(self, parent: str, child: str, label: str) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=0.3)
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw = math.atan2(2 * (r.w * r.z + r.x * r.y),
                             1 - 2 * (r.y * r.y + r.z * r.z))
            print(f"  {parent} → {label}: x={t.x:+.2f}  y={t.y:+.2f}  z={t.z:+.3f}  yaw={math.degrees(yaw):+.1f}°")
        except Exception as e:
            err = str(e).split("\n")[0]
            print(f"  {parent} → {label}: NOT FOUND ({err[:80]})")

    def _occupancy_report(self, slot: str, topic: str, msg: OccupancyGrid) -> None:
        info = msg.info
        res = info.resolution
        print(f"\n[{slot}]  {topic}")
        print(f"  frame_id: '{msg.header.frame_id}'  resolution={res:.3f}m  size={info.width}x{info.height}")
        print(f"  origin: x={info.origin.position.x:.2f} y={info.origin.position.y:.2f}")

        # Whole-map stats (always — global signal of map quality)
        self._occ_value_counts("whole map", msg.data)

        # Cells in 2 m square around robot (use map→base_link TF if available)
        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id, "base_link", Time(),
                timeout=Duration(seconds=0.3),
            )
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    msg.header.frame_id, "b_base_link", Time(),
                    timeout=Duration(seconds=0.3),
                )
                rx, ry = tf.transform.translation.x, tf.transform.translation.y
            except Exception:
                print("  (no base_link TF — skipping per-cell stats)")
                # Whole-map stats
                self._occ_value_counts("whole map", msg.data)
                return
        print(f"  robot at: ({rx:+.2f}, {ry:+.2f})")
        # 2m square around robot
        half_m = 1.0
        gx = int((rx - info.origin.position.x) / res)
        gy = int((ry - info.origin.position.y) / res)
        radius = int(math.ceil(half_m / res))
        w, h = info.width, info.height
        cells = []
        for dy in range(-radius, radius + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                continue
            for dx in range(-radius, radius + 1):
                nx = gx + dx
                if nx < 0 or nx >= w:
                    continue
                cells.append(msg.data[ny * w + nx])
        self._occ_value_counts(f"2 m square around robot ({len(cells)} cells)", cells)
        center = msg.data[gy * w + gx] if 0 <= gx < w and 0 <= gy < h else None
        print(f"  CELL AT ROBOT CENTER: value={center}  ({self._occ_label(center)})")

    @staticmethod
    def _occ_label(v: Optional[int]) -> str:
        if v is None:
            return "outside map"
        if v == -1:
            return "unknown"
        if v < 50:
            return "free"
        return "occupied"

    def _occ_value_counts(self, label: str, values) -> None:
        counts = Counter(values)
        free = sum(c for v, c in counts.items() if v != -1 and v < 50)
        occ = sum(c for v, c in counts.items() if v != -1 and v >= 50)
        unk = counts.get(-1, 0)
        total = free + occ + unk
        if total == 0:
            print(f"  {label}: empty")
            return
        print(f"  {label}:  free={free} ({100*free/total:.1f}%)  "
              f"occ={occ} ({100*occ/total:.1f}%)  unknown={unk} ({100*unk/total:.1f}%)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="robot_a", help="robot namespace (default robot_a)")
    ap.add_argument("--duration", type=float, default=4.0, help="seconds to listen (default 4)")
    # parse_known_args lets us pass through --ros-args -r /tf:=...
    args, ros_argv = ap.parse_known_args(argv)
    # rclpy expects argv with the executable name in [0]
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = HeightAudit(args.ns, args.duration)
        return node.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
