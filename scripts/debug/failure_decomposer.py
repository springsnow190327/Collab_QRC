#!/usr/bin/env python3
"""failure_decomposer — for every collision, print which safety layer let it through.

Subscribes to every layer in the perception → planning → execution chain.
When the dual_robot_collision_monitor reports an OBSTACLE_SCUFF, this node
queries every layer's most-recent state and tells you which one first
classified the obstacle as "free" — the layer that needs fixing.

Layers checked, in order from sensor up to actuator:

  L1 raw cloud           : did the LiDAR even see the obstacle?
  L2 terrain_map (XYZI)  : did terrain_analysis put intensity above threshold?
  L3 /map (OccupancyGrid): did octomap mark the cell occupied?
  L4 free_paths          : did localPlanner know about it?
  L5 chosen /local_path  : did localPlanner choose to avoid it?
  L6 path_safety_filter  : did the post-localPlanner filter catch it?
  L7 cmd_vel_safety_shield: did the omega-kill shield catch the swept clip?
  L8 executed cmd_vel    : what did the robot actually try to do?

Verdict logic: the FIRST layer that didn't see the obstacle (or didn't act
on seeing it) is the one to fix. The output names that layer + a short
"likely cause" pointing at the parameter / code path most likely responsible.

Usage:
  ./scripts/debug/failure_decomposer.py --ns robot_a
  Then run the sim/real autonomy as normal. Decomposer logs to stdout
  and to /{ns}/failure_decomposer_report (one std_msgs/String per scuff).
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


# Search radius (m) around the contact point used to decide
# "did this layer see the obstacle".
PROBE_RADIUS_M = 0.30


def _read_pc2_points(msg: PointCloud2, max_points: int = 80000):
    """Yield (x,y,z, intensity_or_None) tuples. Stops at max_points."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return
    xo = fields["x"].offset
    yo = fields["y"].offset
    zo = fields["z"].offset
    has_i = "intensity" in fields
    io = fields["intensity"].offset if has_i else None
    step = msg.point_step
    n = msg.width * msg.height
    n = min(n, max_points)
    data = msg.data
    for i in range(n):
        b = i * step
        x = struct.unpack_from("<f", data, b + xo)[0]
        y = struct.unpack_from("<f", data, b + yo)[0]
        z = struct.unpack_from("<f", data, b + zo)[0]
        if has_i:
            inten = struct.unpack_from("<f", data, b + io)[0]
        else:
            inten = None
        yield (x, y, z, inten)


def _within(px, py, qx, qy, r):
    return (px - qx) ** 2 + (py - qy) ** 2 <= r * r


class FailureDecomposer(Node):
    def __init__(self, ns: str):
        super().__init__("failure_decomposer")
        self.ns = ns.strip("/")

        # Latest snapshots of each layer
        self.last_raw_cloud:  Optional[PointCloud2]   = None
        self.last_terrain:    Optional[PointCloud2]   = None  # /terrain_map (XYZI)
        self.last_obstacle:   Optional[PointCloud2]   = None  # filter output (XYZ)
        self.last_map:        Optional[OccupancyGrid] = None
        self.last_free_paths: Optional[PointCloud2]   = None
        self.last_path:       Optional[Path]          = None
        self.last_path_status:    Optional[str]       = None
        self.last_shield_status:  Optional[str]       = None
        self.last_cmd_raw:    Optional[TwistStamped]  = None
        self.last_cmd_out:    Optional[TwistStamped]  = None
        self.last_odom:       Optional[Odometry]      = None

        # Recent scuff events (consume from collision_monitor stream).
        # We listen to /collision_events (stub: collision_monitor logs to
        # stdout; we also intercept its textual log via topic /rosout in
        # downstream — for clean ops we expect a SCUFF detector input).
        self.scuff_events = deque(maxlen=64)
        self._last_scuff_t = 0.0

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

        # Subscriptions: every layer we want to interrogate.
        self.create_subscription(
            PointCloud2, f"/{self.ns}/registered_scan_reliable",
            self._cb_raw_cloud, reliable_volatile)
        self.create_subscription(
            PointCloud2, f"/{self.ns}/terrain_map",
            self._cb_terrain, sensor_qos)
        self.create_subscription(
            PointCloud2, f"/{self.ns}/terrain_obstacle_cloud",
            self._cb_obstacle, reliable_volatile)
        self.create_subscription(
            OccupancyGrid, f"/{self.ns}/map",
            self._cb_map, reliable_latched)
        self.create_subscription(
            PointCloud2, f"/{self.ns}/free_paths",
            self._cb_free_paths, sensor_qos)
        self.create_subscription(
            Path, f"/{self.ns}/local_path",
            self._cb_path, reliable_volatile)
        self.create_subscription(
            String, f"/{self.ns}/path_safety_status",
            self._cb_path_status, 5)
        self.create_subscription(
            String, f"/{self.ns}/cmd_vel_shield_status",
            self._cb_shield_status, 5)
        self.create_subscription(
            TwistStamped, f"/{self.ns}/cmd_vel_stamped_raw",
            self._cb_cmd_raw, 5)
        self.create_subscription(
            TwistStamped, f"/{self.ns}/cmd_vel_stamped",
            self._cb_cmd_out, 5)
        self.create_subscription(
            Odometry, f"/{self.ns}/odom/nav",
            self._cb_odom, sensor_qos)

        # Trigger: collision_monitor publishes scuff events as String JSON
        # on /collision_events (we'll add this publisher; for now this
        # node also accepts manual injection via the trigger topic).
        self.create_subscription(
            String, "/collision_events", self._cb_scuff_event, 10)

        self.report_pub = self.create_publisher(
            String, f"/{self.ns}/failure_decomposer_report", 10)

        self.get_logger().info(
            f"failure_decomposer armed for ns=/{self.ns}. "
            "Listening for /collision_events.")

    # ── Subscriber callbacks (cache latest message) ────────────────────
    def _cb_raw_cloud(self, msg):    self.last_raw_cloud = msg
    def _cb_terrain(self, msg):       self.last_terrain = msg
    def _cb_obstacle(self, msg):      self.last_obstacle = msg
    def _cb_map(self, msg):           self.last_map = msg
    def _cb_free_paths(self, msg):    self.last_free_paths = msg
    def _cb_path(self, msg):          self.last_path = msg
    def _cb_path_status(self, msg):   self.last_path_status = msg.data
    def _cb_shield_status(self, msg): self.last_shield_status = msg.data
    def _cb_cmd_raw(self, msg):       self.last_cmd_raw = msg
    def _cb_cmd_out(self, msg):       self.last_cmd_out = msg
    def _cb_odom(self, msg):          self.last_odom = msg

    def _cb_scuff_event(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"non-JSON scuff event: {msg.data[:80]}")
            return
        # Only process events for our namespace
        if event.get("robot", "").strip("/") != self.ns:
            return
        # Throttle: at most one decompose per 2 s (collision_monitor often
        # emits N events per second while stuck on the same pillar).
        now = time.time()
        if now - self._last_scuff_t < 2.0:
            return
        self._last_scuff_t = now
        self.scuff_events.append(event)
        self._decompose(event)

    # ── The actual diagnostic ──────────────────────────────────────────
    def _decompose(self, event: dict) -> None:
        ox, oy, oz = event.get("pos", [None, None, None])
        if ox is None:
            self.get_logger().warn("scuff event has no position; skipping")
            return
        part = event.get("part", "?")
        t_sim = event.get("t_sim", "?")

        report = [
            "",
            "═" * 72,
            f" FAILURE DECOMPOSE  {self.ns}  t_sim={t_sim}s",
            f"   contact: part={part}  pos=({ox:+.2f},{oy:+.2f},{oz:+.2f})",
            f"   probe radius: {PROBE_RADIUS_M:.2f} m around contact",
            "═" * 72,
        ]

        # L1: raw cloud
        l1_seen, l1_count, l1_z_range = self._probe_cloud(
            self.last_raw_cloud, ox, oy)
        report.append(self._line(
            "L1", "raw_cloud (registered_scan_reliable)",
            l1_seen,
            f"{l1_count} pts in {PROBE_RADIUS_M}m disk, z=[{l1_z_range[0]:+.2f},{l1_z_range[1]:+.2f}]"
            if l1_count else "no points (sensor blind to this xy)"))

        # L2: terrain_map (intensity)
        l2_count, l2_max_int = self._probe_terrain_intensity(
            self.last_terrain, ox, oy)
        l2_seen = l2_max_int >= 0.05  # threshold heuristic; will note actual
        report.append(self._line(
            "L2", "terrain_map (intensity)",
            l2_seen,
            f"{l2_count} pts, max intensity={l2_max_int:.3f}m "
            f"({'≥' if l2_seen else '<'} 0.05m threshold heuristic)"
            if l2_count else "no terrain_map points here (terrain_analysis blind)"))

        # L3: /map cell at obstacle
        l3_seen, l3_value = self._probe_occupancy(self.last_map, ox, oy)
        report.append(self._line(
            "L3", "/map cell (octomap projected)",
            l3_seen,
            f"cell value={l3_value} ({'occupied' if l3_seen else ('unknown' if l3_value == -1 else 'free')})"))

        # L4: free_paths (localPlanner candidates near contact)
        l4_seen, l4_count = self._probe_freepaths(
            self.last_free_paths, ox, oy)
        report.append(self._line(
            "L4", "free_paths (localPlanner saw obs near candidates)",
            l4_seen,
            f"{l4_count} candidate-path points within {PROBE_RADIUS_M}m of contact"))

        # L5: chosen path (did localPlanner steer near contact?)
        l5_steers_into = self._chosen_path_passes(self.last_path, ox, oy)
        report.append(self._line(
            "L5", "/local_path (localPlanner's chosen path)",
            not l5_steers_into,  # "did the right thing" = did NOT steer in
            "chosen path passes near contact" if l5_steers_into else "chosen path avoids contact"))

        # L6: path_safety_filter status
        l6_action = self._parse_status(self.last_path_status)
        l6_acted = l6_action in ("rejected", "rejected_start_collision",
                                  "truncated")
        report.append(self._line(
            "L6", "path_safety_filter", l6_acted,
            f"action={l6_action}"))

        # L7: cmd_vel_shield status
        l7_action = self._parse_status(self.last_shield_status)
        l7_acted = l7_action == "omega_killed"
        report.append(self._line(
            "L7", "cmd_vel_safety_shield", l7_acted,
            f"action={l7_action}"))

        # L8: actually executed cmd_vel
        if self.last_cmd_out:
            v = self.last_cmd_out.twist.linear.x
            w = self.last_cmd_out.twist.angular.z
        else:
            v, w = float("nan"), float("nan")
        report.append(self._line(
            "L8", "executed cmd_vel (post-shield)",
            False,
            f"v={v:+.2f} m/s  ω={w:+.2f} rad/s"))

        # ── Verdict: first layer that "didn't see / didn't act"
        verdict = self._verdict(l1_seen, l2_seen, l3_seen, l4_seen,
                                not l5_steers_into,
                                l6_acted, l7_acted)
        report.append("─" * 72)
        report.append(f" VERDICT: {verdict[0]}")
        report.append(f"   most likely fix: {verdict[1]}")
        report.append("═" * 72)

        body = "\n".join(report)
        self.get_logger().info(body)
        # Also publish structured report
        s = String()
        s.data = json.dumps({
            "schema": "failure_decomposer/v1",
            "ns": self.ns,
            "t_sim": t_sim,
            "contact": {"x": ox, "y": oy, "z": oz, "part": part},
            "L1_raw_seen": l1_seen,
            "L2_terrain_max_intensity": l2_max_int,
            "L3_map_value": l3_value,
            "L4_free_paths_obs": l4_seen,
            "L5_path_avoided": not l5_steers_into,
            "L6_path_filter_action": l6_action,
            "L7_shield_action": l7_action,
            "L8_executed_v": v,
            "L8_executed_w": w,
            "verdict": verdict[0],
            "fix_hint": verdict[1],
        })
        self.report_pub.publish(s)

    # ── Layer probes ───────────────────────────────────────────────────
    def _probe_cloud(self, msg, ox, oy):
        """Return (saw_anything, n_points, (z_min,z_max)) for points within
        PROBE_RADIUS_M of (ox, oy)."""
        if msg is None:
            return False, 0, (0.0, 0.0)
        zs = []
        for x, y, z, _ in _read_pc2_points(msg):
            if _within(x, y, ox, oy, PROBE_RADIUS_M):
                zs.append(z)
        if not zs:
            return False, 0, (0.0, 0.0)
        return True, len(zs), (min(zs), max(zs))

    def _probe_terrain_intensity(self, msg, ox, oy):
        """Return (n_points, max_intensity) for points within probe radius."""
        if msg is None:
            return 0, 0.0
        max_i = 0.0
        n = 0
        for x, y, z, inten in _read_pc2_points(msg):
            if inten is None:
                continue
            if _within(x, y, ox, oy, PROBE_RADIUS_M):
                n += 1
                if inten > max_i:
                    max_i = inten
        return n, max_i

    def _probe_occupancy(self, msg, ox, oy):
        if msg is None:
            return False, -2
        info = msg.info
        res = info.resolution
        if res <= 0:
            return False, -2
        gx = int((ox - info.origin.position.x) / res)
        gy = int((oy - info.origin.position.y) / res)
        w, h = info.width, info.height
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False, -3  # outside map
        v = msg.data[gy * w + gx]
        return v >= 50, v

    def _probe_freepaths(self, msg, ox, oy):
        if msg is None:
            return False, 0
        n = 0
        for x, y, _z, _i in _read_pc2_points(msg):
            if _within(x, y, ox, oy, PROBE_RADIUS_M):
                n += 1
        return n > 0, n

    def _chosen_path_passes(self, msg, ox, oy):
        if msg is None or not msg.poses:
            return False
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            if _within(x, y, ox, oy, PROBE_RADIUS_M):
                return True
        return False

    def _parse_status(self, json_str):
        if not json_str:
            return "no_status"
        try:
            d = json.loads(json_str)
            return d.get("action", "?")
        except json.JSONDecodeError:
            return "unparseable"

    # ── Verdict ─────────────────────────────────────────────────────────
    def _verdict(self, l1, l2, l3, l4, l5, l6, l7):
        """First layer that should have stopped this but didn't.
        Returns (description, suggested fix)."""
        if not l1:
            return ("L1 sensor blind",
                    "raw LiDAR didn't see this xy — check sensor mount, range, or occlusion (peer body / robot's own chassis blocking).")
        if not l2:
            return ("L2 terrain_analysis classified as ground",
                    "obstacle's intensity (height above local ground) is below threshold. "
                    "Lower min_obstacle_height_m, OR adjust terrain_analysis voxel size / quantileZ.")
        if not l3:
            return ("L3 octomap missed the obstacle in /map",
                    "terrain_analysis saw it but it didn't make it to /map. "
                    "Check: octomap_server cloud_in remap (must be /terrain_obstacle_cloud), "
                    "octomap z-band filters (must be open after the unification), "
                    "RANSAC ground filter (must be False).")
        if not l4:
            return ("L4 localPlanner didn't see the obstacle in its candidate paths",
                    "free_paths near the contact are empty. "
                    "Check terrain_map subscribe by localPlanner, useTerrainAnalysis=True, "
                    "and obstacleHeightThre matches the threshold (bound to min_obstacle_height_m).")
        if not l5:
            return ("L5 localPlanner picked a path through the obstacle",
                    "free_paths showed the obstacle but the chosen path passes near it. "
                    "Increase costScore (clearance preference), lower pointPerPathThre, "
                    "or check whether path candidates near walls have enough cost penalty.")
        if not l6:
            return ("L6 path_safety_filter let the bad path through",
                    "filter saw the path but didn't reject/truncate. "
                    "Check footprint_radius_m (probably too small for body envelope), "
                    "occ_threshold (might need lowering to count inflation halos), "
                    "and TF remaps (filter must use /{ns}/tf, not global /tf).")
        if not l7:
            return ("L7 cmd_vel_safety_shield didn't kill ω",
                    "robot likely swept its body into the obstacle during a turn. "
                    "Increase predict_horizon_sec, increase footprint length/width, "
                    "or check shield's TF remap (same gotcha as L6).")
        # L1-L7 all "saw it / acted" but contact still happened — physical
        # execution diverged from cmd_vel.
        return ("L8 physical execution diverged from cmd_vel",
                "all safety layers acted but the robot still hit. Likely CHAMP gait "
                "residual / inertia / pathFollower sending pre-shield cmds bypassed. "
                "Check cmd_vel chain (raw vs out timestamps), gait settling time, "
                "and brake_until on the planner side.")

    @staticmethod
    def _line(tag, label, ok, detail):
        mark = "✓" if ok else "✗"
        return f"  {tag} [{mark}] {label}: {detail}"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="robot_a", help="namespace to monitor")
    args, ros_argv = ap.parse_known_args(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = FailureDecomposer(args.ns)
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
