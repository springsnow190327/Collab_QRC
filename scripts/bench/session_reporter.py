#!/usr/bin/env python3
"""Bounded exploration session reporter for headless benchmark runs.

Runs for a fixed wall-clock window and captures three classes of metric:

  1. SAFETY      — wall contact events (from `/mujoco/contacts`, the live
                   mjData.contact stream), tip-over extremes (roll/pitch).
  2. COVERAGE    — explored area from `/<ns>/map` OccupancyGrid.
  3. PROGRESS    — cumulative odom distance, final pose, peak speed.

On exit (duration elapsed OR SIGTERM from a parent launch file), flushes a
final JSON summary to disk and exits 0 so the launch's OnProcessExit handler
shuts the rest of the stack down cleanly.

JSON is also snapshotted every 10 s so a mid-run crash still leaves usable
data behind.

CLI:
    python3 scripts/session_reporter.py \
        --duration 120 \
        --namespace robot \
        --output /tmp/session_reports/latest.json

The script is designed to run as an ExecuteProcess inside
`nav_test_mujoco.launch.py`; see `session_duration_sec` launch arg.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


WALL_PREFIXES = ("wall_", "divider_")
ALLOWED_NON_WALL_GEOMS = {
    "ground",
    "green_marker_1", "green_marker_2", "green_marker_3",
    "box_obstacle_1", "box_obstacle_2",
}
STATUS_INTERVAL_SEC = 10.0
TIPPED_OVER_RAD = math.radians(45.0)


def _is_wall_geom(name: str) -> bool:
    return name.startswith(WALL_PREFIXES)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _roll_pitch_yaw_from_quat(q) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass
class ContactEvent:
    t_sec: float
    robot_geom: str
    wall_geom: str
    pos: tuple[float, float, float]


@dataclass
class SessionMetrics:
    duration_target_sec: float
    namespace: str
    started_at: float
    ended_at: float = 0.0
    elapsed_sec: float = 0.0

    # Coverage
    map_received: bool = False
    map_resolution_m: float = 0.0
    map_width_cells: int = 0
    map_height_cells: int = 0
    free_cells: int = 0
    occupied_cells: int = 0
    unknown_cells: int = 0
    explored_area_m2: float = 0.0
    explored_fraction: float = 0.0
    # Ground-truth-normalized coverage: explored_area_m2 / scene_area_m2.
    # scene_area_m2 is the sim-side observable ground truth (inner room
    # footprint) — for vlm_exploration_scene_no_artifacts.xml this is
    # 12 m x 8 m = 96 m². 0.0 means disabled / unknown.
    scene_area_m2: float = 0.0
    coverage_ratio_of_scene: float = 0.0

    # Odom / progress
    odom_received: bool = False
    start_xy: tuple[float, float] | None = None
    end_xy: tuple[float, float] | None = None
    distance_travelled_m: float = 0.0
    peak_speed_mps: float = 0.0
    peak_roll_deg: float = 0.0
    peak_pitch_deg: float = 0.0
    tipped_over: bool = False

    # Safety
    contact_msg_count: int = 0
    wall_contact_events: list[ContactEvent] = field(default_factory=list)
    unique_geom_pairs_hit: set[tuple[str, str]] = field(default_factory=set)
    hit_wall_count_by_name: dict[str, int] = field(default_factory=dict)
    hit_robot_count_by_name: dict[str, int] = field(default_factory=dict)

    # SLAM quality vs MuJoCo ground truth (relative drift — each stream is
    # normalized to its own start pose, so map-frame vs world-frame offsets
    # don't show up as fake drift).
    gt_received: bool = False
    slam_received: bool = False
    pose_error_samples: int = 0
    trans_error_sum_m: float = 0.0
    trans_error_peak_m: float = 0.0
    trans_error_final_m: float = 0.0
    yaw_error_sum_deg: float = 0.0
    yaw_error_peak_deg: float = 0.0
    yaw_error_final_deg: float = 0.0

    # Outcome
    outcome: str = "running"  # running | completed | sigterm | exception


class SessionReporter(Node):
    def __init__(
        self,
        duration_sec: float,
        namespace: str,
        output_path: Path,
        scene_area_m2: float = 0.0,
    ) -> None:
        super().__init__("session_reporter")
        self._duration = duration_sec
        self._ns = namespace.strip().strip("/") or "robot"
        self._output = output_path
        self._output.parent.mkdir(parents=True, exist_ok=True)

        self._m = SessionMetrics(
            duration_target_sec=duration_sec,
            namespace=self._ns,
            started_at=time.time(),
            scene_area_m2=scene_area_m2,
        )
        self._wall_start = time.monotonic()
        self._last_status_at = self._wall_start
        self._prev_odom_xy: tuple[float, float] | None = None
        # Relative-drift tracking: each stream's first observation is its
        # "anchor", subsequent samples are compared as deltas from anchor.
        self._gt_anchor: tuple[float, float, float] | None = None
        self._slam_anchor: tuple[float, float, float] | None = None
        self._latest_gt_delta: tuple[float, float, float] | None = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, "/mujoco/contacts", self._on_contacts, sensor_qos
        )
        self.create_subscription(
            Odometry, f"/{self._ns}/odom/nav", self._on_odom, 10
        )
        self.create_subscription(
            Odometry, f"/{self._ns}/odom/ground_truth", self._on_odom_gt, 10
        )
        self.create_subscription(
            OccupancyGrid, f"/{self._ns}/map", self._on_map, 1
        )

        self.create_timer(0.25, self._tick)

        print("=" * 70, flush=True)
        scene_str = (
            f"{scene_area_m2:.1f} m² (pass ≥90% → {0.9 * scene_area_m2:.1f} m²)"
            if scene_area_m2 > 0
            else "<disabled>"
        )
        print(
            "  SESSION REPORTER\n"
            f"    duration   : {duration_sec:.0f} s\n"
            f"    namespace  : /{self._ns}\n"
            f"    output     : {output_path}\n"
            f"    scene area : {scene_str}",
            flush=True,
        )
        print("=" * 70, flush=True)

    # ── Subscribers ───────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry) -> None:
        """SLAM odom (Cartographer → carto_odom_bridge → /{ns}/odom/nav)."""
        self._m.odom_received = True
        self._m.slam_received = True
        p = msg.pose.pose.position
        xy = (float(p.x), float(p.y))
        if self._m.start_xy is None:
            self._m.start_xy = xy
        self._m.end_xy = xy
        if self._prev_odom_xy is not None:
            dx = xy[0] - self._prev_odom_xy[0]
            dy = xy[1] - self._prev_odom_xy[1]
            d = math.hypot(dx, dy)
            if d < 1.0:
                self._m.distance_travelled_m += d
        self._prev_odom_xy = xy

        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        if speed > self._m.peak_speed_mps:
            self._m.peak_speed_mps = speed

        roll, pitch, yaw = _roll_pitch_yaw_from_quat(msg.pose.pose.orientation)
        rd, pd = math.degrees(roll), math.degrees(pitch)
        if abs(rd) > abs(self._m.peak_roll_deg):
            self._m.peak_roll_deg = rd
        if abs(pd) > abs(self._m.peak_pitch_deg):
            self._m.peak_pitch_deg = pd
        if abs(roll) > TIPPED_OVER_RAD or abs(pitch) > TIPPED_OVER_RAD:
            self._m.tipped_over = True

        # Pose-error vs ground truth (relative drift).
        if self._slam_anchor is None:
            self._slam_anchor = (xy[0], xy[1], yaw)
            return
        if self._latest_gt_delta is None:
            return
        slam_dx = xy[0] - self._slam_anchor[0]
        slam_dy = xy[1] - self._slam_anchor[1]
        slam_dyaw = _wrap_pi(yaw - self._slam_anchor[2])
        gt_dx, gt_dy, gt_dyaw = self._latest_gt_delta
        ex = gt_dx - slam_dx
        ey = gt_dy - slam_dy
        eyaw_deg = abs(math.degrees(_wrap_pi(gt_dyaw - slam_dyaw)))
        trans_err = math.hypot(ex, ey)
        self._m.pose_error_samples += 1
        self._m.trans_error_sum_m += trans_err
        self._m.yaw_error_sum_deg += eyaw_deg
        if trans_err > self._m.trans_error_peak_m:
            self._m.trans_error_peak_m = trans_err
        if eyaw_deg > self._m.yaw_error_peak_deg:
            self._m.yaw_error_peak_deg = eyaw_deg
        self._m.trans_error_final_m = trans_err
        self._m.yaw_error_final_deg = eyaw_deg

    def _on_odom_gt(self, msg: Odometry) -> None:
        """MuJoCo ground-truth odom (/{ns}/odom/ground_truth)."""
        self._m.gt_received = True
        p = msg.pose.pose.position
        _, _, yaw = _roll_pitch_yaw_from_quat(msg.pose.pose.orientation)
        now = (float(p.x), float(p.y), yaw)
        if self._gt_anchor is None:
            self._gt_anchor = now
            self._latest_gt_delta = (0.0, 0.0, 0.0)
            return
        self._latest_gt_delta = (
            now[0] - self._gt_anchor[0],
            now[1] - self._gt_anchor[1],
            _wrap_pi(now[2] - self._gt_anchor[2]),
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._m.map_received = True
        res = float(msg.info.resolution)
        w = int(msg.info.width)
        h = int(msg.info.height)
        self._m.map_resolution_m = res
        self._m.map_width_cells = w
        self._m.map_height_cells = h

        free = occ = unk = 0
        for v in msg.data:
            if v < 0:
                unk += 1
            elif v >= 50:
                occ += 1
            else:
                free += 1
        self._m.free_cells = free
        self._m.occupied_cells = occ
        self._m.unknown_cells = unk
        cell_area = res * res if res > 0 else 0.0
        known = free + occ
        self._m.explored_area_m2 = known * cell_area
        total = max(1, w * h)
        self._m.explored_fraction = known / total
        if self._m.scene_area_m2 > 0:
            self._m.coverage_ratio_of_scene = \
                self._m.explored_area_m2 / self._m.scene_area_m2

    def _on_contacts(self, msg: String) -> None:
        self._m.contact_msg_count += 1
        t_rel = time.monotonic() - self._wall_start
        for ln in msg.data.split("\n"):
            if not ln:
                continue
            parts = ln.split("|")
            if len(parts) < 3:
                continue
            n1, n2, pos_str = parts[0], parts[1], parts[2]
            w1 = _is_wall_geom(n1)
            w2 = _is_wall_geom(n2)
            if not (w1 or w2):
                continue
            other = n2 if w1 else n1
            wall = n1 if w1 else n2
            if other in ALLOWED_NON_WALL_GEOMS or _is_wall_geom(other):
                continue

            robot_label = other if other and other != "_" else "<unnamed_robot_geom>"
            try:
                pos = tuple(float(x) for x in pos_str.split(","))
            except ValueError:
                pos = (0.0, 0.0, 0.0)
            ev = ContactEvent(
                t_sec=t_rel, robot_geom=robot_label, wall_geom=wall,
                pos=(pos[0], pos[1], pos[2]) if len(pos) == 3 else (0.0, 0.0, 0.0),
            )
            self._m.wall_contact_events.append(ev)
            self._m.unique_geom_pairs_hit.add((robot_label, wall))
            self._m.hit_wall_count_by_name[wall] = \
                self._m.hit_wall_count_by_name.get(wall, 0) + 1
            self._m.hit_robot_count_by_name[robot_label] = \
                self._m.hit_robot_count_by_name.get(robot_label, 0) + 1

    # ── Timer ─────────────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._wall_start
        self._m.elapsed_sec = elapsed

        if now - self._last_status_at >= STATUS_INTERVAL_SEC:
            self._last_status_at = now
            self._print_status(elapsed)
            self._flush_json(final=False)

        if elapsed >= self._duration:
            self._m.outcome = "completed"
            self._finalize_and_exit(0)

    def _print_status(self, elapsed: float) -> None:
        fx = self._m.end_xy or (float("nan"), float("nan"))
        cov_str = (
            f"{self._m.coverage_ratio_of_scene * 100:5.1f}%"
            if self._m.scene_area_m2 > 0 else "  n/a"
        )
        drift_str = (
            f"drift={self._m.trans_error_final_m:.2f}m/"
            f"{self._m.yaw_error_final_deg:.1f}° "
            if self._m.pose_error_samples > 0 else ""
        )
        print(
            f"[session_reporter] t={elapsed:6.1f}/{self._duration:.0f}s "
            f"explored={self._m.explored_area_m2:6.2f} m² "
            f"({cov_str}) "
            f"dist={self._m.distance_travelled_m:5.2f} m "
            f"contacts={len(self._m.wall_contact_events):3d} "
            f"{drift_str}"
            f"pose=({fx[0]:+.2f},{fx[1]:+.2f})",
            flush=True,
        )

    # ── JSON IO ───────────────────────────────────────────────────────
    def _to_dict(self) -> dict[str, Any]:
        m = self._m
        return {
            "outcome": m.outcome,
            "duration_target_sec": m.duration_target_sec,
            "elapsed_sec": round(m.elapsed_sec, 3),
            "started_at_unix": m.started_at,
            "ended_at_unix": m.ended_at,
            "namespace": m.namespace,
            "coverage": {
                "map_received": m.map_received,
                "resolution_m": m.map_resolution_m,
                "width_cells": m.map_width_cells,
                "height_cells": m.map_height_cells,
                "free_cells": m.free_cells,
                "occupied_cells": m.occupied_cells,
                "unknown_cells": m.unknown_cells,
                "explored_area_m2": round(m.explored_area_m2, 3),
                "explored_fraction_of_grid": round(m.explored_fraction, 4),
                "scene_area_m2": m.scene_area_m2,
                "coverage_ratio_of_scene": round(m.coverage_ratio_of_scene, 4),
                "coverage_pass_90pct": (
                    m.scene_area_m2 > 0
                    and m.coverage_ratio_of_scene >= 0.90
                ),
            },
            "progress": {
                "odom_received": m.odom_received,
                "start_xy": list(m.start_xy) if m.start_xy else None,
                "end_xy": list(m.end_xy) if m.end_xy else None,
                "distance_travelled_m": round(m.distance_travelled_m, 3),
                "peak_speed_mps": round(m.peak_speed_mps, 3),
                "peak_roll_deg": round(m.peak_roll_deg, 2),
                "peak_pitch_deg": round(m.peak_pitch_deg, 2),
                "tipped_over": m.tipped_over,
            },
            "slam": {
                "slam_received": m.slam_received,
                "gt_received": m.gt_received,
                "pose_error_samples": m.pose_error_samples,
                "trans_error_peak_m": round(m.trans_error_peak_m, 4),
                "trans_error_mean_m": round(
                    m.trans_error_sum_m / m.pose_error_samples, 4
                ) if m.pose_error_samples else 0.0,
                "trans_error_final_m": round(m.trans_error_final_m, 4),
                "yaw_error_peak_deg": round(m.yaw_error_peak_deg, 3),
                "yaw_error_mean_deg": round(
                    m.yaw_error_sum_deg / m.pose_error_samples, 3
                ) if m.pose_error_samples else 0.0,
                "yaw_error_final_deg": round(m.yaw_error_final_deg, 3),
            },
            "safety": {
                "wall_contact_count": len(m.wall_contact_events),
                "unique_geom_pairs_hit": sorted(
                    [list(p) for p in m.unique_geom_pairs_hit]
                ),
                "hit_walls": dict(sorted(m.hit_wall_count_by_name.items(),
                                         key=lambda x: -x[1])),
                "hit_robot_parts": dict(sorted(m.hit_robot_count_by_name.items(),
                                               key=lambda x: -x[1])),
                "first_contact_sec": (
                    m.wall_contact_events[0].t_sec
                    if m.wall_contact_events else None
                ),
                "events": [
                    {
                        "t_sec": round(ev.t_sec, 3),
                        "robot": ev.robot_geom,
                        "wall": ev.wall_geom,
                        "pos": [round(x, 3) for x in ev.pos],
                    }
                    for ev in m.wall_contact_events[:50]
                ],
                "events_truncated": len(m.wall_contact_events) > 50,
                "contact_msgs_received": m.contact_msg_count,
            },
        }

    def _flush_json(self, final: bool) -> None:
        self._m.ended_at = time.time()
        payload = self._to_dict()
        tmp = self._output.with_suffix(self._output.suffix + ".tmp")
        try:
            tmp.write_text(json.dumps(payload, indent=2))
            os.replace(tmp, self._output)
        except OSError as e:
            print(f"[session_reporter] WARN: failed to write {self._output}: {e}",
                  flush=True)
        if final:
            cov = payload["coverage"]
            print("=" * 70, flush=True)
            print(f"[session_reporter] FINAL outcome={payload['outcome']} "
                  f"elapsed={payload['elapsed_sec']:.1f}s", flush=True)
            if cov["scene_area_m2"] > 0:
                ratio = cov["coverage_ratio_of_scene"]
                passed = cov["coverage_pass_90pct"]
                print(f"  explored  : {cov['explored_area_m2']:.2f} m² / "
                      f"{cov['scene_area_m2']:.1f} m² gt → "
                      f"{ratio * 100:.1f}%  "
                      f"[{'PASS' if passed else 'FAIL'} ≥90%]", flush=True)
            else:
                print(f"  explored  : {cov['explored_area_m2']:.2f} m²  "
                      f"({cov['explored_fraction_of_grid'] * 100:.1f}% of grid)",
                      flush=True)
            print(f"  distance  : {payload['progress']['distance_travelled_m']:.2f} m"
                  f"  peak {payload['progress']['peak_speed_mps']:.2f} m/s", flush=True)
            slam = payload.get("slam", {})
            if slam.get("pose_error_samples", 0) > 0:
                print(
                    f"  slam drift: trans peak {slam['trans_error_peak_m']:.3f} m "
                    f"mean {slam['trans_error_mean_m']:.3f} m "
                    f"final {slam['trans_error_final_m']:.3f} m | "
                    f"yaw peak {slam['yaw_error_peak_deg']:.2f}° "
                    f"mean {slam['yaw_error_mean_deg']:.2f}° "
                    f"final {slam['yaw_error_final_deg']:.2f}°  "
                    f"(n={slam['pose_error_samples']})",
                    flush=True,
                )
            elif not slam.get("gt_received"):
                print("  slam drift: <no ground-truth odom received>", flush=True)
            print(f"  contacts  : {payload['safety']['wall_contact_count']} "
                  f"(unique pairs: {len(payload['safety']['unique_geom_pairs_hit'])})",
                  flush=True)
            if payload["safety"]["hit_walls"]:
                print(f"  walls hit : {payload['safety']['hit_walls']}", flush=True)
            if payload["safety"]["hit_robot_parts"]:
                print(f"  robot hit : {payload['safety']['hit_robot_parts']}",
                      flush=True)
            print(f"  json      : {self._output}", flush=True)
            print("=" * 70, flush=True)

    def _finalize_and_exit(self, code: int) -> None:
        if getattr(self, "_already_finalized", False):
            return
        self._already_finalized = True
        self._flush_json(final=True)
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        # sys.exit inside an rclpy timer/signal callback can be swallowed
        # by the executor, leaving the subprocess alive past its bound.
        # os._exit forces immediate termination, which lets the launch
        # OnProcessExit handler fire and cascade a clean Shutdown.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


def _parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0,
                    help="Session duration in seconds (default 120)")
    ap.add_argument("--namespace", default="robot",
                    help="Robot namespace (default 'robot')")
    ap.add_argument("--output", default="/tmp/session_reports/latest.json",
                    help="Output JSON path (default /tmp/session_reports/latest.json)")
    ap.add_argument("--scene-area-m2", type=float, default=0.0,
                    help="Sim ground-truth observable area in m² (denominator "
                         "for coverage_ratio_of_scene). Defaults to 0 (disabled).")
    return ap.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = SessionReporter(
        duration_sec=args.duration,
        namespace=args.namespace,
        output_path=Path(args.output),
        scene_area_m2=args.scene_area_m2,
    )

    def _sigterm(_signo, _frame):
        # Don't overwrite a completed outcome — when the outer `timeout`
        # arrives after the reporter finished its bounded session but
        # before the launch finished cascading shutdown, we want the
        # report to still say "completed".
        if node._m.outcome == "running":
            node._m.outcome = "sigterm"
        node._finalize_and_exit(0)

    signal.signal(signal.SIGTERM, _sigterm)
    signal.signal(signal.SIGINT, _sigterm)

    try:
        rclpy.spin(node)
    except SystemExit:
        raise
    except Exception as e:
        node._m.outcome = "exception"
        print(f"[session_reporter] exception: {e}", flush=True)
        node._finalize_and_exit(1)


if __name__ == "__main__":
    main()
