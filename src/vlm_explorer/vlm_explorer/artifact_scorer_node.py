#!/usr/bin/env python3
"""Ground truth scorer for artifact detection.

Subscribes to /vlm/artifact_detections and compares against known red block
positions from the Gazebo world file. Publishes live precision/recall/F1
and per-block match status.

Publishes:
  /vlm/artifact_score (String, JSON) — live scoring summary
  /vlm/artifact_gt_markers (MarkerArray) — green=matched, yellow=unmatched GT

Logs a final summary CSV row to ~/.ros/log/artifact_scores/.
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# Ground truth red block positions from vlm_exploration.world
_GROUND_TRUTH = [
    {"id": "red_block_1", "x": 1.8, "y": 3.5, "room": "A_north"},
    {"id": "red_block_2", "x": 3.2, "y": 1.8, "room": "A_center"},
    {"id": "red_block_3", "x": 8.0, "y": 3.0, "room": "B_west"},
    {"id": "red_block_4", "x": 11.2, "y": 1.5, "room": "B_east"},
    {"id": "red_block_5", "x": 4.5, "y": 0.3, "room": "corridor"},
    {"id": "red_block_6", "x": 3.5, "y": -3.0, "room": "CD_south"},
    {"id": "red_block_7", "x": 10.5, "y": -3.2, "room": "D_east"},
    {"id": "red_block_8", "x": 1.2, "y": -2.0, "room": "C_west"},
]


class ArtifactScorerNode(Node):
    def __init__(self):
        super().__init__("artifact_scorer")

        self.declare_parameter("detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("score_topic", "/vlm/artifact_score")
        self.declare_parameter("gt_marker_topic", "/vlm/artifact_gt_markers")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("match_radius_m", 1.5)
        self.declare_parameter("rate", 0.5)
        self.declare_parameter("log_dir", "")

        self._match_radius = self.get_parameter("match_radius_m").value
        self._rate = self.get_parameter("rate").value
        self._marker_frame = str(self.get_parameter("marker_frame_id").value)
        log_dir = str(self.get_parameter("log_dir").value).strip()

        self._score_pub = self.create_publisher(
            String, self.get_parameter("score_topic").value, 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter("gt_marker_topic").value, 10
        )

        self.create_subscription(
            String,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10,
        )

        self._detections: list[dict] = []
        self._last_score: dict = {}
        self._start_time = time.monotonic()

        # Log directory
        if log_dir:
            self._log_dir = Path(log_dir)
        else:
            self._log_dir = Path.home() / ".ros" / "log" / "artifact_scores"
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._log_dir / f"score_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with open(self._csv_path, "w") as f:
            f.write(
                "elapsed_s,n_detections,true_positives,false_positives,"
                "missed,precision,recall,f1,matched_blocks,avg_error_m\n"
            )

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"ArtifactScorer: match_radius={self._match_radius}m "
            f"gt_blocks={len(_GROUND_TRUTH)} log={self._csv_path}"
        )

    def _on_detections(self, msg: String):
        try:
            dets = json.loads(msg.data)
            if isinstance(dets, list):
                self._detections = dets
        except json.JSONDecodeError:
            pass

    def _tick(self):
        score = self._compute_score()
        self._last_score = score

        msg = String()
        msg.data = json.dumps(score)
        self._score_pub.publish(msg)

        self._publish_gt_markers(score)
        self._append_csv(score)

        if score["true_positives"] > 0 or score["false_positives"] > 0:
            self.get_logger().info(
                f"P={score['precision']:.2f} R={score['recall']:.2f} "
                f"F1={score['f1']:.2f} | TP={score['true_positives']} "
                f"FP={score['false_positives']} missed={score['missed']} "
                f"avg_err={score['avg_error_m']:.2f}m"
            )

    def _compute_score(self) -> dict:
        elapsed = time.monotonic() - self._start_time
        n_gt = len(_GROUND_TRUTH)
        dets = list(self._detections)

        # Greedy nearest-neighbor matching: each GT matched to closest det
        gt_matched: dict[str, dict | None] = {g["id"]: None for g in _GROUND_TRUTH}
        det_used: set[int] = set()
        errors: list[float] = []

        # Build all (gt_idx, det_idx, distance) pairs, sort by distance
        pairs = []
        for gi, gt in enumerate(_GROUND_TRUTH):
            for di, det in enumerate(dets):
                dx = det.get("x", 0.0) - gt["x"]
                dy = det.get("y", 0.0) - gt["y"]
                dist = math.hypot(dx, dy)
                if dist <= self._match_radius:
                    pairs.append((gi, di, dist))
        pairs.sort(key=lambda p: p[2])

        for gi, di, dist in pairs:
            gt_id = _GROUND_TRUTH[gi]["id"]
            if gt_matched[gt_id] is not None or di in det_used:
                continue
            gt_matched[gt_id] = {"det_idx": di, "error_m": round(dist, 3)}
            det_used.add(di)
            errors.append(dist)

        tp = len(errors)
        fp = len(dets) - tp
        missed = n_gt - tp
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, n_gt)
        f1 = 2 * precision * recall / max(1e-9, precision + recall)
        avg_err = sum(errors) / max(1, len(errors))

        matches = []
        for gt in _GROUND_TRUTH:
            m = gt_matched[gt["id"]]
            entry = {
                "gt_id": gt["id"],
                "gt_x": gt["x"],
                "gt_y": gt["y"],
                "room": gt["room"],
                "matched": m is not None,
            }
            if m is not None:
                det = dets[m["det_idx"]]
                entry["det_x"] = det.get("x")
                entry["det_y"] = det.get("y")
                entry["error_m"] = m["error_m"]
                entry["confidence"] = det.get("confidence")
                entry["source"] = det.get("source")
            matches.append(entry)

        return {
            "elapsed_s": round(elapsed, 1),
            "n_gt": n_gt,
            "n_detections": len(dets),
            "true_positives": tp,
            "false_positives": fp,
            "missed": missed,
            "precision": round(precision, 3),
            "recall": round(recall, 3),
            "f1": round(f1, 3),
            "avg_error_m": round(avg_err, 3),
            "match_radius_m": self._match_radius,
            "matches": matches,
        }

    def _publish_gt_markers(self, score: dict):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, m in enumerate(score.get("matches", [])):
            # GT position marker
            mk = Marker()
            mk.header.frame_id = self._marker_frame
            mk.header.stamp = stamp
            mk.ns = "gt_blocks"
            mk.id = i
            mk.type = Marker.CYLINDER
            mk.action = Marker.ADD
            mk.pose.position.x = m["gt_x"]
            mk.pose.position.y = m["gt_y"]
            mk.pose.position.z = 0.02
            mk.pose.orientation.w = 1.0
            mk.scale.x = self._match_radius * 2
            mk.scale.y = self._match_radius * 2
            mk.scale.z = 0.02
            if m["matched"]:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 0.8, 0.0, 0.25
            else:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.8, 0.0, 0.25
            ma.markers.append(mk)

            # Label
            lbl = Marker()
            lbl.header.frame_id = self._marker_frame
            lbl.header.stamp = stamp
            lbl.ns = "gt_labels"
            lbl.id = i
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = m["gt_x"]
            lbl.pose.position.y = m["gt_y"]
            lbl.pose.position.z = 0.35
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.1
            lbl.color.r = lbl.color.g = lbl.color.b = lbl.color.a = 1.0
            if m["matched"]:
                lbl.text = f"{m['gt_id']} HIT {m['error_m']:.2f}m"
            else:
                lbl.text = f"{m['gt_id']} MISS"
            ma.markers.append(lbl)

            # Line from GT to detection if matched
            if m["matched"]:
                line = Marker()
                line.header.frame_id = self._marker_frame
                line.header.stamp = stamp
                line.ns = "gt_error_lines"
                line.id = i
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.03
                line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.3, 0.3, 0.8
                from geometry_msgs.msg import Point
                line.points.append(Point(x=m["gt_x"], y=m["gt_y"], z=0.05))
                line.points.append(Point(x=m["det_x"], y=m["det_y"], z=0.05))
                ma.markers.append(line)

        self._marker_pub.publish(ma)

    def _append_csv(self, score: dict):
        matched_ids = [m["gt_id"] for m in score.get("matches", []) if m["matched"]]
        try:
            with open(self._csv_path, "a") as f:
                f.write(
                    f"{score['elapsed_s']},{score['n_detections']},"
                    f"{score['true_positives']},{score['false_positives']},"
                    f"{score['missed']},{score['precision']},{score['recall']},"
                    f"{score['f1']},{';'.join(matched_ids)},{score['avg_error_m']}\n"
                )
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArtifactScorerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._last_score:
            node.get_logger().info(
                f"FINAL: P={node._last_score['precision']:.3f} "
                f"R={node._last_score['recall']:.3f} "
                f"F1={node._last_score['f1']:.3f} "
                f"avg_err={node._last_score['avg_error_m']:.3f}m"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
