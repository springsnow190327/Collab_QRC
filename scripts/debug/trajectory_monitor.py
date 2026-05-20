#!/usr/bin/env python3
"""trajectory_monitor — track the robot's explored extent + correlate stuck events.

Subscribes to /<ns>/odom/nav and continuously tracks:
  - min/max x and y reached (the headline metric: does the trajectory span
    x = +35 → -35 ?),
  - cumulative path length,
  - current CFPA2 status + latest stuck verdict.

Prints a one-line progress report every --report-sec, and on shutdown (SIGINT/
SIGTERM) writes a JSON summary to --summary-file. Designed to be the validation
oracle for the "trajectory spans ±35 within 10%" goal and to run unattended
under explore_autorun.sh.

Usage:
  ./scripts/debug/trajectory_monitor.py --ns robot --report-sec 15 \
      --target-xmax 35 --target-xmin -35 --tol 0.10
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)

from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class TrajectoryMonitor(Node):
    def __init__(self, args):
        super().__init__("trajectory_monitor")
        self.ns = args.ns.strip("/")
        self.report_sec = float(args.report_sec)
        self.xmax_target = float(args.target_xmax)
        self.xmin_target = float(args.target_xmin)
        self.tol = float(args.tol)
        self.summary_file = args.summary_file or os.path.join(
            "/tmp/collab_qrc_logs", f"trajectory_{self.ns}.json")
        os.makedirs(os.path.dirname(self.summary_file), exist_ok=True)

        self.xmin = math.inf
        self.xmax = -math.inf
        self.ymin = math.inf
        self.ymax = -math.inf
        self.path_len = 0.0
        self.last_xy = None
        self.n_samples = 0
        self.t0 = time.monotonic()
        self.cfpa2_status = "?"
        self.last_verdict = "?"
        self.verdict_counts = {}
        self._last_report = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(
            Odometry, f"/{self.ns}/odom/nav", self._cb_odom, sensor_qos)
        self.create_subscription(
            String, f"/{self.ns}/exploration_status", self._cb_status, reliable)
        self.create_subscription(
            String, f"/{self.ns}/stuck_diagnosis", self._cb_diag, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"trajectory_monitor ns=/{self.ns} target x∈[{self.xmin_target},"
            f"{self.xmax_target}] tol={self.tol*100:.0f}% summary={self.summary_file}")

    def _cb_odom(self, m: Odometry):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        self.n_samples += 1
        self.xmin = min(self.xmin, x)
        self.xmax = max(self.xmax, x)
        self.ymin = min(self.ymin, y)
        self.ymax = max(self.ymax, y)
        if self.last_xy is not None:
            self.path_len += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

    def _cb_status(self, m: String):
        self.cfpa2_status = m.data

    def _cb_diag(self, m: String):
        try:
            d = json.loads(m.data)
            v = d.get("verdict", "?")
            self.last_verdict = v
            self.verdict_counts[v] = self.verdict_counts.get(v, 0) + 1
        except json.JSONDecodeError:
            pass

    def _span_progress(self):
        """Fraction of the target +x and -x reach achieved."""
        pos = self.xmax / self.xmax_target if self.xmax_target != 0 else 0.0
        neg = self.xmin / self.xmin_target if self.xmin_target != 0 else 0.0
        return max(0.0, pos), max(0.0, neg)

    def _goal_met(self):
        pos_ok = self.xmax >= self.xmax_target * (1 - self.tol)
        neg_ok = self.xmin <= self.xmin_target * (1 - self.tol)
        return pos_ok and neg_ok

    def _tick(self):
        now = time.monotonic()
        if now - self._last_report < self.report_sec:
            return
        self._last_report = now
        if self.n_samples == 0:
            self.get_logger().info("trajectory_monitor: no odom yet")
            return
        pos, neg = self._span_progress()
        el = now - self.t0
        self.get_logger().info(
            f"[{el:6.0f}s] x∈[{self.xmin:+.1f},{self.xmax:+.1f}] "
            f"y∈[{self.ymin:+.1f},{self.ymax:+.1f}] path={self.path_len:.1f}m "
            f"| +x {pos*100:.0f}% -x {neg*100:.0f}% of ±target "
            f"| cfpa2='{self.cfpa2_status}' verdicts={self.verdict_counts} "
            f"| GOAL_MET={self._goal_met()}")
        self._write_summary()

    def _write_summary(self):
        s = {
            "ns": self.ns,
            "elapsed_s": round(time.monotonic() - self.t0, 1),
            "x_min": round(self.xmin, 2) if self.n_samples else None,
            "x_max": round(self.xmax, 2) if self.n_samples else None,
            "y_min": round(self.ymin, 2) if self.n_samples else None,
            "y_max": round(self.ymax, 2) if self.n_samples else None,
            "path_len_m": round(self.path_len, 1),
            "n_samples": self.n_samples,
            "target_xmax": self.xmax_target,
            "target_xmin": self.xmin_target,
            "tol": self.tol,
            "goal_met": self._goal_met(),
            "cfpa2_status": self.cfpa2_status,
            "verdict_counts": self.verdict_counts,
            "t_wall": time.time(),
        }
        try:
            with open(self.summary_file, "w") as f:
                json.dump(s, f, indent=2)
        except OSError:
            pass

    def finalize(self):
        self._write_summary()
        print("\n" + "═" * 64, flush=True)
        print(f" TRAJECTORY SUMMARY [{self.ns}]", flush=True)
        print(f"   x range: [{self.xmin:+.2f}, {self.xmax:+.2f}]  "
              f"(target [{self.xmin_target}, {self.xmax_target}], tol {self.tol*100:.0f}%)",
              flush=True)
        print(f"   y range: [{self.ymin:+.2f}, {self.ymax:+.2f}]", flush=True)
        print(f"   path length: {self.path_len:.1f} m   samples: {self.n_samples}",
              flush=True)
        print(f"   verdicts: {self.verdict_counts}", flush=True)
        print(f"   GOAL MET: {self._goal_met()}", flush=True)
        print("═" * 64, flush=True)


def main(argv=None):
    user_argv, ros_argv = _split_ros_argv(sys.argv if argv is None else argv)
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="robot")
    ap.add_argument("--report-sec", type=float, default=15.0)
    ap.add_argument("--target-xmax", type=float, default=35.0)
    ap.add_argument("--target-xmin", type=float, default=-35.0)
    ap.add_argument("--tol", type=float, default=0.10)
    ap.add_argument("--summary-file", default=None)
    args = ap.parse_args(user_argv[1:] if len(user_argv) > 1 else [])

    rclpy.init(args=ros_argv)
    node = TrajectoryMonitor(args)

    def _on_sig(signum, frame):
        node.finalize()
        if rclpy.ok():
            rclpy.shutdown()
    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.finalize()
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
