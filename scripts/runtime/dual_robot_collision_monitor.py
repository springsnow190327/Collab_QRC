#!/usr/bin/env python3
"""Monitor /mujoco/contacts for inter-robot collisions between robot A and robot B.

The dual MJCF scene (demo3_dual.xml) names robot-A geoms bare (FL_wheel_collision,
base_collision, head_upper_collision, ...) and robot-B geoms with a "b_" prefix
(b_FL_wheel_collision, b_base_collision, ...). The MuJoCo plugin publishes raw
contact geom pairs as newline-delimited "geom1|geom2|x,y,z" records on the
std_msgs/String topic /mujoco/contacts.

This node:
- Classifies each contact pair as (A-A world, B-B world, A-B inter-robot, other).
- Logs each distinct A↔B geom pair with a timestamp + world position.
- Prints periodic summary.
- Emits a JSON report on shutdown at the path given by --output or the
  dual_robot_collision_report.json default.
"""
from __future__ import annotations
import argparse
import json
import os
import signal
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# Robot part name roots that appear in MJCF (shared by A and B; B has "b_" prefix).
# See src/go2w/go2_gazebo_sim/mujoco/demo3_dual.xml for the canonical list.
ROBOT_PART_PREFIXES = (
    "base_collision",
    "head_upper_collision",
    "head_lower_collision",
    "FL_hip_collision", "FL_thigh_collision",
    "FL_calf_upper_collision", "FL_calf_lower_collision", "FL_wheel_collision",
    "FR_hip_collision", "FR_thigh_collision",
    "FR_calf_upper_collision", "FR_calf_lower_collision", "FR_wheel_collision",
    "RL_hip_collision", "RL_thigh_collision",
    "RL_calf_upper_collision", "RL_calf_lower_collision", "RL_wheel_collision",
    "RR_hip_collision", "RR_thigh_collision",
    "RR_calf_upper_collision", "RR_calf_lower_collision", "RR_wheel_collision",
)


def classify(geom_name: str) -> str:
    """Return 'A', 'B', or 'world' for a geom name."""
    if not geom_name:
        return "world"
    if geom_name.startswith("b_"):
        rest = geom_name[2:]
        if any(rest == p or rest.startswith(p) for p in ROBOT_PART_PREFIXES):
            return "B"
        return "world"
    if any(geom_name == p or geom_name.startswith(p) for p in ROBOT_PART_PREFIXES):
        return "A"
    return "world"


class DualRobotCollisionMonitor(Node):
    def __init__(self, output_path: str | None, verbose: bool):
        super().__init__("dual_robot_collision_monitor")
        self.output_path = output_path
        self.verbose = verbose
        self.t0 = time.time()

        # {(part_A, part_B) → {"hits": int, "first_sec": float, "first_pos": [x,y,z]}}
        self.inter_robot_pairs: dict[tuple[str, str], dict] = {}
        self.contact_msgs = 0
        self.inter_robot_hits_total = 0
        self.last_log_t = 0.0

        # The mujoco plugin publishes /mujoco/contacts with BEST_EFFORT
        # reliability — matching QoS is required or DDS silently drops
        # every message (see CLAUDE.md "QoS mismatches that fail silently").
        self.create_subscription(
            String, "/mujoco/contacts", self._on_contacts,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=50,
            ),
        )
        self.create_timer(10.0, self._periodic_summary)
        self.get_logger().info(
            "dual_robot_collision_monitor started; output="
            f"{self.output_path or '<stdout only>'}"
        )

    def _on_contacts(self, msg: String):
        self.contact_msgs += 1
        now = time.time() - self.t0
        for raw in msg.data.split("\n"):
            line = raw.strip()
            if not line or "|" not in line:
                continue
            parts = line.split("|")
            if len(parts) < 3:
                continue
            g1, g2, pos_str = parts[0], parts[1], parts[2]
            c1, c2 = classify(g1), classify(g2)
            if (c1 == "A" and c2 == "B") or (c1 == "B" and c2 == "A"):
                # Normalise the pair so (A, B) order is canonical
                if c1 == "A":
                    key = (g1, g2)
                else:
                    key = (g2, g1)
                self.inter_robot_hits_total += 1
                if key not in self.inter_robot_pairs:
                    try:
                        xyz = [float(x) for x in pos_str.split(",")]
                    except ValueError:
                        xyz = [float("nan")] * 3
                    self.inter_robot_pairs[key] = {
                        "hits": 1,
                        "first_sec": now,
                        "first_pos": xyz,
                    }
                    if self.verbose:
                        self.get_logger().warn(
                            f"INTER-ROBOT CONTACT @ t={now:.1f}s: "
                            f"A.{key[0]} × B.{key[1]} @ {xyz}"
                        )
                else:
                    self.inter_robot_pairs[key]["hits"] += 1

    def _periodic_summary(self):
        now = time.time() - self.t0
        if now - self.last_log_t < 9.5:
            return
        self.last_log_t = now
        self.get_logger().info(
            f"[t={now:5.0f}s] contact_msgs={self.contact_msgs:>5d}  "
            f"inter-robot hits={self.inter_robot_hits_total:>4d}  "
            f"unique pairs={len(self.inter_robot_pairs):>3d}"
        )

    def write_report(self):
        report = {
            "elapsed_sec": time.time() - self.t0,
            "contact_msgs_received": self.contact_msgs,
            "inter_robot_hits_total": self.inter_robot_hits_total,
            "unique_inter_robot_pairs": len(self.inter_robot_pairs),
            "pairs": [
                {
                    "robot_a_geom": k[0],
                    "robot_b_geom": k[1],
                    "hits": v["hits"],
                    "first_sec": round(v["first_sec"], 3),
                    "first_pos": v["first_pos"],
                }
                for k, v in sorted(
                    self.inter_robot_pairs.items(),
                    key=lambda kv: -kv[1]["hits"],
                )
            ],
        }
        if self.output_path:
            Path(self.output_path).parent.mkdir(parents=True, exist_ok=True)
            Path(self.output_path).write_text(json.dumps(report, indent=2))
            self.get_logger().info(f"wrote report to {self.output_path}")
        else:
            print("\n=== DUAL-ROBOT COLLISION REPORT ===")
            print(json.dumps(report, indent=2))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=os.environ.get(
            "DUAL_COLLISION_OUTPUT",
            "/tmp/dual_robot_collision_report.json",
        ),
        help="Path to write final JSON report (default: $DUAL_COLLISION_OUTPUT "
        "or /tmp/dual_robot_collision_report.json).",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Log each newly-seen inter-robot geom pair (default: summary only).",
    )
    # Strip ROS args so parser doesn't choke.
    if argv is None:
        argv = sys.argv[1:]
    ros_args = []
    if "--ros-args" in argv:
        idx = argv.index("--ros-args")
        argv, ros_args = argv[:idx], argv[idx:]
    args = parser.parse_args(argv)

    rclpy.init(args=ros_args or None)
    node = DualRobotCollisionMonitor(args.output, args.verbose)

    def _shutdown(*_):
        node.write_report()
        rclpy.shutdown()
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.write_report()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
