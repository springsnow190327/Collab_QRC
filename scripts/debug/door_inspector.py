#!/usr/bin/env python3
"""Passive door-task inspector — reads topics only, fires no motors.

Subscribes to:
  /door_task/door_state    (Float64)  — door hinge angle
  /robot_a/odom/nav        (Odometry) — Robot A position
  /robot_b/odom/nav        (Odometry) — Robot B position
  /door_task/fsm_status    (String)   — FSM state

Prints a status line at ~2 Hz and a final PASS/FAIL verdict.
"""

import json
import math
import signal
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


DOOR_X = 4.0
ROOM_A_CENTROID = (2.0, 2.0)
ROOM_B_CENTROID = (6.0, 2.0)
SAME_ROOM_THRESH = 3.5  # both x < 3.5 (room A) or both x > 4.5 (room B)

# Max time to wait for task completion
MAX_WATCH_SEC = 180.0


class DoorInspector(Node):
    def __init__(self):
        super().__init__("door_inspector")

        self._door_angle = 0.0
        self._robot_a_pos = None  # (x, y)
        self._robot_b_pos = None
        self._fsm_state = ""
        self._fsm_status = ""
        self._start_time = time.monotonic()
        self._peak_door_angle = 0.0

        # History for final report
        self._history: list[dict] = []

        self.create_subscription(Float64, "/door_task/door_state", self._on_door, 10)
        self.create_subscription(Odometry, "/robot_a/odom/nav", self._on_a_odom, 10)
        self.create_subscription(Odometry, "/robot_b/odom/nav", self._on_b_odom, 10)
        self.create_subscription(String, "/door_task/fsm_status", self._on_fsm, 10)

        self._timer = self.create_timer(0.5, self._report)  # 2 Hz

        self.get_logger().info("=" * 60)
        self.get_logger().info("DOOR INSPECTOR — passive observer, no motor commands")
        self.get_logger().info("=" * 60)

    def _on_door(self, msg: Float64):
        self._door_angle = msg.data
        self._peak_door_angle = max(self._peak_door_angle, msg.data)

    def _on_a_odom(self, msg: Odometry):
        self._robot_a_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_b_odom(self, msg: Odometry):
        self._robot_b_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_fsm(self, msg: String):
        try:
            d = json.loads(msg.data)
            self._fsm_state = d.get("state", "")
            self._fsm_status = d.get("status", "")
        except json.JSONDecodeError:
            pass

    def _room_label(self, x: float) -> str:
        if x < SAME_ROOM_THRESH:
            return "Room_A"
        elif x > DOOR_X + 0.5:
            return "Room_B"
        else:
            return "DOOR_ZONE"

    def _same_room(self) -> bool:
        if self._robot_a_pos is None or self._robot_b_pos is None:
            return False
        ax = self._robot_a_pos[0]
        bx = self._robot_b_pos[0]
        return (ax < SAME_ROOM_THRESH and bx < SAME_ROOM_THRESH) or \
               (ax > DOOR_X + 0.5 and bx > DOOR_X + 0.5)

    def _report(self):
        elapsed = time.monotonic() - self._start_time

        a_str = "WAITING" if self._robot_a_pos is None else \
            f"({self._robot_a_pos[0]:5.2f},{self._robot_a_pos[1]:5.2f}) [{self._room_label(self._robot_a_pos[0])}]"
        b_str = "WAITING" if self._robot_b_pos is None else \
            f"({self._robot_b_pos[0]:5.2f},{self._robot_b_pos[1]:5.2f}) [{self._room_label(self._robot_b_pos[0])}]"

        same = self._same_room()
        same_str = "YES" if same else "no"

        line = (
            f"[{elapsed:6.1f}s] door={self._door_angle:5.3f}rad "
            f"peak={self._peak_door_angle:5.3f} | "
            f"A={a_str} B={b_str} | "
            f"same_room={same_str} | "
            f"fsm={self._fsm_state}({self._fsm_status})"
        )
        self.get_logger().info(line)

        # Record history
        self._history.append({
            "t": round(elapsed, 1),
            "door": round(self._door_angle, 3),
            "peak": round(self._peak_door_angle, 3),
            "a_x": round(self._robot_a_pos[0], 2) if self._robot_a_pos else None,
            "b_x": round(self._robot_b_pos[0], 2) if self._robot_b_pos else None,
            "same_room": same,
            "fsm": self._fsm_state,
        })

        # Check terminal conditions
        if self._fsm_status == "success":
            self._final_verdict("PASS", "FSM reported success")
        elif self._fsm_status == "failure" and self._fsm_state == "S_FAIL":
            # Only final fail after all replans
            pass  # coordinator may replan
        elif elapsed > MAX_WATCH_SEC:
            self._final_verdict("FAIL", f"timeout after {MAX_WATCH_SEC}s")

        # Also check if both robots are in same room (independent of FSM)
        if same and elapsed > 30.0:
            self._final_verdict("PASS", "both robots in same room")

    def _final_verdict(self, result: str, reason: str):
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  VERDICT: {result} — {reason}")
        self.get_logger().info(f"  Peak door angle: {self._peak_door_angle:.3f} rad ({math.degrees(self._peak_door_angle):.1f} deg)")
        if self._robot_a_pos:
            self.get_logger().info(f"  Robot A final: ({self._robot_a_pos[0]:.2f}, {self._robot_a_pos[1]:.2f}) [{self._room_label(self._robot_a_pos[0])}]")
        if self._robot_b_pos:
            self.get_logger().info(f"  Robot B final: ({self._robot_b_pos[0]:.2f}, {self._robot_b_pos[1]:.2f}) [{self._room_label(self._robot_b_pos[0])}]")
        self.get_logger().info(f"  Same room: {self._same_room()}")
        self.get_logger().info("=" * 60)

        # Print history summary (key moments)
        self.get_logger().info("Timeline:")
        prev_fsm = ""
        for h in self._history:
            if h["fsm"] != prev_fsm or h.get("same_room"):
                self.get_logger().info(
                    f"  t={h['t']:6.1f}s  door={h['door']:.3f}  "
                    f"A_x={h['a_x']}  B_x={h['b_x']}  "
                    f"fsm={h['fsm']}  same_room={h['same_room']}"
                )
                prev_fsm = h["fsm"]

        self.get_logger().info("")
        # Exit
        raise SystemExit(0 if result == "PASS" else 1)


def main():
    rclpy.init()
    node = DoorInspector()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(e.code)
    except KeyboardInterrupt:
        node.get_logger().info("Inspector interrupted")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
