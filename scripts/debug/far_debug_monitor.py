#!/usr/bin/env python3
"""Real-time FAR planner debug monitor.

Prints a one-line summary every second with color-coded warnings:
  pose, goal, waypoint, cmd_vel, FAR planning time, obstacle clearance.

Auto-detects and highlights:
  🔴 STUCK     — hasn't moved > 0.1m in 5s
  🟡 REVERSE   — cmd_vel.x < -0.02 (driving backward)
  🟡 OSCILLATE — waypoint flipped direction 3+ times in 5s
  🟡 WP_BEHIND — waypoint is > 90° behind robot heading
  🔴 CONTACT   — wall contact detected via /mujoco/contacts

Run alongside the sim:
    python3 scripts/far_debug_monitor.py
"""
from __future__ import annotations
import math, time, collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

RED = "\033[91m"
YEL = "\033[93m"
GRN = "\033[92m"
RST = "\033[0m"
DIM = "\033[2m"

def _yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def _has_active_target(gx, gy, wpx, wpy):
    return (gx is not None and gy is not None) or (wpx is not None and wpy is not None)

def _target_satisfied(*, px, py, gx, gy, wpx, wpy, radius=0.35):
    targets = []
    if gx is not None and gy is not None:
        targets.append((gx, gy))
    if wpx is not None and wpy is not None:
        targets.append((wpx, wpy))
    if not targets:
        return False
    return any(math.hypot(px - tx, py - ty) <= radius for tx, ty in targets)

class FarDebugMonitor(Node):
    def __init__(self):
        super().__init__("far_debug_monitor")
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Odometry, "/robot/odom/nav", self._odom, 10)
        self.create_subscription(PointStamped, "/robot/way_point", self._wp, 10)
        self.create_subscription(PointStamped, "/robot/way_point_coord", self._goal, 10)
        # cmd_vel routing varies by launch / robot platform:
        #   - default_nav / astar   → TwistStamped on /robot/cmd_vel_stamped
        #   - Nav2 MPPI Go2W single → plain Twist on /robot/cmd_vel
        #     (go2w_hybrid_cmd_router consumes it to drive wheels + legs)
        #   - Nav2 MPPI Go2 single  → plain Twist on /robot/cmd_vel_legged
        #     (Nav2's cmd_vel is remapped → cmd_vel_legged so CHAMP receives it
        #     directly; /robot/cmd_vel ends up with 0 publishers and the
        #     monitor used to print +0.000 forever, giving the false impression
        #     the robot wasn't moving)
        # Subscribe to ALL three; whichever has traffic populates the readout.
        self.create_subscription(Twist, "/robot/cmd_vel", self._cmd, 10)
        self.create_subscription(Twist, "/robot/cmd_vel_legged", self._cmd, 10)
        self.create_subscription(TwistStamped, "/robot/cmd_vel_stamped", self._cmd_s, 10)
        self.create_subscription(String, "/mujoco/contacts", self._contacts, qos_be)

        self.create_timer(1.0, self._tick)

        self._px = self._py = self._yaw = 0.0
        self._vx = self._wz = 0.0
        self._gx = self._gy = None
        self._wpx = self._wpy = None
        self._contact_count = 0
        self._wall_contact_this_tick = 0
        self._t0 = time.monotonic()

        # Stuck detection
        self._last_move_t = time.monotonic()
        self._last_move_xy = (0.0, 0.0)

        # Oscillation detection
        self._wp_angles = collections.deque(maxlen=10)
        self._wp_flips = 0

        print(f"{GRN}far_debug_monitor started{RST}")
        print(f"{'t':>6} {'pose':>14} {'goal':>14} {'wp':>14} "
              f"{'vx':>6} {'wz':>6} {'flags'}")
        print("-" * 85)

    def _odom(self, msg):
        self._px = msg.pose.pose.position.x
        self._py = msg.pose.pose.position.y
        self._yaw = _yaw(msg.pose.pose.orientation)

    def _wp(self, msg):
        new_x, new_y = msg.point.x, msg.point.y
        if self._wpx is not None:
            old_ang = math.atan2(self._wpy - self._py, self._wpx - self._px)
            new_ang = math.atan2(new_y - self._py, new_x - self._px)
            diff = abs(math.atan2(math.sin(new_ang-old_ang), math.cos(new_ang-old_ang)))
            if diff > 1.5:
                self._wp_flips += 1
            self._wp_angles.append((time.monotonic(), diff))
        self._wpx, self._wpy = new_x, new_y

    def _goal(self, msg):
        self._gx, self._gy = msg.point.x, msg.point.y
        self._last_move_t = time.monotonic()
        self._last_move_xy = (self._px, self._py)

    def _cmd(self, msg):
        self._vx = msg.linear.x
        self._wz = msg.angular.z

    def _cmd_s(self, msg):
        self._vx = msg.twist.linear.x
        self._wz = msg.twist.angular.z

    def _contacts(self, msg):
        WALL = ("wall_", "divider_")
        ALLOWED = {"ground", "green_marker_1", "green_marker_2",
                   "green_marker_3", "box_obstacle_1", "box_obstacle_2"}
        for ln in msg.data.split("\n"):
            if not ln: continue
            parts = ln.split("|")
            if len(parts) < 3: continue
            n1, n2 = parts[0], parts[1]
            w1 = n1.startswith(WALL)
            w2 = n2.startswith(WALL)
            if not (w1 or w2): continue
            other = n2 if w1 else n1
            if other in ALLOWED or other.startswith(WALL): continue
            self._wall_contact_this_tick += 1
            self._contact_count += 1

    def _tick(self):
        now = time.monotonic()
        t = now - self._t0
        flags = []
        active_target = _has_active_target(self._gx, self._gy, self._wpx, self._wpy)
        target_satisfied = _target_satisfied(
            px=self._px,
            py=self._py,
            gx=self._gx,
            gy=self._gy,
            wpx=self._wpx,
            wpy=self._wpy,
        )
        if not active_target or target_satisfied:
            self._last_move_t = now
            self._last_move_xy = (self._px, self._py)

        # Stuck?
        d_from_last = math.hypot(self._px - self._last_move_xy[0],
                                  self._py - self._last_move_xy[1])
        if d_from_last > 0.1:
            self._last_move_t = now
            self._last_move_xy = (self._px, self._py)
        stuck_sec = now - self._last_move_t
        if active_target and not target_satisfied and stuck_sec > 5.0:
            flags.append(f"{RED}STUCK({stuck_sec:.0f}s){RST}")

        # Reverse?
        if self._vx < -0.02:
            flags.append(f"{YEL}REVERSE(vx={self._vx:.2f}){RST}")

        # Waypoint behind?
        if self._wpx is not None:
            wp_ang = math.atan2(self._wpy - self._py, self._wpx - self._px)
            heading_err = abs(math.atan2(math.sin(wp_ang - self._yaw),
                                         math.cos(wp_ang - self._yaw)))
            if heading_err > 1.57:
                flags.append(f"{YEL}WP_BEHIND({math.degrees(heading_err):.0f}°){RST}")

        # Oscillation?
        recent_flips = sum(1 for tt, _ in self._wp_angles if now - tt < 5.0
                           and _ > 1.5)
        if recent_flips >= 3:
            flags.append(f"{YEL}OSCILLATE({recent_flips}flips/5s){RST}")

        # Contact?
        if self._wall_contact_this_tick > 0:
            flags.append(f"{RED}CONTACT({self._wall_contact_this_tick}){RST}")
            self._wall_contact_this_tick = 0

        # Format
        pose_str = f"({self._px:+.1f},{self._py:+.1f})"
        goal_str = f"({self._gx:+.1f},{self._gy:+.1f})" if self._gx is not None else "      —"
        wp_str = f"({self._wpx:+.1f},{self._wpy:+.1f})" if self._wpx is not None else "      —"
        flag_str = " ".join(flags) if flags else f"{DIM}ok{RST}"

        print(f"{t:6.1f} {pose_str:>14} {goal_str:>14} {wp_str:>14} "
              f"{self._vx:+.3f} {self._wz:+.3f} {flag_str}")

def main():
    rclpy.init()
    node = FarDebugMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
