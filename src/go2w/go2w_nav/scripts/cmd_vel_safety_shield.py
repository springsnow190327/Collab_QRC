#!/usr/bin/env python3
"""cmd_vel_safety_shield — execution-time clearance check.

Sits between pathFollower and the actuator (twist_bridge / hybrid
router):

    pathFollower ──/cmd_vel_stamped──> /cmd_vel_stamped_raw
    /cmd_vel_stamped_raw ──┐
    /map ──────────────────┤  cmd_vel_safety_shield
    /odom/nav ─────────────┘
                            └──> /cmd_vel_stamped ──> twist_bridge

For every command received, we measure the robot's footprint against the
current /map. The default stamped mode kills unsafe angular commands but
keeps translation so the robot can drive out of a narrow zone. Optional
linear-stop mode adds a predictive footprint check for legged Nav2
controllers: if the combined command clips, split the command by DOF and
preserve any safe escape component before resorting to a full stop. When
enabled, a bounded reverse-escape reflex may publish a small opposite
body-x command if the requested motion clips but reversing is clear.

This complements CFPA2's pivot-lock (which only blocks goal CHANGES).
The shield enforces the same "no pivot in tight clearance" invariant at
cmd_vel level, so even if the held goal demands rotation, the rotation
is suppressed until the robot has cleared the corridor.

Why we avoid hard-stop first: stopping both linear and angular can leave
the robot frozen in the narrow zone. Preserving safe translation while
killing an unsafe yaw command is usually the practical escape motion.
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
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String


class CmdVelSafetyShield(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_safety_shield")

        self.declare_parameter("clearance_radius_m", 0.50)
        self.declare_parameter("occ_threshold", 50)
        self.declare_parameter("angular_kill_threshold_rad_s", 0.10)
        self.declare_parameter("linear_stop_enabled", False)
        self.declare_parameter("linear_kill_threshold_mps", 0.03)
        self.declare_parameter("reverse_escape_enabled", False)
        self.declare_parameter("reverse_escape_speed_mps", 0.10)
        self.declare_parameter("footprint_length_m", 0.65)
        self.declare_parameter("footprint_width_m", 0.45)
        self.declare_parameter("predict_horizon_sec", 0.4)
        self.declare_parameter("message_type", "stamped")
        self.declare_parameter("input_topic", "cmd_vel_stamped_raw")
        self.declare_parameter("output_topic", "cmd_vel_stamped")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("status_topic", "cmd_vel_shield_status")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("use_tf", True)

        self.radius_m = float(self.get_parameter("clearance_radius_m").value)
        self.occ_threshold = int(self.get_parameter("occ_threshold").value)
        self.angular_kill_thr = float(
            self.get_parameter("angular_kill_threshold_rad_s").value
        )
        self.linear_stop_enabled = bool(
            self.get_parameter("linear_stop_enabled").value
        )
        self.linear_kill_thr = float(
            self.get_parameter("linear_kill_threshold_mps").value
        )
        self.reverse_escape_enabled = bool(
            self.get_parameter("reverse_escape_enabled").value
        )
        self.reverse_escape_speed = abs(float(
            self.get_parameter("reverse_escape_speed_mps").value
        ))
        self.fp_length = float(self.get_parameter("footprint_length_m").value)
        self.fp_width = float(self.get_parameter("footprint_width_m").value)
        self.predict_horizon_sec = float(
            self.get_parameter("predict_horizon_sec").value
        )
        self.message_type = str(self.get_parameter("message_type").value).strip().lower()
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.use_tf = bool(self.get_parameter("use_tf").value)

        self._latest_map: Optional[OccupancyGrid] = None
        self._latest_pose: Optional[tuple[float, float]] = None  # (x, y) in map

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        if self.message_type == "twist":
            self.create_subscription(Twist, in_topic, self._on_cmd_twist, 5)
            self.cmd_pub = self.create_publisher(Twist, out_topic, 5)
        else:
            self.message_type = "stamped"
            self.create_subscription(TwistStamped, in_topic, self._on_cmd_stamped, 5)
            self.cmd_pub = self.create_publisher(TwistStamped, out_topic, 5)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.status_pub = self.create_publisher(String, status_topic, 5)

        if self.use_tf:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None

        self.get_logger().info(
            f"cmd_vel_safety_shield armed. clearance_radius={self.radius_m:.2f}m "
            f"angular_kill>{self.angular_kill_thr:.2f}rad/s | "
            f"linear_stop={self.linear_stop_enabled} "
            f"reverse_escape={self.reverse_escape_enabled} "
            f"type={self.message_type} | "
            f"in={in_topic} out={out_topic} map={map_topic} odom={odom_topic}"
        )

    # ── Callbacks ──────────────────────────────────────────────────────
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_odom(self, msg: Odometry) -> None:
        # odom/nav is in map frame for our setup (slam_odom_relay aligns
        # SLAM origin to map origin at startup). Use as fallback if TF
        # is not available.
        self._latest_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _on_cmd_stamped(self, msg: TwistStamped) -> None:
        vx, vy, wz, action = self._filter_command(
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.angular.z),
        )
        out = TwistStamped()
        out.header = msg.header
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.linear.z = 0.0
        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = wz
        self.cmd_pub.publish(out)
        self._publish_status(action, float(msg.twist.linear.x), float(msg.twist.angular.z))

    def _on_cmd_twist(self, msg: Twist) -> None:
        vx, vy, wz, action = self._filter_command(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
        )
        out = Twist()
        out.linear.x = vx
        out.linear.y = vy
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = wz
        self.cmd_pub.publish(out)
        self._publish_status(action, float(msg.linear.x), float(msg.angular.z))

    def _filter_command(self, vx: float, vy: float, wz: float) -> tuple[float, float, float, str]:
        if self._latest_map is None:
            return vx, vy, wz, "passthrough_no_map"

        # Need full pose (xy + yaw) for predictive check.
        pose = self._lookup_pose_xyyaw()
        if pose is None:
            return vx, vy, wz, "passthrough_no_pose"
        rx, ry, ryaw = pose

        has_linear = math.hypot(vx, vy) > self.linear_kill_thr
        has_angular = abs(wz) > self.angular_kill_thr
        if self.linear_stop_enabled and (has_linear or has_angular):
            motion_clips = self._motion_footprint_clips(
                rx,
                ry,
                ryaw,
                vx=vx,
                vy=vy,
                wz=wz,
                horizon_sec=self.predict_horizon_sec,
            )
            if not motion_clips:
                return vx, vy, wz, "passthrough_motion_safe"

            translation_clips = has_linear and self._motion_footprint_clips(
                rx,
                ry,
                ryaw,
                vx=vx,
                vy=vy,
                wz=0.0,
                horizon_sec=self.predict_horizon_sec,
            )
            rotation_clips = has_angular and self._motion_footprint_clips(
                rx,
                ry,
                ryaw,
                vx=0.0,
                vy=0.0,
                wz=wz,
                horizon_sec=self.predict_horizon_sec,
            )

            if has_linear and not translation_clips:
                self.get_logger().warn(
                    f"omega-killed @ pose ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°): "
                    f"combined motion would clip footprint, translation "
                    f"v=({vx:.2f},{vy:.2f}) is clear, requested ω={wz:.2f} → 0.00"
                )
                return vx, vy, 0.0, "omega_killed_motion_clip"

            if has_angular and not rotation_clips:
                self.get_logger().warn(
                    f"linear-stopped @ pose ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°): "
                    f"combined motion would clip footprint, rotation "
                    f"ω={wz:.2f} is clear, requested v=({vx:.2f},{vy:.2f}) → 0.00"
                )
                return 0.0, 0.0, wz, "linear_stopped_rotation_safe"

            if self.reverse_escape_enabled and abs(vx) > self.linear_kill_thr:
                escape_vx = -math.copysign(
                    min(abs(vx), self.reverse_escape_speed),
                    vx,
                )
                escape_clips = self._motion_footprint_clips(
                    rx,
                    ry,
                    ryaw,
                    vx=escape_vx,
                    vy=0.0,
                    wz=0.0,
                    horizon_sec=self.predict_horizon_sec,
                )
                if not escape_clips:
                    self.get_logger().warn(
                        f"reverse-escape @ pose ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°): "
                        f"requested motion clips footprint; "
                        f"publishing v=({escape_vx:.2f},0.00) ω=0.00"
                    )
                    return escape_vx, 0.0, 0.0, "reverse_escape"

            self.get_logger().warn(
                f"motion-stopped @ pose ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°): "
                f"predicted footprint clip, requested v=({vx:.2f},{vy:.2f}) "
                f"ω={wz:.2f} → stop"
            )
            return 0.0, 0.0, 0.0, "motion_stopped"

        # Small ω is always safe — pass through.
        if abs(wz) <= self.angular_kill_thr:
            return vx, vy, wz, "passthrough_small_omega"

        # PREDICTIVE CHECK: simulate the requested rotation forward by
        # `predict_horizon_sec` and test the rotated footprint. If the
        # rotated body envelope is STILL clear, the rotation is safe
        # even if current pose is hugging a wall (means we're rotating
        # AWAY from it). Only kill ω when the rotation would sweep the
        # body INTO a wall.
        future_yaw = ryaw + wz * self.predict_horizon_sec
        # Sweep the rotation in N samples to catch mid-rotation clips
        sweep_n = 4
        rotation_safe = True
        for k in range(sweep_n + 1):
            t = k / sweep_n
            yaw_k = ryaw + wz * self.predict_horizon_sec * t
            if self._oriented_footprint_clips(rx, ry, yaw_k):
                rotation_safe = False
                break

        if rotation_safe:
            # Rotation OK — pass full cmd through.
            return vx, vy, wz, "passthrough_rotation_safe"

        # Rotation would clip wall — kill ω, keep linear so we can
        # back/forward out of corridor.
        self.get_logger().warn(
            f"omega-killed @ pose ({rx:.2f},{ry:.2f},{math.degrees(ryaw):.0f}°): "
            f"rotation by {math.degrees(wz*self.predict_horizon_sec):.0f}° "
            f"would clip footprint, requested ω={wz:.2f} → 0.00"
        )
        return vx, vy, 0.0, "omega_killed"

    # ── Geometry helpers ───────────────────────────────────────────────
    def _lookup_pose_xy(self) -> Optional[tuple[float, float]]:
        if self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time(),
                    timeout=Duration(seconds=0.05),
                )
                return (
                    float(tf.transform.translation.x),
                    float(tf.transform.translation.y),
                )
            except Exception:
                pass
        return self._latest_pose

    def _lookup_pose_xyyaw(self) -> Optional[tuple[float, float, float]]:
        """Pose in map frame: (x, y, yaw). Prefers TF, falls back to odom."""
        if self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time(),
                    timeout=Duration(seconds=0.05),
                )
                qx = tf.transform.rotation.x
                qy = tf.transform.rotation.y
                qz = tf.transform.rotation.z
                qw = tf.transform.rotation.w
                # Yaw from quaternion (assuming roll/pitch ≈ 0 — true for
                # ground robot)
                siny = 2.0 * (qw * qz + qx * qy)
                cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny, cosy)
                return (
                    float(tf.transform.translation.x),
                    float(tf.transform.translation.y),
                    float(yaw),
                )
            except Exception:
                pass
        if self._latest_pose is None:
            return None
        # Odom fallback: no yaw stored, use 0 (rotation safety check
        # will be slightly off but still useful).
        return (self._latest_pose[0], self._latest_pose[1], 0.0)

    def _oriented_footprint_clips(self, cx: float, cy: float, yaw: float) -> bool:
        """Return True if the oriented rectangular footprint at (cx,cy,yaw)
        contains any occupied cell in /map. Footprint is centered on
        base_link, body x along yaw direction, body y perpendicular."""
        m = self._latest_map
        if m is None:
            return False
        info = m.info
        res = info.resolution
        if res <= 0.0:
            return False
        ox = info.origin.position.x
        oy = info.origin.position.y
        w = int(info.width)
        h = int(info.height)
        # Sample the rectangle's interior on a grid (stride = res).
        half_l = self.fp_length * 0.5
        half_w = self.fp_width * 0.5
        cy_yaw = math.cos(yaw)
        sy_yaw = math.sin(yaw)
        # Number of samples per side: enough to hit each map cell at
        # least once (stride <= res).
        n_l = max(2, int(math.ceil(self.fp_length / res)) + 1)
        n_w = max(2, int(math.ceil(self.fp_width / res)) + 1)
        data = m.data
        for i in range(n_l):
            t_l = -half_l + (2 * half_l) * (i / max(1, n_l - 1))
            for j in range(n_w):
                t_w = -half_w + (2 * half_w) * (j / max(1, n_w - 1))
                # Transform from body frame to map frame
                px = cx + cy_yaw * t_l - sy_yaw * t_w
                py = cy + sy_yaw * t_l + cy_yaw * t_w
                gx = int((px - ox) / res)
                gy = int((py - oy) / res)
                if gx < 0 or gy < 0 or gx >= w or gy >= h:
                    continue
                v = data[gy * w + gx]
                if v >= self.occ_threshold:
                    return True
        return False

    def _motion_footprint_clips(
        self,
        cx: float,
        cy: float,
        yaw: float,
        *,
        vx: float,
        vy: float,
        wz: float,
        horizon_sec: float,
    ) -> bool:
        """Predict whether the commanded body-frame twist clips the map.

        Samples future poses only. If the current pose is already close to
        an obstacle, backing away can still be allowed when its future
        footprint is clear.
        """
        horizon = max(0.0, float(horizon_sec))
        if horizon <= 0.0:
            return False
        steps = max(2, int(math.ceil(horizon / 0.1)))
        dt = horizon / steps
        x = float(cx)
        y = float(cy)
        th = float(yaw)
        for _ in range(steps):
            c = math.cos(th)
            s = math.sin(th)
            x += (c * vx - s * vy) * dt
            y += (s * vx + c * vy) * dt
            th += wz * dt
            if self._oriented_footprint_clips(x, y, th):
                return True
        return False

    def _clearance_blocked(self, rxy: tuple[float, float]) -> bool:
        msg = self._latest_map
        if msg is None:
            return False
        info = msg.info
        res = info.resolution
        if res <= 0.0:
            return False
        rx, ry = rxy
        gx = int((rx - info.origin.position.x) / res)
        gy = int((ry - info.origin.position.y) / res)
        w, h = int(info.width), int(info.height)
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            # Outside map — treat as safe (don't kill ω); robot can still
            # rotate to face goals into known regions. If you'd prefer
            # belt-and-braces, return True here instead.
            return False
        radius_cells = max(1, int(math.ceil(self.radius_m / res)))
        radius_sq = radius_cells * radius_cells
        data = msg.data
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

    def _publish_status(self, action: str, v_in: float, w_in: float) -> None:
        s = String()
        s.data = (
            '{"schema":"cmd_vel_shield/v1","action":"' + action + '"'
            f',"v_in":{v_in:.3f}'
            f',"w_in":{w_in:.3f}'
            "}"
        )
        self.status_pub.publish(s)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = CmdVelSafetyShield()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
