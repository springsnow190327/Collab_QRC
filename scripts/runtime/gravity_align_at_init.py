#!/usr/bin/env python3
"""gravity_align_at_init.py — one-shot map → camera_init alignment.

Replaces the hardcoded `map → camera_init` static transform that
slam.launch.py used to publish on real-robot. The hardcoded value
(roll=-0.0368, pitch=+0.2636) only worked when the robot was started
on FLAT ground — it baked in the Mid-360 mount tilt (≈+15° forward,
≈-2° left) measured under that condition. On any other terrain the
SLAM map ended up tilted by the difference between the actual startup
attitude and the calibrated mount tilt.

This node fixes that by *measuring* the body's actual gravity vector
during the IMU-init window and publishing the matching static TF.

How it works:
  1. Subscribe to /livox/imu (BestEffort, like the Livox driver).
  2. Buffer the first N samples (default 200 ≈ 1.0 s @ 200 Hz).
  3. Verify the robot was static during that window
     (accel std deviation below static_thresh_g, default 0.05 g).
  4. Compute (roll, pitch) from the mean linear_acceleration:
       pitch = atan2(-mean_acc.x, sqrt(mean_acc.y² + mean_acc.z²))
       roll  = atan2( mean_acc.y, mean_acc.z)
     Yaw is unobservable from gravity alone → set to 0.
  5. Publish a *latched* static TF map → camera_init with that
     orientation. Send it on /tf_static via StaticTransformBroadcaster
     (TRANSIENT_LOCAL durability, so late subscribers also receive it).
  6. Stay alive (StaticTransformBroadcaster needs the publisher kept).

If the IMU window is non-static (someone is holding the robot, or
walking gait already started), the node logs a WARN and falls back to
the old hardcoded mount-tilt values so SLAM still has a reasonable
initial alignment.

Why a separate node and not done inside Fast-LIO: Fast-LIO does its
own IMU-bias estimation but DOES NOT gravity-align its world frame —
see vendor/fast_lio/src/IMU_Processing.hpp:195, the rot-to-gravity line
is commented out. So Fast-LIO's `camera_init` is body-at-init in body's
own (tilted) frame. We need an external static to express that frame
in a gravity-aligned `map`.
"""
from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


class GravityAligner(Node):
    def __init__(self) -> None:
        super().__init__("gravity_align_at_init")

        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("samples_required", 200)
        self.declare_parameter("max_wait_sec", 5.0)
        # std-dev threshold above which "robot is moving" — bail out to fallback.
        self.declare_parameter("static_thresh_g", 0.05)
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "camera_init")
        # Hardcoded mount-only fallback (matches the old slam.launch.py value;
        # used if IMU samples are missing or non-static).
        self.declare_parameter("fallback_pitch", 0.263591)
        self.declare_parameter("fallback_roll", -0.036809)

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.n_required = int(self.get_parameter("samples_required").value)
        self.max_wait = float(self.get_parameter("max_wait_sec").value)
        self.static_thresh = float(self.get_parameter("static_thresh_g").value) * 9.80665
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.fb_roll = float(self.get_parameter("fallback_roll").value)
        self.fb_pitch = float(self.get_parameter("fallback_pitch").value)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.samples: list[tuple[float, float, float]] = []
        self.published = False

        # Livox driver publishes /livox/imu BestEffort → match it. (Reliable
        # would silently miss messages with this publisher.)
        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)
        self.start_t = self.get_clock().now()
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"gravity_align_at_init: collecting {self.n_required} static IMU "
            f"samples from {self.imu_topic} (≤{self.max_wait}s); will publish "
            f"static TF {self.parent_frame} → {self.child_frame}"
        )

    def _on_imu(self, msg: Imu) -> None:
        if len(self.samples) < self.n_required:
            self.samples.append(
                (msg.linear_acceleration.x,
                 msg.linear_acceleration.y,
                 msg.linear_acceleration.z)
            )

    def _tick(self) -> None:
        if self.published:
            return
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9
        if len(self.samples) >= self.n_required:
            self._publish_from_samples()
            self.published = True
            return
        if elapsed >= self.max_wait:
            self.get_logger().warn(
                f"only got {len(self.samples)}/{self.n_required} IMU samples "
                f"in {elapsed:.1f}s — falling back to hardcoded mount tilt "
                f"(roll={math.degrees(self.fb_roll):+.2f}°, "
                f"pitch={math.degrees(self.fb_pitch):+.2f}°)"
            )
            self._publish(self.fb_roll, self.fb_pitch, fallback=True)
            self.published = True

    def _publish_from_samples(self) -> None:
        n = len(self.samples)
        sx = sum(s[0] for s in self.samples) / n
        sy = sum(s[1] for s in self.samples) / n
        sz = sum(s[2] for s in self.samples) / n

        var = sum(
            (s[0] - sx) ** 2 + (s[1] - sy) ** 2 + (s[2] - sz) ** 2
            for s in self.samples
        ) / n
        std = math.sqrt(var)

        if std > self.static_thresh:
            self.get_logger().warn(
                f"IMU not static during calibration "
                f"(std={std:.3f} m/s² > {self.static_thresh:.3f}). "
                f"Result may be off — fall back to hardcoded mount tilt."
            )
            self._publish(self.fb_roll, self.fb_pitch, fallback=True)
            return

        norm = math.sqrt(sx * sx + sy * sy + sz * sz)
        if norm < 1.0:  # sanity: gravity is ~9.8, anything < 1 m/s² is junk
            self.get_logger().error(
                f"IMU mean acc norm ({norm:.3f}) is implausibly small. "
                f"IMU not delivering or wrong axis convention. Fallback."
            )
            self._publish(self.fb_roll, self.fb_pitch, fallback=True)
            return

        gx, gy, gz = sx / norm, sy / norm, sz / norm
        # mean_acc reports specific force at rest = -gravity_in_body, so
        # (gx,gy,gz) points UP in body frame. Body's tilt relative to a
        # gravity-aligned world is then:
        pitch = math.atan2(-gx, math.sqrt(gy * gy + gz * gz))
        roll = math.atan2(gy, gz)

        self.get_logger().info(
            f"IMU init OK ({n} samples, std={std:.3f} m/s²): "
            f"mean acc=({sx:+.3f}, {sy:+.3f}, {sz:+.3f}) m/s² "
            f"→ roll={math.degrees(roll):+.2f}°, pitch={math.degrees(pitch):+.2f}°"
        )
        self._publish(roll, pitch, fallback=False)

    def _publish(self, roll: float, pitch: float, fallback: bool) -> None:
        qx, qy, qz, qw = _quat_from_rpy(roll, pitch, 0.0)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.broadcaster.sendTransform(t)
        tag = "FALLBACK" if fallback else "MEASURED"
        self.get_logger().info(
            f"published {tag} static TF {self.parent_frame}→{self.child_frame}: "
            f"roll={math.degrees(roll):+.3f}° pitch={math.degrees(pitch):+.3f}°"
        )


def main() -> None:
    rclpy.init()
    node = GravityAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
