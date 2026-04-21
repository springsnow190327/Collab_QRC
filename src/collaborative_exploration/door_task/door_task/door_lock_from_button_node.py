#!/usr/bin/env python3
"""Analytical door lock driven by the pressure pad.

The MJCF exposes a POSITION actuator on ``door_barrier_slide`` — a
slide joint on an invisible-ish amber box geom that sits in the
doorway when extended (qpos=0) and retracts underground when pulled
down to qpos=-3. MuJoCo computes the restoring force at 500 Hz, so
the barrier is either a real geometric wall or entirely absent —
there is no "stiff spring" middle ground.

This node sets the barrier actuator's target position:

  - button NOT pressed (LOCKED):
      target = 0.0 m (barrier extended, fills the doorway)
    The barrier is a 0.08 × 1.0 × 2.0 m box in the doorway. Any push
    against the door is stopped geometrically by the barrier, not by
    damping on the door hinge. The door panel itself can still wobble
    a few degrees but can never pass the barrier.

  - button pressed (UNLOCKED):
      target = -3.0 m (barrier retracted to z ≈ -2, underground)
    The barrier is no longer in collision with anything in-scene,
    and the door swings under its own FD30 spring (4.5 Nm/rad,
    3.0 Nm·s/rad damping) — a pure fire-rated door with closer.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray


class DoorLockFromButtonNode(Node):
    def __init__(self):
        super().__init__("door_lock_from_button")

        self.declare_parameter("button_topic", "/door_task/button_pressed")
        # The ros2_control controller_manager lives under /mujoco_sim,
        # so the forward_command_controller publishes its commands topic
        # in that namespace too. Publishing to /door_assist_controller/
        # commands goes nowhere (the previous FSM-era config accidentally
        # worked because nothing depended on this write path being live).
        self.declare_parameter(
            "assist_command_topic",
            "/mujoco_sim/door_assist_controller/commands",
        )
        # Slide joint targets: barrier home is at qpos=0 (extended,
        # filling the doorway). Retracted target is +3.0 (barrier body
        # slides UP so the geom sits above the wall at z∈[3, 5]).
        # Upward retract avoids the ground-contact stall that would
        # happen with a negative target (the geom would be blocked
        # from moving down by the floor plane's contact).
        self.declare_parameter("barrier_locked_m", 0.0)
        self.declare_parameter("barrier_unlocked_m", 3.0)
        self.declare_parameter("publish_rate", 200.0)

        self._locked = float(self.get_parameter("barrier_locked_m").value)
        self._unlocked = float(self.get_parameter("barrier_unlocked_m").value)
        rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self._pressed = False
        # ros2_control's forward_command_controller subscribes to its
        # commands topic with BEST_EFFORT reliability. A default Python
        # publisher is RELIABLE, which causes DDS to silently drop every
        # message due to the QoS mismatch — this was the root cause of
        # the "barrier doesn't move" bug. Match the subscriber's QoS
        # explicitly.
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("assist_command_topic").value),
            cmd_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("button_topic").value),
            self._on_button,
            10,
        )
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"DoorLockFromButton: analytical barrier lock, "
            f"locked_m={self._locked:.2f}  unlocked_m={self._unlocked:.2f} "
            f"@ {rate:.0f} Hz"
        )

    def _on_button(self, msg: Bool):
        self._pressed = bool(msg.data)

    def _tick(self):
        target = self._unlocked if self._pressed else self._locked
        msg = Float64MultiArray()
        msg.data = [target]
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DoorLockFromButtonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
