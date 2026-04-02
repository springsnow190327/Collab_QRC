#!/usr/bin/env python3
"""Collision monitor — counts wall crash events from Gazebo contact sensor.

Subscribes to the bumper/contact topic published by the gazebo_ros_bumper
plugin on the robot body.  Each new contact (after a debounce period) is
counted as a crash event.  Publishes a running tally on /collision_count
and logs each event.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Int32


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__("collision_monitor")

        self.declare_parameter("contact_topic", "contact_states")
        self.declare_parameter("debounce_sec", 1.0)
        self.declare_parameter("min_force", 5.0)

        contact_topic = self.get_parameter("contact_topic").as_string()
        self.debounce_sec = self.get_parameter("debounce_sec").as_double()
        self.min_force = self.get_parameter("min_force").as_double()

        self.crash_count = 0
        self.last_crash_time = 0.0

        self.create_subscription(ContactsState, contact_topic, self._on_contact, 10)
        self.count_pub = self.create_publisher(Int32, "collision_count", 10)

        self.get_logger().info(
            f"Collision monitor started: topic={contact_topic} "
            f"debounce={self.debounce_sec}s min_force={self.min_force}N"
        )

    def _on_contact(self, msg: ContactsState):
        if not msg.states:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Check if any contact has sufficient force (filters ground contact noise)
        max_force = 0.0
        contact_body = ""
        for state in msg.states:
            for wrench in state.wrenches:
                f = (
                    wrench.force.x ** 2
                    + wrench.force.y ** 2
                    + wrench.force.z ** 2
                ) ** 0.5
                if f > max_force:
                    max_force = f
                    contact_body = (
                        state.collision2_name
                        if "base" in state.collision1_name
                        else state.collision1_name
                    )

        if max_force < self.min_force:
            return

        # Debounce
        if (now - self.last_crash_time) < self.debounce_sec:
            return

        self.crash_count += 1
        self.last_crash_time = now
        self.get_logger().warn(
            f"COLLISION #{self.crash_count}: force={max_force:.1f}N body={contact_body}"
        )

        count_msg = Int32()
        count_msg.data = self.crash_count
        self.count_pub.publish(count_msg)


def main():
    rclpy.init()
    rclpy.spin(CollisionMonitor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
