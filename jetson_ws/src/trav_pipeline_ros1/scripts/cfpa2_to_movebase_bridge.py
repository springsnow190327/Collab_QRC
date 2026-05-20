#!/usr/bin/env python3
"""cfpa2_to_movebase_bridge — translate CFPA2 way_point into a move_base goal.

ROS 1 (Noetic, rospy) equivalent of scripts/runtime/cfpa2_to_nav2_bridge.py,
for onboard deployment on the real Go2 (Jetson Orin NX, Point-LIO + move_base).

CFPA2's C++ coordinator publishes /<ns>/way_point_coord as PointStamped (an XY
target with no orientation — see roscpp_goal_publisher.hpp + the coordinator's
`goal_topic_suffix: /way_point_coord` default). move_base subscribes to
/<ns>/move_base_simple/goal as PoseStamped (full pose with orientation). This
bridge:

  - subscribes /<ns>/way_point_coord (PointStamped)
  - synthesizes orientation = atan2(goal - robot_pose) so the planner has a
    sensible terminal heading
  - publishes /<ns>/move_base_simple/goal (PoseStamped) only when the goal
    *changed* — re-publishing identical goals at 2 Hz would force move_base to
    abort + replan every tick
  - converts move_base goal ABORTED/REJECTED terminal status into
    /<ns>/nav_status messages (same nav_status/v1 schema as the ROS 2 bridge)
    so CFPA2 can blacklist unreachable frontiers quickly

This is the ROS 1 / move_base counterpart to the Nav2 path. move_base is
SIMPLER than Nav2: there is no bt_navigator action wrapper or BehaviorTreeLog —
move_base directly publishes /<ns>/move_base/status (actionlib_msgs/
GoalStatusArray), and a terminal ABORTED (4) / REJECTED (5) on the active goal
IS the "Nav2 gave up" signal. We translate that into a 3-emit fast-blacklist
burst (CFPA2's fast-BL needs 3 consecutive `unreachable` for the same goal_key).

Run alongside:
  - Point-LIO (publishes /<ns>/Odometry)
  - cfpa2_single_robot_node_cpp (with explore enabled)
  - move_base (the ROS 1 nav stack)
"""

import json
import math

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _ns_topic(ns, topic):
    """Prefix /<ns>/ unless the topic is already absolute (starts with '/')."""
    topic = str(topic)
    if topic.startswith("/"):
        return topic
    return "/{}/{}".format(ns, topic) if ns else "/{}".format(topic)


class Cfpa2ToMovebaseBridge(object):
    def __init__(self):
        # ~private-namespace params with sensible onboard defaults.
        ns = str(rospy.get_param("~namespace", "robot"))
        # CFPA2 C++ coordinator publishes the goal PointStamped on the suffix
        # `/way_point_coord` (config/cfpa2_single_robot.yaml: goal_topic_suffix).
        wp_param = rospy.get_param("~waypoint_topic", "way_point_coord")
        goal_param = rospy.get_param("~goal_topic", "move_base_simple/goal")
        # Point-LIO onboard publishes /<ns>/Odometry (onboard_pointlio_noetic.sh).
        odom_param = rospy.get_param("~odom_topic", "Odometry")
        nav_status_param = rospy.get_param("~nav_status_topic", "nav_status")
        # move_base action server status topic (actionlib convention).
        status_param = rospy.get_param("~movebase_status_topic", "move_base/status")

        # Skip republishing if the new goal is within this distance of the last
        # published goal — CFPA2 republishes its current goal at 2 Hz to keep
        # the channel alive; we don't want move_base to restart every tick.
        self.goal_change_min_m = float(rospy.get_param("~goal_change_min_m", 0.30))
        # Bridge-side threshold kept for parity with the ROS 2 node's param
        # surface; on move_base the terminal status alone is authoritative, so
        # this currently gates nothing extra but is exposed for tuning.
        self._planner_failure_threshold = max(
            1, int(rospy.get_param("~planner_failure_threshold", 3))
        )

        self._wp_topic = _ns_topic(ns, wp_param)
        self._goal_topic = _ns_topic(ns, goal_param)
        self._odom_topic = _ns_topic(ns, odom_param)
        self._nav_status_topic = _ns_topic(ns, nav_status_param)
        self._status_topic = _ns_topic(ns, status_param)

        self._last_pose_x = None
        self._last_pose_y = None
        self._last_goal_x = None
        self._last_goal_y = None
        self._goal_seq = 0
        self._active_goal = None  # (gx, gy)
        self._last_goal_pub_stamp_ns = 0

        # Dedupe move_base terminal events by goal_id so we fire the fast-BL
        # burst once per move_base goal even though status republishes at ~5 Hz.
        self._last_acted_goal_id = None
        # One-shot 3-emit burst state (CFPA2 fast-BL needs 3 consecutive).
        self._burst_remaining = 0
        self._burst_state = None
        self._burst_reason = None
        self._burst_extra = None
        self._burst_timer = None

        self._goal_pub = rospy.Publisher(self._goal_topic, PoseStamped, queue_size=10)
        self._nav_status_pub = rospy.Publisher(
            self._nav_status_topic, String, queue_size=10
        )

        rospy.Subscriber(self._odom_topic, Odometry, self._on_odom, queue_size=5)
        rospy.Subscriber(
            self._wp_topic, PointStamped, self._on_waypoint, queue_size=5
        )
        rospy.Subscriber(
            self._status_topic, GoalStatusArray, self._on_movebase_status,
            queue_size=5,
        )

        rospy.loginfo(
            "bridge armed. %s -> %s; pose from %s; nav_status on %s; "
            "move_base status from %s",
            self._wp_topic, self._goal_topic, self._odom_topic,
            self._nav_status_topic, self._status_topic,
        )

    def _on_odom(self, msg):
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y

    def _emit_nav_status(self, state, reason, extra=None):
        payload = {
            "schema": "nav_status/v1",
            "source": "cfpa2_to_movebase_bridge",
            "state": state,
            "reason": reason,
            "stamp_ns": int(rospy.Time.now().to_nsec()),
            "goal_seq": int(self._goal_seq),
        }
        if self._active_goal is not None:
            payload["goal"] = [
                float(self._active_goal[0]),
                float(self._active_goal[1]),
            ]
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._nav_status_pub.publish(msg)

    def _on_waypoint(self, msg):
        gx, gy = float(msg.point.x), float(msg.point.y)
        # Suppress duplicate / sub-threshold-change goals.
        if (
            self._last_goal_x is not None
            and math.hypot(gx - self._last_goal_x, gy - self._last_goal_y)
            < self.goal_change_min_m
        ):
            return

        # Synthesize orientation pointing from current robot pose to goal.
        # If we have no odom yet, default to facing +x (yaw=0).
        yaw = 0.0
        if self._last_pose_x is not None:
            dx = gx - self._last_pose_x
            dy = gy - self._last_pose_y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = msg.header.frame_id or "map"
        out.pose.position.x = gx
        out.pose.position.y = gy
        out.pose.position.z = 0.0
        # quaternion from yaw alone (z-axis rotation).
        out.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.orientation.w = math.cos(0.5 * yaw)
        self._goal_pub.publish(out)
        self._last_goal_pub_stamp_ns = int(out.header.stamp.to_nsec())

        rospy.loginfo(
            "forwarded goal (%+.2f, %+.2f) yaw=%+.1f deg",
            gx, gy, math.degrees(yaw),
        )
        self._last_goal_x = gx
        self._last_goal_y = gy
        self._goal_seq += 1
        self._active_goal = (gx, gy)
        # A new goal supersedes any in-flight terminal burst.
        self._cancel_burst()
        self._emit_nav_status("navigating", "goal_forwarded")

    def _on_movebase_status(self, msg):
        """Fire nav_status(unreachable) on the first ABORTED/REJECTED we see
        for a move_base goal_id we haven't yet acted on.

        move_base publishes /<ns>/move_base/status as a GoalStatusArray listing
        the recent goals. We pick the most-recently-stamped terminal entry and
        dedupe by goal_id so we only emit once per move_base goal even though
        the status keeps republishing.
        """
        if self._active_goal is None:
            return
        terminal = None  # (stamp_ns, goal_id, status_code)
        for st in msg.status_list:
            code = int(st.status)
            if code not in (GoalStatus.ABORTED, GoalStatus.REJECTED):
                continue
            stamp_ns = int(st.goal_id.stamp.to_nsec())
            # Ignore terminal status that predates our current goal publish.
            if stamp_ns < int(self._last_goal_pub_stamp_ns):
                continue
            goal_id = str(st.goal_id.id)
            if terminal is None or stamp_ns > terminal[0]:
                terminal = (stamp_ns, goal_id, code)
        if terminal is None:
            return
        _, goal_id, code = terminal
        if goal_id == self._last_acted_goal_id:
            return
        self._last_acted_goal_id = goal_id

        state = "unreachable" if code == GoalStatus.ABORTED else "failed"
        reason = (
            "movebase_aborted" if code == GoalStatus.ABORTED
            else "movebase_rejected"
        )
        extra = {"status_code": int(code), "goal_id": goal_id}
        # CFPA2's fast-BL needs N (default 3) consecutive `unreachable` for the
        # same goal_key. A single move_base terminal event only triggers this
        # callback once per goal_id, so spin a tiny one-shot timer to emit 3 in
        # a row (the 2nd + 3rd satisfy CFPA2's consec_threshold).
        self._emit_nav_status(state, reason, extra=extra)
        self._burst_remaining = 2
        self._burst_state = state
        self._burst_reason = reason
        self._burst_extra = extra
        self._cancel_burst_timer_only()
        # 100 ms cadence x 2 follow-ups -> CFPA2 sees 3 emits within ~250 ms.
        self._burst_timer = rospy.Timer(
            rospy.Duration(0.10), self._burst_tick
        )
        rospy.loginfo(
            "move_base %s: goal=(%+.2f,%+.2f) code=%d -> fast BL burst (3 emits)",
            state, self._active_goal[0], self._active_goal[1], code,
        )

    def _burst_tick(self, _event):
        if self._burst_remaining <= 0:
            self._cancel_burst_timer_only()
            return
        self._emit_nav_status(
            self._burst_state, self._burst_reason, extra=self._burst_extra
        )
        self._burst_remaining -= 1
        if self._burst_remaining <= 0:
            self._cancel_burst_timer_only()

    def _cancel_burst_timer_only(self):
        if self._burst_timer is not None:
            try:
                self._burst_timer.shutdown()
            except Exception:
                pass
            self._burst_timer = None

    def _cancel_burst(self):
        self._cancel_burst_timer_only()
        self._burst_remaining = 0


def main():
    rospy.init_node("cfpa2_to_movebase_bridge")
    Cfpa2ToMovebaseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
