#!/usr/bin/env python3
"""Publish slope-verified ramp ascent goals from filtered GridMap layers."""

from __future__ import annotations

import math

import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from trav_cost_filters.ramp_goal_selector import (
    GridMapGeometry,
    RampGoal,
    RampSelectorParams,
    advance_centerline_ascent_goal,
    select_approach_goal,
    select_ramp_ascent_goal,
    select_ramp_ascent_goal_from_points,
)


class RampAscentGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("ramp_ascent_goal")

        self.input_topic = str(
            self.declare_parameter("input_topic", "elevation_map_filtered").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "ramp_ascent_goal").value
        )
        self.robot_frame = str(
            self.declare_parameter("robot_frame", "base_link").value
        )
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.pointcloud_topic = str(
            self.declare_parameter("pointcloud_topic", "registered_scan_reliable").value
        )
        self.use_pointcloud_ramp_detection = bool(
            self.declare_parameter("use_pointcloud_ramp_detection", True).value
        )
        self.pointcloud_stride = max(
            1, int(self.declare_parameter("pointcloud_stride", 4).value)
        )
        self.verified_hold_sec = max(
            0.1, float(self.declare_parameter("verified_hold_sec", 1.5).value)
        )
        self.elevation_layer = str(
            self.declare_parameter("elevation_layer", "elevation").value
        )
        self.traversability_layer = str(
            self.declare_parameter("traversability_layer", "trav_eth").value
        )
        self.slope_layer = str(
            self.declare_parameter("slope_layer", "slope").value
        )
        self.step_residual_layer = str(
            self.declare_parameter("step_residual_layer", "step_residual").value
        )
        self.params = RampSelectorParams(
            min_traversability=float(
                self.declare_parameter("min_traversability", 0.30).value
            ),
            min_slope_rad=math.radians(
                float(self.declare_parameter("min_slope_deg", 5.0).value)
            ),
            max_slope_rad=math.radians(
                float(self.declare_parameter("max_slope_deg", 30.0).value)
            ),
            max_step_residual_m=float(
                self.declare_parameter("max_step_residual_m", 0.06).value
            ),
            min_candidate_cells=int(
                self.declare_parameter("min_candidate_cells", 8).value
            ),
            min_elevation_span_m=float(
                self.declare_parameter("min_elevation_span_m", 0.25).value
            ),
            min_goal_distance_m=float(
                self.declare_parameter("min_goal_distance_m", 0.70).value
            ),
            max_goal_distance_m=float(
                self.declare_parameter("max_goal_distance_m", 4.5).value
            ),
            platform_min_elevation_gain_m=float(
                self.declare_parameter("platform_min_elevation_gain_m", 0.45).value
            ),
            platform_lateral_window_m=float(
                self.declare_parameter("platform_lateral_window_m", 1.5).value
            ),
            platform_forward_window_m=float(
                self.declare_parameter("platform_forward_window_m", 2.5).value
            ),
            preferred_uphill_yaw_rad=self._optional_yaw_param(
                "preferred_uphill_yaw_deg"
            ),
            preferred_uphill_tolerance_rad=math.radians(
                float(self.declare_parameter("preferred_uphill_tolerance_deg", 45.0).value)
            ),
            goal_lookahead_m=self._optional_positive_param("goal_lookahead_m"),
            goal_center_y=self._optional_float_param("goal_center_y"),
            min_x=float(self.declare_parameter("min_x", -1.0e9).value),
            max_x=float(self.declare_parameter("max_x", 1.0e9).value),
            min_y=float(self.declare_parameter("min_y", -1.0e9).value),
            max_y=float(self.declare_parameter("max_y", 1.0e9).value),
        )
        self.approach_enabled = bool(
            self.declare_parameter("approach_enabled", False).value
        )
        self.approach_x = float(self.declare_parameter("approach_x", 0.0).value)
        self.approach_y = float(self.declare_parameter("approach_y", 0.0).value)
        self.approach_step_m = max(
            0.1, float(self.declare_parameter("approach_step_m", 2.0).value)
        )
        self.approach_stop_radius_m = max(
            0.0, float(self.declare_parameter("approach_stop_radius_m", 0.45).value)
        )
        self.monotonic_ascent_enabled = bool(
            self.declare_parameter("monotonic_ascent_enabled", False).value
        )
        self.monotonic_min_ahead_m = max(
            0.0,
            float(
                self.declare_parameter(
                    "monotonic_min_ahead_m",
                    float(self.params.goal_lookahead_m or 0.9),
                ).value
            ),
        )
        self.monotonic_hold_sec = max(
            0.0, float(self.declare_parameter("monotonic_hold_sec", 0.0).value)
        )
        self.monotonic_terminal_hold_enabled = bool(
            self.declare_parameter("monotonic_terminal_hold_enabled", False).value
        )
        self.ascent_terminal_x = self._optional_float_param("ascent_terminal_x")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_goal_xy: tuple[float, float] | None = None
        self.last_mode: str | None = None
        self.last_verified_ramp_ns = 0
        self.last_verified_goal: RampGoal | None = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(PointStamped, self.output_topic, 10)
        self.sub = self.create_subscription(GridMap, self.input_topic, self._on_map, qos)
        self.cloud_sub = None
        if self.use_pointcloud_ramp_detection:
            self.cloud_sub = self.create_subscription(
                PointCloud2, self.pointcloud_topic, self._on_cloud, qos
            )

        self.get_logger().info(
            f"ramp_ascent_goal: {self.input_topic} -> {self.output_topic} "
            f"layers=({self.elevation_layer},{self.traversability_layer},"
            f"{self.slope_layer},{self.step_residual_layer}) "
            f"slope=[{math.degrees(self.params.min_slope_rad):.1f},"
            f"{math.degrees(self.params.max_slope_rad):.1f}]deg "
            f"trav>={self.params.min_traversability:.2f} "
            f"step<={self.params.max_step_residual_m:.2f}m "
            f"monotonic={'on' if self.monotonic_ascent_enabled else 'off'}"
        )

    def _optional_yaw_param(self, name: str) -> float | None:
        deg = float(self.declare_parameter(name, 999.0).value)
        if abs(deg) > 360.0:
            return None
        return math.radians(deg)

    def _optional_positive_param(self, name: str) -> float | None:
        value = float(self.declare_parameter(name, 0.0).value)
        if value <= 0.0:
            return None
        return value

    def _optional_float_param(self, name: str) -> float | None:
        value = float(self.declare_parameter(name, 1.0e9).value)
        if abs(value) >= 1.0e8:
            return None
        return value

    def _layer_array(self, msg: GridMap, layer_name: str) -> np.ndarray | None:
        if layer_name not in msg.layers:
            self.get_logger().warn_throttle(
                self.get_clock(),
                5000,
                f"layer '{layer_name}' not in GridMap; available={list(msg.layers)}",
            )
            return None

        rows = int(round(msg.info.length_y / msg.info.resolution))
        cols = int(round(msg.info.length_x / msg.info.resolution))
        expected = rows * cols
        layer_idx = list(msg.layers).index(layer_name)
        data = np.array(msg.data[layer_idx].data, dtype=np.float32)
        if data.size != expected:
            self.get_logger().warn_throttle(
                self.get_clock(),
                5000,
                f"layer '{layer_name}' size mismatch: data={data.size} expected={expected}",
            )
            return None
        return data.reshape(cols, rows).T

    def _robot_xy(self, frame_id: str) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warn_throttle(
                self.get_clock(),
                5000,
                f"ramp goal skipped: cannot transform {frame_id} <- "
                f"{self.robot_frame}: {exc}",
            )
            return None
        return (float(tf.transform.translation.x), float(tf.transform.translation.y))

    def _transform_cloud_to_map(self, msg: PointCloud2) -> np.ndarray | None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time())
        except TransformException as exc:
            self.get_logger().warn_throttle(
                self.get_clock(),
                5000,
                f"raw ramp detection skipped: cannot transform {self.map_frame} <- "
                f"{msg.header.frame_id}: {exc}",
            )
            return None

        q = tf.transform.rotation
        t = tf.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array(
            [
                [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
                [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
                [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
            ],
            dtype=np.float64,
        )
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)
        points = []
        for idx, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx % self.pointcloud_stride != 0:
                continue
            points.append((point[0], point[1], point[2]))
        if not points:
            return None
        sensor_points = np.asarray(points, dtype=np.float64)
        return sensor_points @ rot.T + trans

    def _publish_goal(self, goal, stamp, frame_id: str) -> None:
        msg_out = PointStamped()
        msg_out.header.stamp = stamp
        msg_out.header.frame_id = frame_id
        msg_out.point.x = goal.x
        msg_out.point.y = goal.y
        msg_out.point.z = goal.elevation_m
        self.pub.publish(msg_out)

        changed = (
            self.last_goal_xy is None
            or math.hypot(goal.x - self.last_goal_xy[0], goal.y - self.last_goal_xy[1]) > 0.35
            or goal.mode != self.last_mode
        )
        if changed:
            self.get_logger().info(
                f"ramp goal {goal.mode}: ({goal.x:.2f},{goal.y:.2f},"
                f"z={goal.elevation_m:.2f}) cells={goal.candidate_cells} "
                f"slope={math.degrees(goal.slope_rad):.1f}deg "
                f"step_residual={goal.step_residual_m:.3f}m"
            )
            self.last_goal_xy = (goal.x, goal.y)
            self.last_mode = goal.mode

    def _ascent_center_y(self) -> float | None:
        if self.params.goal_center_y is not None and math.isfinite(float(self.params.goal_center_y)):
            return float(self.params.goal_center_y)
        if self.approach_enabled:
            return float(self.approach_y)
        return None

    def _inside_ascent_corridor(self, robot_xy: tuple[float, float]) -> bool:
        margin = 0.35
        x = float(robot_xy[0])
        y = float(robot_xy[1])
        if x < float(self.params.min_x) - margin:
            return False
        upper = (
            float(self.ascent_terminal_x)
            if self.ascent_terminal_x is not None
            else float(self.params.max_x)
        )
        if x > upper + margin:
            return False
        if math.isfinite(float(self.params.min_y)) and y < float(self.params.min_y) - margin:
            return False
        if math.isfinite(float(self.params.max_y)) and y > float(self.params.max_y) + margin:
            return False
        return True

    def _monotonic_progress_goal(
        self,
        goal: RampGoal | None,
        robot_xy: tuple[float, float],
    ) -> RampGoal | None:
        if not self.monotonic_ascent_enabled:
            return goal
        if goal is not None and goal.mode not in {"ramp", "platform"}:
            return goal

        if goal is None:
            if self.last_verified_goal is None or self.monotonic_hold_sec <= 0.0:
                return None
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_verified_ramp_ns > int(self.monotonic_hold_sec * 1e9):
                return None
            if not self._inside_ascent_corridor(robot_xy):
                return None

        progressed = advance_centerline_ascent_goal(
            current_goal=goal,
            robot_xy=robot_xy,
            previous_goal_xy=self.last_goal_xy,
            previous_goal=self.last_verified_goal,
            center_y=self._ascent_center_y(),
            min_ahead_m=self.monotonic_min_ahead_m,
            terminal_x=self.ascent_terminal_x,
            min_x=float(self.params.min_x),
            max_x=float(self.params.max_x),
            hold_terminal=self.monotonic_terminal_hold_enabled,
        )
        if progressed is not None and progressed.mode in {"ramp", "platform"}:
            self.last_verified_goal = progressed
        return progressed

    def _on_cloud(self, msg: PointCloud2) -> None:
        if not self.use_pointcloud_ramp_detection:
            return
        robot_xy = self._robot_xy(self.map_frame)
        if robot_xy is None:
            return
        points_map = self._transform_cloud_to_map(msg)
        if points_map is None:
            return
        goal = select_ramp_ascent_goal_from_points(
            points_map,
            robot_xy=robot_xy,
            params=self.params,
        )
        if goal is None:
            goal = self._monotonic_progress_goal(None, robot_xy)
            if goal is not None:
                self._publish_goal(goal, msg.header.stamp, self.map_frame)
            return
        self.last_verified_ramp_ns = self.get_clock().now().nanoseconds
        self.last_verified_goal = goal
        goal = self._monotonic_progress_goal(goal, robot_xy)
        if goal is None:
            return
        self._publish_goal(goal, msg.header.stamp, self.map_frame)

    def _on_map(self, msg: GridMap) -> None:
        elevation = self._layer_array(msg, self.elevation_layer)
        traversability = self._layer_array(msg, self.traversability_layer)
        slope = self._layer_array(msg, self.slope_layer)
        step_residual = self._layer_array(msg, self.step_residual_layer)
        if any(layer is None for layer in (elevation, traversability, slope, step_residual)):
            return

        rows, cols = elevation.shape
        robot_xy = self._robot_xy(msg.header.frame_id)
        if robot_xy is None:
            return

        geometry = GridMapGeometry(
            origin_x=float(msg.info.pose.position.x - msg.info.length_x / 2.0),
            origin_y=float(msg.info.pose.position.y - msg.info.length_y / 2.0),
            resolution=float(msg.info.resolution),
            width=int(cols),
            height=int(rows),
        )
        goal = select_ramp_ascent_goal(
            elevation=elevation,
            traversability=traversability,
            slope=slope,
            step_residual=step_residual,
            geometry=geometry,
            robot_xy=robot_xy,
            params=self.params,
        )
        if goal is None:
            goal = self._monotonic_progress_goal(None, robot_xy)
            if goal is not None:
                self._publish_goal(goal, msg.header.stamp, msg.header.frame_id)
                return
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_verified_ramp_ns < int(self.verified_hold_sec * 1e9):
                return
            if not self.approach_enabled:
                self.get_logger().debug("no slope-verified ramp goal in current GridMap")
                return
            goal = select_approach_goal(
                robot_xy=robot_xy,
                anchor_xy=(self.approach_x, self.approach_y),
                step_m=self.approach_step_m,
                stop_radius_m=self.approach_stop_radius_m,
                center_y=self.approach_y,
                require_anchor_ahead_x=True,
            )
            if goal is None:
                self.get_logger().debug("ramp approach complete; waiting for slope-verified ramp cells")
                return
        else:
            self.last_verified_ramp_ns = self.get_clock().now().nanoseconds
            self.last_verified_goal = goal
            goal = self._monotonic_progress_goal(goal, robot_xy)
            if goal is None:
                return

        self._publish_goal(goal, msg.header.stamp, msg.header.frame_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RampAscentGoalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
