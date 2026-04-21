#!/usr/bin/env python3
"""Centralized multi-robot frontier goal assignment (Burgard-style heuristic).

Legacy mode score:
    score(i, t) = U_t - beta * V_t^i

Committed mode adds Level-1 anti-oscillation controls:
- goal lock timer
- progress-gated switching
- frontier blacklist after repeated failures
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


class MultiRobotGoalAssigner(Node):
    def __init__(self) -> None:
        super().__init__("multi_robot_goal_assigner")

        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("beta", 0.18)
        self.declare_parameter("sensor_range", 3.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("max_targets", 800)
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("use_shared_map", False)
        self.declare_parameter("shared_map_topic", "/disco_slam/global_map")
        self.declare_parameter("shared_map_wait_sec", 8.0)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)
        self.declare_parameter("switch_hysteresis", 0.05)
        self.declare_parameter("switch_min_dist", 0.35)
        self.declare_parameter("min_assign_distance", 0.30)

        # Algorithm-selection and Level-1 stabilization controls.
        self.declare_parameter("algorithm_mode", "legacy")
        self.declare_parameter("goal_lock_sec", 5.0)
        self.declare_parameter("progress_window_sec", 3.0)
        self.declare_parameter("progress_min_delta_m", 0.15)
        self.declare_parameter("blacklist_fail_count", 2)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("blacklist_key_resolution", 0.5)
        self.declare_parameter("reached_blacklist_dist", 0.30)
        self.declare_parameter("reached_blacklist_repeat_count", 3)
        self.declare_parameter("reached_blacklist_ttl_sec", 12.0)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.beta = float(self.get_parameter("beta").value)
        self.sensor_range = max(0.1, float(self.get_parameter("sensor_range").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_targets = max(50, int(self.get_parameter("max_targets").value))
        self.goal_topic_suffix = str(self.get_parameter("goal_topic_suffix").value)
        self.use_shared_map = bool(self.get_parameter("use_shared_map").value)
        self.shared_map_topic = str(self.get_parameter("shared_map_topic").value)
        self.shared_map_wait_sec = max(0.0, float(self.get_parameter("shared_map_wait_sec").value))
        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)
        self.switch_hysteresis = max(0.0, float(self.get_parameter("switch_hysteresis").value))
        self.switch_min_dist = max(0.1, float(self.get_parameter("switch_min_dist").value))
        self.min_assign_distance = max(0.0, float(self.get_parameter("min_assign_distance").value))

        self.algorithm_mode = str(self.get_parameter("algorithm_mode").value).strip().lower()
        if self.algorithm_mode not in {"legacy", "committed"}:
            self.get_logger().warn(
                f"Unknown algorithm_mode='{self.algorithm_mode}', falling back to legacy"
            )
            self.algorithm_mode = "legacy"

        self.goal_lock_sec = max(0.0, float(self.get_parameter("goal_lock_sec").value))
        self.progress_window_sec = max(0.5, float(self.get_parameter("progress_window_sec").value))
        self.progress_min_delta_m = max(0.0, float(self.get_parameter("progress_min_delta_m").value))
        self.blacklist_fail_count = max(1, int(self.get_parameter("blacklist_fail_count").value))
        self.blacklist_ttl_sec = max(0.0, float(self.get_parameter("blacklist_ttl_sec").value))
        self.blacklist_key_resolution = max(
            0.05,
            float(self.get_parameter("blacklist_key_resolution").value),
        )
        self.reached_blacklist_dist = max(0.0, float(self.get_parameter("reached_blacklist_dist").value))
        self.reached_blacklist_repeat_count = max(
            1,
            int(self.get_parameter("reached_blacklist_repeat_count").value),
        )
        self.reached_blacklist_ttl_sec = max(
            0.0,
            float(self.get_parameter("reached_blacklist_ttl_sec").value),
        )

        self.maps: dict[str, OccupancyGrid] = {}
        self.shared_map: Optional[OccupancyGrid] = None
        self.odoms: dict[str, Odometry] = {}
        self.last_goal: dict[str, tuple[float, float]] = {}
        self.last_goal_set_time_ns: dict[str, int] = {}
        self.goal_progress_samples: dict[str, deque[tuple[int, float]]] = {
            ns: deque() for ns in self.namespaces
        }
        self.goal_fail_counts: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.goal_blacklist_until_ns: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.reached_goal_repeat_count: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.reached_goal_last_key: dict[str, Optional[tuple[int, int]]] = {
            ns: None for ns in self.namespaces
        }
        self.last_policy_reason: dict[str, str] = {ns: "init" for ns in self.namespaces}

        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False
        self._start_ns = self.get_clock().now().nanoseconds
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0

        self.goal_pubs = {}
        for ns in self.namespaces:
            self.create_subscription(OccupancyGrid, f"/{ns}/map", lambda m, n=ns: self._map_cb(m, n), 1)
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._odom_cb(m, n), 10)
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_topic_suffix}", 10)
        if self.use_shared_map:
            self.create_subscription(OccupancyGrid, self.shared_map_topic, self._shared_map_cb, 1)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"Coordinator started for {self.namespaces} | mode={self.algorithm_mode} "
            f"beta={self.beta:.2f}, sensor_range={self.sensor_range:.2f}, "
            f"goal_lock_sec={self.goal_lock_sec:.1f}, progress_window_sec={self.progress_window_sec:.1f}, "
            f"progress_min_delta_m={self.progress_min_delta_m:.2f}, "
            f"blacklist_fail_count={self.blacklist_fail_count}, blacklist_ttl_sec={self.blacklist_ttl_sec:.1f}, "
            f"min_assign_distance={self.min_assign_distance:.2f}, "
            f"reached_blacklist_dist={self.reached_blacklist_dist:.2f}, "
            f"reached_blacklist_repeat_count={self.reached_blacklist_repeat_count}, "
            f"reached_blacklist_ttl_sec={self.reached_blacklist_ttl_sec:.1f}, "
            f"use_shared_map={self.use_shared_map} shared_map_topic={self.shared_map_topic} "
            f"shared_map_wait_sec={self.shared_map_wait_sec:.1f}"
        )

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        self.maps[ns] = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self.odoms[ns] = msg

    def _shared_map_cb(self, msg: OccupancyGrid) -> None:
        self.shared_map = msg
        if self._shared_map_fallback_active:
            self.get_logger().info(
                f"Shared map received on {self.shared_map_topic}; switching to shared-map coordination."
            )
        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False

    @staticmethod
    def _grid_index(x: int, y: int, w: int) -> int:
        return y * w + x

    def _world_to_grid(self, msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
        gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
        if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
            return None
        return (gx, gy)

    def _grid_to_world(self, msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
        return (
            msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
        )

    def _is_free(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.free_value

    def _is_unknown(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.unknown_value

    def _extract_frontiers(self, msg: OccupancyGrid) -> list[tuple[float, float]]:
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        out: list[tuple[float, float]] = []
        s = self.frontier_stride

        for gy in range(1, h - 1, s):
            row = gy * w
            for gx in range(1, w - 1, s):
                idx = row + gx
                if not self._is_free(data, idx):
                    continue
                found_unknown = False
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
                    nidx = (gy + dy) * w + (gx + dx)
                    if self._is_unknown(data, nidx):
                        found_unknown = True
                        break
                if found_unknown:
                    out.append(self._grid_to_world(msg, gx, gy))
                    if len(out) >= self.max_targets:
                        return out
        return out

    def _distance_transform(self, msg: OccupancyGrid, start_w: tuple[float, float]) -> dict[int, int]:
        start = self._world_to_grid(msg, start_w[0], start_w[1])
        if start is None:
            return {}

        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        sx, sy = start
        sidx = self._grid_index(sx, sy, w)

        if not self._is_free(data, sidx):
            found = None
            for r in range(1, 13):
                for dy in range(-r, r + 1):
                    ny = sy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        if nx < 0 or nx >= w:
                            continue
                        nidx = self._grid_index(nx, ny, w)
                        if self._is_free(data, nidx):
                            found = (nx, ny, nidx)
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is None:
                return {}
            sx, sy, sidx = found

        q = deque([(sx, sy)])
        dist = {sidx: 0}
        while q:
            cx, cy = q.popleft()
            cidx = self._grid_index(cx, cy, w)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = self._grid_index(nx, ny, w)
                if nidx in dist:
                    continue
                if not self._is_free(data, nidx):
                    continue
                dist[nidx] = dist[cidx] + 1
                q.append((nx, ny))
        return dist

    def _merge_targets(self, target_lists: list[list[tuple[float, float]]], merge_resolution: float) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()
        q = max(0.05, float(merge_resolution))
        for targets in target_lists:
            for wx, wy in targets:
                key = (int(round(wx / q)), int(round(wy / q)))
                if key in seen:
                    continue
                seen.add(key)
                out.append((wx, wy))
                if len(out) >= self.max_targets:
                    return out
        return out

    def _goal_key(self, goal: tuple[float, float]) -> tuple[int, int]:
        q = self.blacklist_key_resolution
        return (int(round(goal[0] / q)), int(round(goal[1] / q)))

    def _prune_blacklist(self, ns: str, now_ns: int) -> None:
        entries = self.goal_blacklist_until_ns[ns]
        expired = [k for k, until_ns in entries.items() if until_ns <= now_ns]
        for key in expired:
            entries.pop(key, None)

    def _is_blacklisted(self, ns: str, goal: tuple[float, float], now_ns: int) -> bool:
        self._prune_blacklist(ns, now_ns)
        key = self._goal_key(goal)
        until_ns = self.goal_blacklist_until_ns[ns].get(key, 0)
        return until_ns > now_ns

    def _register_goal_failure(self, ns: str, goal: tuple[float, float], now_ns: int, reason: str) -> None:
        key = self._goal_key(goal)
        counts = self.goal_fail_counts[ns]
        counts[key] = counts.get(key, 0) + 1
        if counts[key] < self.blacklist_fail_count:
            return

        counts[key] = 0
        if self.blacklist_ttl_sec <= 0.0:
            return

        until_ns = now_ns + int(self.blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting goal ({goal[0]:.2f},{goal[1]:.2f}) for {self.blacklist_ttl_sec:.1f}s "
            f"after repeated {reason} failures."
        )

    def _distance_robot_to_goal(self, ns: str, goal: tuple[float, float]) -> float:
        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        return math.hypot(goal[0] - rx, goal[1] - ry)

    def _goal_too_close(self, ns: str, goal: tuple[float, float]) -> bool:
        if self.min_assign_distance <= 0.0:
            return False
        return self._distance_robot_to_goal(ns, goal) <= self.min_assign_distance

    def _update_reached_goal_blacklist(self, ns: str, now_ns: int) -> None:
        if self.reached_blacklist_ttl_sec <= 0.0 or self.reached_blacklist_dist <= 0.0:
            return

        goal = self.last_goal.get(ns)
        if goal is None:
            self.reached_goal_last_key[ns] = None
            self.reached_goal_repeat_count[ns] = 0
            return

        key = self._goal_key(goal)
        dist = self._distance_robot_to_goal(ns, goal)
        if dist > self.reached_blacklist_dist:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 0
            return

        # Do not repeatedly extend an active blacklist entry for the same key.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            self.reached_goal_repeat_count[ns] = 0
            return

        if self.reached_goal_last_key[ns] == key:
            self.reached_goal_repeat_count[ns] += 1
        else:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 1

        if self.reached_goal_repeat_count[ns] < self.reached_blacklist_repeat_count:
            return

        self.reached_goal_repeat_count[ns] = 0
        until_ns = now_ns + int(self.reached_blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting repeatedly reached goal ({goal[0]:.2f},{goal[1]:.2f}) "
            f"for {self.reached_blacklist_ttl_sec:.1f}s "
            f"after {self.reached_blacklist_repeat_count} near-goal repeats "
            f"(dist<={self.reached_blacklist_dist:.2f}m)."
        )

    def _goal_reachable(self, map_msg: OccupancyGrid, dist_map: dict[int, int], goal: tuple[float, float]) -> bool:
        g = self._world_to_grid(map_msg, goal[0], goal[1])
        if g is None:
            return False
        idx = self._grid_index(g[0], g[1], int(map_msg.info.width))
        return idx in dist_map

    def _update_progress_samples(self, ns: str, now_ns: int) -> None:
        goal = self.last_goal.get(ns)
        if goal is None:
            return

        samples = self.goal_progress_samples[ns]
        samples.append((now_ns, self._distance_robot_to_goal(ns, goal)))

        cutoff_ns = now_ns - int(self.progress_window_sec * 1e9)
        while len(samples) >= 2 and samples[0][0] < cutoff_ns:
            samples.popleft()

    def _progress_delta(self, ns: str) -> Optional[float]:
        samples = self.goal_progress_samples[ns]
        if len(samples) < 2:
            return None
        span_ns = samples[-1][0] - samples[0][0]
        if span_ns < int(0.5 * self.progress_window_sec * 1e9):
            return None
        return samples[0][1] - samples[-1][1]

    def _set_active_goal(self, ns: str, goal: tuple[float, float], now_ns: int) -> None:
        prev = self.last_goal.get(ns)
        if prev is None or math.hypot(prev[0] - goal[0], prev[1] - goal[1]) > 1e-6:
            self.last_goal_set_time_ns[ns] = now_ns
            self.goal_progress_samples[ns].clear()
        self.last_goal[ns] = goal

    def _set_policy_reason(self, ns: str, reason: str) -> None:
        self.last_policy_reason[ns] = reason

    def _apply_switch_hysteresis(self, ns: str, goal: tuple[float, float], assignment_score: float) -> tuple[float, float]:
        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return goal

        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        dist_to_last = math.hypot(last[0] - rx, last[1] - ry)
        move = math.hypot(goal[0] - last[0], goal[1] - last[1])

        # Only apply hold logic while still traveling to the previous goal.
        if dist_to_last > self.switch_min_dist:
            if move < self.switch_min_dist:
                self._set_policy_reason(ns, "hold/hysteresis_small_move")
                return last
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/hysteresis_low_score")
                return last
        self._set_policy_reason(ns, "switch/hysteresis_ok")
        return goal

    def _apply_goal_policy(
        self,
        ns: str,
        candidate_goal: tuple[float, float],
        assignment_score: float,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        now_ns: int,
    ) -> tuple[float, float]:
        if self.algorithm_mode != "committed":
            return self._apply_switch_hysteresis(ns, candidate_goal, assignment_score)

        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return candidate_goal

        if self._is_blacklisted(ns, candidate_goal, now_ns):
            self._set_policy_reason(ns, "hold/candidate_blacklisted")
            return last

        dist_to_last = self._distance_robot_to_goal(ns, last)
        reached_last = dist_to_last <= self.switch_min_dist
        last_reachable = self._goal_reachable(map_msg, dist_map, last)
        hard_failure = not last_reachable

        last_set_ns = self.last_goal_set_time_ns.get(ns, 0)
        lock_active = (
            self.goal_lock_sec > 0.0
            and last_set_ns > 0
            and (now_ns - last_set_ns) < int(self.goal_lock_sec * 1e9)
        )

        if lock_active and not hard_failure and not reached_last:
            self._set_policy_reason(ns, "hold/goal_lock_active")
            return last

        delta = self._progress_delta(ns)
        stalled = delta is not None and delta < self.progress_min_delta_m

        candidate_move = math.hypot(candidate_goal[0] - last[0], candidate_goal[1] - last[1])
        if candidate_move < self.switch_min_dist:
            self._set_policy_reason(ns, "hold/small_candidate_move")
            return last

        if not reached_last and not hard_failure:
            # Keep commitment while making sufficient progress.
            if not stalled:
                self._set_policy_reason(ns, "hold/progressing")
                return last
            # Only switch on weak assignment scores when already stalled.
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/stalled_but_low_score")
                return last

        if not reached_last and (hard_failure or stalled):
            reason = "unreachable" if hard_failure else "stalled"
            self._register_goal_failure(ns, last, now_ns, reason)
            self._set_policy_reason(ns, f"switch/{reason}")
            return candidate_goal

        self._set_policy_reason(ns, "switch/reached_or_improved")
        return candidate_goal

    def _maybe_log_summary(
        self,
        targets_total: int,
        per_ns_frontiers: dict[str, int],
        per_ns_reachable: dict[str, int],
        per_ns_assigned: dict[str, tuple[float, float]],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_summary_ns == 0:
            self._last_summary_ns = now_ns
            return
        if (now_ns - self._last_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_summary_ns = now_ns

        parts = []
        for ns in self.namespaces:
            goal = per_ns_assigned.get(ns)
            goal_txt = "None" if goal is None else f"({goal[0]:.2f},{goal[1]:.2f})"
            dist_txt = "-"
            age_txt = "-"
            if goal is not None and ns in self.odoms:
                dist_txt = f"{self._distance_robot_to_goal(ns, goal):.2f}"
            set_ns = self.last_goal_set_time_ns.get(ns, 0)
            if set_ns > 0:
                age_txt = f"{max(0.0, (now_ns - set_ns) / 1e9):.1f}"
            policy = self.last_policy_reason.get(ns, "-")
            parts.append(
                f"{ns}:frontiers={per_ns_frontiers.get(ns, 0)} "
                f"reachable={per_ns_reachable.get(ns, 0)} goal={goal_txt} "
                f"d={dist_txt} age={age_txt}s policy={policy}"
            )
        self.get_logger().info(
            f"ASSIGN step[{self.algorithm_mode}]: targets={targets_total} | " + " | ".join(parts)
        )

    def _publish_goal(self, ns: str, map_msg: OccupancyGrid, goal_w: tuple[float, float]) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id or "world"
        msg.point.x = goal_w[0]
        msg.point.y = goal_w[1]
        msg.point.z = 0.0
        self.goal_pubs[ns].publish(msg)

    def _tick(self) -> None:
        if any(ns not in self.maps or ns not in self.odoms for ns in self.namespaces):
            return

        now_ns = self.get_clock().now().nanoseconds
        for ns in self.namespaces:
            self._prune_blacklist(ns, now_ns)
            self._update_reached_goal_blacklist(ns, now_ns)

        using_shared_map = self.use_shared_map and self.shared_map is not None
        target_map: OccupancyGrid
        if self.use_shared_map:
            if using_shared_map:
                target_map = self.shared_map  # type: ignore[assignment]
                self._warned_missing_shared_map = False
                self._shared_map_fallback_active = False
            else:
                waited_sec = (now_ns - self._start_ns) / 1e9
                if waited_sec >= self.shared_map_wait_sec:
                    # Fail-open: continue coordinated assignment on local map inputs
                    # until shared map becomes available.
                    target_map = self.maps[self.namespaces[0]]
                    if not self._shared_map_fallback_active:
                        self.get_logger().warn(
                            f"Shared map not available on {self.shared_map_topic} after {waited_sec:.1f}s; "
                            "falling back to per-robot maps."
                        )
                        self._shared_map_fallback_active = True
                else:
                    if not self._warned_missing_shared_map:
                        self.get_logger().warn(
                            f"Waiting for shared map on {self.shared_map_topic}; "
                            f"fallback in {max(0.0, self.shared_map_wait_sec - waited_sec):.1f}s."
                        )
                        self._warned_missing_shared_map = True
                    return
        else:
            target_map = self.maps[self.namespaces[0]]

        per_ns_targets: dict[str, list[tuple[float, float]]] = {}
        for ns in self.namespaces:
            per_ns_targets[ns] = self._extract_frontiers(self.maps[ns])

        if using_shared_map:
            targets = self._extract_frontiers(target_map)
        else:
            merge_res = max(0.1, float(target_map.info.resolution) * 2.0)
            targets = self._merge_targets([per_ns_targets[ns] for ns in self.namespaces], merge_res)

        if not targets:
            return

        dist_maps = {}
        for ns in self.namespaces:
            od = self.odoms[ns]
            # When shared map is unavailable we fail-open to per-robot maps.
            cost_map = target_map if using_shared_map else self.maps[ns]
            dist_maps[ns] = self._distance_transform(cost_map, (od.pose.pose.position.x, od.pose.pose.position.y))
            if self.algorithm_mode == "committed":
                self._update_progress_samples(ns, now_ns)

        utilities = [1.0 for _ in targets]
        unassigned = set(self.namespaces)
        assigned: dict[str, int] = {}
        assignment_scores: dict[str, float] = {}
        sigma = max(self.sensor_range * 0.5, 1e-3)

        while unassigned:
            best_pair = None
            best_score = -1e18

            for ns in list(unassigned):
                msg = target_map if using_shared_map else self.maps[ns]
                dist_map = dist_maps[ns]
                if not dist_map:
                    continue
                for ti, (wx, wy) in enumerate(targets):
                    if self._goal_too_close(ns, (wx, wy)):
                        continue
                    if self._is_blacklisted(ns, (wx, wy), now_ns):
                        continue
                    g = self._world_to_grid(msg, wx, wy)
                    if g is None:
                        continue
                    idx = self._grid_index(g[0], g[1], int(msg.info.width))
                    if idx not in dist_map:
                        continue
                    cost_m = float(dist_map[idx]) * msg.info.resolution
                    score = utilities[ti] - self.beta * cost_m
                    if score > best_score:
                        best_score = score
                        best_pair = (ns, ti, score)

            if best_pair is None:
                break

            ns, ti, score = best_pair
            assigned[ns] = ti
            assignment_scores[ns] = float(score)
            unassigned.remove(ns)

            tx, ty = targets[ti]
            for j, (wx, wy) in enumerate(targets):
                d = math.hypot(wx - tx, wy - ty)
                if d > self.sensor_range:
                    continue
                p = math.exp(-0.5 * (d / sigma) * (d / sigma))
                utilities[j] = max(0.0, utilities[j] - p)

        # Fail-open: if collaborative assignment cannot find a reachable shared target
        # for a robot, use that robot's nearest local frontier.
        candidate_goals: dict[str, tuple[float, float]] = {}
        for ns in list(unassigned):
            local_targets = [
                goal
                for goal in per_ns_targets.get(ns, [])
                if not self._goal_too_close(ns, goal)
                if not self._is_blacklisted(ns, goal, now_ns)
            ]
            if not local_targets:
                continue
            od = self.odoms[ns]
            rx = float(od.pose.pose.position.x)
            ry = float(od.pose.pose.position.y)
            nearest_idx = min(
                range(len(local_targets)),
                key=lambda i: math.hypot(local_targets[i][0] - rx, local_targets[i][1] - ry),
            )
            candidate_goals[ns] = local_targets[nearest_idx]
            assignment_scores.setdefault(ns, 1.0)
            unassigned.remove(ns)

        for ns, ti in assigned.items():
            candidate_goals[ns] = targets[ti]

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                # No new assignment candidate, hold current goal if any.
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/no_candidate_no_previous_goal")
                    continue
                self._set_policy_reason(ns, "hold/no_candidate")
                goal = held
            else:
                msg_for_ns = target_map if using_shared_map else self.maps[ns]
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate,
                    assignment_score=assignment_scores.get(ns, 0.0),
                    map_msg=msg_for_ns,
                    dist_map=dist_maps.get(ns, {}),
                    now_ns=now_ns,
                )

            self._set_active_goal(ns, goal, now_ns)
            self._publish_goal(ns, self.maps[ns], goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable: dict[str, int] = {}
        for ns in self.namespaces:
            msg = target_map if using_shared_map else self.maps[ns]
            dist_map = dist_maps.get(ns, {})
            if not dist_map:
                per_ns_reachable[ns] = 0
                continue
            reachable = 0
            for wx, wy in targets:
                if self._goal_too_close(ns, (wx, wy)):
                    continue
                if self._is_blacklisted(ns, (wx, wy), now_ns):
                    continue
                g = self._world_to_grid(msg, wx, wy)
                if g is None:
                    continue
                idx = self._grid_index(g[0], g[1], int(msg.info.width))
                if idx in dist_map:
                    reachable += 1
            per_ns_reachable[ns] = reachable

        per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
        self._maybe_log_summary(
            targets_total=len(targets),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiRobotGoalAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
