#!/usr/bin/env python3
"""Single-robot CFPA2 frontier planner."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from std_msgs.msg import String

from .cfpa2_coordinator_node import CFPA2Coordinator
from .cluster_tracker import ClusterTracker
from .frontier_3d import extract_3d_frontiers, project_to_traversability_goal


class CFPA2SingleRobotNode(CFPA2Coordinator):
    def __init__(self) -> None:
        super().__init__(
            node_name="cfpa2_single_robot",
            default_namespaces=["robot"],
            startup_label="cfpa2_single_robot",
            planner_desc="Single-robot CFPA2",
        )

        default_ns = self.namespaces[0] if self.namespaces else "robot"
        self.declare_parameter("robot_namespace", default_ns)
        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip().strip("/") or default_ns

        # verbose_logs: gates the loud throttled debug logs (per-tick reasons,
        # full summary). When False (default), only state-change events are
        # logged so the operator can see what's happening at a glance.
        self.declare_parameter("verbose_logs", False)
        self.verbose_logs = bool(self.get_parameter("verbose_logs").value)
        self.declare_parameter("frontier_3d_min_unknown_volume_m3", 1.0)
        self.declare_parameter("frontier_3d_min_frontier_voxels", 50)
        self.declare_parameter("frontier_3d_border_margin_cells", 3)
        self.declare_parameter("frontier_3d_geodesic_voronoi", False)
        self.declare_parameter("frontier_3d_goal_search_radius_m", 2.0)
        # Z-band filter for "actionable" unknown voxels. Default [-0.2, 1.5] m:
        # covers from just below floor up to platform-top + small clearance.
        # Air above 1.5 m is non-actionable for a ground robot (Mid-360 at
        # z≈0.4 can't carve those voxels free), so excluding them stops the
        # planner from chasing phantom clusters.
        self.declare_parameter("frontier_3d_z_band_min_m", -0.2)
        self.declare_parameter("frontier_3d_z_band_max_m",  1.5)
        self.frontier_3d_min_unknown_volume_m3 = max(
            0.0, float(self.get_parameter("frontier_3d_min_unknown_volume_m3").value)
        )
        self.frontier_3d_min_frontier_voxels = max(
            1, int(self.get_parameter("frontier_3d_min_frontier_voxels").value)
        )
        self.frontier_3d_border_margin_cells = max(
            0, int(self.get_parameter("frontier_3d_border_margin_cells").value)
        )
        self.frontier_3d_geodesic_voronoi = bool(
            self.get_parameter("frontier_3d_geodesic_voronoi").value
        )
        self.frontier_3d_goal_search_radius_m = max(
            0.1, float(self.get_parameter("frontier_3d_goal_search_radius_m").value)
        )
        self.frontier_3d_z_band_min_m = float(
            self.get_parameter("frontier_3d_z_band_min_m").value)
        self.frontier_3d_z_band_max_m = float(
            self.get_parameter("frontier_3d_z_band_max_m").value)

        if len(self.namespaces) != 1:
            raise ValueError(
                "cfpa2_single_robot_node requires exactly one namespace; "
                f"received {self.namespaces}"
            )
        if self.robot_namespace != self.namespaces[0]:
            self.get_logger().warn(
                "robot_namespace parameter does not match namespaces[0]; "
                f"using topic namespace '{self.namespaces[0]}'."
            )
        if self.use_shared_map:
            self.get_logger().warn(
                "cfpa2_single_robot_node ignores use_shared_map=true; "
                "single-robot planning always uses the local map."
            )
            self.use_shared_map = False
        if self.output_mode != "waypoint_coord":
            self.get_logger().warn(
                f"cfpa2_single_robot_node requires output_mode=waypoint_coord; "
                f"overriding '{self.output_mode}'."
            )
            self.output_mode = "waypoint_coord"

        # Single source of truth for exploration state; consumed by
        # exploration_metrics_logger to emit structured event lines and to
        # decide when to fire exploration_complete + cancel Nav2.
        ns = self.namespaces[0]
        self._status_pub = self.create_publisher(
            String, f"/{ns}/exploration_status", 10)
        self._last_status: str = ""
        self.declare_parameter("startup_delay_sec", 0.0)
        self.startup_delay_sec = max(
            0.0, float(self.get_parameter("startup_delay_sec").value)
        )
        self._startup_start_ns = 0

        self.declare_parameter("ramp_ascent_enabled", False)
        self.declare_parameter("ramp_ascent_goal_topic_suffix", "/ramp_ascent_goal")
        self.declare_parameter("ramp_ascent_goal_stale_sec", 2.0)
        self.declare_parameter("ramp_ascent_max_goal_distance_m", 5.0)
        self.declare_parameter("ramp_ascent_require_grid_reachable", True)
        self.declare_parameter("ramp_ascent_reachability_occ_threshold", 100)
        self.declare_parameter("ramp_ascent_exclusive", False)
        self.declare_parameter("ramp_ascent_ignore_blacklist", False)
        self.declare_parameter("ramp_ascent_switch_min_dist_m", 0.25)
        self.declare_parameter("ramp_ascent_utility", 100000.0)
        self.declare_parameter("ramp_ascent_corridor_lock_sec", 0.0)
        self.declare_parameter("ramp_ascent_lock_min_x", -1.0e9)
        self.declare_parameter("ramp_ascent_lock_max_x", 1.0e9)
        self.declare_parameter("ramp_ascent_lock_max_abs_y", 1.0e9)
        self.ramp_ascent_enabled = bool(
            self.get_parameter("ramp_ascent_enabled").value
        )
        ramp_suffix = str(
            self.get_parameter("ramp_ascent_goal_topic_suffix").value
        ).strip()
        if not ramp_suffix.startswith("/"):
            ramp_suffix = "/" + ramp_suffix if ramp_suffix else "/ramp_ascent_goal"
        self.ramp_ascent_goal_topic_suffix = ramp_suffix
        self.ramp_ascent_goal_stale_sec = max(
            0.1, float(self.get_parameter("ramp_ascent_goal_stale_sec").value)
        )
        self.ramp_ascent_max_goal_distance_m = max(
            0.1, float(self.get_parameter("ramp_ascent_max_goal_distance_m").value)
        )
        self.ramp_ascent_require_grid_reachable = bool(
            self.get_parameter("ramp_ascent_require_grid_reachable").value
        )
        self.ramp_ascent_reachability_occ_threshold = int(
            max(
                1,
                min(
                    101,
                    int(self.get_parameter("ramp_ascent_reachability_occ_threshold").value),
                ),
            )
        )
        self.ramp_ascent_exclusive = bool(
            self.get_parameter("ramp_ascent_exclusive").value
        )
        self.ramp_ascent_ignore_blacklist = bool(
            self.get_parameter("ramp_ascent_ignore_blacklist").value
        )
        self.ramp_ascent_switch_min_dist_m = max(
            0.05, float(self.get_parameter("ramp_ascent_switch_min_dist_m").value)
        )
        self.ramp_ascent_utility = float(
            self.get_parameter("ramp_ascent_utility").value
        )
        self.ramp_ascent_corridor_lock_sec = max(
            0.0, float(self.get_parameter("ramp_ascent_corridor_lock_sec").value)
        )
        self.ramp_ascent_lock_min_x = float(
            self.get_parameter("ramp_ascent_lock_min_x").value
        )
        self.ramp_ascent_lock_max_x = float(
            self.get_parameter("ramp_ascent_lock_max_x").value
        )
        self.ramp_ascent_lock_max_abs_y = max(
            0.0, float(self.get_parameter("ramp_ascent_lock_max_abs_y").value)
        )
        self._ramp_goal_by_ns: dict[str, tuple[float, float]] = {}
        self._ramp_goal_rx_ns: dict[str, int] = {}
        self._active_goal_is_ramp: dict[str, bool] = {}
        if self.ramp_ascent_enabled:
            ramp_topic = f"/{ns}{self.ramp_ascent_goal_topic_suffix}"
            self.create_subscription(
                PointStamped,
                ramp_topic,
                lambda m, n=ns: self._ramp_ascent_goal_cb(m, n),
                10,
            )
            self.get_logger().info(
                f"[{ns}] ramp ascent goal ← {ramp_topic} "
                f"(max_dist={self.ramp_ascent_max_goal_distance_m:.1f}m)"
            )

        # Pause flag set by exploration_metrics_logger via /<ns>/exploration_complete.
        # When set, _tick_impl returns early and stops publishing goals.
        self._paused: bool = False
        self.create_subscription(
            String, f"/{ns}/exploration_complete",
            self._on_exploration_complete, 10)
        self._last_3d_frontier_warn_ns = 0

        # Cross-frame frontier-cluster tracker. Without this, every tick
        # re-extracts clusters from scratch and CFPA2 cannot tell whether
        # the SAME cluster has been tried-and-failed before. Result is a
        # whack-a-mole loop where the robot reaches a cluster's projection,
        # the cluster's volume doesn't shrink (e.g. it lives in air voxels
        # above a ramp that Mid-360 can't probe), and the same goal is
        # re-issued forever. The tracker bookkeeps each cluster's volume
        # trajectory + how many goal attempts it's seen, and exposes a
        # ``non_actionable`` flag that CFPA2 uses to skip dead-end clusters.
        # Bumped from 3 → 8 and 30 → 90: with z-band filter the cluster
        # really IS shrinkable when robot reaches the right vantage, but
        # the cluster centroid projects to "ramp base" for many cycles
        # before the robot actually climbs the ramp and starts shrinking
        # the on-ramp UNK voxels. Giving up after 3 blacklists × 14 s = 42 s
        # is way too aggressive — the robot needs more rope.
        self.declare_parameter("cluster_track_max_attempts", 8)
        self.declare_parameter("cluster_track_stale_after_sec", 90.0)
        self.declare_parameter("cluster_track_shrink_pct", 0.05)
        self.declare_parameter("cluster_track_overlap_thresh", 0.30)
        self._cluster_tracker = ClusterTracker(
            match_overlap_thresh=float(
                self.get_parameter("cluster_track_overlap_thresh").value),
            shrink_thresh_pct=float(
                self.get_parameter("cluster_track_shrink_pct").value),
            stale_after_sec=float(
                self.get_parameter("cluster_track_stale_after_sec").value),
            max_attempts=int(
                self.get_parameter("cluster_track_max_attempts").value),
        )
        # Map goal world_xy → tracker_id so record_attempt can be called
        # when a goal is finally published downstream.
        self._goal_to_tracker_id: dict[tuple[float, float], int] = {}

    def _ramp_ascent_goal_cb(self, msg: PointStamped, ns: str) -> None:
        goal = (float(msg.point.x), float(msg.point.y))
        if not (math.isfinite(goal[0]) and math.isfinite(goal[1])):
            return
        self._ramp_goal_by_ns[ns] = goal
        self._ramp_goal_rx_ns[ns] = self.get_clock().now().nanoseconds

    def _ramp_ascent_goal_if_valid(
        self,
        *,
        ns: str,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        now_ns: int,
    ) -> Optional[tuple[float, float]]:
        if not getattr(self, "ramp_ascent_enabled", False):
            return None
        goal = self._ramp_goal_by_ns.get(ns)
        if goal is None or not self._ramp_ascent_goal_is_fresh(ns=ns, now_ns=now_ns):
            return None
        if ns not in self.odoms:
            return None
        if not self._goal_is_finite(goal):
            return None
        if self._distance_robot_to_goal(ns, goal) > float(self.ramp_ascent_max_goal_distance_m):
            return None
        if self._goal_too_close(ns, goal):
            return None
        if not self.ramp_ascent_ignore_blacklist and self._is_blacklisted(ns, goal, now_ns):
            return None
        ramp_occ_threshold = int(
            getattr(
                self,
                "ramp_ascent_reachability_occ_threshold",
                max(int(getattr(self, "occ_thresh", 50)), 100),
            )
        )
        ramp_dist_map = dist_map
        if (
            self.ramp_ascent_require_grid_reachable
            and ramp_occ_threshold != int(getattr(self, "occ_thresh", 50))
        ):
            odom = self.odoms.get(ns)
            if odom is None:
                return None
            ramp_dist_map = self._distance_transform(
                map_msg,
                (
                    float(odom.pose.pose.position.x),
                    float(odom.pose.pose.position.y),
                ),
                occ_threshold=ramp_occ_threshold,
                allow_unknown=False,
            )
        if self.ramp_ascent_require_grid_reachable and not self._goal_reachable(
            map_msg, ramp_dist_map, goal
        ):
            return None
        if not self._goal_has_obstacle_clearance(
            map_msg,
            goal,
            occ_threshold=ramp_occ_threshold,
        ):
            return None
        return goal

    def _ramp_ascent_goal_is_fresh(self, *, ns: str, now_ns: int) -> bool:
        if not getattr(self, "ramp_ascent_enabled", False):
            return False
        if self._ramp_goal_by_ns.get(ns) is None:
            return False
        rx_ns = self._ramp_goal_rx_ns.get(ns, 0)
        if rx_ns <= 0:
            return False
        return now_ns - rx_ns <= int(float(self.ramp_ascent_goal_stale_sec) * 1e9)

    def _ramp_ascent_corridor_lock_active(self, *, ns: str, now_ns: int) -> bool:
        if not getattr(self, "ramp_ascent_enabled", False):
            return False
        if not getattr(self, "ramp_ascent_exclusive", False):
            return False
        lock_sec = float(getattr(self, "ramp_ascent_corridor_lock_sec", 0.0))
        if lock_sec <= 0.0:
            return False
        odom = getattr(self, "odoms", {}).get(ns)
        if odom is None:
            return False
        x = float(odom.pose.pose.position.x)
        y = float(odom.pose.pose.position.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        return (
            float(getattr(self, "ramp_ascent_lock_min_x", -1.0e9))
            <= x
            <= float(getattr(self, "ramp_ascent_lock_max_x", 1.0e9))
            and abs(y) <= float(getattr(self, "ramp_ascent_lock_max_abs_y", 1.0e9))
        )

    def _ramp_goal_forces_switch(
        self,
        *,
        ns: str,
        candidate_goal: tuple[float, float],
        ramp_goal_key: tuple[int, int] | None,
    ) -> bool:
        if ramp_goal_key is None:
            return False
        if self._goal_key(candidate_goal) != ramp_goal_key:
            return False
        if not self._active_goal_is_ramp.get(ns, False):
            return True
        held = self.last_goal.get(ns)
        if held is None:
            return True
        switch_dist = max(
            0.05,
            float(
                getattr(
                    self,
                    "ramp_ascent_switch_min_dist_m",
                    min(0.35, float(getattr(self, "switch_min_dist", 0.35))),
                )
            ),
        )
        return math.hypot(candidate_goal[0] - held[0], candidate_goal[1] - held[1]) > switch_dist

    def _is_active_ramp_goal(self, ns: str, goal: tuple[float, float] | None) -> bool:
        if goal is None:
            return False
        if not getattr(self, "ramp_ascent_enabled", False):
            return False
        if not getattr(self, "_active_goal_is_ramp", {}).get(ns, False):
            return False
        ramp_goal = getattr(self, "_ramp_goal_by_ns", {}).get(ns)
        return ramp_goal is not None and self._goal_key(goal) == self._goal_key(ramp_goal)

    def _update_reached_goal_blacklist(
        self,
        ns: str,
        now_ns: int,
        map_msg: Optional[OccupancyGrid] = None,
    ) -> None:
        goal = getattr(self, "last_goal", {}).get(ns)
        if self._is_active_ramp_goal(ns, goal):
            self.reached_goal_last_key[ns] = self._goal_key(goal)
            self.reached_goal_repeat_count[ns] = 0
            return
        super()._update_reached_goal_blacklist(ns, now_ns, map_msg)

    def _startup_delay_active(self, *, now_ns: int) -> bool:
        delay_sec = float(getattr(self, "startup_delay_sec", 0.0))
        if delay_sec <= 0.0:
            return False
        start_ns = int(getattr(self, "_startup_start_ns", 0))
        if now_ns <= 0:
            return True
        if start_ns <= 0 or now_ns < start_ns:
            self._startup_start_ns = now_ns
            start_ns = now_ns
        return now_ns - start_ns < int(delay_sec * 1e9)

    def _publish_status(self, status: str) -> None:
        """Publish a state change. No-op when status unchanged (avoids spam)."""
        if status == self._last_status:
            return
        self._last_status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[exploration_status] {status}")

    def _on_reached_blacklist(self, ns: str, goal: tuple[float, float]) -> None:
        """Called from coordinator when a goal is reach-blacklisted.

        Increments the relevant ClusterTracker's attempt_count so the
        ``non_actionable`` flag can fire after max_attempts blacklists
        with no volume shrink in between.
        """
        tid = self._cluster_tracker.record_attempt(goal)
        # Unconditional log — we need to see whether the hook fires at all
        # and whether attempts actually accumulate on the right tracker.
        if tid is not None:
            snap = dict(
                (t[0], (t[1], t[2], t[3]))
                for t in self._cluster_tracker.debug_snapshot()
            )
            vol, att, since = snap.get(tid, (-1.0, -1, -1.0))
            self.get_logger().warn(
                f"[cluster_track] blacklist credited tracker id={tid} "
                f"attempts={att} last_vol={vol:.1f}m³ since_shrink={since:.1f}s"
            )
        else:
            self.get_logger().warn(
                f"[cluster_track] blacklist NOT credited (no tracker matched goal "
                f"{goal}); trackers={len(self._cluster_tracker._tracked)}"
            )

    def _on_exploration_complete(self, msg: String) -> None:
        if self._paused:
            return
        self._paused = True
        reason = msg.data or "unspecified"
        self.get_logger().warn(
            f"[exploration_complete] reason={reason} — pausing CFPA2 "
            f"goal publication. Send empty or 'resume' on /<ns>/exploration_complete "
            f"to re-enable.")
        self._publish_status("paused")

    def _extract_frontiers_with_scores(
        self, ns: str, planning_map: OccupancyGrid, now_ns: int
    ) -> tuple[list[tuple[float, float]], dict[tuple[float, float], float]]:
        if self.ig_dimension != "3d":
            return self._extract_frontiers(planning_map), {}

        voxel_entry = self.voxels_3d.get(ns)
        if voxel_entry is None:
            if now_ns - self._last_3d_frontier_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"[{ns}] ig_dimension=3d but no voxels_3d cached yet; "
                    "falling back to 2D frontier extraction."
                )
                self._last_3d_frontier_warn_ns = now_ns
            return self._extract_frontiers(planning_map), {}

        w = int(planning_map.info.width)
        h = int(planning_map.info.height)
        if w <= 0 or h <= 0 or len(planning_map.data) != w * h:
            if now_ns - self._last_3d_frontier_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"[{ns}] planning_map malformed ({w}x{h}, len={len(planning_map.data)}); "
                    "falling back to 2D frontier extraction."
                )
                self._last_3d_frontier_warn_ns = now_ns
            return self._extract_frontiers(planning_map), {}

        vs, ox, oy, oz, _nx, _ny, _nz, voxel_data = voxel_entry
        trav_grid = np.array(planning_map.data, dtype=np.int8).reshape(h, w)
        clearance_cells = int(
            np.ceil(self.cfpa2_frontier_obstacle_clearance_m / max(1e-6, float(planning_map.info.resolution)))
        )
        # Robot xy for "farthest frontier voxel" centroid selection — without
        # it, ring-shaped frontiers have their mean centroid AT the robot,
        # making the goal = current pose = no exploration.
        robot_xy_for_frontier: Optional[tuple[float, float]] = None
        if ns in self.odoms:
            od = self.odoms[ns]
            robot_xy_for_frontier = (
                float(od.pose.pose.position.x),
                float(od.pose.pose.position.y),
            )
        clusters = extract_3d_frontiers(
            voxel_data=voxel_data,
            voxel_size_m=vs,
            origin_xyz=(ox, oy, oz),
            min_unknown_volume_m3=self.frontier_3d_min_unknown_volume_m3,
            min_frontier_voxels=self.frontier_3d_min_frontier_voxels,
            border_margin_cells=self.frontier_3d_border_margin_cells,
            geodesic_voronoi=self.frontier_3d_geodesic_voronoi,
            free_value=self.free_value,
            unknown_value=self.unknown_value,
            z_band_min_m=self.frontier_3d_z_band_min_m,
            z_band_max_m=self.frontier_3d_z_band_max_m,
            robot_xy=robot_xy_for_frontier,
        )

        # Dynamic cross-frame tracking: match new clusters to existing
        # trackers via WORLD-coord AABB overlap (voxels_3d is robot-centric
        # so voxel-index AABBs would drift; tracker converts to world
        # internally using the origin we pass here), accumulate volume
        # history and attempt counts. Skip any cluster the tracker says is
        # non_actionable (max_attempts hit + no volume shrink for
        # stale_after_sec) — these are dead ends (e.g. unknown voxels in
        # robot-unobservable air above a ramp).
        tracked = self._cluster_tracker.update(
            clusters, voxel_origin_xyz=(ox, oy, oz), voxel_size_m=vs)
        actionable = [tf for tf in tracked if not tf.non_actionable]
        if len(tracked) != len(actionable) and self.verbose_logs:
            dead_ids = [tf.tracked_id for tf in tracked if tf.non_actionable]
            self.get_logger().info(
                f"[{ns}] cluster tracker dropped {len(dead_ids)} non-actionable: "
                f"ids={dead_ids[:5]}{'...' if len(dead_ids) > 5 else ''}"
            )
        # Re-key the working set so the rest of the function still loops on
        # raw Frontier3DCluster objects, but only the actionable ones.
        clusters_keep: list = [tf.cluster for tf in actionable]
        # Remember tracker id for each cluster centroid so record_attempt can
        # credit the right tracker when a goal is finally published.
        cluster_to_tracker_id: dict[tuple[float, float], int] = {
            (tf.cluster.centroid_world[0], tf.cluster.centroid_world[1]): tf.tracked_id
            for tf in actionable
        }

        raw_goals: list[tuple[float, float]] = []
        goal_scores_by_key: dict[tuple[int, int], float] = {}
        goal_by_key: dict[tuple[int, int], tuple[float, float]] = {}
        goal_tracker_by_key: dict[tuple[int, int], int] = {}
        for cluster in clusters_keep:
            goal = project_to_traversability_goal(
                centroid_xyz=cluster.centroid_world,
                trav_grid=trav_grid,
                trav_resolution_m=float(planning_map.info.resolution),
                trav_origin_xy=(
                    float(planning_map.info.origin.position.x),
                    float(planning_map.info.origin.position.y),
                ),
                search_radius_m=self.frontier_3d_goal_search_radius_m,
                free_value=self.free_value,
            )
            if goal is None:
                continue
            grid_goal = self._world_to_grid(planning_map, goal[0], goal[1])
            if grid_goal is None:
                continue
            if not self._has_frontier_obstacle_clearance(
                planning_map.data,
                grid_goal[0],
                grid_goal[1],
                w,
                h,
                clearance_cells,
            ):
                continue
            key = self._goal_key(goal)
            score = float(cluster.unknown_volume_m3)
            prev = goal_scores_by_key.get(key)
            if prev is not None and prev >= score:
                continue
            goal_scores_by_key[key] = score
            goal_by_key[key] = goal
            tid = cluster_to_tracker_id.get(
                (cluster.centroid_world[0], cluster.centroid_world[1]))
            if tid is not None:
                goal_tracker_by_key[key] = tid

        if goal_by_key:
            raw_goals = list(goal_by_key.values())
        # Stash the goal→tracker_id mapping for record_attempt() at publish time.
        self._goal_to_tracker_id = {
            goal_by_key[k]: goal_tracker_by_key[k]
            for k in goal_tracker_by_key
        }

        # _filter_dead_frontiers checks that each goal has ≥N UNKNOWN cells
        # within a 2D radius — designed for 2D mode where goals are AT a
        # FREE/UNKNOWN boundary in the trav_grid. In 3D mode the goal is the
        # ground-projection of a 3D cluster centroid, which lies INSIDE the
        # FREE area (especially after the Mid-360 blind-zone fill in
        # mapper_node — 3 m disk of forced FREE around the robot). All
        # neighbours are FREE → live_n = 0 → frontier dropped → CFPA2 reports
        # no_frontiers despite the 3D extractor finding clusters. Skip the
        # filter in 3D mode; 3D's own min_unknown_volume_m3 + min_frontier_voxels
        # already gate against trivial clusters.
        if self.ig_dimension == "3d":
            targets = raw_goals
        else:
            targets = self._filter_dead_frontiers(raw_goals, planning_map)
        goal_scores = {
            goal: goal_scores_by_key[self._goal_key(goal)]
            for goal in targets
            if self._goal_key(goal) in goal_scores_by_key
        }
        return targets, goal_scores

    def _cfpa2_single_utility_from_info_gain(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        info_gain: float,
    ) -> float:
        if getattr(self, "goal_satisfied_dist", 0.0) > 0.0 and self._goal_satisfied(
            ns,
            goal,
            map_msg,
        ):
            return -1e18
        dist_m = self._grid_path_cost_m(map_msg, dist_map, goal)
        if dist_m is None or dist_m <= 0.0:
            return -1e18
        if info_gain < 3.0:
            return -1e18
        switch_penalty = self._cfpa2_switch_penalty(ns, goal)
        momentum_bonus = self._cfpa2_momentum_bonus(ns, goal)
        return (
            (self.cfpa2_w_ig * info_gain)
            - (self.cfpa2_w_c * dist_m)
            - (self.cfpa2_w_sw * switch_penalty)
            + (self.cfpa2_w_momentum * momentum_bonus)
        )

    def _tick_impl(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ns = self.namespaces[0]

        # Honour pause flag from exploration_metrics_logger / operator.
        if self._paused:
            self._publish_status("paused")
            return

        if self._startup_delay_active(now_ns=now_ns):
            self._publish_status("settling")
            return

        if ns not in self.maps:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for map topic from: {ns}; no single-robot CFPA2 goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        planning_map = self.maps[ns]
        self._publish_coordinator_map(planning_map)
        self._publish_robot_markers(planning_map)

        if ns not in self.odoms:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for odom/nav from: {ns}; no single-robot CFPA2 goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        self._prune_blacklist(ns, now_ns)
        self._update_reached_goal_blacklist(ns, now_ns)

        targets, goal_scores = self._extract_frontiers_with_scores(ns, planning_map, now_ns)
        if getattr(self, "cfpa2_max_goal_distance_m", 0.0) > 0.0:
            targets = [goal for goal in targets if not self._goal_too_far(ns, goal)]
            goal_scores = {goal: score for goal, score in goal_scores.items() if goal in targets}

        odom = self.odoms[ns]
        dist_map = self._distance_transform(
            planning_map,
            (float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)),
            occ_threshold=getattr(
                self,
                "cfpa2_reachability_occ_threshold",
                getattr(self, "occ_thresh", 50),
            ),
            allow_unknown=bool(
                getattr(self, "cfpa2_reachability_allow_unknown", False)
            ),
        )
        dist_maps = {ns: dist_map}
        ramp_goal = self._ramp_ascent_goal_if_valid(
            ns=ns,
            map_msg=planning_map,
            dist_map=dist_map,
            now_ns=now_ns,
        )
        ramp_goal_fresh = self._ramp_ascent_goal_is_fresh(ns=ns, now_ns=now_ns)
        ramp_goal_key = self._goal_key(ramp_goal) if ramp_goal is not None else None
        ramp_corridor_lock = self._ramp_ascent_corridor_lock_active(ns=ns, now_ns=now_ns)
        if getattr(self, "ramp_ascent_exclusive", False) and (
            ramp_goal_fresh or ramp_corridor_lock
        ):
            targets = [ramp_goal] if ramp_goal is not None else []
            goal_scores = {}
        elif ramp_goal is not None and not any(
            self._goal_key(goal) == ramp_goal_key for goal in targets
        ):
            targets.append(ramp_goal)

        per_ns_targets = {ns: targets}
        if not targets:
            self._publish_status("no_frontiers")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="no_frontiers_after_extract",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                )
            return

        utilities: dict[tuple[float, float], float] = {}
        for goal in targets:
            is_ramp_goal = ramp_goal_key is not None and self._goal_key(goal) == ramp_goal_key
            if self._goal_too_close(ns, goal):
                continue
            if self._goal_too_far(ns, goal) and not is_ramp_goal:
                continue
            if self._is_blacklisted(ns, goal, now_ns):
                continue
            if is_ramp_goal:
                score = self.ramp_ascent_utility
            else:
                info_gain_override = goal_scores.get(goal)
                if info_gain_override is None:
                    score = self._cfpa2_single_utility(
                        ns=ns,
                        goal=goal,
                        map_msg=planning_map,
                        dist_map=dist_map,
                    )
                else:
                    score = self._cfpa2_single_utility_from_info_gain(
                        ns=ns,
                        goal=goal,
                        map_msg=planning_map,
                        dist_map=dist_map,
                        info_gain=info_gain_override,
                    )
            if score > -1e17:
                utilities[goal] = score

        if not utilities:
            self._publish_status("no_reachable")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_reachable_utilities",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns: 0},
                )

        candidate_goal: Optional[tuple[float, float]] = None
        assignment_score = 0.0
        forced_switch = False

        if utilities:
            candidate_goal, assignment_score = max(utilities.items(), key=lambda kv: kv[1])

        # Stop pattern: if even the best frontier is below the min utility
        # threshold, don't assign any goal — robot holds position. A verified
        # ramp ascent goal is not an information-gain frontier and carries its
        # own utility, so the same threshold is still meaningful.
        if candidate_goal is not None and assignment_score < self.cfpa2_min_utility:
            top3 = sorted(utilities.items(), key=lambda kv: kv[1], reverse=True)[:3]
            top_txt = " ".join(f"({g[0]:.1f},{g[1]:.1f})={s:.2f}" for g, s in top3)
            self.get_logger().info(
                f"HOLD [{ns}] best_u={assignment_score:.2f} < min={self.cfpa2_min_utility:.2f} "
                f"| top: {top_txt}"
            )
            candidate_goal = None

        forced_goal = self._maybe_force_cfpa2_stuck_recovery(
            ns=ns,
            now_ns=now_ns,
            utilities=utilities,
            fallback_targets=targets,
        )
        if forced_goal is not None:
            candidate_goal = forced_goal
            assignment_score = utilities.get(forced_goal, assignment_score)
            forced_switch = True

        if candidate_goal is not None and self._ramp_goal_forces_switch(
            ns=ns,
            candidate_goal=candidate_goal,
            ramp_goal_key=ramp_goal_key,
        ):
            forced_switch = True
            self._set_policy_reason(ns, "switch/ramp_ascent_override")

        goal: Optional[tuple[float, float]] = None
        held_goal = self.last_goal.get(ns)

        if candidate_goal is None:
            if held_goal is None:
                self._set_policy_reason(ns, "hold/cfpa2_no_candidate")
                self._publish_status("searching")
                if self.verbose_logs:
                    self._log_no_goal_debug(
                        now_ns=now_ns,
                        reason="cfpa2_no_assignment_published",
                        planning_map=planning_map,
                        per_ns_targets=per_ns_targets,
                        dist_maps=dist_maps,
                        utilities_sizes={ns: len(utilities)},
                        candidate_goals={},
                        per_ns_assigned={},
                    )
                return

            held_failure: Optional[str] = None
            held_blacklisted = self._is_blacklisted(ns, held_goal, now_ns)
            if not held_blacklisted and not self._is_active_ramp_goal(ns, held_goal):
                held_failure = self._held_goal_safety_failure(
                    planning_map,
                    dist_map,
                    held_goal,
                )
                if held_failure is not None:
                    self._blacklist_active_goal(
                        ns,
                        held_goal,
                        now_ns,
                        f"held_goal_{held_failure}",
                    )
                    held_blacklisted = True

            if held_blacklisted:
                fallback = self._cfpa2_best_available_goal(
                    ns=ns,
                    now_ns=now_ns,
                    utilities=utilities,
                    exclude_goal=held_goal,
                    fallback_targets=targets,
                    map_msg=planning_map,
                    dist_map=dist_map,
                )
                if fallback is not None:
                    candidate_goal = fallback
                    assignment_score = utilities.get(fallback, 0.0)
                    forced_switch = True
                    reason = (
                        f"switch/held_goal_{held_failure}"
                        if held_failure is not None
                        else "switch/cfpa2_blacklist_fallback"
                    )
                    self._set_policy_reason(ns, reason)
                else:
                    goal = self._robot_xy(ns)
                    reason = (
                        f"hold/held_goal_{held_failure}_stop"
                        if held_failure is not None
                        else "hold/cfpa2_blacklisted_stop"
                    )
                    self._set_policy_reason(ns, reason)
            else:
                goal = held_goal
                self._set_policy_reason(ns, "hold/cfpa2_keep_previous")

        if goal is None and candidate_goal is not None:
            if forced_switch:
                goal = candidate_goal
            else:
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate_goal,
                    assignment_score=assignment_score,
                    map_msg=planning_map,
                    dist_map=dist_map,
                    now_ns=now_ns,
                )

        if goal is None:
            self._publish_status("searching")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_assignment_published",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns: len(utilities)},
                    candidate_goals={ns: candidate_goal} if candidate_goal is not None else {},
                    per_ns_assigned={},
                )
            return

        self._set_active_goal(ns, goal, now_ns)
        self._active_goal_is_ramp[ns] = (
            ramp_goal_key is not None and self._goal_key(goal) == ramp_goal_key
        )
        self._publish_goal(ns, planning_map, goal)
        self._publish_status("executing")
        # Attempt-crediting is event-driven (see _on_reached_blacklist):
        # one blacklist event = one genuine "tried + reached + nothing
        # changed" engagement. Crediting per goal-publish would over-count
        # since CFPA2 republishes the same goal each tick.

        reachable = 0
        for frontier in targets:
            frontier_is_ramp_goal = (
                ramp_goal_key is not None
                and self._goal_key(frontier) == ramp_goal_key
            )
            if self._goal_too_close(ns, frontier):
                continue
            if self._goal_too_far(ns, frontier) and not frontier_is_ramp_goal:
                continue
            if self._is_blacklisted(ns, frontier, now_ns):
                continue
            if self._goal_reachable(planning_map, dist_map, frontier):
                reachable += 1

        if self.verbose_logs:
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers={ns: len(targets)},
                per_ns_reachable={ns: reachable},
                per_ns_assigned={ns: goal},
                per_ns_utilities={ns: utilities},
            )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CFPA2SingleRobotNode()
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
