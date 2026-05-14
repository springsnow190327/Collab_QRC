"""Cross-frame frontier-cluster tracking.

The single-frame frontier extractor in :mod:`frontier_3d` returns a list of
:class:`Frontier3DCluster` objects per call, with no notion of identity. As a
result CFPA2 sees a "fresh" cluster every tick — even one that hasn't actually
changed — and keeps issuing the same goal in a loop (especially when the
cluster lives in a volume the robot physically can't observe further, e.g.
open air above a ramp where Mid-360 returns nothing).

This module adds **dynamic per-cluster bookkeeping** across frames:

* match new clusters to existing tracked clusters via AABB-overlap
* record each tracked cluster's volume history, attempt count, and the time
  of its last meaningful volume decrease ("shrink")
* flag clusters as ``non_actionable`` once the robot has tried them N times
  AND their unknown-volume has refused to shrink for T seconds — these are
  almost certainly clusters whose unknown voxels live in
  robot-unobservable space

CFPA2 consults the tracker each tick and skips non-actionable clusters when
building its candidate goal list, so the robot stops looping on dead ends and
moves on to genuinely-actionable frontiers (or correctly declares
``no_frontiers`` if nothing useful remains).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import math
import time

from .frontier_3d import Frontier3DCluster


@dataclass
class _TrackedCluster:
    """Per-cluster state retained across frames.

    AABBs are stored in WORLD-COORDINATE METRES, not voxel indices. The
    voxels_3d grid published by nvblox_frontend is robot-centric (its
    origin moves with the robot), so voxel-index AABBs from the same world
    cluster would drift across frames and break the matcher. Converting
    to world metres on ingest fixes the matching invariant.
    """

    id: int
    last_volume_m3: float
    last_aabb_world: Tuple[Tuple[float, float, float], Tuple[float, float, float]]
    last_centroid: Tuple[float, float, float]
    first_seen_time: float
    last_shrink_time: float
    last_seen_time: float
    attempt_count: int = 0

    def non_actionable(
        self, max_attempts: int, stale_after_sec: float, now: float
    ) -> bool:
        """True iff we've tried this cluster enough times AND it has refused
        to shrink for long enough. Either condition alone is not sufficient:

        * attempts ≥ N alone could just mean exploration is steady-state
        * no shrink alone could just mean the robot hasn't reached it yet

        Requiring both filters down to clusters the robot has actually
        engaged with but failed to consume.
        """
        attempts_exhausted = self.attempt_count >= max_attempts
        no_progress = (now - self.last_shrink_time) > stale_after_sec
        return attempts_exhausted and no_progress


@dataclass
class TrackedFrontier:
    """A frontier cluster plus the tracker's verdict on its actionability."""

    cluster: Frontier3DCluster
    tracked_id: int
    attempt_count: int
    age_sec: float
    sec_since_shrink: float
    non_actionable: bool


def _aabb_intersection_world(
    a: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    b: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
) -> float:
    """Metres³ intersection volume of two world-coord AABBs."""
    (a_lo, a_hi), (b_lo, b_hi) = a, b
    dx = max(0.0, min(a_hi[0], b_hi[0]) - max(a_lo[0], b_lo[0]))
    dy = max(0.0, min(a_hi[1], b_hi[1]) - max(a_lo[1], b_lo[1]))
    dz = max(0.0, min(a_hi[2], b_hi[2]) - max(a_lo[2], b_lo[2]))
    return dx * dy * dz


def _aabb_self_world(
    aabb: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
) -> float:
    lo, hi = aabb
    return max(0.0, (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]))


def _voxel_aabb_to_world(
    aabb_voxel: Tuple[Tuple[int, int, int], Tuple[int, int, int]],
    origin_xyz: Tuple[float, float, float],
    voxel_size_m: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Convert a voxel-index AABB into a world-coord (xy in metres) AABB.

    voxels_3d packs data as data[z*ny*nx + y*nx + x], so aabb_voxel uses
    (x, y, z) tuples. The world coord of cell (i,j,k) is
    (origin_x + (i+0.5)*vs, origin_y + (j+0.5)*vs, origin_z + (k+0.5)*vs).
    The AABB encloses the cell footprints, so use lo*vs and (hi+1)*vs.
    """
    lo, hi = aabb_voxel
    ox, oy, oz = origin_xyz
    lo_w = (
        ox + lo[0] * voxel_size_m,
        oy + lo[1] * voxel_size_m,
        oz + lo[2] * voxel_size_m,
    )
    hi_w = (
        ox + (hi[0] + 1) * voxel_size_m,
        oy + (hi[1] + 1) * voxel_size_m,
        oz + (hi[2] + 1) * voxel_size_m,
    )
    return (lo_w, hi_w)


class ClusterTracker:
    """Stateful tracker for frontier clusters across CFPA2 ticks.

    Parameters
    ----------
    match_overlap_thresh : float
        Min AABB-overlap fraction (overlap / smaller-AABB-volume) for a new
        cluster to be matched to an existing tracker. 0.3 is generous —
        clusters can drift somewhat between frames and still match.
    shrink_thresh_pct : float
        Volume decrease (as a fraction of previous volume) that counts as a
        "shrink" event. 0.05 = 5 %.
    stale_after_sec : float
        How long with no shrink before a cluster is eligible for the
        non-actionable flag.
    max_attempts : int
        How many goal-publishes to a cluster before non-actionable becomes
        possible.
    prune_after_sec : float
        Drop trackers whose cluster hasn't been seen for this long
        (cleanup).
    attempt_match_radius_m : float
        XY radius used to associate a published goal with a tracker (the
        nearest tracker within this radius gets the attempt credit).
    """

    def __init__(
        self,
        *,
        match_overlap_thresh: float = 0.3,
        shrink_thresh_pct: float = 0.05,
        stale_after_sec: float = 30.0,
        max_attempts: int = 3,
        prune_after_sec: float = 60.0,
        attempt_match_radius_m: float = 1.5,
        clock_fn=time.monotonic,
    ) -> None:
        self._tracked: Dict[int, _TrackedCluster] = {}
        self._next_id: int = 1
        self._match_overlap_thresh = match_overlap_thresh
        self._shrink_thresh_pct = shrink_thresh_pct
        self._stale_after_sec = stale_after_sec
        self._max_attempts = max_attempts
        self._prune_after_sec = prune_after_sec
        self._attempt_match_radius_m = attempt_match_radius_m
        self._clock = clock_fn

    def update(
        self,
        new_clusters: List[Frontier3DCluster],
        voxel_origin_xyz: Tuple[float, float, float],
        voxel_size_m: float,
    ) -> List[TrackedFrontier]:
        """Match this frame's clusters to existing trackers; return annotated.

        voxels_3d uses a robot-centric grid (origin moves with the robot),
        so cluster.aabb_voxel coords drift across frames for the same world
        cluster. We convert to world-coord AABBs (metres) before matching,
        which gives a stable cross-frame identity.
        """
        now = self._clock()

        # Pre-convert this frame's AABBs to world coords.
        new_world_aabbs: List[
            Tuple[Tuple[float, float, float], Tuple[float, float, float]]
        ] = [
            _voxel_aabb_to_world(nc.aabb_voxel, voxel_origin_xyz, voxel_size_m)
            for nc in new_clusters
        ]

        # Greedy match: for each new cluster, find the best-overlapping
        # existing tracker. Two new clusters won't compete for the same
        # tracker — first-claim wins.
        claimed: set[int] = set()
        matches: Dict[int, int] = {}  # new_idx → tracker_id
        for ni, nc in enumerate(new_clusters):
            nc_world = new_world_aabbs[ni]
            nc_vol = _aabb_self_world(nc_world)
            if nc_vol <= 0.0:
                continue
            best_id, best_score = -1, 0.0
            for tid, tc in self._tracked.items():
                if tid in claimed:
                    continue
                inter = _aabb_intersection_world(nc_world, tc.last_aabb_world)
                if inter <= 0.0:
                    continue
                tc_vol = max(1e-6, _aabb_self_world(tc.last_aabb_world))
                score = inter / min(nc_vol, tc_vol)
                if score > best_score:
                    best_score, best_id = score, tid
            if best_id >= 0 and best_score >= self._match_overlap_thresh:
                matches[ni] = best_id
                claimed.add(best_id)

        # Update trackers + build annotated output
        annotated: List[TrackedFrontier] = []
        seen_ids: set[int] = set()
        for ni, nc in enumerate(new_clusters):
            tid = matches.get(ni)
            nc_world = new_world_aabbs[ni]
            if tid is None:
                tid = self._next_id
                self._next_id += 1
                self._tracked[tid] = _TrackedCluster(
                    id=tid,
                    last_volume_m3=nc.unknown_volume_m3,
                    last_aabb_world=nc_world,
                    last_centroid=nc.centroid_world,
                    first_seen_time=now,
                    last_shrink_time=now,
                    last_seen_time=now,
                )
            tc = self._tracked[tid]
            # Volume shrink detection (compare to previous)
            if nc.unknown_volume_m3 < tc.last_volume_m3 * (1.0 - self._shrink_thresh_pct):
                tc.last_shrink_time = now
            tc.last_volume_m3 = nc.unknown_volume_m3
            tc.last_aabb_world = nc_world
            tc.last_centroid = nc.centroid_world
            tc.last_seen_time = now
            seen_ids.add(tid)
            annotated.append(
                TrackedFrontier(
                    cluster=nc,
                    tracked_id=tid,
                    attempt_count=tc.attempt_count,
                    age_sec=now - tc.first_seen_time,
                    sec_since_shrink=now - tc.last_shrink_time,
                    non_actionable=tc.non_actionable(
                        self._max_attempts, self._stale_after_sec, now
                    ),
                )
            )

        # Prune stale trackers (not seen this frame and not for prune_after_sec)
        if self._prune_after_sec > 0:
            to_drop = [
                tid for tid, tc in self._tracked.items()
                if tid not in seen_ids and (now - tc.last_seen_time) > self._prune_after_sec
            ]
            for tid in to_drop:
                del self._tracked[tid]

        return annotated

    def record_attempt(self, goal_xy: Tuple[float, float]) -> Optional[int]:
        """Credit the closest tracker's attempt_count for this goal.

        Returns the tracker id that got the credit, or None if no tracker
        was within ``attempt_match_radius_m`` of the goal.
        """
        gx, gy = goal_xy
        best_id, best_d2 = -1, self._attempt_match_radius_m * self._attempt_match_radius_m
        for tid, tc in self._tracked.items():
            dx = tc.last_centroid[0] - gx
            dy = tc.last_centroid[1] - gy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_id = tid
        if best_id >= 0:
            self._tracked[best_id].attempt_count += 1
            return best_id
        return None

    def debug_snapshot(self) -> List[Tuple[int, float, int, float]]:
        """For logging: list of (id, volume, attempts, sec_since_shrink)."""
        now = self._clock()
        return [
            (tc.id, tc.last_volume_m3, tc.attempt_count, now - tc.last_shrink_time)
            for tc in self._tracked.values()
        ]
