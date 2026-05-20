#!/usr/bin/env python3
"""Pure planning helpers for the ROS2 MTARE common-executor fallback.

This is not a route script.  It computes frontiers from an occupancy grid and
assigns autonomous exploration goals from map state, robot poses, and peer-goal
separation.  The ROS node wraps these helpers and publishes PointStamped goals.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class FrontierCluster:
    points: tuple[tuple[float, float], ...]
    centroid: tuple[float, float]
    size: int


def _idx(x: int, y: int, width: int) -> int:
    return y * width + x


def _is_free(value: int, free_threshold: int) -> bool:
    return 0 <= value <= free_threshold


def _has_unknown_neighbor(data: list[int], x: int, y: int, width: int, height: int) -> bool:
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height and data[_idx(nx, ny, width)] < 0:
                return True
    return False


def extract_frontier_clusters(
    *,
    data: Iterable[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    min_cluster_size: int = 5,
    free_threshold: int = 20,
) -> list[FrontierCluster]:
    values = list(data)
    frontier_cells: set[tuple[int, int]] = set()
    for y in range(height):
        for x in range(width):
            if _is_free(values[_idx(x, y, width)], free_threshold) and _has_unknown_neighbor(values, x, y, width, height):
                frontier_cells.add((x, y))

    clusters: list[FrontierCluster] = []
    visited: set[tuple[int, int]] = set()
    for seed in sorted(frontier_cells):
        if seed in visited:
            continue
        stack = [seed]
        visited.add(seed)
        cells: list[tuple[int, int]] = []
        while stack:
            cell = stack.pop()
            cells.append(cell)
            cx, cy = cell
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nxt = (cx + dx, cy + dy)
                    if nxt in frontier_cells and nxt not in visited:
                        visited.add(nxt)
                        stack.append(nxt)
        if len(cells) < min_cluster_size:
            continue
        points = tuple(
            (
                origin_x + (x + 0.5) * resolution,
                origin_y + (y + 0.5) * resolution,
            )
            for x, y in cells
        )
        centroid = (
            sum(p[0] for p in points) / len(points),
            sum(p[1] for p in points) / len(points),
        )
        clusters.append(FrontierCluster(points=points, centroid=centroid, size=len(points)))
    return clusters


def _candidate_points(clusters: Iterable[FrontierCluster]) -> list[tuple[float, float, float]]:
    candidates: list[tuple[float, float, float]] = []
    for cluster in clusters:
        gain = math.log1p(cluster.size)
        candidates.append((cluster.centroid[0], cluster.centroid[1], gain))
        stride = max(1, len(cluster.points) // 12)
        for point in cluster.points[::stride]:
            candidates.append((point[0], point[1], gain))
    return candidates


def assign_frontiers(
    *,
    clusters: Iterable[FrontierCluster],
    robot_positions: dict[str, tuple[float, float]],
    previous_goals: dict[str, tuple[float, float]],
    min_peer_goal_separation: float = 2.0,
    distance_weight: float = 0.8,
    information_weight: float = 3.0,
    momentum_weight: float = 0.3,
) -> dict[str, tuple[float, float]]:
    candidates = _candidate_points(clusters)
    assignments: dict[str, tuple[float, float]] = {}
    if not candidates:
        return assignments

    for ns, pos in sorted(robot_positions.items()):
        best: tuple[float, float] | None = None
        best_score = -float("inf")
        prev = previous_goals.get(ns)
        for x, y, gain in candidates:
            if any(math.hypot(x - gx, y - gy) < min_peer_goal_separation for gx, gy in assignments.values()):
                continue
            dist = math.hypot(x - pos[0], y - pos[1])
            score = information_weight * gain - distance_weight * dist
            if prev is not None:
                score -= momentum_weight * math.hypot(x - prev[0], y - prev[1])
            for peer_ns, peer_prev in previous_goals.items():
                if peer_ns != ns and math.hypot(x - peer_prev[0], y - peer_prev[1]) < min_peer_goal_separation:
                    score -= information_weight * 2.0
            if score > best_score:
                best_score = score
                best = (x, y)
        if best is not None:
            assignments[ns] = best
    return assignments
