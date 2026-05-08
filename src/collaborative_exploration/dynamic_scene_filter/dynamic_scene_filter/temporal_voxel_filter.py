from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class DynamicFilterParams:
    voxel_size: float = 0.25
    static_min_observations: int = 3
    static_min_lifetime_sec: float = 2.0
    decay_time_sec: float = 5.0
    dynamic_obstacle_ttl_sec: float = 2.0
    max_static_velocity: float = 0.15
    min_dynamic_velocity: float = 0.35
    near_robot_ignore_radius: float = 0.4


@dataclass
class VoxelRecord:
    first_seen_time: float
    last_seen_time: float
    observation_count: int = 0
    hit_count: int = 0
    miss_count: int = 0
    last_position_centroid: Point3 = (0.0, 0.0, 0.0)
    velocity_estimate: float = 0.0
    static_score: float = 0.0
    dynamic_score: float = 0.0


@dataclass
class FilterResult:
    static_points: list[Point3] = field(default_factory=list)
    dynamic_points: list[Point3] = field(default_factory=list)
    labels: list[str] = field(default_factory=list)
    dynamic_points_filtered: int = 0
    static_points_kept: int = 0
    dynamic_filter_ratio: float = 0.0


class TemporalVoxelFilter:
    def __init__(self, params: DynamicFilterParams | None = None) -> None:
        self.params = params or DynamicFilterParams()
        self.voxels: dict[tuple[int, int, int], VoxelRecord] = {}
        self.dynamic_voxels: dict[tuple[int, int, int], float] = {}

    @property
    def dynamic_voxel_count(self) -> int:
        return len(self.dynamic_voxels)

    def voxel_id(self, point: Point3) -> tuple[int, int, int]:
        size = max(1e-6, float(self.params.voxel_size))
        return (
            math.floor(point[0] / size),
            math.floor(point[1] / size),
            math.floor(point[2] / size),
        )

    @staticmethod
    def _dist(a: Point3, b: Point3) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    def _update_record(self, key: tuple[int, int, int], point: Point3, stamp_sec: float) -> VoxelRecord:
        record = self.voxels.get(key)
        if record is None:
            velocity_hint = 0.0
            history = [
                old for old in self.voxels.values()
                if stamp_sec - old.last_seen_time > 1e-6
                and stamp_sec - old.last_seen_time <= self.params.dynamic_obstacle_ttl_sec
            ]
            if history:
                nearest = min(
                    history,
                    key=lambda old: (
                        self._dist(point, old.last_position_centroid),
                        stamp_sec - old.last_seen_time,
                    ),
                )
                dt_hint = max(1e-6, stamp_sec - nearest.last_seen_time)
                velocity_hint = self._dist(point, nearest.last_position_centroid) / dt_hint
            record = VoxelRecord(
                first_seen_time=stamp_sec,
                last_seen_time=stamp_sec,
                observation_count=0,
                last_position_centroid=point,
                velocity_estimate=velocity_hint,
            )
            self.voxels[key] = record
        dt = max(1e-6, stamp_sec - record.last_seen_time)
        velocity = self._dist(point, record.last_position_centroid) / dt
        record.velocity_estimate = 0.65 * record.velocity_estimate + 0.35 * velocity
        record.last_position_centroid = point
        record.last_seen_time = stamp_sec
        record.observation_count += 1
        record.hit_count += 1
        return record

    def _label(self, key: tuple[int, int, int], point: Point3, record: VoxelRecord, stamp_sec: float) -> str:
        lifetime = stamp_sec - record.first_seen_time
        near_robot = math.hypot(point[0], point[1]) < self.params.near_robot_ignore_radius
        is_static = (
            record.observation_count >= self.params.static_min_observations
            and lifetime >= self.params.static_min_lifetime_sec
            and record.velocity_estimate <= self.params.max_static_velocity
        )
        is_dynamic = (
            not near_robot
            and (
                record.velocity_estimate >= self.params.min_dynamic_velocity
                or (
                    record.observation_count < self.params.static_min_observations
                    and lifetime < self.params.static_min_lifetime_sec
                    and key in self.dynamic_voxels
                )
            )
        )
        if is_static:
            record.static_score += 1.0
            record.dynamic_score = max(0.0, record.dynamic_score - 0.5)
            return "static"
        if is_dynamic:
            record.dynamic_score += 1.0
            self.dynamic_voxels[key] = stamp_sec
            return "dynamic"
        return "unknown"

    def classify_points(self, points: Iterable[Point3], *, stamp_sec: float) -> FilterResult:
        self.prune(stamp_sec)
        result = FilterResult()
        for raw in points:
            point = (float(raw[0]), float(raw[1]), float(raw[2]))
            key = self.voxel_id(point)
            record = self._update_record(key, point, stamp_sec)
            label = self._label(key, point, record, stamp_sec)
            result.labels.append(label)
            if label == "dynamic":
                result.dynamic_points.append(point)
            else:
                # Unknown points are kept in the static stream so early
                # keyframes are not blank; repeated dynamic evidence moves
                # them out on later observations.
                result.static_points.append(point)
        result.dynamic_points_filtered = len(result.dynamic_points)
        result.static_points_kept = len(result.static_points)
        total = result.dynamic_points_filtered + result.static_points_kept
        result.dynamic_filter_ratio = float(result.dynamic_points_filtered) / float(max(1, total))
        return result

    def prune(self, stamp_sec: float) -> None:
        stale_voxels = [
            key for key, record in self.voxels.items()
            if stamp_sec - record.last_seen_time > self.params.decay_time_sec
        ]
        for key in stale_voxels:
            del self.voxels[key]
        stale_dynamic = [
            key for key, last_seen in self.dynamic_voxels.items()
            if stamp_sec - last_seen > self.params.dynamic_obstacle_ttl_sec
        ]
        for key in stale_dynamic:
            del self.dynamic_voxels[key]
