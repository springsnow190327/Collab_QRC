"""ETH-style traversability equations used by the 3D exploration pipeline.

The runtime filter chain computes the same quantities with grid_map filters.
This module keeps the math testable without ROS: PCA normal, slope, roughness,
and a step residual that subtracts the height change explained by a continuous
plane. The last point matters for the demo ramp: a ramp has local height change,
but it is explained by the fitted slope and must not be treated as a wall.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class TraversabilityThresholds:
    slope_crit_rad: float = math.radians(30.0)
    roughness_crit_m: float = 0.05
    step_crit_m: float = 0.20
    step_window_m: float = 0.30
    traversable_threshold: float = 0.30


@dataclass(frozen=True)
class SurfaceMetrics:
    slope_rad: float
    roughness_m: float
    step_height_m: float
    step_residual_m: float
    normal: tuple[float, float, float]


@dataclass(frozen=True)
class SurfaceVerdict:
    score: float
    traversable: bool
    slope_cost: float
    roughness_cost: float
    step_cost: float


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def plane_metrics(
    points_xyz: np.ndarray,
    *,
    local_heights: Sequence[float] | np.ndarray | None = None,
    step_window_m: float = TraversabilityThresholds.step_window_m,
) -> SurfaceMetrics:
    """Fit a local plane with PCA and compute slope/roughness/step metrics."""
    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 3:
        raise ValueError("points_xyz must be an Nx3 array with at least 3 points")

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    cov = centered.T @ centered / pts.shape[0]
    eigvals, eigvecs = np.linalg.eigh(cov)
    normal = eigvecs[:, int(np.argmin(eigvals))]
    if normal[2] < 0:
        normal = -normal

    nz = _clamp01(abs(float(normal[2])))
    slope_rad = math.acos(nz)
    residuals = centered @ normal
    denom = max(pts.shape[0] - 1, 1)
    roughness_m = math.sqrt(float(np.sum(residuals * residuals) / denom))

    heights: Iterable[float]
    heights = pts[:, 2] if local_heights is None else local_heights
    finite_heights = np.asarray(list(heights), dtype=np.float64)
    finite_heights = finite_heights[np.isfinite(finite_heights)]
    if finite_heights.size == 0:
        step_height_m = math.nan
        step_residual_m = math.nan
    else:
        step_height_m = float(finite_heights.max() - finite_heights.min())
        continuous_rise_m = abs(math.tan(slope_rad)) * step_window_m
        step_residual_m = max(0.0, step_height_m - continuous_rise_m)

    return SurfaceMetrics(
        slope_rad=slope_rad,
        roughness_m=roughness_m,
        step_height_m=step_height_m,
        step_residual_m=step_residual_m,
        normal=(float(normal[0]), float(normal[1]), float(normal[2])),
    )


def classify_surface(
    metrics: SurfaceMetrics,
    thresholds: TraversabilityThresholds = TraversabilityThresholds(),
) -> SurfaceVerdict:
    """Convert local terrain metrics into a traversability score in [0, 1]."""
    slope_cost = _clamp01(metrics.slope_rad / thresholds.slope_crit_rad)
    roughness_cost = _clamp01(metrics.roughness_m / thresholds.roughness_crit_m)
    step_cost = _clamp01(metrics.step_residual_m / thresholds.step_crit_m)

    score = _clamp01((1.0 - slope_cost) * (1.0 - roughness_cost) * (1.0 - step_cost))

    return SurfaceVerdict(
        score=score,
        traversable=score >= thresholds.traversable_threshold,
        slope_cost=slope_cost,
        roughness_cost=roughness_cost,
        step_cost=step_cost,
    )
