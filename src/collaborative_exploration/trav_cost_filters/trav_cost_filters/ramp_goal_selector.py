"""Slope-based ramp goal selection from ETH-style traversability layers."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GridMapGeometry:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int


@dataclass(frozen=True)
class RampSelectorParams:
    min_traversability: float = 0.30
    min_slope_rad: float = math.radians(5.0)
    max_slope_rad: float = math.radians(30.0)
    max_step_residual_m: float = 0.06
    min_candidate_cells: int = 8
    min_elevation_span_m: float = 0.25
    min_goal_distance_m: float = 0.70
    max_goal_distance_m: float = 4.5
    platform_min_elevation_gain_m: float = 0.45
    platform_lateral_window_m: float = 1.5
    platform_forward_window_m: float = 2.5
    preferred_uphill_yaw_rad: float | None = None
    preferred_uphill_tolerance_rad: float = math.radians(45.0)
    goal_lookahead_m: float | None = None
    goal_center_y: float | None = None
    min_x: float = -math.inf
    max_x: float = math.inf
    min_y: float = -math.inf
    max_y: float = math.inf


@dataclass(frozen=True)
class RampGoal:
    x: float
    y: float
    elevation_m: float
    score: float
    mode: str
    candidate_cells: int
    slope_rad: float
    step_residual_m: float


def select_approach_goal(
    *,
    robot_xy: tuple[float, float],
    anchor_xy: tuple[float, float],
    step_m: float,
    stop_radius_m: float,
    center_y: float | None = None,
    require_anchor_ahead_x: bool = False,
) -> RampGoal | None:
    """Short-horizon waypoint toward a ramp acquisition anchor."""

    robot = np.array([float(robot_xy[0]), float(robot_xy[1])], dtype=np.float64)
    anchor = np.array([float(anchor_xy[0]), float(anchor_xy[1])], dtype=np.float64)
    if require_anchor_ahead_x and float(anchor[0] - robot[0]) <= max(0.0, float(stop_radius_m)):
        return None
    delta = anchor - robot
    dist = float(np.linalg.norm(delta))
    if dist <= max(0.0, float(stop_radius_m)) or dist <= 1e-6:
        return None
    step = min(max(0.05, float(step_m)), dist)
    if center_y is not None and math.isfinite(float(center_y)):
        dx = float(anchor[0] - robot[0])
        if abs(dx) <= 1e-6:
            goal_x = float(anchor[0])
        else:
            goal_x = float(robot[0] + math.copysign(min(abs(dx), step), dx))
        goal_xy = np.array([goal_x, float(center_y)], dtype=np.float64)
    else:
        goal_xy = robot + delta / dist * step
    return RampGoal(
        x=float(goal_xy[0]),
        y=float(goal_xy[1]),
        elevation_m=0.0,
        score=step,
        mode="approach",
        candidate_cells=0,
        slope_rad=0.0,
        step_residual_m=0.0,
    )


def ramp_candidate_mask(
    *,
    elevation: np.ndarray,
    traversability: np.ndarray,
    slope: np.ndarray,
    step_residual: np.ndarray,
    params: RampSelectorParams = RampSelectorParams(),
) -> np.ndarray:
    """Cells satisfying the ramp equation, excluding walls and discontinuities."""

    elev = np.asarray(elevation, dtype=np.float32)
    trav = np.asarray(traversability, dtype=np.float32)
    slp = np.asarray(slope, dtype=np.float32)
    step = np.asarray(step_residual, dtype=np.float32)
    if not (elev.shape == trav.shape == slp.shape == step.shape):
        raise ValueError("elevation, traversability, slope, and step_residual must share shape")

    finite = np.isfinite(elev) & np.isfinite(trav) & np.isfinite(slp) & np.isfinite(step)
    return (
        finite
        & (trav >= float(params.min_traversability))
        & (slp >= float(params.min_slope_rad))
        & (slp <= float(params.max_slope_rad))
        & (step <= float(params.max_step_residual_m))
    )


def _cell_centres(geometry: GridMapGeometry) -> tuple[np.ndarray, np.ndarray]:
    xs = geometry.origin_x + (np.arange(geometry.width, dtype=np.float32) + 0.5) * geometry.resolution
    ys = geometry.origin_y + (np.arange(geometry.height, dtype=np.float32) + 0.5) * geometry.resolution
    return np.meshgrid(xs, ys)


def _fit_uphill_direction(xs: np.ndarray, ys: np.ndarray, zs: np.ndarray) -> tuple[np.ndarray, float] | None:
    if xs.size < 3:
        return None
    x0 = xs.astype(np.float64) - float(np.mean(xs))
    y0 = ys.astype(np.float64) - float(np.mean(ys))
    z0 = zs.astype(np.float64) - float(np.mean(zs))
    a = np.column_stack([x0, y0])
    try:
        grad, *_ = np.linalg.lstsq(a, z0, rcond=None)
    except np.linalg.LinAlgError:
        return None
    grad_norm = float(np.linalg.norm(grad))
    if grad_norm <= 1e-6:
        return None
    return grad / grad_norm, math.atan(grad_norm)


def _fit_plane(
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
) -> tuple[np.ndarray, float, float, np.ndarray] | None:
    if xs.size < 3:
        return None
    a = np.column_stack(
        [
            xs.astype(np.float64),
            ys.astype(np.float64),
            np.ones(xs.size, dtype=np.float64),
        ]
    )
    try:
        coeff, *_ = np.linalg.lstsq(a, zs.astype(np.float64), rcond=None)
    except np.linalg.LinAlgError:
        return None
    grad = coeff[:2]
    grad_norm = float(np.linalg.norm(grad))
    if grad_norm <= 1e-6:
        return None
    residuals = np.abs(a @ coeff - zs.astype(np.float64))
    return grad / grad_norm, float(coeff[2]), math.atan(grad_norm), residuals


def _clamp_finite(value: float, lower: float, upper: float) -> float:
    out = float(value)
    if math.isfinite(lower):
        out = max(out, float(lower))
    if math.isfinite(upper):
        out = min(out, float(upper))
    return out


def advance_centerline_ascent_goal(
    *,
    current_goal: RampGoal | None,
    robot_xy: tuple[float, float],
    previous_goal_xy: tuple[float, float] | None = None,
    previous_goal: RampGoal | None = None,
    center_y: float | None = None,
    min_ahead_m: float = 0.9,
    terminal_x: float | None = None,
    min_x: float = -math.inf,
    max_x: float = math.inf,
    terminal_tolerance_m: float = 0.08,
    hold_terminal: bool = False,
) -> RampGoal | None:
    """Keep a verified ramp-ascent target moving uphill on the centerline.

    The selector can temporarily see only a short local ramp patch.  Once the
    ramp equation has been verified, the commanded waypoint should not regress
    to the robot's feet; it should stay at least `min_ahead_m` in front and no
    farther than the configured terminal/platform x.
    """

    template = current_goal if current_goal is not None else previous_goal
    if template is None:
        return current_goal

    robot_x = float(robot_xy[0])
    if not math.isfinite(robot_x):
        return current_goal

    upper = float(terminal_x) if terminal_x is not None and math.isfinite(float(terminal_x)) else float(max_x)
    upper = _clamp_finite(upper, min_x, max_x)

    target_x = max(float(template.x), robot_x + max(0.0, float(min_ahead_m)))
    if previous_goal_xy is not None and math.isfinite(float(previous_goal_xy[0])):
        target_x = max(target_x, float(previous_goal_xy[0]))
    target_x = _clamp_finite(target_x, min_x, upper)

    at_terminal = target_x <= robot_x + max(0.0, float(terminal_tolerance_m))
    if at_terminal and not hold_terminal:
        return None

    if center_y is not None and math.isfinite(float(center_y)):
        target_y = _clamp_finite(float(center_y), -math.inf, math.inf)
    elif current_goal is not None:
        target_y = float(current_goal.y)
    elif previous_goal_xy is not None and math.isfinite(float(previous_goal_xy[1])):
        target_y = float(previous_goal_xy[1])
    else:
        target_y = float(template.y)

    dx = target_x - float(template.x)
    target_z = float(template.elevation_m)
    if math.isfinite(float(template.slope_rad)):
        target_z += dx * math.tan(float(template.slope_rad))

    return RampGoal(
        x=float(target_x),
        y=float(target_y),
        elevation_m=float(target_z),
        score=max(float(template.score), float(target_x - robot_x)),
        mode=str(template.mode),
        candidate_cells=int(template.candidate_cells),
        slope_rad=float(template.slope_rad),
        step_residual_m=float(template.step_residual_m),
    )


def _pointcloud_terrain_samples(
    points: np.ndarray,
    *,
    robot_xy: tuple[float, float],
    params: RampSelectorParams,
) -> np.ndarray:
    """Collapse raw PointCloud2 hits into low-residual terrain cells.

    ETH-style traversability is defined on a local height surface, not on
    every return from vertical walls.  A cell whose z-spread is too large is
    therefore treated as a discontinuity and excluded before plane fitting.
    """

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)

    robot = np.array([float(robot_xy[0]), float(robot_xy[1])], dtype=np.float64)
    lookahead = float(params.goal_lookahead_m or 0.0)
    horizon = max(1.0, float(params.max_goal_distance_m) + lookahead + 0.35)
    local = np.linalg.norm(pts[:, :2] - robot, axis=1) <= horizon
    if np.count_nonzero(local) >= int(params.min_candidate_cells):
        pts = pts[local]

    cell_size = 0.20
    xy_min = np.floor(np.min(pts[:, :2], axis=0) / cell_size) * cell_size
    ij = np.floor((pts[:, :2] - xy_min) / cell_size).astype(np.int64)
    order = np.lexsort((ij[:, 1], ij[:, 0]))
    pts_sorted = pts[order]
    ij_sorted = ij[order]

    max_robust_span = max(0.12, 2.5 * float(params.max_step_residual_m))
    max_full_span = max(0.22, 4.0 * float(params.max_step_residual_m))
    terrain: list[tuple[float, float, float]] = []
    start = 0
    while start < pts_sorted.shape[0]:
        end = start + 1
        while end < pts_sorted.shape[0] and np.array_equal(ij_sorted[end], ij_sorted[start]):
            end += 1
        cell = pts_sorted[start:end]
        z = cell[:, 2]
        if z.size >= 2:
            robust_span = float(np.percentile(z, 90.0) - np.percentile(z, 10.0))
            full_span = float(np.max(z) - np.min(z))
            if robust_span > max_robust_span or full_span > max_full_span:
                start = end
                continue
        terrain.append(
            (
                float(np.median(cell[:, 0])),
                float(np.median(cell[:, 1])),
                float(np.percentile(z, 30.0)),
            )
        )
        start = end

    if not terrain:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(terrain, dtype=np.float64)


def _score_goal(
    *,
    rel_xy: np.ndarray,
    uphill: np.ndarray,
    elevation_gain: np.ndarray,
    distance: np.ndarray,
) -> np.ndarray:
    forward = rel_xy @ uphill
    lateral = np.abs(rel_xy[:, 0] * uphill[1] - rel_xy[:, 1] * uphill[0])
    return (1.8 * forward) + (2.2 * elevation_gain) - (0.25 * distance) - (0.35 * lateral)


def select_ramp_ascent_goal(
    *,
    elevation: np.ndarray,
    traversability: np.ndarray,
    slope: np.ndarray,
    step_residual: np.ndarray,
    geometry: GridMapGeometry,
    robot_xy: tuple[float, float],
    params: RampSelectorParams = RampSelectorParams(),
) -> RampGoal | None:
    """Select a local uphill target from traversable ramp evidence.

    A candidate must satisfy the ETH-style traversability equation:

    trav_eth >= threshold, slope_min <= slope <= slope_max,
    step_residual <= threshold.

    This keeps continuous ramps actionable while rejecting vertical walls and
    box edges that have high height change but discontinuous step residual.
    """

    elev = np.asarray(elevation, dtype=np.float32)
    trav = np.asarray(traversability, dtype=np.float32)
    slp = np.asarray(slope, dtype=np.float32)
    step = np.asarray(step_residual, dtype=np.float32)
    if elev.shape != (geometry.height, geometry.width):
        raise ValueError("layer shape does not match GridMapGeometry")

    mask = ramp_candidate_mask(
        elevation=elev,
        traversability=trav,
        slope=slp,
        step_residual=step,
        params=params,
    )
    x_grid, y_grid = _cell_centres(geometry)
    mask &= (
        (x_grid >= float(params.min_x))
        & (x_grid <= float(params.max_x))
        & (y_grid >= float(params.min_y))
        & (y_grid <= float(params.max_y))
    )
    n_candidates = int(np.count_nonzero(mask))
    if n_candidates < int(params.min_candidate_cells):
        return None

    xs = x_grid[mask].astype(np.float64)
    ys = y_grid[mask].astype(np.float64)
    zs = elev[mask].astype(np.float64)
    ramp_min_z = float(np.nanmin(zs))
    ramp_max_z = float(np.nanmax(zs))
    if ramp_max_z - ramp_min_z < float(params.min_elevation_span_m):
        return None
    fit = _fit_plane(xs, ys, zs)
    if fit is None:
        return None
    uphill, intercept, fit_slope, residuals = fit
    if fit_slope < float(params.min_slope_rad) * 0.5:
        return None
    if params.preferred_uphill_yaw_rad is not None:
        preferred = np.array(
            [
                math.cos(float(params.preferred_uphill_yaw_rad)),
                math.sin(float(params.preferred_uphill_yaw_rad)),
            ],
            dtype=np.float64,
        )
        alignment = float(np.clip(uphill @ preferred, -1.0, 1.0))
        heading_error = math.acos(alignment)
        if heading_error > float(params.preferred_uphill_tolerance_rad):
            return None

    robot = np.array([float(robot_xy[0]), float(robot_xy[1])], dtype=np.float64)
    cand_xy = np.column_stack([xs, ys])
    rel = cand_xy - robot
    distance = np.linalg.norm(rel, axis=1)
    forward = rel @ uphill
    valid = (
        (distance >= float(params.min_goal_distance_m))
        & (distance <= float(params.max_goal_distance_m))
        & (forward > -0.20)
    )
    if not np.any(valid):
        return None

    lookahead = params.goal_lookahead_m
    if lookahead is not None and float(lookahead) > 0.0:
        target_forward = _clamp_finite(
            float(lookahead),
            float(params.min_goal_distance_m),
            float(params.max_goal_distance_m),
        )
        lateral_axis = np.array([-uphill[1], uphill[0]], dtype=np.float64)
        if params.goal_center_y is not None and math.isfinite(float(params.goal_center_y)):
            target_xy = robot + uphill * target_forward
            target_xy[1] = float(params.goal_center_y)
        else:
            local_lateral = rel[valid] @ lateral_axis
            target_lateral = float(np.median(local_lateral))
            target_xy = robot + uphill * target_forward + lateral_axis * target_lateral
        target_xy[0] = _clamp_finite(target_xy[0], params.min_x, params.max_x)
        target_xy[1] = _clamp_finite(target_xy[1], params.min_y, params.max_y)

        valid_xy = cand_xy[valid]
        nearest_dist = float(np.min(np.linalg.norm(valid_xy - target_xy, axis=1)))
        evidence_radius_m = max(0.35, 0.5 * target_forward)
        if nearest_dist <= evidence_radius_m:
            local = valid & (np.linalg.norm(cand_xy - target_xy, axis=1) <= evidence_radius_m)
            if np.count_nonzero(local) >= int(params.min_candidate_cells):
                step_residual = float(np.median(step[mask][local]))
            else:
                step_residual = float(np.median(step[mask]))
            target_z = float(target_xy @ (uphill * math.tan(fit_slope)) + intercept)
            return RampGoal(
                x=float(target_xy[0]),
                y=float(target_xy[1]),
                elevation_m=target_z,
                score=float(target_forward - 0.5 * nearest_dist),
                mode="ramp",
                candidate_cells=n_candidates,
                slope_rad=float(fit_slope),
                step_residual_m=step_residual,
            )

    scores = _score_goal(
        rel_xy=rel[valid],
        uphill=uphill,
        elevation_gain=zs[valid] - ramp_min_z,
        distance=distance[valid],
    )
    valid_indices = np.flatnonzero(valid)
    best_idx = int(valid_indices[int(np.argmax(scores))])
    best_mode = "ramp"

    high_trav = (
        np.isfinite(elev)
        & np.isfinite(trav)
        & np.isfinite(slp)
        & np.isfinite(step)
        & (trav >= float(params.min_traversability))
        & (step <= float(params.max_step_residual_m))
        & (slp < float(params.min_slope_rad))
        & (elev >= ramp_min_z + min(float(params.platform_min_elevation_gain_m), 0.6 * (ramp_max_z - ramp_min_z)))
    )
    if np.any(high_trav):
        top_idx = int(np.argmax((cand_xy - cand_xy.mean(axis=0)) @ uphill))
        top_xy = cand_xy[top_idx]
        high_x = x_grid[high_trav].astype(np.float64)
        high_y = y_grid[high_trav].astype(np.float64)
        high_z = elev[high_trav].astype(np.float64)
        high_xy = np.column_stack([high_x, high_y])
        from_top = high_xy - top_xy
        ahead = from_top @ uphill
        lateral = np.abs(from_top[:, 0] * uphill[1] - from_top[:, 1] * uphill[0])
        high_rel = high_xy - robot
        high_dist = np.linalg.norm(high_rel, axis=1)
        platform_valid = (
            (ahead >= -0.20)
            & (ahead <= float(params.platform_forward_window_m))
            & (lateral <= float(params.platform_lateral_window_m))
            & (high_dist >= float(params.min_goal_distance_m))
            & (high_dist <= float(params.max_goal_distance_m))
        )
        if np.any(platform_valid):
            platform_scores = _score_goal(
                rel_xy=high_rel[platform_valid],
                uphill=uphill,
                elevation_gain=high_z[platform_valid] - ramp_min_z,
                distance=high_dist[platform_valid],
            ) + 1.0
            platform_indices = np.flatnonzero(high_trav)[np.flatnonzero(platform_valid)]
            platform_best = int(platform_indices[int(np.argmax(platform_scores))])
            best_row, best_col = np.unravel_index(platform_best, elev.shape)
            best_x = float(x_grid[best_row, best_col])
            best_y = float(y_grid[best_row, best_col])
            best_z = float(elev[best_row, best_col])
            return RampGoal(
                x=best_x,
                y=best_y,
                elevation_m=best_z,
                score=float(np.max(platform_scores)),
                mode="platform",
                candidate_cells=n_candidates,
                slope_rad=float(np.nanmedian(slp[mask])),
                step_residual_m=float(np.nanmedian(step[mask])),
            )

    best_xy = np.array([float(xs[best_idx]), float(ys[best_idx])], dtype=np.float64)
    if params.goal_center_y is not None and math.isfinite(float(params.goal_center_y)):
        best_xy[1] = _clamp_finite(float(params.goal_center_y), params.min_y, params.max_y)
    best_z = float(best_xy @ (uphill * math.tan(fit_slope)) + intercept)
    return RampGoal(
        x=float(best_xy[0]),
        y=float(best_xy[1]),
        elevation_m=best_z,
        score=float(np.max(scores)),
        mode=best_mode,
        candidate_cells=n_candidates,
        slope_rad=float(np.nanmedian(slp[mask])),
        step_residual_m=float(np.nanmedian(step[mask])),
    )


def select_ramp_ascent_goal_from_points(
    points_xyz: np.ndarray,
    *,
    robot_xy: tuple[float, float],
    params: RampSelectorParams = RampSelectorParams(),
) -> RampGoal | None:
    """Fit a ramp plane directly from TF-aligned PointCloud2 samples."""

    pts = np.asarray(points_xyz, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points_xyz must be an Nx3 array")
    finite = np.all(np.isfinite(pts), axis=1)
    bounded = (
        finite
        & (pts[:, 0] >= float(params.min_x))
        & (pts[:, 0] <= float(params.max_x))
        & (pts[:, 1] >= float(params.min_y))
        & (pts[:, 1] <= float(params.max_y))
    )
    pts = pts[bounded]
    if pts.shape[0] < int(params.min_candidate_cells):
        return None
    terrain = _pointcloud_terrain_samples(pts, robot_xy=robot_xy, params=params)
    if terrain.shape[0] >= int(params.min_candidate_cells):
        pts = terrain

    fit = _fit_plane(pts[:, 0], pts[:, 1], pts[:, 2])
    if fit is None:
        return None
    uphill, intercept, fit_slope, residuals = fit
    if fit_slope < float(params.min_slope_rad) or fit_slope > float(params.max_slope_rad):
        return None
    if params.preferred_uphill_yaw_rad is not None:
        preferred = np.array(
            [
                math.cos(float(params.preferred_uphill_yaw_rad)),
                math.sin(float(params.preferred_uphill_yaw_rad)),
            ],
            dtype=np.float64,
        )
        heading_error = math.acos(float(np.clip(uphill @ preferred, -1.0, 1.0)))
        if heading_error > float(params.preferred_uphill_tolerance_rad):
            return None

    min_z = float(np.min(pts[:, 2]))
    max_z = float(np.max(pts[:, 2]))
    elevation_span = max_z - min_z
    longitudinal = pts[:, :2] @ uphill
    support_length = float(np.max(longitudinal) - np.min(longitudinal))
    local_required_span = max(
        0.06,
        min(
            float(params.min_elevation_span_m),
            0.45 * support_length * math.tan(float(params.min_slope_rad)),
        ),
    )
    if elevation_span < float(params.min_elevation_span_m) and (
        support_length < 0.30 or elevation_span < local_required_span
    ):
        return None

    robot = np.array([float(robot_xy[0]), float(robot_xy[1])], dtype=np.float64)
    xy = pts[:, :2]
    rel = xy - robot
    distance = np.linalg.norm(rel, axis=1)
    forward = rel @ uphill
    valid = (
        (distance >= float(params.min_goal_distance_m))
        & (distance <= float(params.max_goal_distance_m))
        & (forward > -0.2)
    )
    if not np.any(valid):
        return None

    lookahead = params.goal_lookahead_m
    if lookahead is not None and float(lookahead) > 0.0:
        target_forward = _clamp_finite(
            float(lookahead),
            float(params.min_goal_distance_m),
            float(params.max_goal_distance_m),
        )
        lateral_axis = np.array([-uphill[1], uphill[0]], dtype=np.float64)
        if params.goal_center_y is not None and math.isfinite(float(params.goal_center_y)):
            target_xy = robot + uphill * target_forward
            target_xy[1] = float(params.goal_center_y)
        else:
            local_lateral = rel[valid] @ lateral_axis
            target_lateral = float(np.median(local_lateral))
            target_xy = robot + uphill * target_forward + lateral_axis * target_lateral
        target_xy[0] = _clamp_finite(target_xy[0], params.min_x, params.max_x)
        target_xy[1] = _clamp_finite(target_xy[1], params.min_y, params.max_y)

        valid_xy = xy[valid]
        nearest_dist = float(np.min(np.linalg.norm(valid_xy - target_xy, axis=1)))
        evidence_radius_m = max(0.35, 0.5 * target_forward)
        if nearest_dist <= evidence_radius_m:
            local = valid & (np.linalg.norm(xy - target_xy, axis=1) <= evidence_radius_m)
            if np.count_nonzero(local) >= int(params.min_candidate_cells):
                step_residual = float(np.median(residuals[local]))
            else:
                step_residual = float(np.median(residuals[valid]))
            target_z = float((target_xy - np.array([0.0, 0.0])) @ (uphill * math.tan(fit_slope)) + intercept)
            return RampGoal(
                x=float(target_xy[0]),
                y=float(target_xy[1]),
                elevation_m=target_z,
                score=float(target_forward - 0.5 * nearest_dist),
                mode="ramp",
                candidate_cells=int(pts.shape[0]),
                slope_rad=float(fit_slope),
                step_residual_m=step_residual,
            )

    scores = _score_goal(
        rel_xy=rel[valid],
        uphill=uphill,
        elevation_gain=pts[valid, 2] - min_z,
        distance=distance[valid],
    )
    valid_indices = np.flatnonzero(valid)
    best_idx = int(valid_indices[int(np.argmax(scores))])
    best_xy = np.array([float(pts[best_idx, 0]), float(pts[best_idx, 1])], dtype=np.float64)
    if params.goal_center_y is not None and math.isfinite(float(params.goal_center_y)):
        best_xy[1] = _clamp_finite(float(params.goal_center_y), params.min_y, params.max_y)
    best_z = float(best_xy @ (uphill * math.tan(fit_slope)) + intercept)
    return RampGoal(
        x=float(best_xy[0]),
        y=float(best_xy[1]),
        elevation_m=best_z,
        score=float(np.max(scores)),
        mode="ramp",
        candidate_cells=int(pts.shape[0]),
        slope_rad=float(fit_slope),
        step_residual_m=0.0,
    )
