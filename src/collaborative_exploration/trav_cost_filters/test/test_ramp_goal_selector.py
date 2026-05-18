import math

import numpy as np

from trav_cost_filters.ramp_goal_selector import (
    GridMapGeometry,
    RampSelectorParams,
    select_approach_goal,
    advance_centerline_ascent_goal,
    goal_has_min_forward_progress,
    hold_recent_verified_goal,
    select_ramp_ascent_goal,
    select_ramp_ascent_goal_from_points,
    RampGoal,
)


def _demo_like_geometry(width=80, height=40, resolution=0.1):
    return GridMapGeometry(
        origin_x=0.0,
        origin_y=-2.0,
        resolution=resolution,
        width=width,
        height=height,
    )


def test_selector_generates_uphill_goal_on_continuous_traversable_ramp():
    geom = _demo_like_geometry()
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 4.0) & (world_x <= 7.0) & (np.abs(world_y) <= 0.7)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 4.0) * math.tan(math.radians(14.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[ramp] = 0.55
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(2.0, 0.0),
        params=RampSelectorParams(max_goal_distance_m=5.0),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert goal.x > 4.0
    assert abs(goal.y) <= 0.7
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=1e-6)


def test_selector_rejects_wall_like_step_even_when_height_changes():
    geom = _demo_like_geometry()
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    wall_face = (world_x >= 4.9) & (world_x <= 5.1) & (np.abs(world_y) <= 0.8)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[wall_face] = 1.0

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[wall_face] = 0.05
    slope[wall_face] = math.radians(80.0)
    step_residual[wall_face] = 0.45

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(2.0, 0.0),
    )

    assert goal is None


def test_selector_rejects_fused_slope_when_wall_veto_is_high():
    geom = _demo_like_geometry()
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    wall_rim = (world_x >= 4.0) & (world_x <= 6.0) & (np.abs(world_y) <= 0.7)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[wall_rim] = 0.3 + (world_x[wall_rim] - 4.0) * math.tan(math.radians(14.0))

    trav_fused = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    wall_cost = np.full_like(elevation, np.nan)
    step_height = np.full_like(elevation, np.nan)
    trav_fused[wall_rim] = 0.9
    slope[wall_rim] = math.radians(14.0)
    step_residual[wall_rim] = 0.01
    wall_cost[wall_rim] = 0.8
    step_height[wall_rim] = 0.35

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav_fused,
        slope=slope,
        step_residual=step_residual,
        wall_cost=wall_cost,
        step_height=step_height,
        geometry=geom,
        robot_xy=(2.0, 0.0),
        params=RampSelectorParams(
            max_goal_distance_m=5.0,
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
            max_wall_cost=0.3,
            max_step_height_m=0.25,
        ),
    )

    assert goal is None


def test_selector_accepts_fused_ramp_when_wall_and_step_veto_are_low():
    geom = _demo_like_geometry()
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 4.0) & (world_x <= 7.0) & (np.abs(world_y) <= 0.7)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 4.0) * math.tan(math.radians(14.0))

    trav_fused = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    wall_cost = np.full_like(elevation, np.nan)
    step_height = np.full_like(elevation, np.nan)
    trav_fused[ramp] = 0.9
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01
    wall_cost[ramp] = 0.0
    step_height[ramp] = 0.08

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav_fused,
        slope=slope,
        step_residual=step_residual,
        wall_cost=wall_cost,
        step_height=step_height,
        geometry=geom,
        robot_xy=(2.0, 0.0),
        params=RampSelectorParams(
            max_goal_distance_m=5.0,
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
            max_wall_cost=0.3,
            max_step_height_m=0.25,
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert goal.x > 4.0


def test_selector_uses_corridor_and_heading_to_reject_sloped_distractor():
    geom = _demo_like_geometry(width=120, height=80, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    real_ramp = (world_x >= 5.8) & (world_x <= 8.0) & (np.abs(world_y) <= 0.8)
    distractor = (world_x >= 2.0) & (world_x <= 4.0) & (world_y >= 2.0) & (world_y <= 3.0)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[real_ramp] = (world_x[real_ramp] - 5.8) * math.tan(math.radians(14.0))
    elevation[distractor] = (world_y[distractor] - 2.0) * math.tan(math.radians(10.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    for mask, deg in ((real_ramp, 14.0), (distractor, 10.0)):
        trav[mask] = 0.55
        slope[mask] = math.radians(deg)
        step_residual[mask] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(2.0, 0.0),
        params=RampSelectorParams(
            max_goal_distance_m=6.0,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
            min_x=5.5,
            max_x=10.5,
            min_y=-1.4,
            max_y=1.4,
        ),
    )

    assert goal is not None
    assert goal.x >= 5.5
    assert abs(goal.y) <= 1.4


def test_gridmap_selector_uses_local_centerline_lookahead_on_ramp():
    geom = _demo_like_geometry(width=120, height=60, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 5.8) & (world_x <= 8.6) & (np.abs(world_y) <= 0.7)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 5.8) * math.tan(math.radians(14.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[ramp] = 0.55
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(5.8, -0.2),
        params=RampSelectorParams(
            min_goal_distance_m=0.35,
            max_goal_distance_m=1.2,
            goal_lookahead_m=0.9,
            goal_center_y=0.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-0.7,
            max_y=0.7,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert 6.45 <= goal.x <= 6.9
    assert abs(goal.y) <= 0.05
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.05)


def test_gridmap_selector_targets_ramp_entry_when_robot_is_lateral_outside_support():
    geom = _demo_like_geometry(width=120, height=70, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 5.8) & (world_x <= 8.6) & (world_y >= 0.0) & (world_y <= 1.0)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 5.8) * math.tan(math.radians(14.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[ramp] = 0.55
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(5.55, 1.65),
        params=RampSelectorParams(
            min_goal_distance_m=0.35,
            max_goal_distance_m=1.8,
            goal_lookahead_m=1.0,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
        ),
    )

    assert goal is not None
    assert goal.mode == "approach"
    assert 5.85 <= goal.x <= 6.25
    assert 0.35 <= goal.y <= 0.65
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.05)


def test_gridmap_selector_keeps_ramp_edge_robot_on_entry_goal_until_centered():
    geom = _demo_like_geometry(width=120, height=70, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 5.8) & (world_x <= 8.6) & (np.abs(world_y) <= 1.0)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 5.8) * math.tan(math.radians(14.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[ramp] = 0.55
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(5.75, -0.85),
        params=RampSelectorParams(
            min_goal_distance_m=0.35,
            max_goal_distance_m=1.8,
            goal_lookahead_m=1.0,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
        ),
    )

    assert goal is not None
    assert goal.mode == "approach"
    assert 6.05 <= goal.x <= 6.30
    assert abs(goal.y) <= 0.10


def test_gridmap_selector_uses_bounded_approach_when_laterally_far_from_support():
    geom = _demo_like_geometry(width=120, height=90, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 5.8) & (world_x <= 8.6) & (np.abs(world_y) <= 0.75)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 5.8) * math.tan(math.radians(14.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[ramp] = 0.8
    slope[ramp] = math.radians(14.0)
    step_residual[ramp] = 0.01

    robot_xy = (6.6, -2.4)
    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=robot_xy,
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_goal_distance_m=0.45,
            max_goal_distance_m=2.0,
            goal_lookahead_m=1.0,
            min_candidate_cells=30,
            min_elevation_span_m=0.12,
            min_support_length_m=0.75,
            min_support_width_m=0.45,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "approach"
    assert math.hypot(goal.x - robot_xy[0], goal.y - robot_xy[1]) <= 2.0 + 1e-6
    assert goal.y > robot_xy[1]
    assert goal.y < -0.40


def test_gridmap_selector_uses_local_ramp_patch_not_global_sloped_noise():
    geom = GridMapGeometry(
        origin_x=-5.0,
        origin_y=-8.0,
        resolution=0.1,
        width=120,
        height=120,
    )
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    local_ramp = (np.abs(world_x) <= 0.6) & (world_y >= 0.6) & (world_y <= 2.8)
    behind_noise = (np.abs(world_x) <= 2.0) & (world_y >= -7.0) & (world_y <= -4.0)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[local_ramp] = (world_y[local_ramp] - 0.6) * math.tan(math.radians(14.0))
    elevation[behind_noise] = 2.0 + (-world_y[behind_noise] - 4.0) * math.tan(
        math.radians(14.0)
    )

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    candidate = local_ramp | behind_noise
    trav[candidate] = 0.8
    slope[candidate] = math.radians(14.0)
    step_residual[candidate] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(0.0, 0.0),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_goal_distance_m=0.45,
            max_goal_distance_m=2.0,
            goal_lookahead_m=1.0,
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert abs(goal.x) <= 0.7
    assert goal.y > 0.6


def test_gridmap_selector_fits_coherent_component_when_local_horizon_has_artifacts():
    geom = _demo_like_geometry(width=130, height=100, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    ramp = (world_x >= 5.8) & (world_x <= 8.6) & (np.abs(world_y) <= 0.75)
    # A disconnected sloped patch in the same sensor horizon.  It satisfies
    # the per-cell ramp equation but belongs to a different terrain surface;
    # mixing it with the ramp makes a single global plane fit incoherent.
    artifact = (
        (world_x >= 5.8)
        & (world_x <= 8.6)
        & (world_y >= 2.0)
        & (world_y <= 2.9)
    )
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[ramp] = (world_x[ramp] - 5.8) * math.tan(math.radians(14.0))
    elevation[artifact] = (
        0.25 + (world_y[artifact] - 2.0) * math.tan(math.radians(14.0))
    )

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    candidate = ramp | artifact
    trav[candidate] = 0.8
    slope[candidate] = math.radians(14.0)
    step_residual[candidate] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(5.7, 0.1),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_goal_distance_m=0.45,
            max_goal_distance_m=2.0,
            goal_lookahead_m=1.0,
            goal_center_y=0.0,
            min_candidate_cells=30,
            min_elevation_span_m=0.12,
            min_support_length_m=0.75,
            min_support_width_m=0.45,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert 6.45 <= goal.x <= 6.85
    assert abs(goal.y) <= 0.05
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.05)


def test_gridmap_selector_rejects_patch_when_fitted_plane_is_below_ramp_slope():
    geom = _demo_like_geometry(width=90, height=60, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    shallow_patch = (world_x >= 2.0) & (world_x <= 4.2) & (np.abs(world_y) <= 0.8)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[shallow_patch] = (world_x[shallow_patch] - 2.0) * math.tan(
        math.radians(6.0)
    )

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[shallow_patch] = 0.8
    # Per-cell normals can overestimate a noisy patch, but the coherent
    # elevation plane is still below the configured ramp slope threshold.
    slope[shallow_patch] = math.radians(9.0)
    step_residual[shallow_patch] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(1.5, 0.0),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_goal_distance_m=0.45,
            max_goal_distance_m=3.0,
            goal_lookahead_m=1.0,
            min_candidate_cells=8,
            min_elevation_span_m=0.12,
        ),
    )

    assert goal is None


def test_gridmap_selector_rejects_compact_sloped_artifact_without_robot_support():
    geom = _demo_like_geometry(width=70, height=50, resolution=0.1)
    yy, xx = np.mgrid[0 : geom.height, 0 : geom.width]
    world_x = geom.origin_x + (xx + 0.5) * geom.resolution
    world_y = geom.origin_y + (yy + 0.5) * geom.resolution

    artifact = (world_x >= 1.0) & (world_x <= 1.5) & (world_y >= -0.15) & (world_y <= 0.15)
    elevation = np.full((geom.height, geom.width), np.nan, dtype=np.float32)
    elevation[artifact] = 0.35 + (world_x[artifact] - 1.0) * math.tan(math.radians(16.0))

    trav = np.full_like(elevation, np.nan)
    slope = np.full_like(elevation, np.nan)
    step_residual = np.full_like(elevation, np.nan)
    trav[artifact] = 0.8
    slope[artifact] = math.radians(16.0)
    step_residual[artifact] = 0.01

    goal = select_ramp_ascent_goal(
        elevation=elevation,
        traversability=trav,
        slope=slope,
        step_residual=step_residual,
        geometry=geom,
        robot_xy=(0.5, 0.0),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            min_goal_distance_m=0.35,
            max_goal_distance_m=2.0,
            goal_lookahead_m=1.0,
            min_candidate_cells=8,
            min_elevation_span_m=0.10,
            min_support_length_m=0.75,
            min_support_width_m=0.45,
        ),
    )

    assert goal is None


def test_approach_goal_steps_toward_ramp_foot_before_slope_is_observed():
    goal = select_approach_goal(
        robot_xy=(2.0, 0.0),
        anchor_xy=(5.6, 0.0),
        step_m=2.0,
        stop_radius_m=0.4,
    )

    assert goal is not None
    assert math.isclose(goal.x, 4.0, abs_tol=1e-6)
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)
    assert goal.mode == "approach"


def test_approach_goal_can_project_to_ramp_centerline():
    goal = select_approach_goal(
        robot_xy=(4.0, 1.0),
        anchor_xy=(5.6, 0.0),
        step_m=0.8,
        stop_radius_m=0.3,
        center_y=0.0,
    )

    assert goal is not None
    assert 4.6 <= goal.x <= 4.8
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)
    assert goal.mode == "approach"


def test_approach_goal_can_refuse_backtracking_after_ramp_foot_is_passed():
    goal = select_approach_goal(
        robot_xy=(6.4, 0.2),
        anchor_xy=(5.6, 0.0),
        step_m=1.3,
        stop_radius_m=0.3,
        center_y=0.0,
        require_anchor_ahead_x=True,
    )

    assert goal is None


def test_pointcloud_selector_fits_uphill_ramp_plane():
    xs = np.linspace(5.8, 8.0, 12)
    ys = np.linspace(-0.6, 0.6, 5)
    points = []
    for x in xs:
        for y in ys:
            z = (x - 5.8) * math.tan(math.radians(14.0))
            points.append((x, y, z))

    goal = select_ramp_ascent_goal_from_points(
        np.array(points, dtype=np.float32),
        robot_xy=(2.0, 0.0),
        params=RampSelectorParams(
            max_goal_distance_m=6.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-1.4,
            max_y=1.4,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert goal.x > 5.8
    assert abs(goal.y) <= 0.6
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.05)


def test_pointcloud_selector_uses_local_centerline_lookahead_on_ramp():
    xs = np.linspace(5.8, 8.6, 20)
    ys = np.linspace(-0.65, 0.65, 9)
    points = []
    for x in xs:
        for y in ys:
            z = (x - 5.8) * math.tan(math.radians(14.0))
            points.append((x, y, z))

    goal = select_ramp_ascent_goal_from_points(
        np.array(points, dtype=np.float32),
        robot_xy=(5.8, -0.2),
        params=RampSelectorParams(
            min_goal_distance_m=0.35,
            max_goal_distance_m=1.2,
            goal_lookahead_m=0.9,
            goal_center_y=0.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-0.7,
            max_y=0.7,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert 6.45 <= goal.x <= 6.9
    assert abs(goal.y) <= 0.05
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.05)


def test_pointcloud_selector_ignores_vertical_wall_outliers_in_ramp_corridor():
    points = []
    slope = math.tan(math.radians(14.0))

    for x in np.linspace(5.8, 8.4, 24):
        for y in np.linspace(-0.5, 0.5, 9):
            z = (x - 5.8) * slope
            points.append((x, y, z))

    for x in np.linspace(8.4, 10.0, 10):
        for y in np.linspace(-0.5, 0.5, 9):
            points.append((x, y, (8.4 - 5.8) * slope))

    # Corridor walls and box edges live in the same bounded PointCloud2 view.
    # The detector must not fit these vertical samples as part of the ramp
    # equation; only low-residual terrain cells should define the goal.
    for x in np.linspace(5.7, 10.0, 18):
        for y in (-0.68, 0.68):
            for z in np.linspace(0.0, 1.2, 9):
                points.append((x, y, z))
    for x in np.linspace(6.2, 6.6, 5):
        for y in np.linspace(-0.2, 0.2, 5):
            for z in np.linspace(0.05, 0.8, 6):
                points.append((x, y, z))

    goal = select_ramp_ascent_goal_from_points(
        np.array(points, dtype=np.float32),
        robot_xy=(5.7, 0.15),
        params=RampSelectorParams(
            min_goal_distance_m=0.35,
            max_goal_distance_m=1.2,
            goal_lookahead_m=0.8,
            goal_center_y=0.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-0.7,
            max_y=0.7,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert 6.35 <= goal.x <= 6.65
    assert abs(goal.y) <= 0.05
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.12)


def test_pointcloud_selector_accepts_short_local_ramp_patch_by_slope_equation():
    points = []
    slope = math.tan(math.radians(14.0))
    for x in np.linspace(6.65, 7.05, 8):
        for y in np.linspace(-0.45, 0.45, 5):
            points.append((x, y, (x - 5.8) * slope))

    goal = select_ramp_ascent_goal_from_points(
        np.array(points, dtype=np.float32),
        robot_xy=(5.42, -0.08),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            max_step_residual_m=0.06,
            min_candidate_cells=8,
            min_elevation_span_m=0.25,
            min_goal_distance_m=0.45,
            max_goal_distance_m=1.6,
            goal_lookahead_m=1.2,
            goal_center_y=0.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-0.7,
            max_y=0.7,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert 6.55 <= goal.x <= 6.70
    assert abs(goal.y) <= 0.05
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.10)


def test_pointcloud_selector_projects_fallback_goal_to_configured_centerline():
    points = []
    slope = math.tan(math.radians(14.0))
    for x in np.linspace(6.5, 8.2, 18):
        for y in np.linspace(0.50, 0.68, 6):
            points.append((x, y, (x - 5.8) * slope))

    goal = select_ramp_ascent_goal_from_points(
        np.array(points, dtype=np.float32),
        robot_xy=(6.0, 0.10),
        params=RampSelectorParams(
            min_slope_rad=math.radians(8.0),
            max_slope_rad=math.radians(30.0),
            max_step_residual_m=0.06,
            min_candidate_cells=8,
            min_elevation_span_m=0.25,
            min_goal_distance_m=0.45,
            max_goal_distance_m=1.6,
            goal_center_y=0.0,
            min_x=5.5,
            max_x=10.5,
            min_y=-0.7,
            max_y=0.7,
            preferred_uphill_yaw_rad=0.0,
            preferred_uphill_tolerance_rad=math.radians(35.0),
        ),
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert goal.x > 6.0
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)


def test_centerline_ascent_goal_keeps_progress_monotonic_when_evidence_regresses():
    evidence_goal = RampGoal(
        x=7.24,
        y=0.0,
        elevation_m=0.35,
        score=1.0,
        mode="ramp",
        candidate_cells=32,
        slope_rad=math.radians(14.0),
        step_residual_m=0.004,
    )

    goal = advance_centerline_ascent_goal(
        current_goal=evidence_goal,
        robot_xy=(7.13, 0.02),
        previous_goal_xy=(8.37, 0.0),
        center_y=0.0,
        min_ahead_m=1.20,
        terminal_x=9.60,
        min_x=5.5,
        max_x=10.5,
    )

    assert goal is not None
    assert goal.mode == "ramp"
    assert goal.x >= 8.37
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)
    assert math.isclose(goal.slope_rad, math.radians(14.0), rel_tol=0.01)


def test_centerline_ascent_goal_can_continue_from_last_verified_ramp_patch():
    previous_goal = RampGoal(
        x=8.37,
        y=0.0,
        elevation_m=0.58,
        score=1.0,
        mode="ramp",
        candidate_cells=61,
        slope_rad=math.radians(14.0),
        step_residual_m=0.003,
    )

    goal = advance_centerline_ascent_goal(
        current_goal=None,
        robot_xy=(8.25, 0.01),
        previous_goal_xy=(previous_goal.x, previous_goal.y),
        previous_goal=previous_goal,
        center_y=0.0,
        min_ahead_m=1.20,
        terminal_x=9.60,
        min_x=5.5,
        max_x=10.5,
    )

    assert goal is not None
    assert 9.40 <= goal.x <= 9.60
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)
    assert goal.candidate_cells == previous_goal.candidate_cells


def test_recent_verified_goal_is_republished_during_sparse_ramp_evidence():
    previous_goal = RampGoal(
        x=6.83,
        y=0.01,
        elevation_m=0.20,
        score=1.0,
        mode="ramp",
        candidate_cells=625,
        slope_rad=math.radians(14.0),
        step_residual_m=0.0,
    )

    held = hold_recent_verified_goal(
        current_goal=None,
        previous_goal=previous_goal,
        last_verified_ns=int(10.0e9),
        now_ns=int(12.5e9),
        hold_sec=4.0,
    )
    expired = hold_recent_verified_goal(
        current_goal=None,
        previous_goal=previous_goal,
        last_verified_ns=int(10.0e9),
        now_ns=int(15.0e9),
        hold_sec=4.0,
    )

    assert held == previous_goal
    assert expired is None


def test_robot_forward_filter_rejects_verified_goal_behind_current_heading():
    behind_goal = RampGoal(
        x=0.40,
        y=-1.18,
        elevation_m=0.41,
        score=1.0,
        mode="ramp",
        candidate_cells=14,
        slope_rad=math.radians(15.0),
        step_residual_m=0.0,
    )
    ahead_goal = RampGoal(
        x=6.80,
        y=0.00,
        elevation_m=0.20,
        score=1.0,
        mode="ramp",
        candidate_cells=625,
        slope_rad=math.radians(14.0),
        step_residual_m=0.0,
    )

    assert not goal_has_min_forward_progress(
        behind_goal,
        robot_xy=(1.65, -0.07),
        robot_yaw_rad=0.0,
        min_forward_m=0.15,
    )
    assert goal_has_min_forward_progress(
        ahead_goal,
        robot_xy=(5.80, -0.30),
        robot_yaw_rad=0.0,
        min_forward_m=0.15,
    )


def test_centerline_ascent_goal_stops_after_terminal_x_is_reached():
    previous_goal = RampGoal(
        x=9.60,
        y=0.0,
        elevation_m=1.0,
        score=1.0,
        mode="ramp",
        candidate_cells=40,
        slope_rad=math.radians(14.0),
        step_residual_m=0.004,
    )

    goal = advance_centerline_ascent_goal(
        current_goal=None,
        robot_xy=(9.58, 0.0),
        previous_goal_xy=(previous_goal.x, previous_goal.y),
        previous_goal=previous_goal,
        center_y=0.0,
        min_ahead_m=1.20,
        terminal_x=9.60,
        min_x=5.5,
        max_x=10.5,
    )

    assert goal is None


def test_centerline_ascent_goal_can_hold_terminal_x_for_hill_brake():
    previous_goal = RampGoal(
        x=9.60,
        y=0.0,
        elevation_m=1.0,
        score=1.0,
        mode="ramp",
        candidate_cells=40,
        slope_rad=math.radians(14.0),
        step_residual_m=0.004,
    )

    goal = advance_centerline_ascent_goal(
        current_goal=None,
        robot_xy=(9.58, 0.0),
        previous_goal_xy=(previous_goal.x, previous_goal.y),
        previous_goal=previous_goal,
        center_y=0.0,
        min_ahead_m=1.20,
        terminal_x=9.60,
        min_x=5.5,
        max_x=10.5,
        hold_terminal=True,
    )

    assert goal is not None
    assert math.isclose(goal.x, 9.60, abs_tol=1e-6)
    assert math.isclose(goal.y, 0.0, abs_tol=1e-6)
