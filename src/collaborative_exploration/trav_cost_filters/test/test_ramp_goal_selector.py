import math

import numpy as np

from trav_cost_filters.ramp_goal_selector import (
    GridMapGeometry,
    RampSelectorParams,
    select_approach_goal,
    advance_centerline_ascent_goal,
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
