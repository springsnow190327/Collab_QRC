import math

import numpy as np
import pytest

from trav_cost_filters.eth_traversability import (
    RampRescueThresholds,
    TraversabilityThresholds,
    classify_surface,
    plane_metrics,
    ramp_rescue_score,
)


def _ramp_points() -> np.ndarray:
    xs = np.linspace(6.2, 9.8, 7)
    ys = np.linspace(-0.6, 0.6, 5)
    pts = []
    for x in xs:
        for y in ys:
            z = 0.25 * (x - 6.0)
            pts.append((x, y, z))
    return np.asarray(pts, dtype=np.float64)


def _wall_edge_points() -> np.ndarray:
    pts = []
    for x in np.linspace(8.0, 8.2, 3):
        for y in np.linspace(-0.1, 0.1, 3):
            pts.append((x, y, 0.0))
            pts.append((x, y, 1.0))
    return np.asarray(pts, dtype=np.float64)


def test_pca_equation_marks_demo_ramp_traversable():
    metrics = plane_metrics(_ramp_points(), local_heights=np.linspace(0.05, 0.125, 3))

    assert math.degrees(metrics.slope_rad) == pytest.approx(14.0, abs=0.6)
    assert metrics.roughness_m < 1e-9
    assert metrics.step_height_m < 0.20
    assert metrics.step_residual_m < 1e-9

    verdict = classify_surface(metrics, TraversabilityThresholds())
    assert verdict.traversable
    assert verdict.score > 0.5


def test_step_equation_vetoes_vertical_wall_or_cliff():
    metrics = plane_metrics(_wall_edge_points(), local_heights=[0.0, 1.0])

    assert metrics.step_height_m >= 1.0
    verdict = classify_surface(metrics, TraversabilityThresholds())

    assert not verdict.traversable
    assert verdict.score == 0.0


def test_ramp_rescue_requires_slope_floor_before_overriding_cnn():
    thresholds = RampRescueThresholds()

    foot_transition = ramp_rescue_score(
        math.radians(6.0),
        step_residual_m=0.01,
        thresholds=thresholds,
    )
    shallow_transition = ramp_rescue_score(
        math.radians(9.0),
        step_residual_m=0.0,
        thresholds=thresholds,
    )
    clean_ramp = ramp_rescue_score(
        math.radians(14.0),
        step_residual_m=0.0,
        thresholds=thresholds,
    )

    assert foot_transition == 0.0
    assert shallow_transition < 0.60
    assert clean_ramp >= 0.95
