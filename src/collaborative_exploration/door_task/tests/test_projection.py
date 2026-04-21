import math

from door_task.perception.projection import (
    CameraIntrinsics,
    pixel_bearing,
    project_to_world,
)


def test_intrinsics_from_fovy_landscape():
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    # HFOV > VFOV for landscape aspect
    assert intr.hfov_rad > math.radians(80.0)


def test_pixel_bearing_center_is_zero():
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    assert math.isclose(pixel_bearing(intr, intr.width / 2.0), 0.0, abs_tol=1e-9)


def test_pixel_bearing_left_edge_is_positive():
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    bearing = pixel_bearing(intr, 0.0)  # left edge
    assert bearing > 0
    assert math.isclose(bearing, intr.hfov_rad / 2.0, abs_tol=1e-6)


def test_project_centered_along_yaw():
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    # Center pixel + yaw=0 + depth 2 m → straight ahead in world +x
    wx, wy = project_to_world(intr, intr.width / 2.0, 0.0, 0.0, 0.0, 2.0)
    assert math.isclose(wx, 2.0, abs_tol=1e-6)
    assert math.isclose(wy, 0.0, abs_tol=1e-6)


def test_project_with_yaw_90deg():
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    wx, wy = project_to_world(intr, intr.width / 2.0, 1.0, 1.0, math.pi / 2.0, 2.0)
    assert math.isclose(wx, 1.0, abs_tol=1e-6)
    assert math.isclose(wy, 3.0, abs_tol=1e-6)


def test_project_off_axis_corrects_for_oblique_range():
    """Depth is camera Z. For an off-center pixel the actual range along
    the viewing ray is depth / cos(bearing) — the projection should
    reflect that and land further from the robot than depth alone."""
    intr = CameraIntrinsics.from_fovy_deg(1280, 720, 80.0)
    wx_center, wy_center = project_to_world(
        intr, intr.width / 2.0, 0.0, 0.0, 0.0, 2.0
    )
    # Left edge pixel: max positive bearing
    wx_edge, wy_edge = project_to_world(intr, 0.0, 0.0, 0.0, 0.0, 2.0)
    # Range along the ray = 2 / cos(hfov/2); hfov is > 80°, so range > 2.
    edge_range = math.hypot(wx_edge, wy_edge)
    assert edge_range > 2.01
    assert wy_edge > 0   # left edge → +y in world when yaw=0
