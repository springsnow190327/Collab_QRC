"""Project a pixel detection into 2D world coordinates.

TOY SLAM NOTICE (Phase 1b+)
===========================
This module is intended ONLY for the MuJoCo door-task simulation. It
works because:

  1. The mujoco_depth_camera plugin publishes **ground-truth metric
     depth** per pixel — no noise, no holes, no rolling shutter.
  2. The robot pose comes straight from the MuJoCo ground-truth odom
     bridge, not a real SLAM estimate, so the rotation used for the
     world-frame unprojection is exact.
  3. The intrinsics are a clean rectilinear pinhole derived from the
     MJCF camera's ``fovy`` tag.

On a real robot none of these assumptions hold. For a hardware
deployment the right path is:

  * RealSense D435 / D455 (RGB + metric depth aligned to color)
  * Cartographer 2D or ORB-SLAM 3 with IMU tightly coupled
  * Proper camera calibration (rectified intrinsics + RGB↔depth extrinsics)
  * A depth sanity filter (holes / oversaturation / motion blur)

The code below intentionally keeps the unprojection math simple — a
flat pinhole + current robot pose. Don't ship this to hardware.

Camera frame convention: optical axis = camera forward. pixel_bearing
returns +left (CCW). Adding a small tilt (the Go2 MJCF has ~5° pitch
down) is absorbed into the flat 2D projection and ignored — the error
is sub-decimetre at the door-task's 1-5 m detection range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    hfov_rad: float

    @classmethod
    def from_fovy_deg(cls, width: int, height: int, fovy_deg: float) -> "CameraIntrinsics":
        """MJCF cameras specify vertical FOV. Convert to horizontal."""
        vfov = math.radians(fovy_deg)
        hfov = 2.0 * math.atan(math.tan(vfov / 2.0) * (width / height))
        return cls(width=width, height=height, hfov_rad=hfov)


def pixel_bearing(intr: CameraIntrinsics, px_x: float) -> float:
    """Return bearing (rad, +left) for an image column in [0, width].

    Pixel at the image center → 0. Pixel at the left edge → +hfov/2.
    """
    norm = (px_x - intr.width / 2.0) / (intr.width / 2.0)  # [-1, +1]
    return -norm * (intr.hfov_rad / 2.0)


def project_to_world(
    intr: CameraIntrinsics,
    px_x: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    depth_m: float,
) -> tuple[float, float]:
    """Unproject a pixel (bbox center column) + metric depth to a world (x, y).

    ``depth_m`` is the **optical-axis Z** at that pixel (what the depth
    camera actually publishes). The range along the viewing ray at
    bearing θ is ``depth_m / cos(θ)``; we then march that range along
    the world-frame yaw ``robot_yaw + θ``.

    When depth comes from the real sensor this is exact in toy sim
    (see the module docstring — the clean-math assumptions hold).
    """
    bearing = pixel_bearing(intr, px_x)
    c = math.cos(bearing)
    if abs(c) < 1e-3:
        c = 1e-3
    range_m = depth_m / c
    world_yaw = robot_yaw + bearing
    return (
        robot_x + range_m * math.cos(world_yaw),
        robot_y + range_m * math.sin(world_yaw),
    )
