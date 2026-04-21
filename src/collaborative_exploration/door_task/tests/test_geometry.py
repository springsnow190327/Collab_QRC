import math

from door_task.core.geometry import clamp, wrap_to_pi, yaw_from_quat


class _Q:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


def test_clamp_bounds():
    assert clamp(5, 0, 10) == 5
    assert clamp(-1, 0, 10) == 0
    assert clamp(99, 0, 10) == 10


def test_wrap_to_pi():
    assert math.isclose(wrap_to_pi(0.0), 0.0)
    assert math.isclose(wrap_to_pi(math.pi + 0.1), -math.pi + 0.1, abs_tol=1e-9)
    assert math.isclose(wrap_to_pi(-math.pi - 0.1), math.pi - 0.1, abs_tol=1e-9)


def test_yaw_from_quat_identity():
    assert math.isclose(yaw_from_quat(_Q(0, 0, 0, 1)), 0.0)


def test_yaw_from_quat_90deg():
    half = math.sin(math.pi / 4)
    assert math.isclose(yaw_from_quat(_Q(0, 0, half, half)), math.pi / 2, abs_tol=1e-9)
