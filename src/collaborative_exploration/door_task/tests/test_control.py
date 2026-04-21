import math

from door_task.core.control import ControlGains, compute_drive_cmd

GAINS = ControlGains()


def test_arrive_returns_zero():
    vx, wz = compute_drive_cmd(0, 0, 0, 0.05, 0.05, 0.5, GAINS)
    assert vx == 0.0 and wz == 0.0


def test_straight_ahead_drives_forward():
    vx, wz = compute_drive_cmd(0, 0, 0, 5.0, 0.0, 0.5, GAINS)
    assert vx > 0
    assert abs(wz) < 1e-6


def test_large_heading_error_turns_in_place():
    # target directly behind → heading error = pi → turn-only
    vx, wz = compute_drive_cmd(0, 0, 0, -5.0, 0.0, 0.5, GAINS)
    assert vx == 0.0
    assert abs(wz) > 0


def test_wz_clamped_to_max():
    vx, wz = compute_drive_cmd(0, 0, 0, -5.0, 0.001, 0.5, GAINS)
    assert abs(wz) <= GAINS.wz_max + 1e-9


def test_vx_request_caps_max():
    vx, _ = compute_drive_cmd(0, 0, 0, 5.0, 0.0, 0.2, GAINS)
    assert vx <= 0.2 + 1e-9
