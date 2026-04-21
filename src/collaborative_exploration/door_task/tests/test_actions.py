import math

from door_task.core.actions import (
    STOP_ACTION,
    fmt_action,
    lower_drive_relative,
    validate_action,
)


def test_validate_drive():
    a = validate_action({"mode": "drive", "tx": 1.0, "ty": 2.0, "vx_max": 0.4})
    assert a == {"mode": "drive", "tx": 1.0, "ty": 2.0, "vx_max": 0.4}


def test_validate_unknown_collapses_to_stop():
    assert validate_action({"mode": "warp"}) == STOP_ACTION
    assert validate_action({}) == STOP_ACTION


def test_validate_drive_relative_defaults():
    a = validate_action({"mode": "drive_relative"})
    assert a["mode"] == "drive_relative"
    assert a["forward_m"] == 0.0
    assert a["heading_deg"] == 0.0


def test_lower_drive_relative_spin_in_place():
    """forward_m=0 must produce vx_max=0 but still place a non-zero phantom
    target so the heading loop has direction."""
    a = lower_drive_relative(
        {"mode": "drive_relative", "forward_m": 0.0, "heading_deg": 90.0, "vx_max": 0.5},
        x=1.0, y=2.0, yaw=0.0,
    )
    assert a["mode"] == "drive"
    assert a["vx_max"] == 0.0
    # 90° CCW from yaw=0 → +y direction → ty > y, tx ≈ x
    assert a["ty"] > 2.0
    assert math.isclose(a["tx"], 1.0, abs_tol=1e-6)


def test_lower_drive_relative_forward_keeps_vx():
    a = lower_drive_relative(
        {"mode": "drive_relative", "forward_m": 1.0, "heading_deg": 0.0, "vx_max": 0.4},
        x=0.0, y=0.0, yaw=0.0,
    )
    assert math.isclose(a["tx"], 1.0)
    assert a["vx_max"] == 0.4


def test_fmt_action_stop():
    assert fmt_action({"mode": "stop"}) == "stop"


def test_fmt_action_drive():
    s = fmt_action({"mode": "drive", "tx": 1.5, "ty": -2.0, "vx_max": 0.5})
    assert "drive" in s and "+1.5" in s
