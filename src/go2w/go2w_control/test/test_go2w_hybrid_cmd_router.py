import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys


def _load_router():
    repo = Path(__file__).resolve().parents[4]
    path = repo / "src" / "go2w" / "go2w_control" / "scripts" / "go2w_hybrid_cmd_router.py"
    spec = importlib.util.spec_from_file_location("go2w_hybrid_cmd_router", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_ramp_force_active_requires_fresh_goal_inside_ramp_corridor():
    router = _load_router()

    assert router.ramp_force_active(
        goal_xy=(6.5, 0.0),
        goal_rx_sec=10.0,
        now_sec=10.5,
        stale_sec=1.0,
        min_x=5.3,
        max_x=9.8,
        max_abs_y=0.9,
    )
    assert not router.ramp_force_active(
        goal_xy=(6.5, 0.0),
        goal_rx_sec=8.0,
        now_sec=10.5,
        stale_sec=1.0,
        min_x=5.3,
        max_x=9.8,
        max_abs_y=0.9,
    )
    assert not router.ramp_force_active(
        goal_xy=(4.0, 0.0),
        goal_rx_sec=10.0,
        now_sec=10.5,
        stale_sec=1.0,
        min_x=5.3,
        max_x=9.8,
        max_abs_y=0.9,
    )


def test_ramp_cmd_limiter_removes_reverse_and_caps_wheel_eligible_speed():
    router = _load_router()

    vx, vy, wz = router.limit_ramp_legged_values(
        linear_x=0.38,
        linear_y=0.12,
        angular_z=0.8,
        max_vx=0.17,
        max_abs_wz=0.35,
    )
    assert vx == 0.17
    assert vy == 0.0
    assert wz == 0.35

    vx, vy, wz = router.limit_ramp_legged_values(
        linear_x=-0.10,
        linear_y=0.0,
        angular_z=-0.8,
        max_vx=0.17,
        max_abs_wz=0.35,
    )
    assert vx == 0.0
    assert vy == 0.0
    assert wz == -0.35


def test_ramp_wheel_limiter_removes_reverse_lateral_and_caps_yaw():
    router = _load_router()

    vx, vy, wz = router.limit_ramp_wheel_values(
        linear_x=0.42,
        linear_y=0.20,
        angular_z=0.75,
        max_vx=0.30,
        max_abs_wz=0.20,
    )
    assert vx == 0.30
    assert vy == 0.0
    assert wz == 0.20

    vx, vy, wz = router.limit_ramp_wheel_values(
        linear_x=-0.10,
        linear_y=-0.20,
        angular_z=-0.75,
        max_vx=0.30,
        max_abs_wz=0.20,
    )
    assert vx == 0.0
    assert vy == 0.0
    assert wz == -0.20


def test_ramp_wheel_limiter_brakes_when_cmd_is_idle_or_stale():
    router = _load_router()

    vx, vy, wz = router.limit_ramp_wheel_values_for_recency(
        linear_x=0.42,
        linear_y=0.0,
        angular_z=0.10,
        cmd_recent=False,
        cmd_idle=False,
        max_vx=0.30,
        max_abs_wz=0.20,
    )
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)

    vx, vy, wz = router.limit_ramp_wheel_values_for_recency(
        linear_x=0.0,
        linear_y=0.0,
        angular_z=0.0,
        cmd_recent=True,
        cmd_idle=True,
        max_vx=0.30,
        max_abs_wz=0.20,
    )
    assert (vx, vy, wz) == (0.0, 0.0, 0.0)


def test_hybrid_motion_config_locks_ramp_corridor_to_wheel_mode():
    repo = Path(__file__).resolve().parents[4]
    text = (
        repo
        / "src"
        / "go2w"
        / "go2w_config"
        / "config"
        / "control"
        / "go2w_hybrid_motion.yaml"
    ).read_text()

    assert "ramp_force_wheel_enabled: true" in text
    assert "ramp_goal_stale_sec: 9999.0" in text
    assert "ramp_force_max_yaw_rate_rps: 0.20" in text


def test_ramp_force_wheel_requests_wheel_even_when_cmd_is_idle_or_stale():
    router = _load_router()
    node = object.__new__(router.Go2WHybridCmdRouter)
    node.cmd_timeout_sec = 0.5
    node.thresholds = router.MotionThresholds(
        idle_linear=0.02,
        idle_lateral=0.02,
        idle_angular=0.05,
        wheel_linear=0.18,
        wheel_lateral=0.05,
        wheel_angular=0.30,
        wheel_curvature=0.45,
    )
    node._wheel_eligible_since_sec = None
    node.wheel_engage_sustain_sec = 0.5
    node.ramp_force_legged_enabled = False
    node.ramp_force_wheel_enabled = True
    node._ramp_goal_xy = (9.60, 0.0)
    node._ramp_goal_rx_sec = 10.0
    node.ramp_goal_stale_sec = 9999.0
    node.ramp_force_min_goal_x = 5.3
    node.ramp_force_max_goal_x = 9.8
    node.ramp_force_max_abs_goal_y = 0.9
    node._last_cmd_time_sec = None

    idle_cmd = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0),
        angular=SimpleNamespace(z=0.0),
    )

    mode, _ = router.Go2WHybridCmdRouter._requested_mode(node, idle_cmd, 20.0)

    assert mode == "wheel"
