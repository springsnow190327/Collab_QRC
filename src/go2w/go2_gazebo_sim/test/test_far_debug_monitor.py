import importlib.util
from pathlib import Path


def _load_monitor():
    repo = Path(__file__).resolve().parents[4]
    path = repo / "scripts" / "debug" / "far_debug_monitor.py"
    spec = importlib.util.spec_from_file_location("far_debug_monitor", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_far_debug_monitor_only_reports_stuck_with_active_target():
    monitor = _load_monitor()

    assert not monitor._has_active_target(None, None, None, None)
    assert monitor._has_active_target(0.0, 0.0, None, None)
    assert monitor._has_active_target(None, None, 0.0, 0.0)


def test_far_debug_monitor_treats_reached_goal_as_not_stuck():
    monitor = _load_monitor()

    assert monitor._target_satisfied(
        px=9.55,
        py=0.02,
        gx=9.60,
        gy=0.0,
        wpx=9.60,
        wpy=0.0,
        radius=0.35,
    )
    assert not monitor._target_satisfied(
        px=8.8,
        py=0.02,
        gx=9.60,
        gy=0.0,
        wpx=9.60,
        wpy=0.0,
        radius=0.35,
    )
