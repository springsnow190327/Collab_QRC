import importlib.util
from collections import deque
from pathlib import Path

from geometry_msgs.msg import PoseStamped


_SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "runtime" / "stuck_watchdog.py"
_SPEC = importlib.util.spec_from_file_location("stuck_watchdog", _SCRIPT)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)
StuckWatchdog = _MOD.StuckWatchdog


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_stuck_watchdog_requests_frontier_replan_when_goal_makes_no_progress():
    node = StuckWatchdog.__new__(StuckWatchdog)
    node._ns = "robot_b"
    node.window_sec = 10.0
    node.threshold_m = 0.20
    node.goal_reached_radius = 0.50
    node.cooldown_sec = 0.0
    node._recovery_in_flight = False
    node._last_recovery_t = -100.0
    node._pose_hist = deque([(0.0, 0.0, 0.0), (10.0, 0.05, 0.0)])
    node._latest_goal = PoseStamped()
    node._latest_goal.pose.position.x = 3.0
    node._latest_goal.pose.position.y = 0.0
    node._now_sec = lambda: 10.0
    node.get_logger = lambda: _Logger()

    recovery_events = []
    node._emit_recovery = recovery_events.append
    node._trigger_recovery = lambda: None
    node._frontier_replan_pub = _Publisher()

    node._check_stuck()

    assert recovery_events == ["stuck_detected"]
    assert len(node._frontier_replan_pub.messages) == 1


def test_stuck_watchdog_suppresses_backup_for_forward_ramp_goal():
    node = StuckWatchdog.__new__(StuckWatchdog)
    node._ns = "robot"
    node.window_sec = 10.0
    node.threshold_m = 0.20
    node.goal_reached_radius = 0.50
    node.cooldown_sec = 0.0
    node._recovery_in_flight = False
    node._last_recovery_t = -100.0
    node._pose_hist = deque([(0.0, 6.45, 0.02), (10.0, 6.56, 0.01)])
    node._latest_goal = PoseStamped()
    node._latest_goal.pose.position.x = 7.22
    node._latest_goal.pose.position.y = 0.0
    node._now_sec = lambda: 10.0
    node.get_logger = lambda: _Logger()
    node.ramp_suppress_enabled = True
    node.ramp_min_x = 5.3
    node.ramp_max_x = 9.8
    node.ramp_max_abs_y = 0.9
    node.ramp_min_forward_goal_m = 0.15

    recovery_events = []
    node._emit_recovery = recovery_events.append
    node._trigger_recovery = lambda: None
    node._frontier_replan_pub = _Publisher()

    node._check_stuck()

    assert recovery_events == []
    assert len(node._frontier_replan_pub.messages) == 0
    assert len(node._pose_hist) == 0
