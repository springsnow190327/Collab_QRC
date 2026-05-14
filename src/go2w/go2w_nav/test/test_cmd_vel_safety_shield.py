import importlib.util
from pathlib import Path

from nav_msgs.msg import OccupancyGrid


_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "cmd_vel_safety_shield.py"
)
_SPEC = importlib.util.spec_from_file_location("cmd_vel_safety_shield", _SCRIPT)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)
CmdVelSafetyShield = _MOD.CmdVelSafetyShield


def _node_with_obstacle_ahead():
    node = CmdVelSafetyShield.__new__(CmdVelSafetyShield)
    node.fp_length = 0.70
    node.fp_width = 0.40
    node.occ_threshold = 50

    msg = OccupancyGrid()
    msg.info.resolution = 0.05
    msg.info.width = 60
    msg.info.height = 40
    msg.info.origin.position.x = -1.0
    msg.info.origin.position.y = -1.0
    msg.data = [0] * (msg.info.width * msg.info.height)
    gx = int((0.45 - msg.info.origin.position.x) / msg.info.resolution)
    gy = int((0.00 - msg.info.origin.position.y) / msg.info.resolution)
    msg.data[gy * msg.info.width + gx] = 100
    node._latest_map = msg
    return node


class _Logger:
    def warn(self, *_args, **_kwargs):
        pass


def _node_with_obstacle_in_rotation_sweep():
    node = _node_with_obstacle_ahead()
    node.linear_stop_enabled = True
    node.reverse_escape_enabled = False
    node.reverse_escape_speed = 0.10
    node.linear_kill_thr = 0.03
    node.angular_kill_thr = 0.10
    node.predict_horizon_sec = 0.5
    node.get_logger = lambda: _Logger()
    node._lookup_pose_xyyaw = lambda: (0.0, 0.0, 0.0)

    node._latest_map.data = [0] * (
        node._latest_map.info.width * node._latest_map.info.height
    )
    ox = node._latest_map.info.origin.position.x
    oy = node._latest_map.info.origin.position.y
    res = node._latest_map.info.resolution
    gx = int((0.25 - ox) / res)
    gy = int((0.30 - oy) / res)
    node._latest_map.data[gy * node._latest_map.info.width + gx] = 100
    return node


def _filter_node_with_obstacle_ahead():
    node = _node_with_obstacle_ahead()
    node.linear_stop_enabled = True
    node.reverse_escape_enabled = True
    node.reverse_escape_speed = 0.10
    node.linear_kill_thr = 0.03
    node.angular_kill_thr = 0.10
    node.predict_horizon_sec = 0.5
    node.get_logger = lambda: _Logger()
    node._lookup_pose_xyyaw = lambda: (0.0, 0.0, 0.0)
    return node


def test_predictive_linear_motion_detects_future_footprint_clip():
    node = _node_with_obstacle_ahead()

    assert node._motion_footprint_clips(
        0.0, 0.0, 0.0,
        vx=0.30, vy=0.0, wz=0.0,
        horizon_sec=0.5,
    )


def test_predictive_linear_motion_allows_backing_away_from_obstacle():
    node = _node_with_obstacle_ahead()

    assert not node._motion_footprint_clips(
        0.0, 0.0, 0.0,
        vx=-0.30, vy=0.0, wz=0.0,
        horizon_sec=0.5,
    )


def test_motion_guard_splits_unsafe_rotation_from_safe_translation():
    node = _node_with_obstacle_in_rotation_sweep()

    vx, vy, wz, action = node._filter_command(0.10, 0.0, 0.8)

    assert (vx, vy) == (0.10, 0.0)
    assert wz == 0.0
    assert action == "omega_killed_motion_clip"


def test_motion_guard_uses_reverse_escape_when_forward_would_clip():
    node = _filter_node_with_obstacle_ahead()

    vx, vy, wz, action = node._filter_command(0.30, 0.0, 0.0)

    assert (vx, vy, wz) == (-0.10, 0.0, 0.0)
    assert action == "reverse_escape"
