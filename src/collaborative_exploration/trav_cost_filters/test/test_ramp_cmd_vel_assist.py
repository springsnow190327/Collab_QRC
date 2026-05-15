import math

from trav_cost_filters.ramp_cmd_vel_assist_node import (
    RampAssistParams,
    compute_ramp_assist_twist,
)


def test_ramp_assist_commands_forward_when_verified_goal_is_ahead():
    cmd = compute_ramp_assist_twist(
        robot_x=5.7,
        robot_y=0.05,
        robot_yaw=0.0,
        goal_x=6.7,
        goal_y=0.0,
        params=RampAssistParams(),
    )

    assert cmd is not None
    assert 0.22 <= cmd.linear.x <= 0.30
    assert abs(cmd.angular.z) < 0.1


def test_ramp_assist_handles_close_approach_goal_without_entering_wheel_mode():
    cmd = compute_ramp_assist_twist(
        robot_x=5.42,
        robot_y=-0.48,
        robot_yaw=0.0,
        goal_x=5.60,
        goal_y=0.0,
        params=RampAssistParams(),
    )

    assert cmd is not None
    assert 0.22 <= cmd.linear.x <= 0.30
    assert cmd.angular.z > 0.0


def test_ramp_assist_stays_off_outside_corridor_or_behind_goal():
    params = RampAssistParams()

    assert compute_ramp_assist_twist(
        robot_x=4.0,
        robot_y=0.0,
        robot_yaw=0.0,
        goal_x=6.0,
        goal_y=0.0,
        params=params,
    ) is None
    assert compute_ramp_assist_twist(
        robot_x=6.7,
        robot_y=0.0,
        robot_yaw=0.0,
        goal_x=6.75,
        goal_y=0.0,
        params=params,
    ) is None


def test_ramp_assist_slows_forward_speed_for_large_heading_error():
    cmd = compute_ramp_assist_twist(
        robot_x=5.7,
        robot_y=0.0,
        robot_yaw=math.radians(60.0),
        goal_x=6.7,
        goal_y=0.0,
        params=RampAssistParams(),
    )

    assert cmd is not None
    assert math.isclose(cmd.linear.x, 0.22)
    assert cmd.angular.z < 0.0
