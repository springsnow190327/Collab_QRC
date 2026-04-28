#!/usr/bin/env python3
"""
rl_sar Go2W policy inference node — drop-in replacement for CHAMP.

Subscribes:
    {ns}/imu/data           sensor_msgs/Imu          (orientation + angular_velocity)
    {ns}/joint_states       sensor_msgs/JointState   (16-DoF: 12 leg + 4 wheel)
    {ns}/cmd_vel            geometry_msgs/Twist      (vx, vy, vyaw)

Publishes:
    {ns}/robot_leg_effort_controller/commands     std_msgs/Float64MultiArray  (12 effort)
    {ns}/robot_wheel_velocity_controller/commands std_msgs/Float64MultiArray  ( 4 velocity)

The PD law is:
    output_dof_pos[i]  = pos_actions_scaled[i] + default_dof_pos[i]   (legs)
    output_dof_vel[i]  = vel_actions_scaled[i]                         (wheels)
    motor_command.kp   = rl_kp                                         (20 legs / 0 wheels)
    motor_command.kd   = rl_kd                                         (0.5 across)
    tau                = kp*(target_q - q) + kd*(target_dq - dq)

For wheels (kp=0) this reduces to a velocity-tracking damper that we publish
directly to the velocity controller (no need to spend the kd torque on it
since the controller runs the velocity loop downstream). For legs we publish
the kp/kd PD torque to an effort forward_command_controller.

Joint ordering: rl_sar's policy was trained with [FR, FL, RR, RL] order
(see policy/go2w/base.yaml `joint_names`). Our /joint_states publishes in
the controller's declared order (FL, FR, RL, RR). The node reorders by
joint name on every callback — robust to topic reorderings.

State machine:
    INIT          start-up; freeze at default_dof_pos with high kp (stand)
    STAND_UP     ramp from current pose → default_dof_pos over STAND_RAMP_S
    LOCOMOTION   feed observations to the policy at policy_rate Hz
    PASSIVE      kp=kd=tau=0 (operator-triggered)

The node uses the same param semantics as rl_sar's config.yaml; defaults
are loaded from the bundled policy/go2w/base.yaml + robot_lab/config.yaml.
"""
from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

DEFAULT_RL_SAR_ROOT = "/home/hanszhu/Research/Collab_QRC/src/vendor/rl_sar"


@dataclass
class PolicyConfig:
    num_dofs: int
    num_observations: int
    observations: List[str]
    history: List[int]
    history_priority: str
    clip_obs: float
    action_scale: np.ndarray
    rl_kp: np.ndarray
    rl_kd: np.ndarray
    default_dof_pos: np.ndarray
    wheel_indices: List[int]
    lin_vel_scale: float
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    commands_scale: np.ndarray
    torque_limits: np.ndarray
    joint_names: List[str]


def _quat_rotate_inverse(q_xyzw: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate v by the inverse of quaternion q (xyzw)."""
    qx, qy, qz, qw = q_xyzw
    a = v * (2.0 * qw * qw - 1.0)
    b = np.cross(np.array([qx, qy, qz]), v) * 2.0 * qw
    c = np.array([qx, qy, qz]) * (np.dot(np.array([qx, qy, qz]), v) * 2.0)
    return a - b + c


def _load_config(rl_sar_root: str) -> PolicyConfig:
    base_yaml = os.path.join(rl_sar_root, "policy/go2w/base.yaml")
    cfg_yaml = os.path.join(rl_sar_root, "policy/go2w/robot_lab/config.yaml")
    with open(base_yaml) as f:
        base = yaml.safe_load(f)["go2w"]
    with open(cfg_yaml) as f:
        cfg = yaml.safe_load(f)["go2w/robot_lab"]
    return PolicyConfig(
        num_dofs=int(cfg["num_of_dofs"]),
        num_observations=int(cfg["num_observations"]),
        observations=list(cfg["observations"]),
        history=list(cfg.get("observations_history") or []),
        history_priority=str(cfg.get("observations_history_priority", "time")),
        clip_obs=float(cfg["clip_obs"]),
        action_scale=np.asarray(cfg["action_scale"], dtype=np.float32),
        rl_kp=np.asarray(cfg["rl_kp"], dtype=np.float32),
        rl_kd=np.asarray(cfg["rl_kd"], dtype=np.float32),
        default_dof_pos=np.asarray(cfg["default_dof_pos"], dtype=np.float32),
        wheel_indices=list(cfg["wheel_indices"]),
        lin_vel_scale=float(cfg["lin_vel_scale"]),
        ang_vel_scale=float(cfg["ang_vel_scale"]),
        dof_pos_scale=float(cfg["dof_pos_scale"]),
        dof_vel_scale=float(cfg["dof_vel_scale"]),
        commands_scale=np.asarray(cfg["commands_scale"], dtype=np.float32),
        torque_limits=np.asarray(cfg["torque_limits"], dtype=np.float32),
        joint_names=list(base["joint_names"]),
    )


# Order in our MJCF / controllers (FL, FR, RL, RR per quadrant; legs then wheels).
LEG_JOINT_ORDER_LOCAL = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]
WHEEL_JOINT_ORDER_LOCAL = ["FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"]


class State:
    INIT = "INIT"
    STAND_UP = "STAND_UP"
    LOCOMOTION = "LOCOMOTION"
    PASSIVE = "PASSIVE"


class RLLocomotionNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_locomotion_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("rl_sar_root", DEFAULT_RL_SAR_ROOT)
        self.declare_parameter("policy_path",
                               os.path.join(DEFAULT_RL_SAR_ROOT,
                                            "policy/go2w/robot_lab/policy.pt"))
        self.declare_parameter("policy_rate_hz", 50.0)
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("stand_up_seconds", 2.0)
        self.declare_parameter("stand_up_kp", 70.0)
        self.declare_parameter("stand_up_kd", 5.0)
        self.declare_parameter("auto_stand_up", True)
        self.declare_parameter("auto_locomotion_after_stand", True)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        # When cmd_vel is stale OR ‖cmd‖ < this threshold, freeze the policy
        # and hold a PD stand pose. The Go2W rl_sar policy at zero command
        # still outputs ~0.5 rad/s wheel velocity (training residual), which
        # drifts the robot at idle. Setting > 0 disables the policy in
        # idle and publishes pure stand-up PD instead.
        self.declare_parameter("idle_cmd_threshold", 0.05)
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("leg_command_topic", "robot_leg_effort_controller/commands")
        self.declare_parameter("wheel_command_topic", "robot_wheel_velocity_controller/commands")

        rl_sar_root = self.get_parameter("rl_sar_root").value
        self.cfg = _load_config(rl_sar_root)
        if self.cfg.num_dofs != 16:
            raise RuntimeError(f"Expected 16 DoF, config says {self.cfg.num_dofs}")
        if self.cfg.num_observations != 57:
            self.get_logger().warning(
                f"config num_observations={self.cfg.num_observations}, expected 57"
            )

        # ── Joint name remap (our /joint_states order → policy order) ────
        # policy order: cfg.joint_names
        # local order:  LEG_JOINT_ORDER_LOCAL + WHEEL_JOINT_ORDER_LOCAL
        local_order = LEG_JOINT_ORDER_LOCAL + WHEEL_JOINT_ORDER_LOCAL
        self.local_to_policy = np.array(
            [self.cfg.joint_names.index(n) for n in local_order],
            dtype=np.int32,
        )  # local_to_policy[local_idx] = policy_idx
        self.policy_to_local = np.empty_like(self.local_to_policy)
        for li, pi in enumerate(self.local_to_policy):
            self.policy_to_local[pi] = li
        # Local-frame index lists for slicing the published commands.
        self.local_leg_indices = list(range(0, 12))
        self.local_wheel_indices = list(range(12, 16))

        # ── Policy ────────────────────────────────────────────────────────
        policy_path = self.get_parameter("policy_path").value
        self.get_logger().info(f"loading policy: {policy_path}")
        self.model = torch.jit.load(policy_path, map_location="cpu")
        self.model.eval()
        for p in self.model.parameters():
            p.requires_grad_(False)

        # ── State buffers (policy-ordered) ────────────────────────────────
        self.last_actions = np.zeros(self.cfg.num_dofs, dtype=np.float32)
        self.last_dof_pos = self.cfg.default_dof_pos.copy()
        self.last_dof_vel = np.zeros(self.cfg.num_dofs, dtype=np.float32)
        self.last_quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.last_ang_vel = np.zeros(3, dtype=np.float32)
        self.commands = np.zeros(3, dtype=np.float32)
        self.last_cmd_vel_time: Optional[float] = None
        self.have_imu = False
        self.have_joint_states = False
        self.lock = threading.Lock()

        # ── State machine ─────────────────────────────────────────────────
        self.state = State.INIT
        self.stand_up_t0 = self.get_clock().now()
        self.stand_start_pos = self.cfg.default_dof_pos.copy()

        # ── ROS I/O ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Imu, self.get_parameter("imu_topic").value,
                                 self._on_imu, sensor_qos)
        self.create_subscription(JointState, self.get_parameter("joint_states_topic").value,
                                 self._on_joint_states, 10)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value,
                                 self._on_cmd_vel, 10)
        self.leg_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("leg_command_topic").value, 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("wheel_command_topic").value, 10)

        policy_rate = float(self.get_parameter("policy_rate_hz").value)
        self.policy_dt = 1.0 / policy_rate
        self.create_timer(self.policy_dt, self._on_tick)

        if self.get_parameter("auto_stand_up").value:
            self.state = State.STAND_UP
            self.stand_up_t0 = self.get_clock().now()
            self.get_logger().info("auto stand-up requested → STAND_UP")
        self.get_logger().info(
            f"rl_locomotion: dofs={self.cfg.num_dofs}, "
            f"obs={self.cfg.num_observations}, policy_rate={policy_rate:.1f} Hz"
        )

    # ── Subscriptions ─────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu) -> None:
        with self.lock:
            self.last_quat_xyzw = np.array([
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w,
            ], dtype=np.float32)
            self.last_ang_vel = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ], dtype=np.float32)
            self.have_imu = True

    def _on_joint_states(self, msg: JointState) -> None:
        # Build dof_pos / dof_vel in policy order, indexed by joint name.
        if not msg.name:
            return
        idx_in_msg: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}
        dof_pos_pol = np.zeros(self.cfg.num_dofs, dtype=np.float32)
        dof_vel_pol = np.zeros(self.cfg.num_dofs, dtype=np.float32)
        for pi, jn in enumerate(self.cfg.joint_names):
            mi = idx_in_msg.get(jn)
            if mi is None:
                # Joint missing — keep last value to avoid NaN spikes.
                dof_pos_pol[pi] = self.last_dof_pos[pi]
                dof_vel_pol[pi] = self.last_dof_vel[pi]
                continue
            dof_pos_pol[pi] = float(msg.position[mi]) if mi < len(msg.position) else 0.0
            dof_vel_pol[pi] = float(msg.velocity[mi]) if mi < len(msg.velocity) else 0.0
        with self.lock:
            self.last_dof_pos = dof_pos_pol
            self.last_dof_vel = dof_vel_pol
            self.have_joint_states = True

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self.lock:
            self.commands = np.array([
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.angular.z),
            ], dtype=np.float32)
            self.last_cmd_vel_time = self.get_clock().now().nanoseconds * 1e-9

    # ── Tick ──────────────────────────────────────────────────────────────
    def _on_tick(self) -> None:
        if not (self.have_imu and self.have_joint_states):
            return  # waiting for first sensor data

        if self.state == State.STAND_UP:
            self._tick_stand_up()
            return
        if self.state == State.LOCOMOTION:
            self._tick_locomotion()
            return
        if self.state == State.PASSIVE:
            self._publish_zero()
            return

    def _tick_stand_up(self) -> None:
        ramp = float(self.get_parameter("stand_up_seconds").value)
        kp = float(self.get_parameter("stand_up_kp").value)
        kd = float(self.get_parameter("stand_up_kd").value)
        with self.lock:
            q = self.last_dof_pos.copy()
            dq = self.last_dof_vel.copy()
        # Latch start pose AND reset t0 on the first stand-up tick that
        # actually has sensor data — otherwise alpha is computed from the
        # constructor's wall-clock t0 and ramps to 1.0 immediately.
        if not hasattr(self, "_stand_start_set"):
            self.stand_start_pos = q.copy()
            self.stand_up_t0 = self.get_clock().now()
            self._stand_start_set = True
        t = (self.get_clock().now() - self.stand_up_t0).nanoseconds * 1e-9
        alpha = max(0.0, min(1.0, t / max(ramp, 1e-3)))
        target_q = (1.0 - alpha) * self.stand_start_pos + alpha * self.cfg.default_dof_pos
        # PD legs only, wheels passive (kd damping).
        tau_pol = kp * (target_q - q) - kd * dq
        # Wheels: zero target torque (just PD damping is fine to keep them still).
        tau_pol[self.cfg.wheel_indices] = -kd * dq[self.cfg.wheel_indices]
        tau_pol = np.clip(tau_pol, -self.cfg.torque_limits, self.cfg.torque_limits)
        self._publish_outputs(tau_pol, wheel_vel_targets_pol=np.zeros(4, dtype=np.float32))
        if alpha >= 1.0:
            if self.get_parameter("auto_locomotion_after_stand").value:
                self.state = State.LOCOMOTION
                self.get_logger().info("STAND_UP complete → LOCOMOTION")
            else:
                # Hold pose until commanded.
                pass

    def _tick_locomotion(self) -> None:
        with self.lock:
            quat = self.last_quat_xyzw.copy()
            ang_vel = self.last_ang_vel.copy()
            dof_pos = self.last_dof_pos.copy()
            dof_vel = self.last_dof_vel.copy()
            cmd = self.commands.copy()
            last_cmd_t = self.last_cmd_vel_time
            last_actions = self.last_actions.copy()
        # cmd_vel watchdog — zero target if stale or never received.
        timeout = float(self.get_parameter("cmd_vel_timeout_sec").value)
        if last_cmd_t is None:
            cmd = np.zeros(3, dtype=np.float32)
        elif timeout > 0:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if now_s - last_cmd_t > timeout:
                cmd = np.zeros(3, dtype=np.float32)
        # Idle freeze: at near-zero command the rl_sar policy still emits
        # residual wheel velocity. Hold a PD stand pose instead of running
        # the policy in this regime.
        idle_thresh = float(self.get_parameter("idle_cmd_threshold").value)
        if idle_thresh > 0 and float(np.max(np.abs(cmd))) < idle_thresh:
            self._tick_idle_hold(dof_pos, dof_vel)
            return

        # ── Build observation in policy order (57-d) ─────────────────────
        gravity_vec = _quat_rotate_inverse(
            quat, np.array([0.0, 0.0, -1.0], dtype=np.float32)
        ).astype(np.float32)
        ang_vel_scaled = ang_vel * self.cfg.ang_vel_scale
        cmd_scaled = cmd * self.cfg.commands_scale
        dof_pos_rel = (dof_pos - self.cfg.default_dof_pos) * self.cfg.dof_pos_scale
        dof_pos_rel[self.cfg.wheel_indices] = 0.0  # wheels always 0 in obs
        dof_vel_scaled = dof_vel * self.cfg.dof_vel_scale

        obs_parts: List[np.ndarray] = []
        for term in self.cfg.observations:
            if term == "ang_vel":
                obs_parts.append(ang_vel_scaled)
            elif term == "gravity_vec":
                obs_parts.append(gravity_vec)
            elif term == "commands":
                obs_parts.append(cmd_scaled)
            elif term == "dof_pos":
                obs_parts.append(dof_pos_rel)
            elif term == "dof_vel":
                obs_parts.append(dof_vel_scaled)
            elif term == "actions":
                obs_parts.append(last_actions)
            elif term == "lin_vel":
                # Not available in MuJoCo without ground-truth — zero out.
                obs_parts.append(np.zeros(3, dtype=np.float32))
            else:
                self.get_logger().warning(f"unknown obs term: {term}; using zeros")
                obs_parts.append(np.zeros(3, dtype=np.float32))
        obs = np.concatenate(obs_parts).astype(np.float32)
        if obs.size != self.cfg.num_observations:
            self.get_logger().warning(
                f"obs size mismatch: built {obs.size}, expected {self.cfg.num_observations}"
            )
        obs = np.clip(obs, -self.cfg.clip_obs, self.cfg.clip_obs)

        # ── Forward ──────────────────────────────────────────────────────
        with torch.no_grad():
            actions = self.model.forward(
                torch.from_numpy(obs).unsqueeze(0)
            ).squeeze(0).cpu().numpy().astype(np.float32)
        # Policy outputs unclamped raw actions; rl_sar clamps with config bounds.
        # We rely on torque_limits clamp downstream.

        # ── Build commanded targets (policy-ordered) ─────────────────────
        actions_scaled = actions * self.cfg.action_scale
        pos_actions_scaled = actions_scaled.copy()
        pos_actions_scaled[self.cfg.wheel_indices] = 0.0
        vel_actions_scaled = np.zeros_like(actions_scaled)
        vel_actions_scaled[self.cfg.wheel_indices] = actions_scaled[self.cfg.wheel_indices]

        target_q_pol = pos_actions_scaled + self.cfg.default_dof_pos
        target_dq_pol = vel_actions_scaled
        # tau = kp*(target_q - q) + kd*(target_dq - dq)   (rl_sar formula:
        # rl_kp*(all_actions_scaled + default - q) - rl_kd*dq, equivalent
        # because target_dq for non-wheels is 0 and rl_kp for wheels is 0).
        tau_pol = (
            self.cfg.rl_kp * (target_q_pol - dof_pos)
            + self.cfg.rl_kd * (target_dq_pol - dof_vel)
        )
        tau_pol = np.clip(tau_pol, -self.cfg.torque_limits, self.cfg.torque_limits)

        with self.lock:
            self.last_actions = actions

        self._publish_outputs(tau_pol, wheel_vel_targets_pol=target_dq_pol[self.cfg.wheel_indices])

    def _tick_idle_hold(self, dof_pos: np.ndarray, dof_vel: np.ndarray) -> None:
        """Hold default pose with stand-up PD; zero wheel velocity."""
        kp = float(self.get_parameter("stand_up_kp").value)
        kd = float(self.get_parameter("stand_up_kd").value)
        target_q = self.cfg.default_dof_pos
        tau_pol = kp * (target_q - dof_pos) - kd * dof_vel
        # Wheels: pure damping (no propulsion).
        for i in self.cfg.wheel_indices:
            tau_pol[i] = -kd * dof_vel[i]
        tau_pol = np.clip(tau_pol, -self.cfg.torque_limits, self.cfg.torque_limits)
        self._publish_outputs(tau_pol, wheel_vel_targets_pol=np.zeros(4, dtype=np.float32))

    # ── Output publishing ─────────────────────────────────────────────────
    def _publish_outputs(self,
                        tau_pol: np.ndarray,
                        wheel_vel_targets_pol: np.ndarray) -> None:
        # tau_pol: 16 in policy order. Take legs only (non-wheel indices in
        # policy order are 0..11) → reorder to LEG_JOINT_ORDER_LOCAL.
        tau_local = tau_pol[self.policy_to_local]
        leg_msg = Float64MultiArray()
        leg_msg.data = [float(x) for x in tau_local[: 12]]
        self.leg_pub.publish(leg_msg)

        # Wheels: send velocity targets to the velocity controller in our
        # local wheel order (FL, FR, RL, RR).
        # wheel_vel_targets_pol is in policy wheel-slot order:
        #   policy idx 12=FR_foot, 13=FL_foot, 14=RR_foot, 15=RL_foot.
        # Map to local wheel order: FL, FR, RL, RR.
        pol_wheel_names = self.cfg.joint_names[12:16]
        local_wheel_names = WHEEL_JOINT_ORDER_LOCAL
        pol_to_local_wheel = [pol_wheel_names.index(n) for n in local_wheel_names]
        wheel_local = wheel_vel_targets_pol[pol_to_local_wheel]
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [float(x) for x in wheel_local]
        self.wheel_pub.publish(wheel_msg)

    def _publish_zero(self) -> None:
        leg_msg = Float64MultiArray(); leg_msg.data = [0.0] * 12
        wheel_msg = Float64MultiArray(); wheel_msg.data = [0.0] * 4
        self.leg_pub.publish(leg_msg)
        self.wheel_pub.publish(wheel_msg)


def main() -> None:
    rclpy.init()
    node = RLLocomotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
