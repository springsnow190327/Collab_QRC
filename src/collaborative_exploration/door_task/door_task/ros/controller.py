"""ROS wrapper for the dual-robot VLM door-task controller.

Wires sub/pub/timers and delegates pure logic to ``door_task.core`` and
``door_task.llm``. Strategy comes from the slow planner + fast executer
(both VLM calls); the 10 Hz heading P-loop tracks whatever drive target
is in the shared action slot.
"""

from __future__ import annotations

import json
import math
import os
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8, String

from door_task.core.actions import (
    STOP_ACTION,
    fmt_action,
    lower_drive_relative,
    validate_action,
)
from door_task.core.control import ControlGains, compute_drive_cmd
from door_task.core.geometry import clamp, yaw_from_quat
from door_task.core.memory import default_world_memory
from door_task.core.rendering import (
    compose_vlm_image,
    encode_image_b64,
    image_msg_to_np,
)
from door_task.llm.backend import call_vlm, parse_action_json
from door_task.prompts.loader import EXECUTER_PROMPT, PLANNER_PROMPT
from door_task.prompts.user_prompt import build_user_prompt


class VLMControllerNode(Node):
    def __init__(self):
        super().__init__("vlm_controller")

        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("vlm_provider", "xai")
        self.declare_parameter("vlm_model", "grok-4-1-fast-non-reasoning")
        self.declare_parameter("vlm_temperature", 0.3)
        self.declare_parameter("vlm_max_tokens", 512)
        self.declare_parameter("vlm_timeout_sec", 8.0)
        self.declare_parameter("vlm_tick_sec", 1.0)
        self.declare_parameter("planner_tick_sec", 6.0)
        self.declare_parameter("planner_model", "")
        self.declare_parameter("planner_timeout_sec", 30.0)
        self.declare_parameter("planner_max_tokens", 768)
        self.declare_parameter("planner_temperature", 0.2)
        self.declare_parameter("control_tick_sec", 0.1)
        self.declare_parameter("vx_max", 0.55)
        self.declare_parameter("wz_max", 0.6)
        self.declare_parameter("heading_kp", 2.0)
        self.declare_parameter("heading_err_turn_only", 0.40)
        self.declare_parameter("arrive_tol", 0.20)
        self.declare_parameter("action_stale_sec", 4.0)
        self.declare_parameter(
            "camera_topics",
            ["/front_camera/color/image_raw", "/b_front_camera/color/image_raw"],
        )
        self.declare_parameter("map_origin_offset_x", [2.0, 6.0])
        self.declare_parameter("map_origin_offset_y", [2.0, 2.0])

        self._namespaces = [str(x) for x in self.get_parameter("robot_namespaces").value]
        self._provider = str(self.get_parameter("vlm_provider").value)
        self._model = str(self.get_parameter("vlm_model").value)
        self._temperature = float(self.get_parameter("vlm_temperature").value)
        self._max_tokens = int(self.get_parameter("vlm_max_tokens").value)
        self._timeout = float(self.get_parameter("vlm_timeout_sec").value)
        self._vlm_tick_period = float(self.get_parameter("vlm_tick_sec").value)
        self._planner_tick_period = float(self.get_parameter("planner_tick_sec").value)
        _planner_model = str(self.get_parameter("planner_model").value).strip()
        self._planner_model = _planner_model or self._model
        self._planner_timeout = float(self.get_parameter("planner_timeout_sec").value)
        self._planner_max_tokens = int(self.get_parameter("planner_max_tokens").value)
        self._planner_temperature = float(self.get_parameter("planner_temperature").value)
        self._ctrl_tick_period = float(self.get_parameter("control_tick_sec").value)
        self._gains = ControlGains(
            vx_max=float(self.get_parameter("vx_max").value),
            wz_max=float(self.get_parameter("wz_max").value),
            heading_kp=float(self.get_parameter("heading_kp").value),
            turn_only_thresh=float(self.get_parameter("heading_err_turn_only").value),
            arrive_tol=float(self.get_parameter("arrive_tol").value),
        )
        self._action_stale = float(self.get_parameter("action_stale_sec").value)
        cam_topics = list(self.get_parameter("camera_topics").value)
        offx = list(self.get_parameter("map_origin_offset_x").value)
        offy = list(self.get_parameter("map_origin_offset_y").value)
        self._map_offset: dict[str, tuple[float, float]] = {}
        for i, ns in enumerate(self._namespaces):
            ox = float(offx[i]) if i < len(offx) else 0.0
            oy = float(offy[i]) if i < len(offy) else 0.0
            self._map_offset[ns] = (ox, oy)

        # ── State ─────────────────────────────────────────────────────────
        self._odoms: dict[str, Odometry] = {}
        self._cam_frames: dict[str, np.ndarray] = {}
        self._maps: dict[str, Optional[OccupancyGrid]] = {ns: None for ns in self._namespaces}
        self._button_pressed = False
        self._button_ever_pressed = False
        self._start_time = self.get_clock().now().nanoseconds / 1e9

        self._actions: dict[str, dict] = {ns: dict(STOP_ACTION) for ns in self._namespaces}
        self._action_updated_time = 0.0
        self._action_lock = threading.Lock()

        # ── Executer (fast) ───────────────────────────────────────────────
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm_ctrl")
        self._pending_future: Optional[Future] = None
        self._vlm_calls = 0
        self._vlm_successes = 0
        self._ready_logged = False

        # ── Planner (slow) ────────────────────────────────────────────────
        self._planner_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm_planner")
        self._planner_pending: Optional[Future] = None
        self._plan: dict = {}
        self._world_memory: dict = default_world_memory()
        self._plan_lock = threading.Lock()
        self._planner_calls = 0
        self._planner_successes = 0
        self._executer_reports: list[dict] = []
        self._max_executer_reports = 8

        # ── Subscriptions ─────────────────────────────────────────────────
        for ns in self._namespaces:
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg),
                10,
            )
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        if len(cam_topics) != len(self._namespaces):
            raise ValueError("camera_topics length must match robot_namespaces")
        for ns, topic in zip(self._namespaces, cam_topics):
            self.create_subscription(
                Image, topic,
                lambda msg, _ns=ns: self._on_camera(_ns, msg),
                cam_qos,
            )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        for ns in self._namespaces:
            self.create_subscription(
                OccupancyGrid, f"/{ns}/map",
                lambda msg, _ns=ns: self._on_map(_ns, msg),
                map_qos,
            )
        self.create_subscription(Bool, "/door_task/button_pressed", self._on_button, 10)
        # Perception world_dict (optional — node may not be running).
        self._world_dict_json: str = ""
        self._world_dict_lock = threading.Lock()
        self.create_subscription(
            String, "/perception/world_dict", self._on_world_dict, 10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._cmd_pubs = {
            ns: self.create_publisher(Twist, f"/{ns}/cmd_vel_legged", 10)
            for ns in self._namespaces
        }
        self._stop_pubs = {
            ns: self.create_publisher(Int8, f"/{ns}/stop", 10) for ns in self._namespaces
        }
        self._debug_pub = self.create_publisher(String, "/vlm_debug/state", 10)
        self._last_planner_payload: dict = {}
        self._last_executer_payload: dict = {}
        self._last_image_b64: str = ""

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(self._ctrl_tick_period, self._control_tick)
        self.create_timer(self._vlm_tick_period, self._vlm_tick_cb)
        self.create_timer(self._planner_tick_period, self._planner_tick_cb)
        self.create_timer(2.0, self._mute_nav)
        self._mute_nav()

        self.get_logger().info(
            f"VLMController up | exec={self._provider}/{self._model} | "
            f"planner={self._provider}/{self._planner_model} | "
            f"ctrl={self._ctrl_tick_period}s exec={self._vlm_tick_period}s "
            f"planner={self._planner_tick_period}s"
        )

    # ── Subscribers ───────────────────────────────────────────────────────
    def _on_odom(self, ns, msg):
        self._odoms[ns] = msg

    def _on_camera(self, ns, msg):
        try:
            self._cam_frames[ns] = image_msg_to_np(msg)
        except Exception as exc:
            self.get_logger().warn(f"{ns} camera decode failed: {exc}")

    def _on_map(self, ns, msg):
        ox, oy = self._map_offset.get(ns, (0.0, 0.0))
        if ox != 0.0 or oy != 0.0:
            msg.info.origin.position.x += ox
            msg.info.origin.position.y += oy
        self._maps[ns] = msg

    def _on_button(self, msg: Bool):
        self._button_pressed = bool(msg.data)
        if self._button_pressed:
            self._button_ever_pressed = True

    def _on_world_dict(self, msg: String):
        with self._world_dict_lock:
            self._world_dict_json = msg.data

    def _world_dict_snapshot(self) -> str:
        with self._world_dict_lock:
            return self._world_dict_json

    # ── Fast control loop ─────────────────────────────────────────────────
    def _mute_nav(self):
        """Latch /stop=1 so any lingering nav planner ignores planner output.
        The door launch already skips the nav sub-launch when the LLM
        controller owns cmd_vel; this is belt-and-suspenders."""
        msg = Int8(data=1)
        for pub in self._stop_pubs.values():
            pub.publish(msg)

    def _control_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        with self._action_lock:
            actions = {ns: dict(a) for ns, a in self._actions.items()}
            updated = self._action_updated_time
        stale = updated > 0 and (now - updated) > self._action_stale

        for ns in self._namespaces:
            msg = Twist()
            if stale:
                self._cmd_pubs[ns].publish(msg)
                continue
            vx, wz = self._compute_cmd(ns, actions[ns])
            msg.linear.x = clamp(vx, -self._gains.vx_max, self._gains.vx_max)
            msg.angular.z = clamp(wz, -self._gains.wz_max, self._gains.wz_max)
            self._cmd_pubs[ns].publish(msg)

    def _compute_cmd(self, ns: str, action: dict) -> tuple[float, float]:
        if action.get("mode") != "drive":
            return 0.0, 0.0
        od = self._odoms.get(ns)
        if od is None:
            return 0.0, 0.0
        try:
            tx = float(action["tx"])
            ty = float(action["ty"])
            vx_req = float(action.get("vx_max", self._gains.vx_max))
        except (KeyError, TypeError, ValueError):
            return 0.0, 0.0
        x = od.pose.pose.position.x
        y = od.pose.pose.position.y
        yaw = yaw_from_quat(od.pose.pose.orientation)
        return compute_drive_cmd(x, y, yaw, tx, ty, vx_req, self._gains)

    # ── Readiness gate ────────────────────────────────────────────────────
    def _data_ready(self) -> bool:
        if any(ns not in self._odoms for ns in self._namespaces):
            return False
        if any(ns not in self._cam_frames for ns in self._namespaces):
            return False
        for ns in self._namespaces:
            grid = self._maps.get(ns)
            if grid is None:
                return False
            known = sum(1 for c in grid.data if c != -1)
            if known < 200:
                return False
        return True

    # ── Slow VLM ticks ────────────────────────────────────────────────────
    def _planner_tick_cb(self):
        if not self._data_ready():
            return
        if self._planner_pending is not None and not self._planner_pending.done():
            return
        obs = self._build_observation()
        try:
            image_b64 = self._render_scene_b64(obs)
        except Exception as exc:
            self.get_logger().warn(f"planner render failed: {exc}")
            return
        self._planner_pending = self._planner_pool.submit(
            self._run_planner_query, obs, image_b64
        )
        self._planner_pending.add_done_callback(self._on_planner_done)
        self._planner_calls += 1

    def _vlm_tick_cb(self):
        if not self._data_ready():
            return
        if not self._ready_logged:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self._start_time
            self.get_logger().info(f"VLM controller ready ({elapsed:.1f}s after start)")
            self._ready_logged = True
        if self._pending_future is not None and not self._pending_future.done():
            return
        obs = self._build_observation()
        try:
            image_b64 = self._render_scene_b64(obs)
        except Exception as exc:
            self.get_logger().warn(f"render failed: {exc}")
            return
        self._pending_future = self._pool.submit(self._run_query, obs, image_b64)
        self._pending_future.add_done_callback(self._on_query_done)
        self._vlm_calls += 1

    def _build_observation(self) -> dict:
        now = self.get_clock().now().nanoseconds / 1e9
        obs: dict = {
            "elapsed_sec": now - self._start_time,
            "button_pressed": self._button_pressed,
            "button_ever_pressed": self._button_ever_pressed,
        }
        with self._action_lock:
            last_actions = {ns: dict(a) for ns, a in self._actions.items()}
        for ns in self._namespaces:
            od = self._odoms[ns]
            yaw = yaw_from_quat(od.pose.pose.orientation)
            obs[ns] = {
                "x": od.pose.pose.position.x,
                "y": od.pose.pose.position.y,
                "yaw_deg": math.degrees(yaw),
            }
        obs["last_action"] = last_actions
        return obs

    def _render_scene_b64(self, obs: dict) -> str:
        ns_a = self._namespaces[0]
        ns_b = self._namespaces[1] if len(self._namespaces) > 1 else None
        img, panel_meta = compose_vlm_image(
            cam_a=self._cam_frames.get(ns_a),
            cam_b=self._cam_frames.get(ns_b) if ns_b else None,
            map_a=self._maps.get(ns_a),
            map_b=self._maps.get(ns_b) if ns_b else None,
            pose_a=(obs[ns_a]["x"], obs[ns_a]["y"]),
            pose_b=(obs[ns_b]["x"], obs[ns_b]["y"]) if ns_b else None,
        )
        obs["panel_meta"] = panel_meta
        dump_dir = os.environ.get("VLM_RENDER_DUMP_DIR", "")
        if dump_dir:
            try:
                os.makedirs(dump_dir, exist_ok=True)
                from PIL import Image as PILImage  # noqa: PLC0415
                path = os.path.join(dump_dir, f"render_{self._vlm_calls:04d}.png")
                PILImage.fromarray(img).save(path)
            except Exception as exc:
                self.get_logger().warn(f"render dump failed: {exc}")
        b64 = encode_image_b64(img)
        self._last_image_b64 = b64
        return b64

    def _run_query(self, obs, image_b64) -> dict:
        with self._plan_lock:
            plan_snapshot = dict(self._plan) if self._plan else {}
            memory_snapshot = dict(self._world_memory) if self._world_memory else {}
        user_prompt = build_user_prompt(obs)
        user_prompt += "\n\nWORLD MEMORY (from planner, persistent):\n"
        user_prompt += json.dumps(memory_snapshot, indent=2)
        if plan_snapshot:
            user_prompt += "\n\nCURRENT PLAN (from slow planner):\n"
            user_prompt += json.dumps(plan_snapshot, indent=2)
        else:
            user_prompt += "\n\nCURRENT PLAN: (none yet — emit safe stop / wait actions)"
        wd = self._world_dict_snapshot()
        if wd:
            user_prompt += "\n\nPERCEPTION WORLD_DICT (YOLO+tracker, rolling):\n" + wd
        raw = call_vlm(
            self._provider, self._model,
            EXECUTER_PROMPT, user_prompt, image_b64,
            temperature=self._temperature,
            max_tokens=self._max_tokens,
            timeout=self._timeout,
        )
        return parse_action_json(raw)

    def _run_planner_query(self, obs, image_b64) -> dict:
        with self._plan_lock:
            prev_plan = dict(self._plan) if self._plan else {}
            prev_memory = dict(self._world_memory) if self._world_memory else {}
            reports = list(self._executer_reports)
            self._executer_reports = []
        user_prompt = build_user_prompt(obs)
        user_prompt += "\n\nPREVIOUS WORLD MEMORY:\n"
        user_prompt += json.dumps(prev_memory, indent=2)
        if prev_plan:
            user_prompt += "\n\nPREVIOUS PLAN:\n"
            user_prompt += json.dumps(prev_plan, indent=2)
        if reports:
            user_prompt += "\n\nRECENT EXECUTER REPORTS (since your last call):\n"
            user_prompt += json.dumps(reports, indent=2)
        else:
            user_prompt += "\n\nRECENT EXECUTER REPORTS: (none)"
        wd = self._world_dict_snapshot()
        if wd:
            user_prompt += "\n\nPERCEPTION WORLD_DICT (YOLO+tracker, rolling):\n" + wd
        raw = call_vlm(
            self._provider, self._planner_model,
            PLANNER_PROMPT, user_prompt, image_b64,
            temperature=self._planner_temperature,
            max_tokens=self._planner_max_tokens,
            timeout=self._planner_timeout,
        )
        return parse_action_json(raw)

    # ── Result handlers ───────────────────────────────────────────────────
    def _on_planner_done(self, future: Future):
        try:
            plan = future.result()
        except Exception as exc:
            self.get_logger().warn(f"planner query failed: {exc}")
            return
        if not isinstance(plan, dict):
            return
        new_memory = plan.pop("world_memory", None)
        with self._plan_lock:
            self._plan = plan
            if isinstance(new_memory, dict):
                self._world_memory = new_memory
        self._planner_successes += 1
        reason = str(plan.get("reason", ""))[:100]
        pillar_known = (
            isinstance(new_memory, dict)
            and isinstance(new_memory.get("pillar"), dict)
            and bool(new_memory["pillar"].get("known"))
        )
        self.get_logger().info(
            f"PLAN {self._planner_successes}/{self._planner_calls} "
            f"| pillar_known={pillar_known} | {reason}"
        )
        self._last_planner_payload = {
            "call_no": self._planner_successes,
            "reason": str(plan.get("reason", "")),
            "robot_a": plan.get("robot_a", {}),
            "robot_b": plan.get("robot_b", {}),
            "world_memory_out": new_memory if isinstance(new_memory, dict) else {},
        }
        self._publish_debug_snapshot()

    def _on_query_done(self, future: Future):
        try:
            parsed = future.result()
        except Exception as exc:
            self.get_logger().warn(f"VLM query failed: {exc}")
            return
        try:
            actions = {
                ns: validate_action(parsed.get(ns, STOP_ACTION))
                for ns in self._namespaces
            }
        except Exception as exc:
            self.get_logger().warn(f"action parse error: {exc} | raw={parsed}")
            return

        report = parsed.get("report")
        if isinstance(report, dict) and (
            report.get("uncertain") or report.get("request_help") or report.get("discoveries")
        ):
            with self._plan_lock:
                self._executer_reports.append(
                    {
                        "t": self.get_clock().now().nanoseconds / 1e9 - self._start_time,
                        "report": report,
                    }
                )
                if len(self._executer_reports) > self._max_executer_reports:
                    self._executer_reports.pop(0)

        # Lower drive_relative → drive using the current pose so the heading
        # loop has an absolute target to track.
        for ns in self._namespaces:
            a = actions[ns]
            if a.get("mode") != "drive_relative":
                continue
            od = self._odoms.get(ns)
            if od is None:
                actions[ns] = dict(STOP_ACTION)
                continue
            x = od.pose.pose.position.x
            y = od.pose.pose.position.y
            yaw = yaw_from_quat(od.pose.pose.orientation)
            actions[ns] = lower_drive_relative(a, x, y, yaw)

        now = self.get_clock().now().nanoseconds / 1e9
        with self._action_lock:
            self._actions = actions
            self._action_updated_time = now

        self._vlm_successes += 1
        reason = str(parsed.get("reason", ""))[:80]
        self.get_logger().info(
            f"VLM {self._vlm_successes}/{self._vlm_calls}: "
            f"A={fmt_action(actions['robot_a'])} "
            f"B={fmt_action(actions['robot_b'])} | {reason}"
        )
        self._last_executer_payload = {
            "call_no": self._vlm_successes,
            "reason": str(parsed.get("reason", "")),
            "robot_a": {"fmt": fmt_action(actions["robot_a"]), "raw": actions["robot_a"]},
            "robot_b": {"fmt": fmt_action(actions["robot_b"]), "raw": actions["robot_b"]},
            "report": report if isinstance(report, dict) else {},
        }
        self._publish_debug_snapshot()

    # ── Debug snapshot ────────────────────────────────────────────────────
    def _publish_debug_snapshot(self):
        now = self.get_clock().now().nanoseconds / 1e9 - self._start_time
        with self._plan_lock:
            plan_snap = dict(self._plan) if self._plan else {}
            memory_snap = dict(self._world_memory) if self._world_memory else {}
            reports_snap = list(self._executer_reports)
        pose = {}
        for ns in self._namespaces:
            od = self._odoms.get(ns)
            if od is None:
                continue
            yaw = yaw_from_quat(od.pose.pose.orientation)
            pose[ns] = {
                "x": od.pose.pose.position.x,
                "y": od.pose.pose.position.y,
                "yaw_deg": math.degrees(yaw),
            }
        state = {
            "t": now,
            "image_b64": self._last_image_b64,
            "planner": {
                "calls": self._planner_calls,
                "successes": self._planner_successes,
                "period_s": self._planner_tick_period,
                "model": self._planner_model,
                "system_prompt_chars": len(PLANNER_PROMPT),
                "last": self._last_planner_payload,
            },
            "executer": {
                "calls": self._vlm_calls,
                "successes": self._vlm_successes,
                "period_s": self._vlm_tick_period,
                "model": self._model,
                "system_prompt_chars": len(EXECUTER_PROMPT),
                "last": self._last_executer_payload,
            },
            "world_memory": memory_snap,
            "plan": plan_snap,
            "recent_reports": reports_snap,
            "pose": pose,
            "button_pressed": self._button_pressed,
            "button_ever_pressed": self._button_ever_pressed,
        }
        try:
            self._debug_pub.publish(String(data=json.dumps(state, default=str)))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VLMControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
