#!/usr/bin/env python3
"""Low-frequency VLM advisor for ROS2 exploration.

This node is intentionally conservative:
- the baseline explorer keeps producing goals continuously
- VLM goals are published on a side topic and consumed through a mux
- if inference is slow, disabled, or fails, the baseline path is untouched
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .prompting import TOOL_SCHEMAS, build_system_prompt, build_user_prompt
from .vlm_backends import VLMBackendError, extract_json_object, query_vlm, resolve_api_key, resolve_provider


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class VLMCoordinatorNode(Node):
    def __init__(self):
        super().__init__("vlm_coordinator")

        self.declare_parameter("robot_namespaces", ["robot"])
        self.declare_parameter("rendered_map_topic", "/vlm/rendered_map")
        self.declare_parameter("scene_json_topic", "/vlm/scene_json")
        self.declare_parameter("artifact_detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("tool_requests_topic", "/vlm/tool_requests")
        self.declare_parameter("tool_status_topic", "/vlm/tool_status")
        self.declare_parameter("goal_topic_suffix", "/vlm_way_point")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("replan_period_sec", 15.0)
        self.declare_parameter("goal_repeat_period_sec", 0.5)
        self.declare_parameter("primary_goal_ttl_sec", 2.0)
        self.declare_parameter("vlm_enabled", True)
        self.declare_parameter("vlm_provider", "auto")  # auto | xai | openai | anthropic
        self.declare_parameter("vlm_model", "")
        self.declare_parameter("vlm_temperature", 0.1)
        self.declare_parameter("vlm_max_tokens", 768)
        self.declare_parameter("vlm_timeout_sec", 15.0)
        self.declare_parameter("artifact_reach_radius_m", 1.0)
        self.declare_parameter("max_scene_json_chars", 4000)
        self.declare_parameter("mission_prompt", "")
        self.declare_parameter("vlm_log_dir", "")

        self._namespaces = [str(x) for x in self.get_parameter("robot_namespaces").value]
        self._goal_suffix = str(self.get_parameter("goal_topic_suffix").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._replan_sec = max(2.0, float(self.get_parameter("replan_period_sec").value))
        self._goal_repeat_sec = max(0.2, float(self.get_parameter("goal_repeat_period_sec").value))
        self._goal_ttl_sec = max(0.5, float(self.get_parameter("primary_goal_ttl_sec").value))
        self._vlm_enabled = bool(self.get_parameter("vlm_enabled").value)
        self._vlm_provider = str(self.get_parameter("vlm_provider").value).strip() or "auto"
        self._vlm_model = str(self.get_parameter("vlm_model").value).strip()
        self._vlm_temperature = float(self.get_parameter("vlm_temperature").value)
        self._vlm_max_tokens = int(self.get_parameter("vlm_max_tokens").value)
        self._vlm_timeout_sec = float(self.get_parameter("vlm_timeout_sec").value)
        self._artifact_reach_radius_m = float(self.get_parameter("artifact_reach_radius_m").value)
        self._max_scene_json_chars = int(self.get_parameter("max_scene_json_chars").value)
        self._mission_prompt = str(self.get_parameter("mission_prompt").value).strip()

        # VLM history logging
        log_dir_param = str(self.get_parameter("vlm_log_dir").value).strip()
        if not log_dir_param:
            log_dir_param = os.path.join(
                os.environ.get("ROS_LOG_DIR", os.path.expanduser("~/.ros/log")),
                "vlm_history",
            )
        run_stamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_dir = Path(log_dir_param) / run_stamp
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._cycle_count = 0
        (self._log_dir / "index.json").write_text("[]")
        self.get_logger().info(f"VLM history logging to {self._log_dir}")

        self._rendered_img: Image | None = None
        self._scene_json: str | None = None
        self._artifact_detections: list[dict] = []
        self._tool_status: dict[str, dict] = {}
        self._odoms: dict[str, Odometry] = {}
        self._last_goals: dict[str, tuple[float, float]] = {}
        self._last_goal_stamp: dict[str, float] = {}
        self._last_scene_fingerprint = ""
        self._seen_artifacts: set[str] = set()

        self._worker_lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self._pending_result: dict | None = None
        self._pending_error: str | None = None
        self._pending_fingerprint = ""

        self.create_subscription(Image, self.get_parameter("rendered_map_topic").value, self._on_rendered, 10)
        self.create_subscription(String, self.get_parameter("scene_json_topic").value, self._on_scene_json, 10)
        self.create_subscription(
            String, self.get_parameter("artifact_detections_topic").value, self._on_artifact_detections, 10
        )
        self.create_subscription(String, self.get_parameter("tool_status_topic").value, self._on_tool_status, 10)
        for ns in self._namespaces:
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda msg, _ns=ns: self._on_odom(_ns, msg), 10)

        self._goal_pubs = {
            ns: self.create_publisher(PointStamped, f"/{ns}{self._goal_suffix}", 10) for ns in self._namespaces
        }
        self._tool_pub = self.create_publisher(String, self.get_parameter("tool_requests_topic").value, 10)

        self._replan_timer = self.create_timer(self._replan_sec, self._replan_tick)
        self._result_timer = self.create_timer(0.2, self._drain_worker_result)
        self._goal_repeat_timer = self.create_timer(self._goal_repeat_sec, self._republish_goals)

        provider = resolve_provider(self._vlm_provider)
        self.get_logger().info(
            "VLMCoordinator started | "
            f"enabled={self._vlm_enabled} provider={provider} model={self._resolved_model_name(provider)} "
            f"goal_topic={self._goal_suffix} period={self._replan_sec:.1f}s"
        )

    def _on_rendered(self, msg: Image):
        self._rendered_img = msg

    def _on_scene_json(self, msg: String):
        self._scene_json = msg.data

    def _on_artifact_detections(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self._artifact_detections = payload if isinstance(payload, list) else []
        except json.JSONDecodeError:
            self._artifact_detections = []

    def _on_tool_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        artifact_id = str(payload.get("artifact_id", "")).strip()
        if artifact_id:
            self._tool_status[artifact_id] = payload

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _resolved_model_name(self, provider: str) -> str:
        if self._vlm_model:
            return self._vlm_model
        if provider == "xai":
            return "grok-4-1-fast-non-reasoning"
        if provider == "anthropic":
            return "claude-3-5-sonnet-latest"
        return "gpt-4o-mini"

    def _encode_image_b64(self) -> Optional[str]:
        msg = self._rendered_img
        if msg is None:
            return None
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            try:
                from PIL import Image as PILImage  # type: ignore

                pil_img = PILImage.fromarray(img)
                buf = io.BytesIO()
                pil_img.save(buf, format="JPEG", quality=72)
                return base64.b64encode(buf.getvalue()).decode("ascii")
            except ImportError:
                header = f"P6\n{msg.width} {msg.height}\n255\n".encode()
                return base64.b64encode(header + img.tobytes()).decode("ascii")
        except (ValueError, IndexError, OSError):
            return None

    def _build_scene(self) -> dict | None:
        if self._scene_json is None:
            return None
        try:
            scene = json.loads(self._scene_json)
        except json.JSONDecodeError:
            return None

        scene["artifact_detections"] = self._artifact_detections
        scene["tool_schemas"] = [t["name"] for t in TOOL_SCHEMAS]
        scene["current_goals"] = {
            ns: [round(x, 2), round(y, 2)] for ns, (x, y) in sorted(self._last_goals.items())
        }
        scene["artifacts_seen"] = sorted(self._seen_artifacts)
        scene["tool_status"] = self._tool_status
        scene["artifact_proximity"] = self._artifact_proximity()

        text = json.dumps(scene)
        if len(text) > self._max_scene_json_chars:
            scene = dict(scene)
            scene["tool_status"] = {k: v for k, v in list(self._tool_status.items())[-4:]}
        return scene

    def _artifact_proximity(self) -> list[dict]:
        proximity = []
        for det in self._artifact_detections:
            artifact_id = str(det.get("id", ""))
            dx = float(det.get("x", 0.0))
            dy = float(det.get("y", 0.0))
            for ns, odom in self._odoms.items():
                rx = float(odom.pose.pose.position.x)
                ry = float(odom.pose.pose.position.y)
                dist = math.hypot(rx - dx, ry - dy)
                if dist <= self._artifact_reach_radius_m:
                    proximity.append({"robot": ns, "artifact_id": artifact_id, "distance_m": round(dist, 2)})
        return proximity

    def _scene_fingerprint(self, scene: dict) -> str:
        compact = {
            "robot_states": scene.get("robot_states", {}),
            "artifact_detections": [
                {
                    "id": str(det.get("id", "")),
                    "x": round(float(det.get("x", 0.0)), 1),
                    "y": round(float(det.get("y", 0.0)), 1),
                    "label": str(det.get("label", "")),
                }
                for det in scene.get("artifact_detections", [])
            ],
            "current_goals": scene.get("current_goals", {}),
            "artifacts_seen": scene.get("artifacts_seen", []),
        }
        return json.dumps(compact, sort_keys=True)

    def _log_cycle(self, system_prompt: str, user_prompt: str, scene: dict,
                   image_b64: str | None, model: str, provider: str,
                   raw_response: str | None, parsed: dict | None,
                   error: str | None, latency_sec: float):
        """Persist a full VLM cycle to disk for the debug viewer."""
        self._cycle_count += 1
        cycle_id = f"{self._cycle_count:04d}"
        cycle_dir = self._log_dir / cycle_id
        cycle_dir.mkdir(exist_ok=True)
        try:
            # Save image
            if image_b64:
                (cycle_dir / "rendered_map.jpg").write_bytes(base64.b64decode(image_b64))
            # Save prompts and response
            (cycle_dir / "prompt.json").write_text(json.dumps({
                "system_prompt": system_prompt,
                "user_prompt": user_prompt,
                "scene": scene,
            }, indent=2))
            (cycle_dir / "response.json").write_text(json.dumps({
                "raw": raw_response,
                "parsed": parsed,
                "error": error,
            }, indent=2))
            # Append to index
            entry = {
                "cycle": cycle_id,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "model": model,
                "provider": provider,
                "latency_sec": round(latency_sec, 2),
                "error": error,
                "has_tool_calls": bool(parsed and parsed.get("tool_calls")),
            }
            index_path = self._log_dir / "index.json"
            try:
                index = json.loads(index_path.read_text())
            except (json.JSONDecodeError, FileNotFoundError):
                index = []
            index.append(entry)
            index_path.write_text(json.dumps(index, indent=2))
        except OSError as exc:
            self.get_logger().warn(f"Failed to write VLM log: {exc}")

    def _replan_tick(self):
        if not self._vlm_enabled:
            return
        if self._rendered_img is None:
            return
        if self._worker is not None and self._worker.is_alive():
            return
        provider = resolve_provider(self._vlm_provider)
        if provider == "none" or not resolve_api_key(provider):
            return

        scene = self._build_scene()
        if scene is None:
            return
        # Block VLM until map has meaningful free space (Cartographer needs time to start)
        map_info = scene.get("map_info", {})
        free_cells = int(map_info.get("free_cells", 0))
        if free_cells < 200:
            self.get_logger().info(
                f"Map not ready ({free_cells} free cells < 200), skipping VLM call",
                throttle_duration_sec=10.0,
            )
            return
        fingerprint = self._scene_fingerprint(scene)
        if fingerprint == self._last_scene_fingerprint:
            return

        image_b64 = self._encode_image_b64()
        if image_b64 is None:
            return

        model = self._resolved_model_name(provider)
        system_prompt = build_system_prompt(mission=self._mission_prompt)
        user_prompt = build_user_prompt(scene)
        self._pending_error = None
        self._pending_result = None

        def _worker():
            t0 = time.monotonic()
            try:
                raw = query_vlm(
                    provider=provider,
                    model=model,
                    system_prompt=system_prompt,
                    user_prompt=user_prompt,
                    image_b64=image_b64,
                    temperature=self._vlm_temperature,
                    max_tokens=self._vlm_max_tokens,
                    timeout_sec=self._vlm_timeout_sec,
                )
                latency = time.monotonic() - t0
                parsed = extract_json_object(raw)
                with self._worker_lock:
                    self._pending_result = {
                        "raw": raw, "parsed": parsed,
                        "system_prompt": system_prompt, "user_prompt": user_prompt,
                        "scene": scene, "image_b64": image_b64,
                        "model": model, "provider": provider, "latency_sec": latency,
                    }
                    self._pending_fingerprint = fingerprint
            except VLMBackendError as exc:
                latency = time.monotonic() - t0
                with self._worker_lock:
                    self._pending_error = str(exc)
                    self._pending_fingerprint = fingerprint
                self._log_cycle(
                    system_prompt, user_prompt, scene, image_b64,
                    model, provider, None, None, str(exc), latency,
                )

        self._worker = threading.Thread(target=_worker, daemon=True)
        self._worker.start()

    def _drain_worker_result(self):
        with self._worker_lock:
            result = self._pending_result
            error = self._pending_error
            fingerprint = self._pending_fingerprint
            self._pending_result = None
            self._pending_error = None
            self._pending_fingerprint = ""

        if error:
            self.get_logger().warn(f"VLM inference failed: {error}")
            self._last_scene_fingerprint = fingerprint
            return
        if result is None:
            return

        parsed = result.get("parsed")
        raw = str(result.get("raw", ""))

        # Log full cycle to disk
        self._log_cycle(
            result.get("system_prompt", ""), result.get("user_prompt", ""),
            result.get("scene", {}), result.get("image_b64"),
            result.get("model", ""), result.get("provider", ""),
            raw, parsed, None, result.get("latency_sec", 0.0),
        )

        if parsed is None:
            self.get_logger().warn(f"VLM response was not valid JSON: {raw[:240]}")
            self._last_scene_fingerprint = fingerprint
            return

        self.get_logger().info(f"VLM response: {raw[:240]}")
        self._execute_tool_calls(parsed)
        self._last_scene_fingerprint = fingerprint

    def _execute_tool_calls(self, response: dict):
        tool_calls = response.get("tool_calls", [])
        if not isinstance(tool_calls, list):
            self.get_logger().warn("VLM response did not contain a tool_calls list")
            return
        for tc in tool_calls:
            name = str(tc.get("name", "")).strip()
            args = tc.get("arguments", {}) or {}
            if name == "assign_waypoints":
                self._handle_assign_waypoints(args)
            elif name == "mark_artifact_seen":
                self._handle_mark_artifact_seen(args)
            elif name == "interact_with_artifact":
                self._handle_interact_with_artifact(args)
            elif name:
                self.get_logger().warn(f"Unknown VLM tool: {name}")

    def _handle_assign_waypoints(self, args: dict):
        assignments = args.get("assignments", [])
        if not isinstance(assignments, list):
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for item in assignments:
            ns = str(item.get("robot", "")).strip()
            if ns not in self._goal_pubs:
                continue
            x = float(item.get("x", 0.0))
            y = float(item.get("y", 0.0))
            reason = str(item.get("reason", "")).strip()
            self._last_goals[ns] = (x, y)
            self._last_goal_stamp[ns] = now_sec
            self._publish_goal(ns, x, y)
            self.get_logger().info(f"VLM override -> {ns}: ({x:.2f}, {y:.2f}) reason={reason}")

    def _handle_mark_artifact_seen(self, args: dict):
        artifact_id = str(args.get("artifact_id", "")).strip()
        robot = str(args.get("robot", "")).strip()
        reason = str(args.get("reason", "")).strip()
        if artifact_id:
            self._seen_artifacts.add(artifact_id)
        self.get_logger().info(
            f"VLM semantic hit -> robot={robot or 'unknown'} artifact={artifact_id or 'unknown'} reason={reason}"
        )

    def _handle_interact_with_artifact(self, args: dict):
        payload = {
            "tool": "interact_with_artifact",
            "robot": str(args.get("robot", "")).strip(),
            "artifact_id": str(args.get("artifact_id", "")).strip(),
            "action": str(args.get("action", "inspect")).strip(),
            "reason": str(args.get("reason", "")).strip(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._tool_pub.publish(msg)
        self.get_logger().info(
            f"VLM tool request -> robot={payload['robot']} artifact={payload['artifact_id']} action={payload['action']}"
        )

    def _publish_goal(self, ns: str, x: float, y: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0
        self._goal_pubs[ns].publish(msg)

    def _republish_goals(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for ns, goal in list(self._last_goals.items()):
            stamp = self._last_goal_stamp.get(ns, 0.0)
            if now_sec - stamp > self._goal_ttl_sec:
                continue
            self._publish_goal(ns, goal[0], goal[1])


def main(args=None):
    rclpy.init(args=args)
    node = VLMCoordinatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
