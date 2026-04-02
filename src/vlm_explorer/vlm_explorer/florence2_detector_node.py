#!/usr/bin/env python3
"""Florence-2 per-robot visual detector node.

Runs Microsoft Florence-2 locally on GPU for three tasks:
  1. Open detection (<OD>) at a steady rate — ambient object awareness
  2. Phrase grounding (<CAPTION_TO_PHRASE_GROUNDING>) — "do I see the goal?"
  3. Detailed caption (<MORE_DETAILED_CAPTION>) on ROI — rich description on trigger

Publishes detections in the same JSON format as artifact_detector_node so the
existing VLM coordinator and map renderer consume them without changes.

VRAM: ~1.5 GB for florence-2-large in fp16, ~0.5 GB for florence-2-base.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _ensure_flash_attn_shim():
    """Install a dummy flash_attn module if the real one is missing.

    Florence-2's HuggingFace code unconditionally imports flash_attn at module
    level.  When flash_attn is not installed (common on laptops / Jetson), the
    import check in transformers raises ImportError.  A lightweight shim lets
    the import succeed; Florence-2 then falls back to SDPA or eager attention.
    """
    import importlib.machinery
    import importlib.util
    import sys
    import types

    if importlib.util.find_spec("flash_attn") is not None:
        return  # real package available

    def _dummy(name):
        mod = types.ModuleType(name)
        mod.__spec__ = importlib.machinery.ModuleSpec(name, None)
        mod.__path__ = []
        mod.__version__ = "2.0.0"
        return mod

    for name in ("flash_attn", "flash_attn.flash_attn_func", "flash_attn.bert_padding"):
        sys.modules.setdefault(name, _dummy(name))


def _load_florence2(model_id: str, device: str):
    """Load Florence-2 model and processor. Called once at startup."""
    import torch

    _ensure_flash_attn_shim()
    from transformers import AutoModelForCausalLM, AutoProcessor

    processor = AutoProcessor.from_pretrained(model_id, trust_remote_code=True)
    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        trust_remote_code=True,
        torch_dtype=torch.float16,
        attn_implementation="sdpa",
    ).to(device)
    model.eval()
    return model, processor


def _run_florence2(model, processor, pil_image, task: str, text_input: str, device: str) -> dict:
    """Run a single Florence-2 inference and return parsed results."""
    import torch

    prompt = task if not text_input else task + text_input
    inputs = processor(text=prompt, images=pil_image, return_tensors="pt").to(device)
    # Match pixel_values dtype to model weights (fp16 when model is half-precision)
    model_dtype = next(model.parameters()).dtype
    if inputs["pixel_values"].dtype != model_dtype:
        inputs["pixel_values"] = inputs["pixel_values"].to(model_dtype)
    with torch.no_grad():
        generated_ids = model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=512,
            num_beams=3,
        )
    generated_text = processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
    result = processor.post_process_generation(
        generated_text,
        task=task,
        image_size=(pil_image.width, pil_image.height),
    )
    return result


class Florence2DetectorNode(Node):
    def __init__(self):
        super().__init__("florence2_detector")

        # ── Parameters ──
        self.declare_parameter("robot_namespaces", ["robot"])
        self.declare_parameter("detections_topic", "/vlm/artifact_detections")
        self.declare_parameter("descriptions_topic", "/vlm/scene_descriptions")
        self.declare_parameter("goal_prompt", "")
        self.declare_parameter("model_id", "microsoft/Florence-2-large")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("detection_rate", 2.0)
        self.declare_parameter("grounding_rate", 1.0)
        self.declare_parameter("description_cooldown_sec", 10.0)
        self.declare_parameter("grounding_confidence_threshold", 0.3)
        self.declare_parameter("assumed_depth_m", 2.0)
        self.declare_parameter("camera_hfov_rad", 2.0944)
        self.declare_parameter("dedup_radius_m", 0.8)
        self.declare_parameter("min_bbox_area_frac", 0.002)

        self._namespaces = [str(x) for x in self.get_parameter("robot_namespaces").value]
        self._goal_prompt = str(self.get_parameter("goal_prompt").value).strip()
        self._model_id = str(self.get_parameter("model_id").value).strip()
        self._device = str(self.get_parameter("device").value).strip()
        self._detection_rate = max(0.1, float(self.get_parameter("detection_rate").value))
        self._grounding_rate = max(0.1, float(self.get_parameter("grounding_rate").value))
        self._desc_cooldown = float(self.get_parameter("description_cooldown_sec").value)
        self._grounding_conf = float(self.get_parameter("grounding_confidence_threshold").value)
        self._assumed_depth = float(self.get_parameter("assumed_depth_m").value)
        self._hfov = float(self.get_parameter("camera_hfov_rad").value)
        self._dedup_r = float(self.get_parameter("dedup_radius_m").value)
        self._min_bbox_frac = float(self.get_parameter("min_bbox_area_frac").value)

        # ── Publishers ──
        self._det_pub = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )
        self._desc_pub = self.create_publisher(
            String, self.get_parameter("descriptions_topic").value, 10
        )

        # ── Subscribers ──
        self._images: dict[str, Image] = {}
        self._odoms: dict[str, Odometry] = {}
        for ns in self._namespaces:
            self.create_subscription(
                Image, f"/{ns}/front_camera/image_raw",
                lambda msg, _ns=ns: self._on_image(_ns, msg), 10,
            )
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg), 10,
            )

        # Allow goal prompt to be changed at runtime
        self.create_subscription(
            String, "/vlm/goal_prompt",
            self._on_goal_prompt, 10,
        )

        # ── State ──
        self._registry: list[dict] = []
        self._last_desc_time: dict[str, float] = {}
        self._model = None
        self._processor = None
        self._model_ready = False
        self._model_lock = threading.Lock()

        # ── Load model in background thread ──
        self._loader_thread = threading.Thread(target=self._load_model, daemon=True)
        self._loader_thread.start()

        # ── Timers ──
        self._det_timer = self.create_timer(1.0 / self._detection_rate, self._detection_tick)
        self._grounding_timer = self.create_timer(1.0 / self._grounding_rate, self._grounding_tick)

        self.get_logger().info(
            f"Florence2Detector: model={self._model_id} device={self._device} "
            f"ns={self._namespaces} goal='{self._goal_prompt}' "
            f"det_rate={self._detection_rate}Hz ground_rate={self._grounding_rate}Hz"
        )

    # ── Callbacks ──

    def _on_image(self, ns: str, msg: Image):
        self._images[ns] = msg

    def _on_odom(self, ns: str, msg: Odometry):
        self._odoms[ns] = msg

    def _on_goal_prompt(self, msg: String):
        new_prompt = msg.data.strip()
        if new_prompt != self._goal_prompt:
            self._goal_prompt = new_prompt
            self.get_logger().info(f"Goal prompt updated: '{self._goal_prompt}'")

    # ── Model loading ──

    def _load_model(self):
        self.get_logger().info(f"Loading Florence-2 model '{self._model_id}' on {self._device}...")
        try:
            model, processor = _load_florence2(self._model_id, self._device)
            with self._model_lock:
                self._model = model
                self._processor = processor
                self._model_ready = True
            self.get_logger().info("Florence-2 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load Florence-2: {e}")

    # ── Image conversion ──

    def _msg_to_pil(self, msg: Image):
        """Convert ROS Image to PIL Image."""
        from PIL import Image as PILImage

        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
            if msg.encoding == "bgr8":
                img = img[:, :, ::-1]
            return PILImage.fromarray(img)
        except (ValueError, IndexError):
            return None

    # ── Task 1: Open detection (<OD>) ──

    def _detection_tick(self):
        if not self._model_ready:
            return

        for ns in self._namespaces:
            if ns not in self._images or ns not in self._odoms:
                continue
            pil_img = self._msg_to_pil(self._images[ns])
            if pil_img is None:
                continue

            with self._model_lock:
                result = _run_florence2(
                    self._model, self._processor, pil_img,
                    "<OD>", "", self._device,
                )

            detections = self._parse_od_result(ns, result, pil_img.width, pil_img.height)
            self._merge_detections(detections)

        self._publish_detections()

    def _parse_od_result(self, ns: str, result: dict, img_w: int, img_h: int) -> list[dict]:
        """Parse Florence-2 <OD> output into detection dicts."""
        detections = []
        od = result.get("<OD>", {})
        bboxes = od.get("bboxes", [])
        labels = od.get("labels", [])
        img_area = max(1, img_w * img_h)

        odom = self._odoms.get(ns)
        if odom is None:
            return detections

        for bbox, label in zip(bboxes, labels):
            x1, y1, x2, y2 = bbox
            bbox_area = (x2 - x1) * (y2 - y1)
            if bbox_area / img_area < self._min_bbox_frac:
                continue

            cx = (x1 + x2) / 2.0
            world_x, world_y = self._project_to_world(cx, img_w, odom)
            detections.append({
                "label": label.strip().lower(),
                "x": round(world_x, 2),
                "y": round(world_y, 2),
                "confidence": round(bbox_area / img_area, 3),
                "robot": ns,
                "source": "florence2_od",
            })
        return detections

    # ── Task 2: Phrase grounding (<CAPTION_TO_PHRASE_GROUNDING>) ──

    def _grounding_tick(self):
        if not self._model_ready:
            return
        if not self._goal_prompt:
            return

        for ns in self._namespaces:
            if ns not in self._images or ns not in self._odoms:
                continue
            pil_img = self._msg_to_pil(self._images[ns])
            if pil_img is None:
                continue

            with self._model_lock:
                result = _run_florence2(
                    self._model, self._processor, pil_img,
                    "<CAPTION_TO_PHRASE_GROUNDING>", self._goal_prompt, self._device,
                )

            hits = self._parse_grounding_result(ns, result, pil_img, self._goal_prompt)
            if hits:
                self._merge_detections(hits)
                self._publish_detections()

                # Trigger detailed description if cooldown elapsed
                now = time.monotonic()
                last = self._last_desc_time.get(ns, 0.0)
                if now - last >= self._desc_cooldown:
                    self._last_desc_time[ns] = now
                    self._describe_scene(ns, pil_img, hits)

    def _parse_grounding_result(
        self, ns: str, result: dict, pil_img, goal: str,
    ) -> list[dict]:
        """Parse grounding result and filter by confidence."""
        detections = []
        gr = result.get("<CAPTION_TO_PHRASE_GROUNDING>", {})
        bboxes = gr.get("bboxes", [])
        labels = gr.get("labels", [])
        img_w, img_h = pil_img.width, pil_img.height
        img_area = max(1, img_w * img_h)

        odom = self._odoms.get(ns)
        if odom is None:
            return detections

        for bbox, label in zip(bboxes, labels):
            x1, y1, x2, y2 = bbox
            bbox_area = (x2 - x1) * (y2 - y1)
            bbox_frac = bbox_area / img_area
            if bbox_frac < self._grounding_conf:
                continue

            cx = (x1 + x2) / 2.0
            world_x, world_y = self._project_to_world(cx, img_w, odom)
            detections.append({
                "label": f"goal:{goal}",
                "x": round(world_x, 2),
                "y": round(world_y, 2),
                "confidence": round(min(1.0, bbox_frac * 5), 2),
                "robot": ns,
                "source": "florence2_grounding",
                "bbox": [round(v, 1) for v in [x1, y1, x2, y2]],
            })
        return detections

    # ── Task 3: Detailed description on trigger ──

    def _describe_scene(self, ns: str, pil_img, trigger_dets: list[dict]):
        """Run <MORE_DETAILED_CAPTION> and publish the description."""
        with self._model_lock:
            result = _run_florence2(
                self._model, self._processor, pil_img,
                "<MORE_DETAILED_CAPTION>", "", self._device,
            )

        caption = result.get("<MORE_DETAILED_CAPTION>", "")
        if not caption:
            return

        payload = {
            "robot": ns,
            "goal_prompt": self._goal_prompt,
            "caption": caption,
            "trigger_detections": trigger_dets,
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self._desc_pub.publish(msg)

        self.get_logger().info(
            f"[{ns}] Florence-2 description: {str(caption)[:120]}"
        )

    # ── World projection ──

    def _project_to_world(self, pixel_cx: float, img_w: int, odom: Odometry) -> tuple[float, float]:
        """Project a horizontal pixel center to a world (x, y) estimate."""
        bearing_offset = ((pixel_cx / max(1, img_w)) - 0.5) * self._hfov
        robot_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        world_bearing = robot_yaw + bearing_offset
        wx = odom.pose.pose.position.x + self._assumed_depth * math.cos(world_bearing)
        wy = odom.pose.pose.position.y + self._assumed_depth * math.sin(world_bearing)
        return wx, wy

    # ── Detection registry ──

    def _merge_detections(self, new_dets: list[dict]):
        for d in new_dets:
            merged = False
            for existing in self._registry:
                dx = float(d["x"]) - float(existing["x"])
                dy = float(d["y"]) - float(existing["y"])
                if math.hypot(dx, dy) < self._dedup_r:
                    if float(d.get("confidence", 0)) > float(existing.get("confidence", 0)):
                        existing.update(d)
                    merged = True
                    break
            if not merged:
                d["id"] = f"florence_{len(self._registry)}"
                self._registry.append(d)

    def _publish_detections(self):
        msg = String()
        msg.data = json.dumps(self._registry)
        self._det_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Florence2DetectorNode()
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
