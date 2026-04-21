"""Perception ROS node — YOLO + IoU tracker + CLIP inspector + world_dict.

Subscribes to two robot front cameras (color + **metric depth** from
the mujoco_depth_camera plugin), plus odom. Runs YOLOv8n on a worker
thread, tracks bboxes per camera with IoU, scores each crop with CLIP
for open-vocab labels, and unprojects bbox centers to world (x, y)
using the real depth sampled at each bbox centroid.

TOY SLAM NOTICE
===============
The unprojection relies on ground-truth depth (no noise, no holes) and
ground-truth odometry. This is adequate for the MuJoCo door-task but
NOT for real hardware — see ``perception/projection.py`` for the path
a real robot should take. The node falls back to ``fallback_depth_m``
only if the depth topic is silent, which on real hardware is the
wrong fallback (it just hides the problem).
"""

from __future__ import annotations

import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from door_task.core.geometry import yaw_from_quat
from door_task.core.rendering import image_msg_to_np
from door_task.perception.detector import YoloDetector
from door_task.perception.inspector import SemanticInspector
from door_task.perception.projection import CameraIntrinsics, project_to_world
from door_task.perception.tracker import IouTracker
from door_task.perception.world_dict import WorldDict


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.declare_parameter("robot_namespaces", ["robot_a", "robot_b"])
        self.declare_parameter(
            "camera_topics",
            ["/front_camera/color/image_raw", "/b_front_camera/color/image_raw"],
        )
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fovy_deg", 80.0)
        # Real depth from /{cam}/depth/image_rect_raw is primary; this is
        # the fallback used only if the depth frame is missing (should
        # rarely trigger in sim).
        self.declare_parameter("fallback_depth_m", 2.5)
        self.declare_parameter(
            "depth_topics",
            [
                "/front_camera/depth/image_rect_raw",
                "/b_front_camera/depth/image_rect_raw",
            ],
        )
        # Scene bounds clamp — reject any world_xy that lands outside the
        # door-task room, which would be a projection / SLAM hallucination.
        self.declare_parameter("scene_x_min", 0.0)
        self.declare_parameter("scene_x_max", 8.0)
        self.declare_parameter("scene_y_min", 0.0)
        self.declare_parameter("scene_y_max", 4.0)
        self.declare_parameter("detect_rate", 5.0)
        self.declare_parameter("conf_threshold", 0.20)
        self.declare_parameter("max_detections", 10)
        self.declare_parameter("model_id", "yolov8n.pt")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("publish_rate", 4.0)
        self.declare_parameter("merge_radius_m", 0.6)
        self.declare_parameter("decay_sec", 30.0)
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("max_misses", 5)
        # Phase 1b semantic inspector
        self.declare_parameter("inspector_enabled", True)
        self.declare_parameter(
            "inspector_queries",
            [
                "red button",
                "red pressure pad",
                "door",
                "wall",
                "floor",
                "robot",
                "unknown object",
            ],
        )
        self.declare_parameter("inspector_window", 5)
        self.declare_parameter("inspector_model_id", "openai/clip-vit-base-patch32")
        self.declare_parameter("inspector_min_confidence", 0.35)

        self._namespaces = [str(x) for x in self.get_parameter("robot_namespaces").value]
        cam_topics = list(self.get_parameter("camera_topics").value)
        if len(cam_topics) != len(self._namespaces):
            raise ValueError("camera_topics length must match robot_namespaces")

        self._intr = CameraIntrinsics.from_fovy_deg(
            int(self.get_parameter("camera_width").value),
            int(self.get_parameter("camera_height").value),
            float(self.get_parameter("camera_fovy_deg").value),
        )
        self._fallback_depth = float(self.get_parameter("fallback_depth_m").value)
        self._scene_bounds = (
            float(self.get_parameter("scene_x_min").value),
            float(self.get_parameter("scene_x_max").value),
            float(self.get_parameter("scene_y_min").value),
            float(self.get_parameter("scene_y_max").value),
        )

        self._detector = YoloDetector(
            model_id=str(self.get_parameter("model_id").value),
            device=str(self.get_parameter("device").value),
            conf_threshold=float(self.get_parameter("conf_threshold").value),
            max_detections=int(self.get_parameter("max_detections").value),
        )
        self._world_dict = WorldDict(
            merge_radius_m=float(self.get_parameter("merge_radius_m").value),
            decay_sec=float(self.get_parameter("decay_sec").value),
        )
        self._trackers = {
            ns: IouTracker(
                iou_threshold=float(self.get_parameter("iou_threshold").value),
                max_misses=int(self.get_parameter("max_misses").value),
            )
            for ns in self._namespaces
        }
        if bool(self.get_parameter("inspector_enabled").value):
            self._inspector = SemanticInspector(
                queries=[str(q) for q in self.get_parameter("inspector_queries").value],
                window=int(self.get_parameter("inspector_window").value),
                model_id=str(self.get_parameter("inspector_model_id").value),
                device=str(self.get_parameter("device").value),
                min_confidence=float(self.get_parameter("inspector_min_confidence").value),
            )
        else:
            self._inspector = None

        self._frames: dict[str, np.ndarray] = {}
        self._depth_frames: dict[str, np.ndarray] = {}
        self._odoms: dict[str, Odometry] = {}
        self._frames_lock = threading.Lock()
        self._dict_lock = threading.Lock()
        self._busy = False
        self._depth_miss_warned = False

        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        depth_topics = list(self.get_parameter("depth_topics").value)
        if len(depth_topics) != len(self._namespaces):
            raise ValueError("depth_topics length must match robot_namespaces")
        for ns, topic, dtopic in zip(self._namespaces, cam_topics, depth_topics):
            self.create_subscription(
                Image, topic,
                lambda msg, _ns=ns: self._on_image(_ns, msg),
                cam_qos,
            )
            self.create_subscription(
                Image, dtopic,
                lambda msg, _ns=ns: self._on_depth(_ns, msg),
                cam_qos,
            )
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav",
                lambda msg, _ns=ns: self._on_odom(_ns, msg),
                10,
            )

        self._pub = self.create_publisher(String, "/perception/world_dict", 10)
        # Debug annotated camera frames — published as a JSON blob mapping
        # namespace to a base64 JPEG of the RGB frame with YOLO boxes + IoU
        # track ids + CLIP semantic labels overlaid. Consumed by the web
        # dashboard so the user can see what the detector is actually doing.
        #
        # BEST_EFFORT + depth=1: image payloads are ~20 KB per frame, so
        # we don't want DDS reliable backpressure stalling the detect loop
        # if the dashboard is a slow consumer. The dashboard polls /state
        # at 4 Hz anyway so a single-slot buffer is plenty.
        dbg_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._debug_image_pub = self.create_publisher(
            String, "/perception/debug_image", dbg_qos
        )
        self._latest_annotated: dict[str, str] = {}

        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="yolo")
        # Background model warm-up so the first tick doesn't stall.
        threading.Thread(target=self._preload, daemon=True).start()

        self.create_timer(1.0 / float(self.get_parameter("detect_rate").value), self._detect_tick)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish_tick)
        self._t0 = time.monotonic()

        self.get_logger().info(
            f"PerceptionNode up | ns={self._namespaces} | model={self._detector.model_id} "
            f"| device={self._detector.device} | depth=real (fallback={self._fallback_depth:.1f}m)"
        )

    def _preload(self) -> None:
        try:
            self._detector.preload()
            self.get_logger().info(f"YOLO model loaded: {self._detector.model_id}")
        except Exception as exc:
            self.get_logger().error(f"YOLO preload failed: {exc}")
        if self._inspector is not None:
            try:
                self._inspector.preload()
                self.get_logger().info(
                    f"CLIP inspector loaded: queries={self._inspector.queries}"
                )
            except Exception as exc:
                self.get_logger().error(f"CLIP inspector preload failed: {exc}")
                self._inspector = None

    def _on_image(self, ns: str, msg: Image) -> None:
        try:
            arr = image_msg_to_np(msg)
        except Exception as exc:
            self.get_logger().warn(f"{ns} image decode failed: {exc}")
            return
        with self._frames_lock:
            self._frames[ns] = arr

    def _on_depth(self, ns: str, msg: Image) -> None:
        """Decode the mujoco_depth_camera depth frame into a float32 meters array.

        The plugin publishes ``32FC1`` with linearized metric depth along
        the optical axis. We also accept ``16UC1`` (RealSense-style mm)
        in case we reuse this node on hardware.
        """
        try:
            h, w = int(msg.height), int(msg.width)
            if h == 0 or w == 0:
                return
            enc = str(msg.encoding).lower()
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if enc in ("32fc1", ""):
                arr = raw.view(np.float32).reshape((h, w))
            elif enc == "16uc1":
                arr = raw.view(np.uint16).reshape((h, w)).astype(np.float32) * 0.001
            else:
                self.get_logger().warn(f"{ns} depth encoding {enc!r} unsupported")
                return
        except Exception as exc:
            self.get_logger().warn(f"{ns} depth decode failed: {exc}")
            return
        with self._frames_lock:
            self._depth_frames[ns] = arr

    def _sample_depth(self, ns: str, cx: float, cy: float) -> Optional[float]:
        """Median of a 5x5 patch around (cx, cy) in the latest depth frame.

        Rejects NaN/Inf and values outside [0.05, 20.0] m. Returns None
        if no valid sample is available, so the caller can fall back.
        """
        with self._frames_lock:
            frame = self._depth_frames.get(ns)
        if frame is None:
            return None
        h, w = frame.shape
        ix = int(round(cx))
        iy = int(round(cy))
        if not (0 <= ix < w and 0 <= iy < h):
            return None
        x0, x1 = max(0, ix - 2), min(w, ix + 3)
        y0, y1 = max(0, iy - 2), min(h, iy + 3)
        patch = frame[y0:y1, x0:x1]
        mask = np.isfinite(patch) & (patch > 0.05) & (patch < 20.0)
        if not mask.any():
            return None
        return float(np.median(patch[mask]))

    def _in_scene(self, x: float, y: float) -> bool:
        x_min, x_max, y_min, y_max = self._scene_bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _on_odom(self, ns: str, msg: Odometry) -> None:
        self._odoms[ns] = msg

    def _detect_tick(self) -> None:
        if not self._detector.is_ready() or self._busy:
            return
        with self._frames_lock:
            frames = dict(self._frames)
        if not frames:
            return
        self._busy = True
        self._pool.submit(self._run_detection, frames)

    def _run_detection(self, frames: dict[str, np.ndarray]) -> None:
        try:
            now = time.monotonic() - self._t0
            for ns, frame in frames.items():
                od = self._odoms.get(ns)
                if od is None:
                    continue
                rx = od.pose.pose.position.x
                ry = od.pose.pose.position.y
                ryaw = yaw_from_quat(od.pose.pose.orientation)
                try:
                    detections = self._detector.run(frame)
                except Exception as exc:
                    self.get_logger().warn(f"{ns} YOLO failed: {exc}")
                    continue
                tracked = self._trackers[ns].step(detections)
                # Phase 1b: score each tracked crop with CLIP over the
                # configured open-vocab queries, pool across window.
                semantic: dict[int, tuple[str, float]] = {}
                if self._inspector is not None and self._inspector.is_ready() and tracked:
                    try:
                        bboxes = {tid: det.bbox_xyxy for tid, det in tracked.items()}
                        scores = self._inspector.observe(frame, bboxes)
                        semantic = {tid: (s.label, s.confidence) for tid, s in scores.items()}
                        self._inspector.prune(set(self._trackers[ns].tracks.keys()))
                    except Exception as exc:
                        self.get_logger().warn(f"{ns} CLIP inspector failed: {exc}")
                # Debug: annotate the frame with boxes + track IDs + semantic
                # labels and immediately publish so the dashboard sees every
                # detector frame, not just every slow _publish_tick. This
                # also works fine even when `tracked` is empty — the user
                # still sees the raw camera view.
                try:
                    self._latest_annotated[ns] = self._annotate_frame(
                        frame, tracked, semantic
                    )
                except Exception as exc:
                    self.get_logger().warn(f"{ns} debug annotate failed: {exc}")
                if self._latest_annotated:
                    payload = {
                        "t": round(now, 2),
                        "cameras": dict(self._latest_annotated),
                    }
                    try:
                        self._debug_image_pub.publish(String(data=json.dumps(payload)))
                    except Exception:
                        pass
                with self._dict_lock:
                    for tid, det in tracked.items():
                        depth = self._sample_depth(ns, det.cx, det.cy)
                        if depth is None:
                            if not self._depth_miss_warned:
                                self.get_logger().warn(
                                    f"{ns} depth sample missing — using "
                                    f"fallback {self._fallback_depth:.1f}m. "
                                    "Check that mujoco_depth_camera is publishing."
                                )
                                self._depth_miss_warned = True
                            depth = self._fallback_depth
                        wx, wy = project_to_world(
                            self._intr, det.cx, rx, ry, ryaw, depth
                        )
                        if not self._in_scene(wx, wy):
                            # Out-of-bounds projection — skip so we don't
                            # poison the world_dict with ghost entries.
                            # (Real hardware should log these instead of
                            # silently dropping, but for toy sim this is
                            # safer than letting the planner drive the
                            # robot into a wall.)
                            continue
                        sem_label, sem_conf = semantic.get(tid, ("", 0.0))
                        self._world_dict.observe(
                            world_xy=(wx, wy),
                            color_label=det.color_label(),
                            yolo_class=det.yolo_class,
                            rgb=det.dominant_rgb,
                            now=now,
                            semantic_label=sem_label,
                            semantic_conf=sem_conf,
                        )
        finally:
            self._busy = False

    def _annotate_frame(
        self,
        frame: np.ndarray,
        tracked: dict,
        semantic: dict[int, tuple[str, float]],
    ) -> str:
        """Draw YOLO bboxes + track IDs + CLIP semantic labels on an RGB
        frame and return a base64 JPEG string. Uses PIL only (already a
        hard dep); no OpenCV."""
        from PIL import Image as PILImage  # noqa: PLC0415
        from PIL import ImageDraw, ImageFont  # noqa: PLC0415
        import base64 as _base64  # noqa: PLC0415
        import io as _io  # noqa: PLC0415

        # Downsample so the dashboard payload stays small (target ~640 px wide).
        h, w = frame.shape[:2]
        target_w = 640
        if w > target_w:
            stride = max(1, w // target_w)
            frame = frame[::stride, ::stride]
            scale = 1.0 / stride
        else:
            scale = 1.0

        pim = PILImage.fromarray(frame, mode="RGB")
        draw = ImageDraw.Draw(pim)
        try:
            font = ImageFont.load_default()
        except Exception:
            font = None

        for tid, det in tracked.items():
            x0, y0, x1, y1 = (c * scale for c in det.bbox_xyxy)
            sem_label, sem_conf = semantic.get(tid, ("", 0.0))
            # Color by semantic: red for red-button matches, green for
            # confident semantic hits, yellow for tracked-but-unlabeled.
            if sem_label and ("red" in sem_label.lower() and sem_conf >= 0.55):
                outline = (255, 64, 64)
            elif sem_label and sem_conf >= 0.55:
                outline = (64, 255, 128)
            else:
                outline = (255, 210, 64)
            draw.rectangle([x0, y0, x1, y1], outline=outline, width=2)
            label = (
                f"#{tid} {sem_label or det.yolo_class or '?'} {sem_conf:.2f}"
                if sem_label
                else f"#{tid} {det.yolo_class or '?'} {det.conf:.2f}"
            )
            draw.text((x0 + 2, max(0, y0 - 12)), label, fill=outline, font=font)

        buf = _io.BytesIO()
        pim.save(buf, format="JPEG", quality=60)
        return _base64.b64encode(buf.getvalue()).decode("ascii")

    def _publish_tick(self) -> None:
        now = time.monotonic() - self._t0
        with self._dict_lock:
            snap = self._world_dict.snapshot(now)
        try:
            self._pub.publish(String(data=json.dumps(snap)))
        except Exception:
            pass
        # debug_image is published directly from _run_detection now so
        # the dashboard sees every detector frame rather than the slower
        # world_dict publish rate.


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
