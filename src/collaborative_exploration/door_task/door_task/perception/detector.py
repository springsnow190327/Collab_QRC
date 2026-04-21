"""YOLOv8n class-agnostic detector wrapper.

Pure compute: takes a numpy RGB image and returns a list of Detection
objects. The model loads on first use (or via .preload()) so the ROS
node can spin without blocking on the YOLO weights download.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import Optional

import numpy as np


@dataclass
class Detection:
    bbox_xyxy: tuple[float, float, float, float]   # pixel coords
    conf: float
    yolo_class: str
    dominant_rgb: tuple[int, int, int] = (0, 0, 0)  # mean color inside the box

    @property
    def cx(self) -> float:
        x0, _, x1, _ = self.bbox_xyxy
        return (x0 + x1) / 2.0

    @property
    def cy(self) -> float:
        _, y0, _, y1 = self.bbox_xyxy
        return (y0 + y1) / 2.0

    @property
    def area(self) -> float:
        x0, y0, x1, y1 = self.bbox_xyxy
        return max(0.0, x1 - x0) * max(0.0, y1 - y0)

    def color_label(self) -> str:
        """Coarse color bucket from the dominant RGB. The Phase 1a planner
        uses this as a stand-in for semantic labels until OWL-ViT lands."""
        r, g, b = self.dominant_rgb
        if r > 140 and r > g + 30 and r > b + 30:
            return "red"
        if g > 140 and g > r + 30 and g > b + 30:
            return "green"
        if b > 140 and b > r + 30 and b > g + 30:
            return "blue"
        if r > 200 and g > 200 and b > 200:
            return "white"
        if r < 60 and g < 60 and b < 60:
            return "black"
        return "other"


def _dominant_rgb(image: np.ndarray, bbox: tuple[float, float, float, float]) -> tuple[int, int, int]:
    h, w = image.shape[:2]
    x0 = max(0, int(bbox[0]))
    y0 = max(0, int(bbox[1]))
    x1 = min(w, int(bbox[2]))
    y1 = min(h, int(bbox[3]))
    if x1 <= x0 or y1 <= y0:
        return (0, 0, 0)
    crop = image[y0:y1, x0:x1]
    if crop.size == 0:
        return (0, 0, 0)
    mean = crop.reshape(-1, crop.shape[-1]).mean(axis=0)
    return tuple(int(v) for v in mean[:3])  # type: ignore[return-value]


class YoloDetector:
    """Thin wrapper around ultralytics.YOLO with a per-instance lock.

    Designed to be created once per process and shared across cameras
    (YOLO is GPU-bound; serializing calls avoids cudaMalloc thrash).
    """

    def __init__(
        self,
        model_id: str = "yolov8n.pt",
        device: str = "cuda",
        conf_threshold: float = 0.25,
        max_detections: int = 10,
    ) -> None:
        self.model_id = model_id
        self.device = device
        self.conf_threshold = conf_threshold
        self.max_detections = max_detections
        self._model = None
        self._lock = Lock()

    def preload(self) -> None:
        if self._model is not None:
            return
        from ultralytics import YOLO  # noqa: PLC0415
        self._model = YOLO(self.model_id)

    def is_ready(self) -> bool:
        return self._model is not None

    def run(self, image_rgb: np.ndarray) -> list[Detection]:
        if self._model is None:
            self.preload()
        with self._lock:
            results = self._model.predict(  # type: ignore[union-attr]
                image_rgb,
                conf=self.conf_threshold,
                device=self.device,
                verbose=False,
                max_det=self.max_detections,
            )
        if not results:
            return []
        r = results[0]
        names = r.names if hasattr(r, "names") else {}
        out: list[Detection] = []
        boxes = getattr(r, "boxes", None)
        if boxes is None:
            return out
        for i in range(len(boxes)):
            xyxy = boxes.xyxy[i].cpu().numpy().tolist()
            conf = float(boxes.conf[i].item())
            cls_idx = int(boxes.cls[i].item())
            cls_name = str(names.get(cls_idx, str(cls_idx)))
            color = _dominant_rgb(image_rgb, tuple(xyxy))  # type: ignore[arg-type]
            out.append(
                Detection(
                    bbox_xyxy=tuple(xyxy),  # type: ignore[arg-type]
                    conf=conf,
                    yolo_class=cls_name,
                    dominant_rgb=color,
                )
            )
        return out
