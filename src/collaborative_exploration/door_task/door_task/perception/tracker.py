"""Per-camera IoU tracker (SORT-lite, no Kalman filter).

Greedy IoU matching of new detections against existing tracks. A track
that misses too many consecutive frames is dropped. Tracks are NOT
shared across cameras — the world_dict layer does cross-camera
association by world_xy proximity.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import count
from typing import Iterable

from .detector import Detection


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0 = max(ax0, bx0)
    iy0 = max(ay0, by0)
    ix1 = min(ax1, bx1)
    iy1 = min(ay1, by1)
    iw = max(0.0, ix1 - ix0)
    ih = max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


@dataclass
class Track:
    track_id: int
    last_det: Detection
    hits: int = 1
    misses: int = 0


class IouTracker:
    def __init__(self, iou_threshold: float = 0.3, max_misses: int = 5) -> None:
        self._iou_threshold = iou_threshold
        self._max_misses = max_misses
        self._tracks: dict[int, Track] = {}
        self._ids = count(1)

    @property
    def tracks(self) -> dict[int, Track]:
        return self._tracks

    def step(self, detections: Iterable[Detection]) -> dict[int, Detection]:
        """Match `detections` to existing tracks. Returns track_id → matched detection."""
        dets = list(detections)
        assigned_dets: set[int] = set()
        matched: dict[int, Detection] = {}

        for tid, trk in list(self._tracks.items()):
            best_i = -1
            best_iou = self._iou_threshold
            for i, d in enumerate(dets):
                if i in assigned_dets:
                    continue
                v = iou(trk.last_det.bbox_xyxy, d.bbox_xyxy)
                if v >= best_iou:
                    best_iou = v
                    best_i = i
            if best_i >= 0:
                d = dets[best_i]
                trk.last_det = d
                trk.hits += 1
                trk.misses = 0
                matched[tid] = d
                assigned_dets.add(best_i)
            else:
                trk.misses += 1
                if trk.misses > self._max_misses:
                    del self._tracks[tid]

        for i, d in enumerate(dets):
            if i in assigned_dets:
                continue
            tid = next(self._ids)
            self._tracks[tid] = Track(track_id=tid, last_det=d)
            matched[tid] = d

        return matched
