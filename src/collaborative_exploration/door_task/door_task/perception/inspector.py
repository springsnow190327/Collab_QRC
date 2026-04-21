"""Open-vocabulary semantic inspector with temporal softmax pooling.

Phase 1b: CLIP scores per-detection crops against a fixed set of
open-vocab text queries (e.g. "red button", "door", "other robot").
For each IoU track, the inspector keeps a rolling window of the most
recent `window` softmax vectors; pooling sums the probabilities across
the window, renormalizes, and picks the top query — stable detections
dominate fleeting ones.

No ROS imports; the class is pure compute. The ROS perception_node
calls `observe()` once per tracker step.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class SemanticScore:
    label: str
    confidence: float


def _crop(image: np.ndarray, bbox: tuple[float, float, float, float]) -> Optional[np.ndarray]:
    h, w = image.shape[:2]
    x0 = max(0, int(bbox[0]))
    y0 = max(0, int(bbox[1]))
    x1 = min(w, int(bbox[2]))
    y1 = min(h, int(bbox[3]))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return image[y0:y1, x0:x1]


def temporal_pool(history: Iterable[np.ndarray]) -> np.ndarray:
    """Phase 1b pooling:

    1. Sum per-query probabilities across the window.
    2. Normalize by the number of frames so the result remains a
       probability-ish vector (each entry is the mean P across the
       window). A single noisy detection with P=0.9 contributes 0.9/N,
       five stable detections of P=0.5 contribute 0.5. The stable
       track wins.
    3. Row-normalize one more time so the returned vector sums to 1.
    """
    arr = np.stack(list(history), axis=0)  # (N, Q)
    mean = arr.mean(axis=0)
    s = float(mean.sum())
    if s <= 0:
        return np.ones_like(mean) / len(mean)
    return mean / s


class SemanticInspector:
    """CLIP-backed crop classifier with per-track temporal pooling.

    Parameters
    ----------
    queries      : list of open-vocab text labels to score against
    window       : number of recent frames to pool per track
    model_id     : HuggingFace CLIP model id
    device       : 'cuda' or 'cpu'
    min_confidence : below this pooled prob, return label=''
    """

    def __init__(
        self,
        queries: list[str],
        window: int = 5,
        model_id: str = "openai/clip-vit-base-patch32",
        device: str = "cuda",
        min_confidence: float = 0.35,
    ) -> None:
        self._queries = list(queries)
        self._window = window
        self._model_id = model_id
        self._device = device
        self._min_conf = min_confidence
        self._model = None
        self._processor = None
        self._text_features = None
        self._history: dict[int, deque] = {}
        self._lock = Lock()

    @property
    def queries(self) -> list[str]:
        return list(self._queries)

    def is_ready(self) -> bool:
        return self._model is not None

    def preload(self) -> None:
        """Load the CLIP model. Text tokens are pre-computed once; full
        forward passes compute similarity each inference call (the
        canonical CLIPModel.forward path, version-independent)."""
        if self._model is not None:
            return
        import torch  # noqa: PLC0415
        from transformers import CLIPModel, CLIPProcessor  # noqa: PLC0415

        self._torch = torch
        self._processor = CLIPProcessor.from_pretrained(self._model_id)
        self._model = CLIPModel.from_pretrained(self._model_id).to(self._device).eval()

    def forget(self, track_id: int) -> None:
        with self._lock:
            self._history.pop(track_id, None)

    def prune(self, live_track_ids: set[int]) -> None:
        with self._lock:
            dead = [tid for tid in self._history if tid not in live_track_ids]
            for tid in dead:
                del self._history[tid]

    def observe(
        self,
        image_rgb: np.ndarray,
        tracks: dict[int, tuple[float, float, float, float]],
    ) -> dict[int, SemanticScore]:
        """Score each tracked bbox on the current frame, update per-track
        history, and return the pooled best label per track.
        """
        if self._model is None:
            return {}
        if not tracks:
            return {}

        crops: list[np.ndarray] = []
        tids: list[int] = []
        for tid, bbox in tracks.items():
            crop = _crop(image_rgb, bbox)
            if crop is None:
                continue
            crops.append(crop)
            tids.append(tid)
        if not crops:
            return {}

        probs_per_crop = self._score_crops(crops)  # (K, Q)

        out: dict[int, SemanticScore] = {}
        with self._lock:
            for i, tid in enumerate(tids):
                hist = self._history.get(tid)
                if hist is None:
                    hist = deque(maxlen=self._window)
                    self._history[tid] = hist
                hist.append(probs_per_crop[i])
                pooled = temporal_pool(hist)
                best_idx = int(pooled.argmax())
                conf = float(pooled[best_idx])
                label = self._queries[best_idx] if conf >= self._min_conf else ""
                out[tid] = SemanticScore(label=label, confidence=conf)
        return out

    def _score_crops(self, crops: list[np.ndarray]) -> np.ndarray:
        """Run CLIP on a batch of crops; return softmax(probs) per crop over queries.

        Uses the canonical ``CLIPModel(**inputs).logits_per_image`` path —
        this is version-stable across transformers 4.x and 5.x where the
        convenience methods (``get_text_features`` / ``get_image_features``)
        have changed return types.
        """
        import torch  # noqa: PLC0415
        from PIL import Image as PILImage  # noqa: PLC0415

        pil_crops = [PILImage.fromarray(c, mode="RGB") for c in crops]
        with torch.no_grad():
            inputs = self._processor(
                text=self._queries,
                images=pil_crops,
                return_tensors="pt",
                padding=True,
            )
            inputs = {k: v.to(self._device) for k, v in inputs.items()}
            out = self._model(**inputs)
            logits_per_image = out.logits_per_image  # (K, Q), already temperature-scaled
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()
        return probs
