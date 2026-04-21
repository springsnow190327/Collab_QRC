"""Rolling spatial memory keyed by track id and color label.

For Phase 1a the entries store enough for the planner to query
"give me the most-confident red object" and get a world (x, y).
Confidence accumulates each time the same track is observed; entries
older than `decay_sec` drop unless re-seen.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass
class WorldEntry:
    entry_id: int
    world_xy: tuple[float, float]
    color_label: str
    yolo_class: str
    rgb: tuple[int, int, int]
    hits: int
    last_seen_t: float
    first_seen_t: float
    semantic_label: str = ""
    semantic_conf: float = 0.0

    def confidence(self, now: float, decay_sec: float) -> float:
        age = max(0.0, now - self.last_seen_t)
        recency = max(0.0, 1.0 - age / decay_sec)
        return min(1.0, 0.2 + 0.1 * self.hits) * recency

    def to_dict(self, now: float, decay_sec: float) -> dict:
        d = asdict(self)
        d["world_xy"] = list(self.world_xy)
        d["rgb"] = list(self.rgb)
        d["confidence"] = round(self.confidence(now, decay_sec), 3)
        d["semantic_conf"] = round(self.semantic_conf, 3)
        d["age_sec"] = round(now - self.last_seen_t, 2)
        return d


class WorldDict:
    """Cross-camera fusion by world-xy nearest-neighbor.

    A new observation is merged into the closest existing entry within
    `merge_radius_m`; otherwise it spawns a new entry. Entries decay out
    once their last observation is older than `decay_sec`.
    """

    def __init__(
        self,
        merge_radius_m: float = 0.6,
        decay_sec: float = 30.0,
    ) -> None:
        self._merge_r = merge_radius_m
        self._decay_sec = decay_sec
        self._entries: dict[int, WorldEntry] = {}
        self._next_id = 1

    @property
    def decay_sec(self) -> float:
        return self._decay_sec

    def observe(
        self,
        world_xy: tuple[float, float],
        color_label: str,
        yolo_class: str,
        rgb: tuple[int, int, int],
        now: float,
        semantic_label: str = "",
        semantic_conf: float = 0.0,
    ) -> WorldEntry:
        # find nearest entry
        best_id = -1
        best_dist = self._merge_r
        for eid, e in self._entries.items():
            dx = e.world_xy[0] - world_xy[0]
            dy = e.world_xy[1] - world_xy[1]
            d = math.hypot(dx, dy)
            if d < best_dist:
                best_dist = d
                best_id = eid
        if best_id >= 0:
            e = self._entries[best_id]
            # exponential moving average of position
            alpha = 0.3
            ex = (1 - alpha) * e.world_xy[0] + alpha * world_xy[0]
            ey = (1 - alpha) * e.world_xy[1] + alpha * world_xy[1]
            e.world_xy = (ex, ey)
            e.hits += 1
            e.last_seen_t = now
            if color_label != "other":
                e.color_label = color_label
            e.rgb = rgb
            # Keep the highest-confidence semantic label seen so far,
            # or overwrite if the current one is more confident.
            if semantic_label and semantic_conf > e.semantic_conf:
                e.semantic_label = semantic_label
                e.semantic_conf = semantic_conf
            return e
        eid = self._next_id
        self._next_id += 1
        e = WorldEntry(
            entry_id=eid,
            world_xy=world_xy,
            color_label=color_label,
            yolo_class=yolo_class,
            rgb=rgb,
            hits=1,
            last_seen_t=now,
            first_seen_t=now,
            semantic_label=semantic_label,
            semantic_conf=semantic_conf,
        )
        self._entries[eid] = e
        return e

    def prune(self, now: float) -> None:
        dead = [eid for eid, e in self._entries.items() if now - e.last_seen_t > self._decay_sec]
        for eid in dead:
            del self._entries[eid]

    def query_by_color(self, color: str, now: float) -> Optional[WorldEntry]:
        candidates = [
            e for e in self._entries.values()
            if e.color_label == color and now - e.last_seen_t <= self._decay_sec
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda e: e.confidence(now, self._decay_sec))

    def query_by_semantic(self, label: str, now: float) -> Optional[WorldEntry]:
        """Return the freshest entry whose pooled CLIP label matches `label`.

        Ranks by semantic_conf × spatial confidence — both a stable track
        and a clear semantic match are required to win.
        """
        candidates = [
            e for e in self._entries.values()
            if e.semantic_label == label and now - e.last_seen_t <= self._decay_sec
        ]
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda e: e.semantic_conf * e.confidence(now, self._decay_sec),
        )

    def snapshot(self, now: float, top_k: int = 10) -> dict:
        self.prune(now)
        items = [e.to_dict(now, self._decay_sec) for e in self._entries.values()]
        items.sort(key=lambda d: d["confidence"], reverse=True)
        return {
            "now": round(now, 2),
            "decay_sec": self._decay_sec,
            "entries": items[:top_k],
        }
