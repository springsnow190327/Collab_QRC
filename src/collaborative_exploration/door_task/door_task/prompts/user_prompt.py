"""User-prompt builder.

Converts an observation dict (poses + button state + panel metadata)
into the text body that accompanies the 2x2 image fed to the VLM.
"""

from __future__ import annotations

import json


def build_user_prompt(obs: dict) -> str:
    ra, rb = obs["robot_a"], obs["robot_b"]
    last_a = obs.get("last_action", {}).get("robot_a", {})
    last_b = obs.get("last_action", {}).get("robot_b", {})
    panels = obs.get("panel_meta", {})
    ma = panels.get("map_a", {})
    mb = panels.get("map_b", {})

    def _panel_extent(meta: dict, label: str) -> str:
        if not meta or meta.get("scale", 0) <= 0:
            return f"  map_{label}: not yet mapped"
        return (
            f"  map_{label}: world x ∈ [{meta['world_x_min']:+.1f}, "
            f"{meta['world_x_max']:+.1f}], "
            f"y ∈ [{meta['world_y_min']:+.1f}, {meta['world_y_max']:+.1f}]. "
            f"Grid lines on the panel are at every 1 m of world space; "
            f"axis labels are burned into each corner."
        )

    return "\n".join(
        [
            f"t = {obs['elapsed_sec']:.1f} s",
            "",
            "SLAM pose (shared world frame, meters / degrees):",
            f"  robot_a: ({ra['x']:+.2f}, {ra['y']:+.2f})  yaw={ra['yaw_deg']:+.0f}",
            f"  robot_b: ({rb['x']:+.2f}, {rb['y']:+.2f})  yaw={rb['yaw_deg']:+.0f}",
            "",
            f"button_pressed = {obs.get('button_pressed', False)}   "
            f"button_ever_pressed = {obs.get('button_ever_pressed', False)}",
            "",
            f"last_action: robot_a={json.dumps(last_a)}  robot_b={json.dumps(last_b)}",
            "",
            "Image attached (2x2 layout):",
            "  top-left  = robot_a front camera",
            "  top-right = robot_b front camera",
            "  bottom-left  = robot_a SLAM occupancy grid (red dot = own pose)",
            "  bottom-right = robot_b SLAM occupancy grid (red dot = own pose)",
            "SLAM maps show black=occupied, light-green=free, gray=unknown.",
            "Each SLAM panel has burned-in world coordinate labels at the",
            "corners and 1m grid lines — you can read off any feature's",
            "approximate world (x, y) by counting cells from the corner.",
            "Panel extents (this tick):",
            _panel_extent(ma, "a"),
            _panel_extent(mb, "b"),
            "",
            "Output JSON action for both robots.",
        ]
    )
