from __future__ import annotations

import json
from typing import Any


TOOL_SCHEMAS = [
    {
        "name": "assign_waypoints",
        "description": (
            "Optionally override the baseline explorer with waypoint assignments for one or more robots. "
            "Only use this when the image or detections provide strong semantic evidence."
        ),
        "parameters": {
            "assignments": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "robot": {"type": "string"},
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "reason": {"type": "string"},
                    },
                    "required": ["robot", "x", "y"],
                },
            }
        },
    },
    {
        "name": "mark_artifact_seen",
        "description": (
            "Record that a candidate artifact has been semantically recognized. "
            "Use this when the camera view strongly suggests the target object has been found."
        ),
        "parameters": {
            "robot": {"type": "string"},
            "artifact_id": {"type": "string"},
            "label": {"type": "string"},
            "reason": {"type": "string"},
        },
    },
    {
        "name": "interact_with_artifact",
        "description": (
            "Request a placeholder interaction with a detected artifact. "
            "Use only when the robot is already near the target."
        ),
        "parameters": {
            "robot": {"type": "string"},
            "artifact_id": {"type": "string"},
            "action": {"type": "string"},
            "reason": {"type": "string"},
        },
    },
]


def build_system_prompt(mission: str = "") -> str:
    mission_block = ""
    if mission:
        mission_block = f"""
USER MISSION: {mission}
Prioritize directing the robot toward anything that matches or could match this mission.
When the robot is near a candidate, use mark_artifact_seen to log it.
"""
    return f"""\
You are a low-frequency semantic exploration advisor layered on top of an existing ROS2 frontier explorer.
{mission_block}
Important operating rules:
1. The baseline exploration stack is already running and keeps coverage growing on its own.
2. Only override the baseline with assign_waypoints when the rendered map, camera-derived detections, or scene context provide meaningful semantic guidance.
3. If the scene is ambiguous, return an empty tool_calls list so the baseline planner continues unchanged.
4. Keep robots spread out and avoid churn. Do not oscillate goals unless there is new evidence.
5. Use interact_with_artifact only when the robot is already close to a detected artifact.
6. Respond with ONLY a JSON object in the format {{"tool_calls": [...]}} and no extra text.
"""


def build_user_prompt(scene: dict[str, Any]) -> str:
    return (
        "Current scene snapshot:\n"
        "```json\n"
        f"{json.dumps(scene, indent=2)}\n"
        "```\n\n"
        "Use the rendered map image plus this JSON to decide whether semantic evidence warrants overriding the baseline explorer. "
        "If not, return {\"tool_calls\": []}."
    )
