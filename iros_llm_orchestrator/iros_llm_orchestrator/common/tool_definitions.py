"""Tool definitions (OpenAI/Ollama function-calling format) for user chat channel 3.

These are passed verbatim to the LLM backend in the `tools` field.
"""

from __future__ import annotations

import json

TOOL_DEFINITIONS: list[dict] = [
    {
        "type": "function",
        "function": {
            "name": "check_occupancy",
            "description": (
                "Get the local occupancy grid around a robot from its lidar. "
                "Returns free and occupied cell centers in map-frame coordinates. "
                "Use before placing a robot at a precise position or planning a formation."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "robot_id": {
                        "type": "string",
                        "description": "Robot namespace, e.g. 'robot_3'",
                    }
                },
                "required": ["robot_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_positions",
            "description": (
                "Look up named geometric points on the map: room corners, doorways, "
                "chokepoints. Use before sending a robot to a spatially specific "
                "location like 'top-right corner of Electrical'."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "room": {
                        "type": "string",
                        "description": (
                            "Room name as it appears in the map config's geometry "
                            "section, e.g. 'electrical'"
                        ),
                    },
                    "qualifier": {
                        "type": "string",
                        "description": (
                            "Dot-path qualifier, e.g. 'corners.top_right', "
                            "'doorways.north', 'center'"
                        ),
                    },
                },
                "required": ["room", "qualifier"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_position",
            "description": (
                "Get the current map-frame pose (x, y, yaw) of a specific robot."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "robot_id": {
                        "type": "string",
                        "description": "Robot namespace, e.g. 'robot_3'",
                    }
                },
                "required": ["robot_id"],
            },
        },
    },
]


def parse_ollama_tool_calls(message: dict) -> list[dict] | None:
    """Extract tool calls from an Ollama /api/chat response message dict.

    Returns list of {"name": str, "arguments": dict, "call_id": str}
    or None if no tool calls are present.
    """
    raw_calls = message.get("tool_calls")
    if not raw_calls:
        return None
    result: list[dict] = []
    for i, tc in enumerate(raw_calls):
        fn = tc.get("function", {})
        name = fn.get("name", "")
        args = fn.get("arguments", {})
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except Exception:
                args = {}
        if not isinstance(args, dict):
            args = {}
        result.append({"name": name, "arguments": args, "call_id": f"ollama_{i}"})
    return result or None


def parse_openai_tool_calls(choice: dict) -> list[dict] | None:
    """Extract tool calls from an OpenAI /v1/chat/completions response choice dict.

    Returns list of {"name": str, "arguments": dict, "call_id": str}
    or None if no tool calls are present.
    """
    message = choice.get("message", {})
    raw_calls = message.get("tool_calls")
    if not raw_calls:
        return None
    result: list[dict] = []
    for tc in raw_calls:
        fn = tc.get("function", {})
        name = fn.get("name", "")
        args = fn.get("arguments", {})
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except Exception:
                args = {}
        if not isinstance(args, dict):
            args = {}
        result.append({
            "name": name,
            "arguments": args,
            "call_id": tc.get("id", f"call_{name}"),
        })
    return result or None
