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
    {
        "type": "function",
        "function": {
            "name": "find_free_group_goals_in_room",
            "description": (
                "Find deterministic, occupancy-aware per-robot MAPF goals "
                "for ordinary group movement into a named room. Read-only: "
                "does not move robots. Use when sending a group into a room "
                "that may already contain robots, or when placing a group "
                "near/around another group without a formation."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "room": {
                        "type": "string",
                        "description": "Named room/location, e.g. 'cafeteria'.",
                    },
                    "robot_ids": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": "Moving robot ids to place.",
                    },
                    "avoid_robot_ids": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": (
                            "Robots whose current footprints must be avoided, "
                            "e.g. the group already in the room."
                        ),
                    },
                    "prefer_near_group": {
                        "type": "array",
                        "items": {"type": "integer"},
                        "description": (
                            "Optional robot ids to place near/around while "
                            "still avoiding their footprints."
                        ),
                    },
                    "placement_mode": {
                        "type": "string",
                        "enum": ["cluster", "around_group", "line"],
                        "description": (
                            "cluster for compact free placement; around_group "
                            "for commands like 'orange around yellow'."
                        ),
                    },
                    "avoid_existing_robots": {
                        "type": "boolean",
                        "description": (
                            "Avoid live robot footprints not in robot_ids."
                        ),
                    },
                    "min_clearance_m": {
                        "type": "number",
                        "description": "Required free space between robot footprints.",
                    },
                    "goal_spacing_m": {
                        "type": "number",
                        "description": "Preferred center-to-center goal spacing.",
                    },
                    "candidate_spacing_m": {
                        "type": "number",
                        "description": "Room sampling grid spacing.",
                    },
                },
                "required": ["room", "robot_ids"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "find_group_placement_in_room",
            "description": (
                "Find deterministic, non-overlapping leader/follower goal "
                "placements for one or more robot groups forming formations "
                "inside the same named room. Read-only: does not move robots. "
                "Use before planning multiple groups into one room or before "
                "activating formations there."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "room": {
                        "type": "string",
                        "description": "Named room/location, e.g. 'cafeteria'.",
                    },
                    "groups": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "name": {
                                    "type": "string",
                                    "description": "Group label, e.g. 'green'.",
                                },
                                "robot_ids": {
                                    "type": "array",
                                    "items": {"type": "integer"},
                                    "description": (
                                        "Robot ids in leader-first order. "
                                        "First id becomes the leader."
                                    ),
                                },
                                "formation": {
                                    "type": "string",
                                    "description": "Formation name: wedge, line, or column.",
                                },
                            },
                            "required": ["name", "robot_ids", "formation"],
                        },
                    },
                    "avoid_existing_robots": {
                        "type": "boolean",
                        "description": (
                            "Avoid live robot footprints not included in the requested groups."
                        ),
                    },
                    "min_clearance_m": {
                        "type": "number",
                        "description": "Required free space between robot footprints.",
                    },
                },
                "required": ["room", "groups"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "verify_plan_execution_state",
            "description": (
                "Verify read-only post-execution state for an LLM plan. "
                "Checks expected formations, follower offset errors, recent "
                "formation failures, and optional room placement. Does not "
                "move robots or call control services."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "original_user_request": {
                        "type": "string",
                        "description": "Original operator command.",
                    },
                    "expected": {
                        "type": "object",
                        "description": (
                            "Optional expected outcome with room and groups. "
                            "Groups may include name, robot_ids, formation, "
                            "and formation_id."
                        ),
                    },
                    "last_plan": {
                        "type": "object",
                        "description": "The plan JSON that was just executed.",
                    },
                    "tolerance_m": {
                        "type": "number",
                        "description": "Follower offset tolerance in metres.",
                    },
                },
                "required": ["original_user_request", "last_plan"],
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
