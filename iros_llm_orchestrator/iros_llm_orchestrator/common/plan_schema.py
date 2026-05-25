"""JSON schema for the channel-3 chat response, used for structured outputs.

Mirrors the plan tree validated by ``plan_executor._validate_node``. Passed to
the LLM backend as a constrained-decoding schema (Ollama ``format`` / OpenAI
``response_format: json_schema``) so the model can only emit well-formed JSON
with valid node shapes — eliminating the malformed-JSON failure mode.

Note: JSON schema cannot express the cross-field rule len(goals)==len(robot_ids);
that stays enforced in ``plan_executor._validate_node`` and stated in the prompt.
"""

from __future__ import annotations

# Recursive plan node — sequence/parallel reference back into #/$defs/node.
_NODE_SCHEMA = {
    "oneOf": [
        {
            "type": "object",
            "properties": {
                "type": {"const": "mapf"},
                "robot_ids": {"type": "array", "items": {"type": "integer"}},
                "goals": {
                    "type": "array",
                    "items": {"type": "array", "items": {"type": "number"}},
                },
                # spread=true: goals is ONE center point; the server arranges the
                # robots into a distinct grid around it. Omit/false: goals is one
                # distinct point per robot (length must equal robot_ids).
                "spread": {"type": "boolean"},
                "reason": {"type": "string"},
            },
            "required": ["type", "robot_ids", "goals", "reason"],
        },
        {
            "type": "object",
            "properties": {
                "type": {"const": "formation"},
                "formation_id": {"type": "string"},
                "leader_ns": {"type": "string"},
                "follower_ns": {"type": "array", "items": {"type": "string"}},
                "offsets_x": {"type": "array", "items": {"type": "number"}},
                "offsets_y": {"type": "array", "items": {"type": "number"}},
                "reason": {"type": "string"},
            },
            "required": ["type", "formation_id", "leader_ns", "reason"],
        },
        {
            "type": "object",
            "properties": {
                "type": {"const": "idle"},
                "reason": {"type": "string"},
            },
            "required": ["type", "reason"],
        },
        {
            "type": "object",
            "properties": {
                "type": {"const": "sequence"},
                "steps": {"type": "array", "items": {"$ref": "#/$defs/node"}},
            },
            "required": ["type", "steps"],
        },
        {
            "type": "object",
            "properties": {
                "type": {"const": "parallel"},
                "steps": {"type": "array", "items": {"$ref": "#/$defs/node"}},
            },
            "required": ["type", "steps"],
        },
    ]
}

PLAN_RESPONSE_SCHEMA: dict = {
    "type": "object",
    "properties": {
        "reasoning": {"type": "string"},
        "reply": {"type": "string"},
        "plan": {"$ref": "#/$defs/node"},
    },
    "required": ["reply", "plan"],
    "$defs": {"node": _NODE_SCHEMA},
}
