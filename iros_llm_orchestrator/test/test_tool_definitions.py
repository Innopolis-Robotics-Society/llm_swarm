"""Tests for tool_definitions.py — parse_ollama_tool_calls and parse_openai_tool_calls."""

import json
import pytest

from iros_llm_orchestrator.common.tool_definitions import (
    TOOL_DEFINITIONS,
    parse_ollama_tool_calls,
    parse_openai_tool_calls,
)


# ---------------------------------------------------------------------------
# TOOL_DEFINITIONS structure
# ---------------------------------------------------------------------------

def test_tool_definitions_count():
    assert len(TOOL_DEFINITIONS) == 3


def test_tool_definitions_names():
    names = {t["function"]["name"] for t in TOOL_DEFINITIONS}
    assert names == {"check_occupancy", "get_positions", "get_robot_position"}


def test_tool_definitions_have_required_fields():
    for t in TOOL_DEFINITIONS:
        assert t["type"] == "function"
        fn = t["function"]
        assert "name" in fn
        assert "description" in fn
        assert "parameters" in fn
        assert fn["parameters"]["type"] == "object"


# ---------------------------------------------------------------------------
# parse_ollama_tool_calls
# ---------------------------------------------------------------------------

def test_parse_ollama_no_tool_calls():
    msg = {"role": "assistant", "content": "hello"}
    assert parse_ollama_tool_calls(msg) is None


def test_parse_ollama_empty_tool_calls():
    msg = {"role": "assistant", "tool_calls": []}
    assert parse_ollama_tool_calls(msg) is None


def test_parse_ollama_single_tool_call():
    msg = {
        "role": "assistant",
        "tool_calls": [
            {"function": {"name": "get_robot_position", "arguments": {"robot_id": "robot_3"}}}
        ],
    }
    result = parse_ollama_tool_calls(msg)
    assert result is not None
    assert len(result) == 1
    assert result[0]["name"] == "get_robot_position"
    assert result[0]["arguments"] == {"robot_id": "robot_3"}
    assert result[0]["call_id"] == "ollama_0"


def test_parse_ollama_stringified_arguments():
    args_str = json.dumps({"room": "nw_hall", "qualifier": "corners.top_right"})
    msg = {
        "role": "assistant",
        "tool_calls": [
            {"function": {"name": "get_positions", "arguments": args_str}}
        ],
    }
    result = parse_ollama_tool_calls(msg)
    assert result is not None
    assert result[0]["arguments"]["room"] == "nw_hall"
    assert result[0]["arguments"]["qualifier"] == "corners.top_right"


def test_parse_ollama_multiple_calls():
    msg = {
        "role": "assistant",
        "tool_calls": [
            {"function": {"name": "get_robot_position", "arguments": {"robot_id": "robot_1"}}},
            {"function": {"name": "check_occupancy",    "arguments": {"robot_id": "robot_2"}}},
        ],
    }
    result = parse_ollama_tool_calls(msg)
    assert result is not None
    assert len(result) == 2
    assert result[0]["call_id"] == "ollama_0"
    assert result[1]["call_id"] == "ollama_1"


# ---------------------------------------------------------------------------
# parse_openai_tool_calls
# ---------------------------------------------------------------------------

def test_parse_openai_no_tool_calls():
    choice = {"message": {"role": "assistant", "content": "hello"}}
    assert parse_openai_tool_calls(choice) is None


def test_parse_openai_single_tool_call():
    choice = {
        "message": {
            "role": "assistant",
            "content": None,
            "tool_calls": [
                {
                    "id": "call_abc123",
                    "type": "function",
                    "function": {
                        "name": "check_occupancy",
                        "arguments": json.dumps({"robot_id": "robot_5"}),
                    },
                }
            ],
        }
    }
    result = parse_openai_tool_calls(choice)
    assert result is not None
    assert len(result) == 1
    assert result[0]["name"] == "check_occupancy"
    assert result[0]["arguments"] == {"robot_id": "robot_5"}
    assert result[0]["call_id"] == "call_abc123"


def test_parse_openai_stringified_arguments():
    choice = {
        "message": {
            "tool_calls": [
                {
                    "id": "call_xyz",
                    "function": {
                        "name": "get_positions",
                        "arguments": '{"room": "central_hub", "qualifier": "center"}',
                    },
                }
            ]
        }
    }
    result = parse_openai_tool_calls(choice)
    assert result is not None
    assert result[0]["arguments"]["room"] == "central_hub"


def test_parse_openai_invalid_json_arguments_returns_empty_dict():
    choice = {
        "message": {
            "tool_calls": [
                {
                    "id": "call_bad",
                    "function": {"name": "get_robot_position", "arguments": "not json"},
                }
            ]
        }
    }
    result = parse_openai_tool_calls(choice)
    assert result is not None
    assert result[0]["arguments"] == {}
