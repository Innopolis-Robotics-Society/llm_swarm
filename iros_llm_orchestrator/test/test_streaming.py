"""Tests for stream_with_tools interface — base class and backend stubs.

Covers:
  - LLMClientBase default: delegates to generate_with_tools, yields terminal only.
  - MockLLMClient stub: yields terminal, no chunk events.
  - Custom clients: chunk ordering, empty-chunk filtering.
  - chunk_cb contract: receives non-empty chunks; never empty strings.
"""

import asyncio
import json

import pytest

from iros_llm_orchestrator.common.llm_factory import LLMClientBase
from iros_llm_orchestrator.common.mock_client import MockLLMClient


def _run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


async def _collect(gen):
    events = []
    async for event in gen:
        events.append(event)
    return events


# ---------------------------------------------------------------------------
# LLMClientBase default stream_with_tools
# ---------------------------------------------------------------------------

def test_base_default_yields_text_terminal():
    class _Client(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {"type": "text", "content": "hello"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert len(events) == 1
    assert events[0] == {"type": "text", "content": "hello"}


def test_base_default_yields_tool_calls_terminal():
    class _Client(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {
                "type": "tool_calls",
                "calls": [{"name": "get_robot_position",
                           "arguments": {"robot_id": "robot_0"},
                           "call_id": "c0"}],
            }

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert len(events) == 1
    assert events[0]["type"] == "tool_calls"
    assert events[0]["calls"][0]["name"] == "get_robot_position"


def test_base_default_emits_no_chunk_events():
    class _Client(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {"type": "text", "content": "plan"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert not any(e["type"] == "chunk" for e in events)


def test_base_default_exactly_one_terminal_event():
    class _Client(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {"type": "text", "content": "x"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    terminal = [e for e in events if e["type"] in ("text", "tool_calls")]
    assert len(terminal) == 1


# ---------------------------------------------------------------------------
# MockLLMClient stub
# ---------------------------------------------------------------------------

def test_mock_stream_with_tools_yields_one_event():
    client = MockLLMClient()
    events = _run(_collect(client.stream_with_tools(
        [{"role": "user", "content": "stop"}], [])))
    assert len(events) == 1


def test_mock_stream_with_tools_no_chunk_events():
    client = MockLLMClient()
    events = _run(_collect(client.stream_with_tools(
        [{"role": "user", "content": "stop"}], [])))
    assert not any(e["type"] == "chunk" for e in events)


def test_mock_stream_with_tools_terminal_type():
    client = MockLLMClient()
    events = _run(_collect(client.stream_with_tools(
        [{"role": "user", "content": "stop"}], [])))
    assert events[0]["type"] in ("text", "tool_calls")


# ---------------------------------------------------------------------------
# Custom streaming clients — chunk ordering
# ---------------------------------------------------------------------------

def test_chunks_precede_terminal():
    """All chunk events appear before the single terminal event."""
    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": "part1"}
            yield {"type": "chunk", "content": "part2"}
            yield {"type": "text", "content": "part1part2"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    types = [e["type"] for e in events]
    assert types == ["chunk", "chunk", "text"]


def test_accumulated_text_matches_terminal():
    """Text accumulated from chunks equals terminal content."""
    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": "hel"}
            yield {"type": "chunk", "content": "lo"}
            yield {"type": "text", "content": "hello"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    accumulated = "".join(e["content"] for e in events if e["type"] == "chunk")
    terminal_content = next(e["content"] for e in events if e["type"] == "text")
    assert accumulated == terminal_content


def test_tool_calls_terminal_after_chunks():
    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": "thinking..."}
            yield {"type": "tool_calls", "calls": [
                {"name": "check_occupancy", "arguments": {"robot_id": "robot_0"},
                 "call_id": "c0"}
            ]}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert events[0]["type"] == "chunk"
    assert events[-1]["type"] == "tool_calls"


# ---------------------------------------------------------------------------
# chunk_cb contract
# ---------------------------------------------------------------------------

def test_chunk_cb_receives_all_nonempty_chunks():
    received: list[str] = []

    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": "alpha"}
            yield {"type": "chunk", "content": "beta"}
            yield {"type": "text", "content": "alphabeta"}

    async def _inner():
        async for event in _Client().stream_with_tools([], []):
            if event["type"] == "chunk":
                token = event.get("content", "")
                if token:
                    received.append(token)

    _run(_inner())
    assert received == ["alpha", "beta"]


def test_chunk_cb_skips_empty_content():
    received: list[str] = []

    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": ""}
            yield {"type": "chunk", "content": "real"}
            yield {"type": "chunk", "content": ""}
            yield {"type": "text", "content": "real"}

    async def _inner():
        async for event in _Client().stream_with_tools([], []):
            if event["type"] == "chunk":
                token = event.get("content", "")
                if token:
                    received.append(token)

    _run(_inner())
    assert received == ["real"]


def test_chunk_cb_not_called_when_no_chunks():
    called = []

    class _Client(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {"type": "text", "content": "direct"}

    async def _inner():
        async for event in _Client().stream_with_tools([], []):
            if event["type"] == "chunk":
                called.append(event.get("content", ""))

    _run(_inner())
    assert called == []


# ---------------------------------------------------------------------------
# Terminal event is always last
# ---------------------------------------------------------------------------

def test_terminal_is_last_event_text():
    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": "a"}
            yield {"type": "chunk", "content": "b"}
            yield {"type": "text", "content": "ab"}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert events[-1]["type"] == "text"


def test_terminal_is_last_event_tool_calls():
    class _Client(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": ""}
            yield {"type": "tool_calls", "calls": [
                {"name": "get_positions", "arguments": {"room": "nw_hall", "qualifier": "center"},
                 "call_id": "c1"}
            ]}

    events = _run(_collect(_Client().stream_with_tools([], [])))
    assert events[-1]["type"] == "tool_calls"
