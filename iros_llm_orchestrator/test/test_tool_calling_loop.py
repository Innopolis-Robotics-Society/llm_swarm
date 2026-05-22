"""Integration test for the tool calling loop in generate_with_tools + ToolExecutor.

Uses a fake LLM client that returns tool_calls on the first turn, then a
final JSON plan on the second turn.  Validates that:
  - the tool executor is called with the right arguments
  - the messages list is extended correctly
  - the final plan is returned from the loop
"""

import asyncio
import json
from typing import Any
from unittest.mock import AsyncMock, MagicMock

import pytest

from iros_llm_orchestrator.common.llm_factory import LLMClientBase
from iros_llm_orchestrator.common.tool_definitions import TOOL_DEFINITIONS
from iros_llm_orchestrator.common.tool_executor import ToolExecutor


def _run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


# ---------------------------------------------------------------------------
# Fake LLM client
# ---------------------------------------------------------------------------

class _FakeLLMClient(LLMClientBase):
    """Returns tool_calls on first call, a text plan on second call."""

    def __init__(self, tool_name: str, tool_args: dict, final_json: str) -> None:
        self._call_count = 0
        self._tool_name = tool_name
        self._tool_args = tool_args
        self._final_json = final_json

    async def generate_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        self._call_count += 1
        if self._call_count == 1:
            return {
                "type": "tool_calls",
                "calls": [
                    {
                        "name": self._tool_name,
                        "arguments": self._tool_args,
                        "call_id": "call_test_001",
                    }
                ],
            }
        return {"type": "text", "content": self._final_json}

    async def generate(self, prompt, prompt_kind="decision") -> str:
        return self._final_json


# ---------------------------------------------------------------------------
# Minimal ToolExecutor with injectable call result
# ---------------------------------------------------------------------------

def _make_tool_executor(result: dict) -> ToolExecutor:
    node = MagicMock()
    pose_cache = MagicMock()
    pose_cache.snapshot.return_value = {3: {"x": 5.2, "y": -3.1, "yaw": 1.57,
                                             "stale": False, "stale_ms": 45}}
    ex = ToolExecutor(node=node, pose_cache=pose_cache, map_cfg={})
    async def _fixed_call(name, arguments):
        return result
    ex.call = _fixed_call
    return ex


# ---------------------------------------------------------------------------
# Helper: run the tool loop (extracted from user_chat_node logic)
# ---------------------------------------------------------------------------

async def _run_tool_loop(
    llm: LLMClientBase,
    executor: ToolExecutor,
    messages: list[dict],
    max_iterations: int = 6,
    chunk_cb=None,
) -> str:
    msgs = list(messages)
    for _ in range(max_iterations):
        if chunk_cb is not None:
            # Use stream_with_tools, forward non-empty chunks to callback
            result = None
            async for event in llm.stream_with_tools(msgs, TOOL_DEFINITIONS):
                if event["type"] == "chunk":
                    token = event.get("content", "")
                    if token:
                        chunk_cb(token)
                else:
                    result = event
            result = result or {"type": "text", "content": ""}
        else:
            result = await llm.generate_with_tools(msgs, TOOL_DEFINITIONS)

        if result["type"] == "text":
            return result["content"]

        calls = result["calls"]
        tool_calls_field = [
            {
                "id": c.get("call_id", f"call_{c['name']}"),
                "type": "function",
                "function": {
                    "name": c["name"],
                    "arguments": json.dumps(c.get("arguments", {})),
                },
            }
            for c in calls
        ]
        msgs.append({"role": "assistant", "content": None, "tool_calls": tool_calls_field})

        for call in calls:
            name = call["name"]
            args = call["arguments"]
            call_id = call.get("call_id", "")
            try:
                tool_result = await executor.call(name, args)
            except Exception as exc:
                tool_result = {"error": str(exc)}
            result_str = json.dumps(tool_result, ensure_ascii=False)
            msgs.append({"role": "tool", "tool_call_id": call_id, "content": result_str})

    raise RuntimeError("tool loop exceeded max iterations")


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

FINAL_PLAN = json.dumps({
    "reasoning": "robot_3 is at the queried position",
    "reply": "Sending robot 3 to its current position.",
    "plan": {"type": "mapf", "robot_ids": [3], "goals": [[5.2, -3.1]], "reason": "test"},
})


def test_tool_loop_two_turns():
    """First turn returns get_robot_position call; second turn returns plan."""
    tool_result = {"robot_id": "robot_3", "x": 5.2, "y": -3.1, "yaw": 1.57,
                   "stale": False, "stale_ms": 45}
    llm = _FakeLLMClient("get_robot_position", {"robot_id": "robot_3"}, FINAL_PLAN)
    executor = _make_tool_executor(tool_result)
    messages = [{"role": "user", "content": "where is robot_3?"}]

    text = _run(_run_tool_loop(llm, executor, messages))
    assert text == FINAL_PLAN
    assert llm._call_count == 2


def test_tool_loop_single_turn_no_tools():
    """If LLM never requests tools, loop exits after one turn."""
    llm = _FakeLLMClient("unused", {}, FINAL_PLAN)

    async def _direct(messages, tools):
        return {"type": "text", "content": FINAL_PLAN}
    llm.generate_with_tools = _direct

    executor = _make_tool_executor({})
    messages = [{"role": "user", "content": "stop"}]

    text = _run(_run_tool_loop(llm, executor, messages))
    assert text == FINAL_PLAN


def test_tool_loop_injects_tool_result_into_messages():
    """Tool result message is appended to the conversation before the second LLM call."""
    captured_messages: list[list[dict]] = []

    class _CaptureLLM(LLMClientBase):
        call_count = 0

        async def generate_with_tools(self, messages, tools):
            captured_messages.append(list(messages))
            self.call_count += 1
            if self.call_count == 1:
                return {
                    "type": "tool_calls",
                    "calls": [{"name": "get_robot_position",
                                "arguments": {"robot_id": "robot_0"},
                                "call_id": "cid_0"}],
                }
            return {"type": "text", "content": FINAL_PLAN}

    tool_result = {"robot_id": "robot_0", "x": 1.0, "y": 2.0, "yaw": 0.0,
                   "stale": False, "stale_ms": 10}
    executor = _make_tool_executor(tool_result)
    messages = [{"role": "user", "content": "get position of robot_0"}]

    _run(_run_tool_loop(_CaptureLLM(), executor, messages))

    second_call_msgs = captured_messages[1]
    roles = [m["role"] for m in second_call_msgs]
    assert "tool" in roles

    tool_msg = next(m for m in second_call_msgs if m["role"] == "tool")
    parsed = json.loads(tool_msg["content"])
    assert parsed["x"] == 1.0


def test_tool_loop_raises_on_max_iterations():
    """Loop raises RuntimeError when max_iterations is exceeded."""
    class _InfiniteToolLLM(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            return {
                "type": "tool_calls",
                "calls": [{"name": "get_robot_position",
                            "arguments": {"robot_id": "robot_0"},
                            "call_id": "c0"}],
            }

    executor = _make_tool_executor({"robot_id": "robot_0", "x": 0.0, "y": 0.0,
                                     "yaw": 0.0, "stale": False, "stale_ms": 0})
    with pytest.raises(RuntimeError, match="max iterations"):
        _run(_run_tool_loop(_InfiniteToolLLM(),
                            executor,
                            [{"role": "user", "content": "x"}],
                            max_iterations=3))


def test_chunk_cb_receives_tokens_during_tool_loop():
    """chunk_cb is called for each non-empty chunk emitted by stream_with_tools."""
    received: list[str] = []

    class _StreamingFakeLLM(LLMClientBase):
        _call_count = 0

        async def stream_with_tools(self, messages, tools):
            self._call_count += 1
            if self._call_count == 1:
                yield {"type": "chunk", "content": "thinking..."}
                yield {
                    "type": "tool_calls",
                    "calls": [{
                        "name": "get_robot_position",
                        "arguments": {"robot_id": "robot_3"},
                        "call_id": "c0",
                    }],
                }
            else:
                yield {"type": "chunk", "content": '{"reasoning":"ok"'}
                yield {"type": "chunk", "content": ',"reply":"done","plan":{"type":"idle","reason":"x"}}'}
                yield {"type": "text",
                       "content": '{"reasoning":"ok","reply":"done","plan":{"type":"idle","reason":"x"}}'}

    tool_result = {"robot_id": "robot_3", "x": 0.0, "y": 0.0,
                   "yaw": 0.0, "stale": False, "stale_ms": 0}
    executor = _make_tool_executor(tool_result)
    messages = [{"role": "user", "content": "where is robot_3?"}]

    text = _run(_run_tool_loop(
        _StreamingFakeLLM(), executor, messages, chunk_cb=received.append))
    assert len(received) >= 1
    assert any("thinking" in c for c in received)


def test_chunk_cb_not_called_for_empty_chunks():
    """Empty-content chunk events are silently skipped by the chunk_cb gate."""
    received: list[str] = []

    class _EmptyChunkLLM(LLMClientBase):
        async def stream_with_tools(self, messages, tools):
            yield {"type": "chunk", "content": ""}
            yield {"type": "chunk", "content": "real token"}
            yield {"type": "text", "content": "real token"}

    executor = _make_tool_executor({})
    messages = [{"role": "user", "content": "test"}]

    _run(_run_tool_loop(
        _EmptyChunkLLM(), executor, messages, chunk_cb=received.append))
    assert received == ["real token"]


def test_chunk_cb_none_uses_generate_with_tools():
    """When chunk_cb is None the loop falls back to generate_with_tools (no streaming)."""
    generate_calls: list[int] = []
    stream_calls: list[int] = []

    class _TrackingLLM(LLMClientBase):
        async def generate_with_tools(self, messages, tools):
            generate_calls.append(1)
            return {"type": "text", "content": FINAL_PLAN}

        async def stream_with_tools(self, messages, tools):
            stream_calls.append(1)
            yield {"type": "text", "content": FINAL_PLAN}

    executor = _make_tool_executor({})
    messages = [{"role": "user", "content": "stop"}]

    _run(_run_tool_loop(_TrackingLLM(), executor, messages, chunk_cb=None))
    assert len(generate_calls) == 1
    assert len(stream_calls) == 0


def test_tool_loop_handles_tool_executor_error():
    """If the tool executor raises, the error dict is injected into messages."""
    captured_tool_msgs: list[dict] = []

    class _ErrorExecutor:
        async def call(self, name, arguments):
            raise RuntimeError("laser timeout")

    class _SingleToolLLM(LLMClientBase):
        _done = False

        async def generate_with_tools(self, messages, tools):
            if not self._done:
                self._done = True
                return {
                    "type": "tool_calls",
                    "calls": [{"name": "check_occupancy",
                                "arguments": {"robot_id": "robot_5"},
                                "call_id": "c1"}],
                }
            return {"type": "text", "content": FINAL_PLAN}

    # Run the loop manually so we can capture messages
    async def _inner():
        msgs = [{"role": "user", "content": "check robot_5"}]
        llm = _SingleToolLLM()
        executor_obj = _ErrorExecutor()
        for _ in range(6):
            result = await llm.generate_with_tools(msgs, TOOL_DEFINITIONS)
            if result["type"] == "text":
                break
            calls = result["calls"]
            tool_calls_field = [{
                "id": c.get("call_id", f"call_{c['name']}"),
                "type": "function",
                "function": {
                    "name": c["name"],
                    "arguments": json.dumps(c.get("arguments", {})),
                },
            } for c in calls]
            msgs.append({"role": "assistant", "content": None,
                         "tool_calls": tool_calls_field})
            for call in calls:
                try:
                    tr = await executor_obj.call(call["name"], call["arguments"])
                except Exception as exc:
                    tr = {"error": str(exc)}
                tm = {"role": "tool", "tool_call_id": call["call_id"],
                      "content": json.dumps(tr)}
                msgs.append(tm)
                captured_tool_msgs.append(tm)
        return msgs

    _run(_inner())
    assert len(captured_tool_msgs) == 1
    payload = json.loads(captured_tool_msgs[0]["content"])
    assert "error" in payload
    assert "timeout" in payload["error"]
