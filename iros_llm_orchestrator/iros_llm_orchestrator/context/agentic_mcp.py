"""Agentic read-only MCP tool loop for /llm/chat.

The LLM can ask for observations through a JSON protocol, but this module
keeps MCP access behind a broker that validates names, args, budgets, timeouts,
and result size. It never exposes raw MCP client/session objects to the model.
"""

from __future__ import annotations

import asyncio
import json
import os
import traceback
from dataclasses import dataclass
from typing import Any, Awaitable, Callable

from iros_llm_orchestrator.context.provider import (
    BLOCKED_MCP_TOOLS,
    DEFAULT_MCP_READ_TOOLS,
    safe_str,
    to_jsonable,
)


AskLlm = Callable[[list[dict]], Awaitable[str]]
ParseFinal = Callable[[str], tuple[str, dict]]
ToolRunner = Callable[[str, dict], Awaitable[Any]]
ToolArgValidator = Callable[[str, dict], tuple[bool, str]]


@dataclass
class AgenticMcpConfig:
    enabled: bool = False
    max_rounds: int = 3
    max_tools_per_round: int = 3
    tool_timeout_sec: float = 2.0
    max_result_chars: int = 6000


@dataclass
class AgenticResponse:
    mode: str
    raw: str
    obj: dict[str, Any] | None = None
    error: str = ''


class AgenticMcpError(RuntimeError):
    """Raised when the agentic loop cannot reach a final answer safely."""


def parse_agentic_response(raw: str) -> AgenticResponse:
    """Parse the JSON protocol without raising.

    Backward compatibility: a normal ``{"reply": "...", "plan": {...}}``
    response is treated as ``mode=final`` even if the mode field is absent.
    """
    text = raw or ''
    try:
        obj = _extract_first_json_object(text)
    except ValueError as exc:
        return AgenticResponse(mode='invalid', raw=text, error=str(exc))

    mode = str(obj.get('mode') or '').strip().lower()
    if mode == 'tool_request':
        tools = obj.get('tools')
        if not isinstance(tools, list):
            return AgenticResponse(
                mode='invalid',
                raw=text,
                obj=obj,
                error='tool_request.tools must be a list',
            )
        return AgenticResponse(mode='tool_request', raw=text, obj=obj)
    if mode == 'final':
        return AgenticResponse(mode='final', raw=text, obj=obj)
    if 'reply' in obj and 'plan' in obj:
        return AgenticResponse(mode='final', raw=text, obj=obj)
    if 'decision' in obj:
        return AgenticResponse(mode='final', raw=text, obj=obj)
    return AgenticResponse(
        mode='invalid',
        raw=text,
        obj=obj,
        error=f'unsupported agentic MCP response mode: {mode or "<missing>"}',
    )


def agentic_instruction_message(
    allowed_tools: list[str] | tuple[str, ...],
    *,
    max_tools_per_round: int,
) -> dict:
    """Return the system instruction that advertises the JSON protocol."""
    allowed = ', '.join(str(name) for name in allowed_tools)
    content = (
        'You may request read-only ROS/MCP observations before the final plan. '
        'MCP is observation only: never request write/control tools, never '
        'publish topics, call services, send action goals, set parameters, '
        'send /cmd_vel, or bypass PlanExecutor, /llm/command, or the Behavior '
        'Tree. Return exactly one JSON object.\n'
        'For observations, return:\n'
        '{"mode":"tool_request","tools":[{"name":"subscribe_once","args":'
        '{"topic":"/bt/state","msg_type":"iros_llm_swarm_interfaces/msg/BTState"}}],'
        '"reason":"Need current BT state"}\n'
        'For the final answer, return:\n'
        '{"mode":"final","reply":"...","plan":{...}}\n'
        f'Allowed read-only tools: {allowed}.\n'
        f'Request at most {max_tools_per_round} tools per round. '
        'After tool results, either request more read-only observations if '
        'necessary or return mode=final.'
    )
    return {'role': 'system', 'content': content}


def add_agentic_instruction(
    messages: list[dict],
    allowed_tools: list[str] | tuple[str, ...],
    *,
    max_tools_per_round: int,
) -> list[dict]:
    """Insert the agentic protocol instruction before the current user turn."""
    out = list(messages)
    msg = agentic_instruction_message(
        allowed_tools,
        max_tools_per_round=max_tools_per_round,
    )
    if out and out[-1].get('role') == 'user':
        out.insert(len(out) - 1, msg)
    else:
        out.append(msg)
    return out


class McpToolBroker:
    """Validate and execute read-only MCP tool requests."""

    def __init__(
        self,
        *,
        allowed_tools: list[str] | tuple[str, ...],
        runner: ToolRunner,
        max_tools_per_round: int,
        tool_timeout_sec: float,
        max_result_chars: int,
        logger: Any | None = None,
        arg_validator: ToolArgValidator | None = None,
    ):
        allowed = []
        for name in allowed_tools or []:
            text = safe_str(name, 120).strip()
            if text and text in DEFAULT_MCP_READ_TOOLS and text not in allowed:
                allowed.append(text)
        self.allowed_tools = allowed
        self.runner = runner
        self.max_tools_per_round = max(0, int(max_tools_per_round))
        self.tool_timeout_sec = max(0.1, float(tool_timeout_sec))
        self.max_result_chars = max(200, int(max_result_chars))
        self.logger = logger
        self.arg_validator = arg_validator

    async def execute_tool_request(self, request: dict) -> dict:
        tools = request.get('tools')
        if not isinstance(tools, list):
            tools = []
        if len(tools) > self.max_tools_per_round:
            self._warn(
                f'agentic MCP requested {len(tools)} tools; '
                f'limiting to {self.max_tools_per_round}')
        selected = tools[:self.max_tools_per_round]
        names = [
            safe_str(t.get('name'), 120) if isinstance(t, dict) else '<invalid>'
            for t in selected
        ]
        self._info(f'agentic MCP requested tools: {names}')
        results = []
        for item in selected:
            result = await self._execute_one(item)
            results.append(result)
        ok_keys = [
            self._result_key(item)
            for item in results
            if item.get('status') == 'ok'
        ]
        if ok_keys:
            self._info(f'agentic MCP successful tool result keys: {ok_keys}')
        return {
            'mode': 'tool_result',
            'results': results,
        }

    async def _execute_one(self, item: Any) -> dict:
        valid, name, args, reason = self.validate_tool_call(item)
        if not valid:
            label = name or '<invalid>'
            self._warn(f'agentic MCP rejected tool {label}: {reason}')
            return {
                'name': label,
                'status': 'rejected',
                'error': reason,
            }
        try:
            raw = await asyncio.wait_for(
                self.runner(name, args),
                timeout=self.tool_timeout_sec,
            )
            return {
                'name': name,
                'args': args,
                'status': 'ok',
                'result': self._bound_result(raw),
            }
        except asyncio.TimeoutError:
            detail = _format_tool_failure(
                name,
                args,
                'TimeoutError',
                f'tool timed out after {self.tool_timeout_sec:.2f}s',
            )
            self._warn(f'agentic MCP tool timeout: {detail}')
            return {
                'name': name,
                'args': args,
                'status': 'timeout',
                'error': detail,
            }
        except Exception as exc:
            detail = _format_tool_exception(name, args, exc)
            self._warn(f'agentic MCP tool error: {detail}')
            return {
                'name': name,
                'args': args,
                'status': 'error',
                'error': detail,
            }

    def validate_tool_call(self, item: Any) -> tuple[bool, str, dict, str]:
        if not isinstance(item, dict):
            return False, '', {}, 'tool entry must be an object'
        name = safe_str(item.get('name'), 120).strip()
        if not name:
            return False, '', {}, 'tool name is required'
        if name in BLOCKED_MCP_TOOLS:
            return False, name, {}, 'tool is blocked because it can write/control ROS'
        if name not in DEFAULT_MCP_READ_TOOLS:
            return False, name, {}, 'tool is not in the read-only MCP allowlist'
        if name not in self.allowed_tools:
            return False, name, {}, 'tool is not enabled by mcp_tool_allowlist'
        raw_args = item.get('args') or {}
        if not isinstance(raw_args, dict):
            return False, name, {}, 'tool args must be an object'
        args = to_jsonable(raw_args)
        ok, reason = _args_are_safe(name, args)
        if not ok:
            return False, name, {}, reason
        if self.arg_validator is not None:
            ok, reason = self.arg_validator(name, args)
            if not ok:
                return False, name, {}, reason
        return True, name, args, ''

    def _bound_result(self, value: Any) -> Any:
        data = to_jsonable(value)
        text = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        if len(text) <= self.max_result_chars:
            return data
        preview_len = max(0, self.max_result_chars - 120)
        return {
            'truncated': True,
            'max_chars': self.max_result_chars,
            'preview': text[:preview_len],
        }

    @staticmethod
    def _result_key(result: dict) -> str:
        name = str(result.get('name') or '')
        args = result.get('args') or {}
        if name == 'subscribe_once' and args.get('topic'):
            return f"subscribe_once:{args['topic']}"
        if name == 'get_action_status' and args.get('action_name'):
            return f"get_action_status:{args['action_name']}"
        return name

    def _info(self, message: str):
        _log(self.logger, 'info', message)

    def _warn(self, message: str):
        _log(self.logger, 'warn', message)


async def run_agentic_mcp_loop(
    messages: list[dict],
    *,
    ask_llm: AskLlm,
    parse_final: ParseFinal,
    broker: McpToolBroker,
    config: AgenticMcpConfig,
    logger: Any | None = None,
    label: str = 'chat',
) -> tuple[str, dict, str]:
    """Run the JSON tool-request loop and return the parsed final response."""
    conversation = add_agentic_instruction(
        messages,
        broker.allowed_tools,
        max_tools_per_round=config.max_tools_per_round,
    )
    rounds_used = 0
    while True:
        round_no = rounds_used + 1
        _log(logger, 'info', f'agentic MCP {label}: LLM round {round_no}')
        raw = await ask_llm(conversation)
        parsed = parse_agentic_response(raw)
        if parsed.mode == 'final':
            _log(logger, 'info', f'agentic MCP {label}: final mode reached')
            reply, plan = parse_final(raw)
            return reply, plan, raw
        if parsed.mode == 'invalid':
            raise AgenticMcpError(f'invalid agentic MCP response: {parsed.error}')

        if rounds_used >= max(0, int(config.max_rounds)):
            _log(logger, 'warn', f'agentic MCP {label}: tool budget exhausted')
            conversation.append({'role': 'assistant', 'content': raw})
            conversation.append({
                'role': 'user',
                'content': json.dumps({
                    'mode': 'tool_result',
                    'results': [{
                        'status': 'rejected',
                        'error': 'agentic MCP tool budget exhausted',
                    }],
                    'instruction': 'Return mode=final now. Do not request more tools.',
                }, ensure_ascii=False),
            })
            final_raw = await ask_llm(conversation)
            final_parsed = parse_agentic_response(final_raw)
            if final_parsed.mode == 'final':
                _log(logger, 'info', f'agentic MCP {label}: final mode reached')
                reply, plan = parse_final(final_raw)
                return reply, plan, final_raw
            raise AgenticMcpError('agentic MCP tool budget exhausted')

        rounds_used += 1
        tools = (parsed.obj or {}).get('tools') or []
        names = [
            safe_str(t.get('name'), 120) if isinstance(t, dict) else '<invalid>'
            for t in tools
        ]
        _log(
            logger,
            'info',
            f'agentic MCP {label}: round {rounds_used} requested {names}',
        )
        tool_result = await broker.execute_tool_request(parsed.obj or {})
        conversation.append({'role': 'assistant', 'content': raw})
        conversation.append({
            'role': 'user',
            'content': _format_tool_result_for_llm(tool_result),
        })


def _format_tool_result_for_llm(tool_result: dict) -> str:
    payload = {
        **tool_result,
        'instruction': (
            'These are read-only observations. Use them only as current state. '
            'Return another mode=tool_request only if more read-only context is '
            'essential; otherwise return mode=final with reply and plan.'
        ),
    }
    return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))


def _format_tool_exception(name: str, args: dict, exc: BaseException) -> str:
    return _format_tool_failure(
        name,
        args,
        type(exc).__name__,
        str(exc) or '<empty>',
        traceback_summary=_traceback_summary(exc),
    )


def _format_tool_failure(
    name: str,
    args: dict,
    exc_type: str,
    exc_msg: str,
    *,
    traceback_summary: str = '',
) -> str:
    parts = [
        f'tool={safe_str(name, 120)}',
        f'args={_format_args(args)}',
        f'exc_type={safe_str(exc_type, 120)}',
        f'exc="{safe_str(exc_msg, 500)}"',
    ]
    if traceback_summary:
        parts.append(f'traceback={traceback_summary}')
    return ' '.join(parts)


def _format_args(args: dict) -> str:
    try:
        text = json.dumps(to_jsonable(args), ensure_ascii=False,
                          separators=(',', ':'))
    except Exception:
        text = str(args)
    return safe_str(text.replace('\n', ' ').replace('\r', ' '), 400)


def _traceback_summary(exc: BaseException) -> str:
    tb = traceback.TracebackException.from_exception(exc)
    frames = list(tb.stack)[-3:]
    if not frames:
        return ''
    return ' -> '.join(
        f'{os.path.basename(frame.filename)}:{frame.lineno}:{frame.name}'
        for frame in frames
    )


def _args_are_safe(name: str, args: dict) -> tuple[bool, str]:
    try:
        text = json.dumps(args, ensure_ascii=False, separators=(',', ':'))
    except (TypeError, ValueError):
        return False, 'tool args must be JSON serializable'
    if len(text) > 2000:
        return False, 'tool args are too large'
    for key, value in args.items():
        key_text = safe_str(key, 120)
        if any(ch in key_text for ch in ('\x00', '\r', '\n')):
            return False, 'tool arg keys must be single-line strings'
        if _contains_unsafe_string(value):
            return False, 'tool args contain unsafe control characters'
    topic = args.get('topic')
    if topic is not None:
        topic_text = safe_str(topic, 240).strip()
        if not topic_text.startswith('/'):
            return False, 'topic args must be absolute ROS names'
    if name == 'subscribe_once':
        if not args.get('topic') or not args.get('msg_type'):
            return False, 'subscribe_once requires topic and msg_type'
    if name == 'get_action_status':
        action_name = safe_str(args.get('action_name'), 240).strip()
        if not action_name:
            return False, 'get_action_status requires action_name'
        if not action_name.startswith('/'):
            return False, 'action_name must be an absolute ROS action name'
    return True, ''


def _contains_unsafe_string(value: Any) -> bool:
    if isinstance(value, str):
        return '\x00' in value or len(value) > 500
    if isinstance(value, list):
        return any(_contains_unsafe_string(item) for item in value[:50])
    if isinstance(value, dict):
        return any(
            _contains_unsafe_string(k) or _contains_unsafe_string(v)
            for k, v in list(value.items())[:50]
        )
    return False


def _extract_first_json_object(text: str) -> dict[str, Any]:
    start = text.find('{')
    if start == -1:
        raise ValueError('no JSON object in LLM output')
    depth = 0
    end = -1
    in_string = False
    escaped = False
    for i in range(start, len(text)):
        ch = text[i]
        if in_string:
            if escaped:
                escaped = False
            elif ch == '\\':
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    if end == -1:
        raise ValueError('JSON object not closed')
    try:
        obj = json.loads(text[start:end])
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc}') from exc
    if not isinstance(obj, dict):
        raise ValueError('top-level JSON value must be an object')
    return obj


def _log(logger: Any | None, level: str, message: str):
    if logger is None:
        return
    method = getattr(logger, level, None)
    if method is None and level == 'warn':
        method = getattr(logger, 'warning', None)
    if method is not None:
        method(message)
