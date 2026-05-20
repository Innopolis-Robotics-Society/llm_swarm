"""Unit tests for the agentic read-only MCP broker and loop."""

import asyncio
import json
from pathlib import Path

import pytest

from iros_llm_orchestrator.context.agentic_mcp import (
    AgenticMcpConfig,
    AgenticMcpError,
    McpToolBroker,
    parse_agentic_response,
    run_agentic_mcp_loop,
)


def _run(coro):
    return asyncio.run(coro)


def _tool_request(name='subscribe_once', args=None):
    return json.dumps({
        'mode': 'tool_request',
        'tools': [{
            'name': name,
            'args': args or {
                'topic': '/bt/state',
                'msg_type': 'iros_llm_swarm_interfaces/msg/BTState',
            },
        }],
        'reason': 'Need current state',
    })


def test_tool_request_parser_accepts_protocol():
    parsed = parse_agentic_response(_tool_request())

    assert parsed.mode == 'tool_request'
    assert parsed.obj['tools'][0]['name'] == 'subscribe_once'


def test_parser_accepts_decision_final_protocol():
    parsed = parse_agentic_response(json.dumps({
        'mode': 'final',
        'decision': 'wait',
        'reason': 'temporary stall',
    }))

    assert parsed.mode == 'final'
    assert parsed.obj['decision'] == 'wait'


def test_invalid_json_handled_safely():
    parsed = parse_agentic_response('not json at all')

    assert parsed.mode == 'invalid'
    assert 'no JSON object' in parsed.error


def test_blocked_tool_rejected_without_runner_call():
    called = []

    async def runner(name, args):
        called.append((name, args))
        return {}

    broker = McpToolBroker(
        allowed_tools=['get_topics', 'call_service'],
        runner=runner,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [{'name': 'call_service', 'args': {'service': '/reset'}}],
    }))

    assert result['results'][0]['status'] == 'rejected'
    assert 'blocked' in result['results'][0]['error']
    assert called == []


def test_allowlisted_tool_acceptance():
    async def runner(name, args):
        return {'observed': name, 'args': args}

    broker = McpToolBroker(
        allowed_tools=['subscribe_once'],
        runner=runner,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [{
            'name': 'subscribe_once',
            'args': {
                'topic': '/bt/state',
                'msg_type': 'iros_llm_swarm_interfaces/msg/BTState',
            },
        }],
    }))

    assert result['results'][0]['status'] == 'ok'
    assert result['results'][0]['result']['observed'] == 'subscribe_once'


def test_get_action_status_requires_action_name_arg():
    called = []

    async def runner(name, args):
        called.append((name, args))
        return {'status': 'accepted'}

    broker = McpToolBroker(
        allowed_tools=['get_action_status'],
        runner=runner,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    wrong = _run(broker.execute_tool_request({
        'tools': [{
            'name': 'get_action_status',
            'args': {'action': '/swarm/set_goals'},
        }],
    }))
    right = _run(broker.execute_tool_request({
        'tools': [{
            'name': 'get_action_status',
            'args': {'action_name': '/swarm/set_goals'},
        }],
    }))

    assert wrong['results'][0]['status'] == 'rejected'
    assert 'action_name' in wrong['results'][0]['error']
    assert right['results'][0]['status'] == 'ok'
    assert called == [
        ('get_action_status', {'action_name': '/swarm/set_goals'}),
    ]


def test_max_tools_per_round_enforced():
    called = []

    async def runner(name, args):
        called.append(name)
        return {'ok': True}

    broker = McpToolBroker(
        allowed_tools=['get_topics', 'get_nodes'],
        runner=runner,
        max_tools_per_round=1,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [
            {'name': 'get_topics', 'args': {}},
            {'name': 'get_nodes', 'args': {}},
        ],
    }))

    assert len(result['results']) == 1
    assert called == ['get_topics']


def test_max_rounds_enforced():
    responses = [
        _tool_request('get_topics', {}),
        _tool_request('get_nodes', {}),
        _tool_request('get_services', {}),
    ]
    calls = []

    async def ask_llm(messages):
        return responses.pop(0)

    async def runner(name, args):
        calls.append(name)
        return {'ok': True}

    def parse_final(raw):
        obj = json.loads(raw)
        return obj['reply'], obj['plan']

    broker = McpToolBroker(
        allowed_tools=['get_topics', 'get_nodes'],
        runner=runner,
        max_tools_per_round=2,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    with pytest.raises(AgenticMcpError, match='budget'):
        _run(run_agentic_mcp_loop(
            [{'role': 'user', 'content': 'status?'}],
            ask_llm=ask_llm,
            parse_final=parse_final,
            broker=broker,
            config=AgenticMcpConfig(
                enabled=True,
                max_rounds=1,
                max_tools_per_round=2,
                tool_timeout_sec=0.1,
                max_result_chars=6000,
            ),
        ))

    assert calls == ['get_topics']


def test_final_answer_still_uses_existing_parser_path():
    root = Path(__file__).resolve().parents[1]
    source = root / 'iros_llm_orchestrator' / 'chat_server.py'
    text = source.read_text(encoding='utf-8')

    assert 'parse_final=self._parse_and_postprocess' in text
    assert '_parse_response(raw)' in text


def test_remediation_still_uses_plan_executor_path():
    root = Path(__file__).resolve().parents[1]
    source = root / 'iros_llm_orchestrator' / 'chat_server.py'
    text = source.read_text(encoding='utf-8')

    assert 'agentic_enabled=self._mcp_agentic_enable_for_remediation' in text
    assert 'ok, failure_info = await self._execute_plan(r_plan)' in text
    assert 'PlanExecutor(' in text
