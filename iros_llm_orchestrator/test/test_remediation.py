"""Unit tests for the remediation loop.

Covers the pure-Python pieces (no rclpy spin): planned-call selection by
leaf type, the remediation prompt builder, is_help_request detection, and
the PlanExecutor failed_leaf bookkeeping. The chat-server side is exercised
by stubbing the context provider; we don't spin a ROS executor here.
"""

import asyncio
import json

import pytest

from iros_llm_orchestrator.common import user_prompt
from iros_llm_orchestrator.common.plan_executor import PlanExecutor
from iros_llm_orchestrator.common.user_prompt import build_remediation_prompt
from iros_llm_orchestrator.context.mcp_readonly_provider import (
    McpReadonlyContextProvider,
    summarize_for_remediation,
)
from iros_llm_orchestrator.context.no_execute import is_help_request
from iros_llm_orchestrator.context.provider import ChatContextConfig


# ---------------------------------------------------------------------------
# is_help_request
# ---------------------------------------------------------------------------

def test_is_help_request_detects_needs_help_prefix():
    assert is_help_request(
        {'type': 'idle', 'reason': 'needs_help: where should I put them?'})


def test_is_help_request_detects_clarify_prefix():
    assert is_help_request(
        {'type': 'idle', 'reason': 'clarify: did you mean east or west?'})


def test_is_help_request_ignores_other_idle_reasons():
    assert not is_help_request(
        {'type': 'idle', 'reason': 'operator stop'})
    assert not is_help_request(
        {'type': 'idle', 'reason': 'reply_only: status report'})


def test_is_help_request_walks_containers():
    plan = {
        'type': 'sequence',
        'steps': [
            {'type': 'mapf', 'robot_ids': [0], 'goals': [[0.0, 0.0]]},
            {'type': 'idle', 'reason': 'needs_help: nested'},
        ],
    }
    assert is_help_request(plan)


def test_is_help_request_handles_none_and_non_dict():
    assert not is_help_request(None)
    assert not is_help_request('idle')
    assert not is_help_request({})


# ---------------------------------------------------------------------------
# _planned_calls_for
# ---------------------------------------------------------------------------

def _provider(**overrides) -> McpReadonlyContextProvider:
    cfg = ChatContextConfig(
        provider='mcp_readonly',
        include_bt_state=False,
        include_formations=False,
        include_map_summary=False,
        include_recent_events=False,
        mcp_enabled=True,
        **overrides,
    )
    return McpReadonlyContextProvider(cfg)


def _call_names(calls):
    return [
        f"{name}:{args.get('topic') or args.get('action') or ''}"
        if name in {'subscribe_once', 'get_action_status'}
        else name
        for name, args in calls
    ]


def test_planned_calls_for_set_formation_forces_bt_and_formations():
    prov = _provider()  # include_* flags off
    calls = prov._planned_calls_for({'leaf_type': 'formation'})
    names = _call_names(calls)
    assert 'subscribe_once:/bt/state' in names
    assert 'subscribe_once:/formations/status' in names


def test_planned_calls_for_disable_formation_also_pulls_formations():
    prov = _provider()
    calls = prov._planned_calls_for({'leaf_type': 'disable_formation'})
    names = _call_names(calls)
    assert 'subscribe_once:/formations/status' in names


def test_planned_calls_for_mapf_includes_action_status():
    prov = _provider()
    calls = prov._planned_calls_for({'leaf_type': 'mapf'})
    names = _call_names(calls)
    assert 'subscribe_once:/bt/state' in names
    assert 'get_action_status:/swarm/set_goals' in names
    assert 'subscribe_once:/formations/status' not in names


def test_planned_calls_for_unknown_leaf_still_pulls_bt_state():
    prov = _provider()
    calls = prov._planned_calls_for({})
    names = _call_names(calls)
    assert 'subscribe_once:/bt/state' in names
    # No formation/action-status query for unknown failure types.
    assert 'subscribe_once:/formations/status' not in names
    assert 'get_action_status:/swarm/set_goals' not in names


# ---------------------------------------------------------------------------
# summarize_for_remediation
# ---------------------------------------------------------------------------

def test_summarize_drops_topology_listings_keeps_diagnostic_keys():
    snapshot = {
        'timestamp': 't',
        'source': 'mcp_readonly_remediation',
        'warnings': ['x'],
        'failure_summary': {'leaf_type': 'formation'},
        'bt_state': {'mode': 'idle', 'action_status': 'ERROR'},
        'formations': [{'formation_id': 'a', 'failure_code': 'FOLLOWER_STUCK'}],
        'robots': {'active': []},
        'recent_events': ['ev'],
        'map': {'name': 'cave', 'known_zones': ['hub']},
        'mcp': {
            'get_topics': ['/topic'],
            'get_nodes': ['/node'],
            'get_services': ['/srv'],
            'get_actions': ['/act'],
            'subscribe_once:/bt/state': [{'mode': 'idle'}],
            'subscribe_once:/formations/status': [{'failure_code': 1}],
            'get_action_status': {'status': 'SUCCEEDED'},
        },
    }
    slim = summarize_for_remediation(snapshot)
    assert slim['source'] == 'mcp_readonly_remediation'
    assert slim['failure_summary']['leaf_type'] == 'formation'
    assert slim['bt_state']['action_status'] == 'ERROR'
    assert slim['formations'][0]['failure_code'] == 'FOLLOWER_STUCK'
    # Only the name is kept from map (not known_zones)
    assert slim['map'] == {'name': 'cave'}
    # subscribe_once + action_status keys preserved; topology dropped
    assert 'subscribe_once:/bt/state' in slim['mcp']
    assert 'subscribe_once:/formations/status' in slim['mcp']
    assert 'get_action_status' in slim['mcp']
    assert 'get_topics' not in slim['mcp']
    assert 'get_nodes' not in slim['mcp']


def test_summarize_handles_non_dict():
    assert summarize_for_remediation(None) == {}
    assert summarize_for_remediation('garbage') == {}


# ---------------------------------------------------------------------------
# build_remediation_prompt
# ---------------------------------------------------------------------------

@pytest.fixture
def stub_user_prompt(monkeypatch):
    """Avoid touching the map_descriptions YAMLs from unit tests."""
    monkeypatch.setattr(user_prompt, '_user_system', lambda _m: 'system')
    monkeypatch.setattr(user_prompt, '_get_examples', lambda _m: [])


def test_build_remediation_prompt_contains_attempts_and_fresh_ctx(stub_user_prompt):
    fresh = {
        'source': 'mcp_readonly_remediation',
        'bt_state': {'action_status': 'ERROR'},
    }
    attempts = [
        {'leaf_type': 'formation', 'last_error': 'follower 17 stuck',
         'failed_at_phase': 'phase2'},
    ]
    messages = build_remediation_prompt(
        original_user_message='yellow form a line at hub',
        original_plan={
            'type': 'sequence',
            'steps': [{'type': 'formation', 'formation_id': 'yellow_line',
                       'leader_ns': 'robot_16'}],
        },
        attempts=attempts,
        failure_info=attempts[-1],
        fresh_runtime_context=fresh,
        history=[],
        map_name='cave',
    )
    blob = json.dumps(messages, ensure_ascii=False)
    # Original prompt content
    assert 'yellow form a line at hub' in blob
    # Fresh runtime context fingerprint
    assert 'mcp_readonly_remediation' in blob
    # Per-attempt log
    assert 'follower 17 stuck' in blob
    assert 'phase=phase2' in blob
    # Original failed plan echoed back to the LLM as assistant message
    assert any(m['role'] == 'assistant' and 'yellow_line' in m['content']
               for m in messages)
    # Remediation rubric present
    assert any('needs_help:' in m['content'] for m in messages
               if m['role'] == 'system')


def test_build_remediation_prompt_skips_runtime_when_no_context(stub_user_prompt):
    """Caller may pass fresh_runtime_context=None (e.g. CLI without MCP)."""
    messages = build_remediation_prompt(
        original_user_message='cyan to hub',
        original_plan={'type': 'idle'},
        attempts=[{'leaf_type': 'mapf', 'last_error': 'no path',
                   'failed_at_phase': 'phase1'}],
        failure_info={'leaf_type': 'mapf', 'last_error': 'no path',
                      'failed_at_phase': 'phase1'},
        fresh_runtime_context=None,
        history=[],
        map_name='cave',
    )
    # When runtime_context is None build_user_prompt does not inject a
    # runtime-context system message — the only system messages should be
    # the user_chat_system prompt and the remediation rubric.
    blob = json.dumps(messages, ensure_ascii=False)
    assert 'mcp_readonly_remediation' not in blob
    assert 'no path' in blob


# ---------------------------------------------------------------------------
# PlanExecutor.failed_leaf
# ---------------------------------------------------------------------------

def test_plan_executor_records_failed_leaf_in_sequence():
    """The second leaf of a sequence fails — failed_leaf points at it."""
    calls: list[dict] = []

    async def send_fn(node):
        calls.append(node)
        return node.get('reason') != 'fail-me'

    plan = {
        'type': 'sequence',
        'steps': [
            {'type': 'idle', 'reason': 'ok-leaf'},
            {'type': 'idle', 'reason': 'fail-me'},
            {'type': 'idle', 'reason': 'never-runs'},
        ],
    }
    executor = PlanExecutor(send_fn=send_fn)
    ok = asyncio.run(executor.run(plan))
    assert ok is False
    assert executor.failed_leaf is not None
    assert executor.failed_leaf.get('reason') == 'fail-me'
    # Third leaf must not have been attempted.
    reasons = [c.get('reason') for c in calls]
    assert 'never-runs' not in reasons


def test_plan_executor_failed_leaf_resets_on_new_run():
    async def fail_first(node):
        return False

    async def succeed(node):
        return True

    executor = PlanExecutor(send_fn=fail_first)
    asyncio.run(executor.run({'type': 'idle', 'reason': 'r'}))
    assert executor.failed_leaf is not None

    executor._send = succeed
    asyncio.run(executor.run({'type': 'idle', 'reason': 'r'}))
    assert executor.failed_leaf is None
