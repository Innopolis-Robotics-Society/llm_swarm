"""Unit/static tests for Channel 3 read-only context injection."""

import asyncio
from types import SimpleNamespace

from iros_llm_orchestrator.common import user_prompt
from iros_llm_orchestrator.context import (
    BLOCKED_MCP_TOOLS,
    DEFAULT_MCP_READ_TOOLS,
    ChatContextConfig,
    NoneContextProvider,
    make_context_provider,
)
from iros_llm_orchestrator.context.mcp_readonly_provider import (
    McpReadonlyContextProvider,
)
from iros_llm_orchestrator.context.no_execute import (
    should_skip_reply_only_execution,
)
from iros_llm_orchestrator.context.provider import (
    bound_context,
    safe_mcp_allowlist,
)
from iros_llm_orchestrator.context.ros_readonly_provider import (
    derive_robot_summary,
    normalize_bt_state,
    normalize_formations_status,
    normalize_llm_event,
)


def test_default_context_provider_is_none():
    provider = make_context_provider(None, ChatContextConfig())

    assert isinstance(provider, NoneContextProvider)
    assert asyncio.run(provider.get_context())['source'] == 'none'


def test_ros_readonly_normalizes_fake_messages():
    bt = SimpleNamespace(
        mode='mapf',
        action_status='RUNNING',
        active_action='MapfPlan',
        action_summary='moving cyan',
        last_error='',
        robot_ids=[0, 1],
        goals=[SimpleNamespace(x=1.23456, y=2.34567)],
        formation_id='',
        leader_ns='',
        llm_thinking=False,
        formation_state=2,
        formation_failure_code=0,
        formation_failure_reason='',
        formation_max_error_m=0.2,
        formation_mean_error_m=0.1,
        stamp_ms=123,
    )
    event = SimpleNamespace(
        channel=3,
        trigger='what are robots doing?',
        output='reply',
        reason='reply_only',
    )
    formations = SimpleNamespace(formations=[
        SimpleNamespace(
            formation_id='wedge',
            leader_ns='robot_0',
            follower_ns=['robot_1'],
            state=2,
            failure_code=0,
            failure_reason='',
            max_error_m=0.2,
            mean_error_m=0.1,
        )
    ])

    bt_context = normalize_bt_state(bt)
    robots = derive_robot_summary(bt, {'robot_groups': {'cyan': {'ids': [0, 1, 2]}}})

    assert bt_context['formation_state'] == 'STABLE'
    assert bt_context['goals'] == [[1.235, 2.346]]
    assert robots == {'active': [0, 1], 'idle': [2], 'unknown': []}
    assert 'channel=3' in normalize_llm_event(event)
    assert normalize_formations_status(formations)[0]['status'] == 'STABLE'


def test_context_is_capped_by_max_chars():
    context = {
        'timestamp': 'now',
        'source': 'ros_readonly',
        'recent_events': ['x' * 500 for _ in range(20)],
        'mcp': {'huge': 'y' * 5000},
    }

    bounded = bound_context(context, 300)

    assert bounded['source'] == 'ros_readonly'
    assert any('truncated' in warning for warning in bounded['warnings'])


def test_prompt_includes_runtime_context_only_when_enabled(monkeypatch):
    monkeypatch.setattr(user_prompt, '_user_system', lambda _map: 'system')
    monkeypatch.setattr(user_prompt, '_get_examples', lambda _map: [])

    no_context = user_prompt.build_user_prompt(
        'hello',
        map_name='cave',
        runtime_context={'source': 'none'},
    )
    with_context = user_prompt.build_user_prompt(
        'what are the robots doing right now?',
        map_name='cave',
        runtime_context={
            'source': 'ros_readonly',
            'bt_state': {'mode': 'idle'},
        },
    )

    assert len(no_context) == 2
    assert len(with_context) == 3
    assert 'Read-only current ROS/system context' in with_context[1]['content']
    assert '"source":"ros_readonly"' in with_context[1]['content']


def test_mcp_allowlist_excludes_write_tools_and_ping_robots():
    allowed, warnings = safe_mcp_allowlist([
        *DEFAULT_MCP_READ_TOOLS,
        'publish_once',
        'call_service',
        'ping_robot',
        'ping_robots',
    ])

    assert 'ping_robot' in BLOCKED_MCP_TOOLS
    assert 'ping_robots' in BLOCKED_MCP_TOOLS
    assert 'ping_robots' not in DEFAULT_MCP_READ_TOOLS
    assert 'publish_once' not in allowed
    assert 'call_service' not in allowed
    assert 'ping_robot' not in allowed
    assert 'ping_robots' not in allowed
    assert len(warnings) == 4


def test_mcp_unavailable_fallback_returns_warning_context():
    provider = McpReadonlyContextProvider(ChatContextConfig(
        provider='mcp_readonly',
        timeout_sec=0.1,
        mcp_enabled=True,
        mcp_transport='unsupported',
    ))

    context = asyncio.run(provider.get_context())

    assert context['source'] == 'mcp_readonly'
    assert context['warnings']
    assert any(
        'MCP server unavailable' in warning
        or 'rosbridge is not reachable' in warning
        or 'uvx not found' in warning
        or 'mcp Python SDK is not installed' in warning
        for warning in context['warnings']
    )


def test_reply_only_guard_skips_factual_questions_but_allows_stop():
    plan = {'type': 'idle', 'reason': 'reply_only: status answer'}
    context = {'source': 'ros_readonly'}

    assert should_skip_reply_only_execution(
        'what are the robots doing right now?',
        plan,
        context,
    )
    assert should_skip_reply_only_execution(
        'is cyan team already in formation?',
        {'type': 'idle', 'reason': 'status answer'},
        context,
    )
    assert not should_skip_reply_only_execution(
        'stop',
        {'type': 'idle', 'reason': 'operator stop'},
        context,
    )
    assert not should_skip_reply_only_execution(
        'cyan go to center',
        {'type': 'mapf', 'robot_ids': [0], 'goals': [[0.0, 0.0]]},
        context,
    )
