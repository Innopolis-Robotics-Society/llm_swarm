"""Unit tests for command_prompt_builder.build_command_prompt()."""

from dataclasses import dataclass

from iros_llm_orchestrator.common.command_prompt import (
    SYSTEM_PROMPT,
    build_command_prompt,
)
from iros_llm_orchestrator.common.scenarios import COMMAND_SCENARIOS


@dataclass
class FakeBTState:
    mode: str = 'mapf'
    action_status: str = 'WARN'
    active_action: str = 'MapfPlan'
    action_summary: str = 'robot 3 stalled'
    last_error: str = 'robot_3 stalled'
    stamp_ms: int = 1234


def test_system_prompt_present():
    p = build_command_prompt(COMMAND_SCENARIOS, [], FakeBTState())
    assert SYSTEM_PROMPT.strip().splitlines()[0] in p


def test_all_scenarios_included():
    p = build_command_prompt(COMMAND_SCENARIOS, [], FakeBTState())
    for scenario in COMMAND_SCENARIOS:
        assert scenario['trigger']['active_action'] in p
        assert scenario['command']['mode'] in p


def test_current_situation_present():
    trigger = FakeBTState(
        action_status='ERROR',
        active_action='SetFormation',
        last_error='retry budget exhausted',
    )
    history = [FakeBTState(stamp_ms=i) for i in range(5)]
    p = build_command_prompt(COMMAND_SCENARIOS, history, trigger)
    assert '# Current situation' in p
    assert 'status=ERROR' in p
    assert 'retry budget exhausted' in p


def test_history_tail_limit():
    history = [FakeBTState(stamp_ms=i) for i in range(100)]
    p = build_command_prompt(COMMAND_SCENARIOS, history, FakeBTState(), tail=5)
    # Only the last 5 snapshots of the current situation should appear.
    # The string 't=0ms' is from the earliest snapshot and must be dropped.
    current = p.split('# Current situation', 1)[1]
    assert 't=95ms' in current
    assert 't=0ms' not in current
