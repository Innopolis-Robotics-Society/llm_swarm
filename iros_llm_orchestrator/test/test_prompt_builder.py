"""Unit tests for decision_prompt.build_decision_prompt()."""

from iros_llm_orchestrator.common.decision_prompt import (
    SYSTEM_PROMPT, build_decision_prompt)
from iros_llm_orchestrator.common.scenarios import (
    DECISION_SCENARIOS as SCENARIOS)


def test_prompt_contains_system_prompt():
    prompt = build_decision_prompt(
        scenarios=SCENARIOS,
        level='WARN',
        event='robot_2 stalled',
        log_buffer=['[t=1000ms] WARN: robot_2 stalled'],
    )
    assert SYSTEM_PROMPT.strip() in prompt


def test_prompt_contains_examples_section():
    prompt = build_decision_prompt(
        scenarios=SCENARIOS,
        level='INFO',
        event='planner progressing normally',
        log_buffer=['[t=2000ms] INFO: progressing'],
    )
    assert '# Examples' in prompt
    assert '## Input' in prompt
    assert '## Decision' in prompt


def test_prompt_contains_every_scenario_event():
    prompt = build_decision_prompt(
        scenarios=SCENARIOS,
        level='INFO',
        event='heartbeat',
        log_buffer=[],
    )
    for scenario in SCENARIOS:
        assert scenario['event'] in prompt


def test_prompt_contains_current_situation():
    prompt = build_decision_prompt(
        scenarios=SCENARIOS,
        level='WARN',
        event='fatal collision detected',
        log_buffer=['[t=3200ms status=failed] WARN: collision'],
    )
    assert '# Current situation' in prompt
    assert 'level: WARN' in prompt
    assert 'event: fatal collision detected' in prompt
    assert '[t=3200ms status=failed] WARN: collision' in prompt


def test_prompt_respects_tail_limit():
    logs = [f'log_{i}' for i in range(30)]
    prompt = build_decision_prompt(
        scenarios=[],
        level='INFO',
        event='tail test',
        log_buffer=logs,
        tail=5,
    )
    assert 'log_29' in prompt
    assert 'log_25' in prompt
    assert 'log_24' not in prompt
    assert 'log_0' not in prompt


def test_prompt_with_empty_log_buffer():
    prompt = build_decision_prompt(
        scenarios=SCENARIOS,
        level='WARN',
        event='service-level failure',
        log_buffer=[],
    )
    assert '# Current situation' in prompt
    assert 'event: service-level failure' in prompt


def test_prompt_with_empty_scenarios_still_well_formed():
    prompt = build_decision_prompt(
        scenarios=[],
        level='WARN',
        event='X',
        log_buffer=['a', 'b'],
    )
    assert SYSTEM_PROMPT.strip() in prompt
    assert '# Current situation' in prompt
    assert 'event: X' in prompt


def test_scenarios_library_covers_all_categories():
    seen = {(s['level'], s['decision']['decision']) for s in SCENARIOS}
    # spec calls for: WARN->wait, WARN->replan, WARN->abort, INFO->wait
    assert ('WARN', 'wait') in seen
    assert ('WARN', 'replan') in seen
    assert ('WARN', 'abort') in seen
    assert ('INFO', 'wait') in seen


def test_scenarios_library_size_meets_minimum():
    # TZ calls for 8-10 examples
    assert len(SCENARIOS) >= 8
