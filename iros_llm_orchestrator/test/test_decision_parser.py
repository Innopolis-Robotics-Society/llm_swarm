"""Unit tests for decision_parser.parse_llm_decision()."""

from iros_llm_orchestrator.decision_parser import parse_llm_decision


def test_plain_json_wait():
    raw = '{"decision": "wait", "reason": "all good"}'
    assert parse_llm_decision(raw) == 'wait'


def test_plain_json_replan():
    raw = '{"decision": "replan", "reason": "robot stalled"}'
    assert parse_llm_decision(raw) == 'replan'


def test_plain_json_abort():
    raw = '{"decision": "abort", "reason": "fatal collision"}'
    assert parse_llm_decision(raw) == 'abort'


def test_fenced_json_with_lang():
    raw = '```json\n{"decision": "abort", "reason": "fatal"}\n```'
    assert parse_llm_decision(raw) == 'abort'


def test_fenced_json_no_lang():
    raw = '```\n{"decision": "replan", "reason": "needs replan"}\n```'
    assert parse_llm_decision(raw) == 'replan'


def test_text_before_and_after_json():
    raw = (
        'Let me think about this...\n'
        '{"decision": "wait", "reason": "system still progressing"}\n'
        'Done thinking.'
    )
    assert parse_llm_decision(raw) == 'wait'


def test_invalid_decision_value_falls_back_to_wait():
    raw = '{"decision": "panic", "reason": "bad enum"}'
    assert parse_llm_decision(raw) == 'wait'


def test_empty_decision_value_falls_back_to_wait():
    raw = '{"decision": "", "reason": "neutral"}'
    assert parse_llm_decision(raw) == 'wait'


def test_unterminated_json_falls_back_to_wait():
    raw = '{"decision": "replan", "reason": "missing end"'
    assert parse_llm_decision(raw) == 'wait'


def test_non_string_decision_value_falls_back_to_wait():
    raw = '{"decision": 42, "reason": "type error"}'
    assert parse_llm_decision(raw) == 'wait'


def test_empty_input_falls_back_to_wait():
    assert parse_llm_decision('') == 'wait'


def test_none_like_input_falls_back_to_wait():
    assert parse_llm_decision(None) == 'wait'


def test_case_insensitive_decision():
    raw = '{"decision": "AbOrT", "reason": "mixed case"}'
    assert parse_llm_decision(raw) == 'abort'


def test_prose_without_json_falls_back_to_wait():
    raw = 'I think the robots should probably keep going for now.'
    assert parse_llm_decision(raw) == 'wait'
