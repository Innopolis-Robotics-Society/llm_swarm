"""Unit tests for command_parser.parse_llm_command()."""

import pytest

from iros_llm_orchestrator.command_parser import parse_llm_command


def test_idle_minimal():
    out = parse_llm_command('{"mode": "idle", "reason": "halt"}')
    assert out['mode'] == 'idle'
    assert out['reason'] == 'halt'


def test_mapf_valid():
    raw = (
        '{"mode": "mapf", "robot_ids": [0, 1],'
        ' "goals": [[1.0, 2.0], [3.0, 4.0]], "reason": "replan"}'
    )
    out = parse_llm_command(raw)
    assert out['mode'] == 'mapf'
    assert out['robot_ids'] == [0, 1]
    assert out['goals'] == [[1.0, 2.0], [3.0, 4.0]]


def test_formation_valid():
    raw = (
        '{"mode": "formation", "formation_id": "line",'
        ' "leader_ns": "robot_1",'
        ' "follower_ns": ["robot_2"],'
        ' "offsets_x": [-1.0], "offsets_y": [0.0],'
        ' "reason": "reform"}'
    )
    out = parse_llm_command(raw)
    assert out['mode'] == 'formation'
    assert out['formation_id'] == 'line'
    assert out['leader_ns'] == 'robot_1'
    assert out['follower_ns'] == ['robot_2']


def test_fenced_json():
    raw = '```json\n{"mode": "idle", "reason": "stop"}\n```'
    assert parse_llm_command(raw)['mode'] == 'idle'


def test_surrounding_text():
    raw = 'Here is my answer: {"mode": "idle", "reason": "x"} — done.'
    assert parse_llm_command(raw)['mode'] == 'idle'


def test_mapf_missing_goals_raises():
    with pytest.raises(ValueError):
        parse_llm_command('{"mode": "mapf", "robot_ids": [0]}')


def test_mapf_length_mismatch_raises():
    with pytest.raises(ValueError):
        parse_llm_command(
            '{"mode": "mapf", "robot_ids": [0, 1], "goals": [[1.0, 2.0]]}'
        )


def test_formation_missing_leader_raises():
    with pytest.raises(ValueError):
        parse_llm_command('{"mode": "formation", "formation_id": "line"}')


def test_formation_length_mismatch_raises():
    with pytest.raises(ValueError):
        parse_llm_command(
            '{"mode": "formation", "formation_id": "l", "leader_ns": "r0",'
            ' "follower_ns": ["r1", "r2"], "offsets_x": [1.0], "offsets_y": [0.0, 0.0]}'
        )


def test_invalid_mode_raises():
    with pytest.raises(ValueError):
        parse_llm_command('{"mode": "explode", "reason": "x"}')


def test_invalid_json_raises():
    with pytest.raises(ValueError):
        parse_llm_command('{not json}')


def test_empty_raises():
    with pytest.raises(ValueError):
        parse_llm_command('')


def test_no_json_object_raises():
    with pytest.raises(ValueError):
        parse_llm_command('just some prose without braces')
