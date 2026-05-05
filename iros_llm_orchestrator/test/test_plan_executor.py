"""Unit tests for plan_executor — flatten_parallel + parse_plan."""

import pytest

from iros_llm_orchestrator.common.plan_executor import (
    flatten_parallel, parse_plan)


def _mapf(ids, goals, reason='r'):
    return {'type': 'mapf', 'robot_ids': list(ids), 'goals': list(goals),
            'reason': reason}


def test_flatten_merges_two_disjoint_mapf():
    p = {'type': 'parallel', 'steps': [
        _mapf([0, 1], [[1.0, 1.0], [2.0, 2.0]]),
        _mapf([2, 3], [[3.0, 3.0], [4.0, 4.0]])]}
    out = flatten_parallel(p)
    assert len(out) == 1
    assert out[0]['type'] == 'mapf'
    assert out[0]['robot_ids'] == [0, 1, 2, 3]
    assert out[0]['goals']     == [[1.0, 1.0], [2.0, 2.0],
                                    [3.0, 3.0], [4.0, 4.0]]


def test_flatten_dedup_overlapping_robot_ids():
    """Same robot in two parallel branches → last-write-wins."""
    p = {'type': 'parallel', 'steps': [
        _mapf([0, 1], [[1.0, 1.0], [2.0, 2.0]]),
        _mapf([1, 2], [[9.9, 9.9], [3.0, 3.0]])]}
    out = flatten_parallel(p)
    assert len(out) == 1
    merged = out[0]
    assert sorted(merged['robot_ids']) == [0, 1, 2]
    # robot 1's goal must come from the second branch (last write wins).
    rid_to_goal = dict(zip(merged['robot_ids'], merged['goals']))
    assert rid_to_goal[1] == [9.9, 9.9]
    assert rid_to_goal[0] == [1.0, 1.0]
    assert rid_to_goal[2] == [3.0, 3.0]


def test_flatten_idle_takes_priority():
    p = {'type': 'parallel', 'steps': [
        _mapf([0], [[1.0, 1.0]]),
        {'type': 'idle', 'reason': 'stop'}]}
    out = flatten_parallel(p)
    assert out == [{'type': 'idle', 'reason': 'idle in parallel branch'}]


def test_flatten_mixes_mapf_and_formation_sequentially():
    p = {'type': 'parallel', 'steps': [
        _mapf([0], [[1.0, 1.0]]),
        {'type': 'formation', 'formation_id': 'line',
         'leader_ns': 'robot_0', 'follower_ns': [], 'offsets_x': [],
         'offsets_y': [], 'reason': 'line'}]}
    out = flatten_parallel(p)
    # Merged mapf first, then formation runs sequentially.
    assert len(out) == 2
    assert out[0]['type'] == 'mapf'
    assert out[1]['type'] == 'formation'


def test_parse_plan_unwraps_reply_envelope():
    raw = '{"reply": "ok", "plan": {"type": "idle", "reason": "x"}}'
    out = parse_plan(raw)
    assert out['type'] == 'idle'


def test_parse_plan_rejects_mismatched_lengths():
    bad = {'type': 'mapf', 'robot_ids': [0, 1], 'goals': [[1.0, 1.0]]}
    with pytest.raises(ValueError):
        parse_plan(bad)
