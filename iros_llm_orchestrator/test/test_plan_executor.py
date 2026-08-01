"""Unit tests for plan_executor — flatten_parallel + parse_plan."""

import asyncio

import pytest

from iros_llm_orchestrator.common.plan_executor import (
    PlanExecutor, coerce_robot_id, flatten_parallel, parse_plan)
from iros_llm_orchestrator.context.pose_cache import (
    compute_formation_staging)


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


def test_flatten_parallel_expands_single_center_spread_before_merge():
    p = {'type': 'parallel', 'steps': [
        _mapf([8, 9, 10, 11], [[2.7, 10.1]], 'green to cafeteria') | {
            'spread': True,
        },
        _mapf([16, 17, 18, 19], [[2.7, 10.1]], 'yellow to cafeteria') | {
            'spread': True,
        },
    ]}

    out = flatten_parallel(p)

    assert len(out) == 1
    assert out[0]['robot_ids'] == [8, 9, 10, 11, 16, 17, 18, 19]
    assert len(out[0]['goals']) == 8
    assert len({tuple(goal) for goal in out[0]['goals']}) == 8


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


# ---------------------------------------------------------------------------
# Formation auto-staging
# ---------------------------------------------------------------------------

def _line_formation():
    return {
        'type': 'formation',
        'formation_id': 'magenta_line',
        'leader_ns': 'robot_4',
        'follower_ns': ['robot_5', 'robot_6', 'robot_7'],
        'offsets_x': [-1.5, -3.0, -4.5],
        'offsets_y': [0.0, 0.0, 0.0],
        'reason': 'line behind robot_4',
    }


def test_compute_staging_skips_when_followers_already_in_position():
    snapshot = {
        4: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        5: {'x': -1.5, 'y': 0.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        6: {'x': -3.0, 'y': 0.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        7: {'x': -4.5, 'y': 0.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
    }
    assert compute_formation_staging(_line_formation(), snapshot) is None


def test_compute_staging_emits_mapf_for_out_of_position_followers():
    snapshot = {
        4: {'x': -9.45, 'y': -6.65, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        5: {'x': -7.95, 'y': -6.65, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        6: {'x': -9.45, 'y': -5.15, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
        7: {'x': -7.95, 'y': -5.15, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
    }
    staging = compute_formation_staging(_line_formation(), snapshot)
    assert staging is not None
    assert staging['type'] == 'mapf'
    assert staging['robot_ids'] == [5, 6, 7]
    # Targets are leader_xy + offset (leader yaw=0, so straight subtraction).
    assert staging['goals'] == [
        [-10.95, -6.65], [-12.45, -6.65], [-13.95, -6.65]]
    assert 'magenta_line' in staging['reason']


def test_compute_staging_respects_leader_yaw_rotation():
    # Leader facing +y (yaw = pi/2). A (-1.5, 0) offset (behind in body frame)
    # rotates to (0, -1.5) in world frame.
    import math
    snapshot = {
        4: {'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2,
            'stale_ms': 50, 'stale': False},
        5: {'x': 5.0, 'y': 5.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
    }
    node = {
        'type': 'formation', 'formation_id': 'f', 'leader_ns': 'robot_4',
        'follower_ns': ['robot_5'], 'offsets_x': [-1.5], 'offsets_y': [0.0],
    }
    staging = compute_formation_staging(node, snapshot)
    assert staging is not None
    tx, ty = staging['goals'][0]
    assert abs(tx - 0.0) < 1e-3
    assert abs(ty - (-1.5)) < 1e-3


def test_compute_staging_returns_none_when_leader_pose_missing():
    snapshot = {
        5: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale_ms': 50, 'stale': False},
    }
    assert compute_formation_staging(_line_formation(), snapshot) is None


def test_compute_staging_returns_none_when_leader_stale():
    snapshot = {
        4: {'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'stale_ms': 5000, 'stale': True},
    }
    assert compute_formation_staging(_line_formation(), snapshot) is None


def test_executor_prepends_staging_for_formation_when_hook_returns_mapf():
    sent: list[dict] = []

    async def fake_send(node: dict) -> bool:
        sent.append(node)
        return True

    def hook(_node):
        return {
            'type': 'mapf', 'robot_ids': [5, 6, 7],
            'goals': [[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]],
            'reason': 'auto-stage 3 follower(s) for magenta_line',
        }

    executor = PlanExecutor(
        send_fn=fake_send, formation_prestage_hook=hook)
    plan = _line_formation()
    ok = asyncio.run(executor.run(plan))

    assert ok
    assert len(sent) == 2
    assert sent[0]['type'] == 'mapf'
    assert sent[0]['robot_ids'] == [5, 6, 7]
    assert sent[1]['type'] == 'formation'


def test_executor_skips_staging_when_hook_returns_none():
    sent: list[dict] = []

    async def fake_send(node: dict) -> bool:
        sent.append(node)
        return True

    executor = PlanExecutor(
        send_fn=fake_send, formation_prestage_hook=lambda _n: None)
    ok = asyncio.run(executor.run(_line_formation()))

    assert ok
    assert len(sent) == 1
    assert sent[0]['type'] == 'formation'


def test_executor_aborts_when_staging_step_fails():
    sent: list[dict] = []

    async def fake_send(node: dict) -> bool:
        sent.append(node)
        return node['type'] != 'mapf'   # staging mapf fails

    def hook(_n):
        return {'type': 'mapf', 'robot_ids': [5], 'goals': [[1.0, 0.0]],
                'reason': 'stage'}

    executor = PlanExecutor(
        send_fn=fake_send, formation_prestage_hook=hook)
    ok = asyncio.run(executor.run(_line_formation()))

    assert not ok
    # Staging step ran, formation never dispatched.
    assert len(sent) == 1
    assert sent[0]['type'] == 'mapf'
    assert executor.failed_leaf['type'] == 'mapf'


# ---------------------------------------------------------------------------
# coerce_robot_id + robot_id / goal normalisation
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('value,expected', [
    (10, 10),
    (0, 0),
    ('10', 10),
    ('robot_10', 10),
    ('robot10', 10),
    ('robot-7', 7),
    ('  robot_3  ', 3),
    ('ROBOT_5', 5),
    (10.0, 10),
])
def test_coerce_robot_id_accepts(value, expected):
    assert coerce_robot_id(value) == expected


@pytest.mark.parametrize('value', [
    'orange', 'robot_', '', 'robot_x', True, False, 1.5, None, [3],
])
def test_coerce_robot_id_rejects(value):
    with pytest.raises(ValueError):
        coerce_robot_id(value)


def test_parse_plan_normalises_string_robot_ids():
    """The exact qwen2.5:7b failure: robot_ids emitted as 'robot_N' strings."""
    bad = {'type': 'mapf',
           'robot_ids': ['robot_10', 'robot_1'],
           'goals': [[1.0, 2.0], [3.0, 4.0]]}
    out = parse_plan(bad)
    assert out['robot_ids'] == [10, 1]
    assert all(isinstance(r, int) for r in out['robot_ids'])


def test_parse_plan_normalises_goal_coordinates_to_float():
    out = parse_plan({'type': 'mapf', 'robot_ids': [0], 'goals': [['1.5', 2]]})
    assert out['goals'] == [[1.5, 2.0]]
    assert all(isinstance(c, float) for c in out['goals'][0])


def test_parse_plan_rejects_unparseable_robot_id():
    bad = {'type': 'mapf', 'robot_ids': ['orange'], 'goals': [[1.0, 1.0]]}
    with pytest.raises(ValueError):
        parse_plan(bad)


def test_parse_plan_rejects_non_numeric_goal():
    bad = {'type': 'mapf', 'robot_ids': [0], 'goals': [['x', 'y']]}
    with pytest.raises(ValueError):
        parse_plan(bad)


def test_flatten_parallel_normalises_string_robot_ids():
    p = {'type': 'parallel', 'steps': [
        _mapf(['robot_0'], [[1.0, 1.0]]),
        _mapf(['robot_2'], [[3.0, 3.0]])]}
    out = flatten_parallel(p)
    assert sorted(out[0]['robot_ids']) == [0, 2]
