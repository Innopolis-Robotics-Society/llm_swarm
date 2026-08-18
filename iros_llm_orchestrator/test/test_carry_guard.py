"""Tests for the one-carrier-per-carry-task guard.

The first test is the plan that actually broke cell B twice, copied out of
paper/results/sessions/20260818_085836.
"""

import pytest

from iros_llm_orchestrator.common.carry_guard import enforce_single_carrier


# carry_security_to_admin as it ships in common_scenarios.yaml.
TASKS = {
    'carry_security_to_admin': {
        'type': 'carry',
        'position': [-15.99, 3.65],
        'dropoff': [12.29, -5.06],
        'radius': 1.5,
    },
    'task_electrical': {
        'type': 'point',
        'position': [-6.31, -3.66],
        'radius': 1.5,
    },
}


def _broken_plan():
    """The plan from run 20260818_085836, trimmed to what matters."""
    return {
        'type': 'parallel',
        'steps': [
            {'type': 'mapf', 'robot_ids': [4, 12],
             'goals': [[-5.56, -3.66], [-7.06, -3.66]],
             'reason': 'magenta and orange to electrical task'},
            {'type': 'sequence', 'steps': [
                {'type': 'mapf', 'robot_ids': [1], 'goals': [[-15.99, 3.65]],
                 'reason': 'cyan to security pickup'},
                {'type': 'mapf', 'robot_ids': [1], 'goals': [[12.29, -5.06]],
                 'reason': 'cyan deliver to admin dropoff'},
            ]},
            {'type': 'mapf', 'robot_ids': [5], 'goals': [[-15.99, 3.65]],
             'reason': 'magenta to security pickup zone'},
        ],
    }


def _leaves(node, out=None):
    out = [] if out is None else out
    if node.get('type') in ('sequence', 'parallel'):
        for step in node.get('steps') or []:
            _leaves(step, out)
    elif node.get('type') == 'mapf':
        out.append(node)
    return out


def test_guard_drops_the_stray_robot_and_keeps_the_one_with_the_delivery_leg():
    plan, records = enforce_single_carrier(_broken_plan(), TASKS)

    assert len(records) == 1
    assert records[0]['task'] == 'carry_security_to_admin'
    assert records[0]['kept'] == 1
    assert records[0]['dropped'] == [5]

    routed = [rid for leaf in _leaves(plan) for rid in leaf['robot_ids']]
    assert 5 not in routed, 'the stray robot must not reach the pickup'
    assert routed.count(1) == 2, 'the carrier keeps both pickup and dropoff'
    # The unrelated point-task leaf is untouched.
    assert [4, 12] in [leaf['robot_ids'] for leaf in _leaves(plan)]


def test_guard_removes_the_leaf_left_with_no_robots():
    plan, _ = enforce_single_carrier(_broken_plan(), TASKS)
    reasons = [leaf.get('reason') for leaf in _leaves(plan)]
    assert 'magenta to security pickup zone' not in reasons


def test_a_sound_plan_is_returned_untouched():
    """One carrier, one pickup: the guard must be invisible."""
    plan = {
        'type': 'sequence',
        'steps': [
            {'type': 'mapf', 'robot_ids': [1], 'goals': [[-15.99, 3.65]]},
            {'type': 'mapf', 'robot_ids': [1], 'goals': [[12.29, -5.06]]},
        ],
    }
    guarded, records = enforce_single_carrier(plan, TASKS)
    assert records == []
    assert guarded == plan


def test_a_robot_just_outside_the_radius_is_not_a_carrier():
    """1.6 m from the pickup is outside radius 1.5 -- not a duplicate."""
    plan = {
        'type': 'parallel',
        'steps': [
            {'type': 'mapf', 'robot_ids': [1], 'goals': [[-15.99, 3.65]]},
            {'type': 'mapf', 'robot_ids': [5], 'goals': [[-14.39, 3.65]]},
        ],
    }
    _, records = enforce_single_carrier(plan, TASKS)
    assert records == []


def test_spread_distance_is_not_enough_to_escape_the_zone():
    """The regression this guard exists for.

    _spread_near_duplicate_goals separates duplicate goals by 1.0 m, which
    is inside the 1.5 m carry radius. A geometric fix would leave both
    robots latching the task, so the guard must still act on spread goals.
    """
    plan = {
        'type': 'parallel',
        'steps': [
            {'type': 'mapf', 'robot_ids': [1], 'goals': [[-16.49, 3.65]]},
            {'type': 'mapf', 'robot_ids': [5], 'goals': [[-15.49, 3.65]]},
        ],
    }
    _, records = enforce_single_carrier(plan, TASKS)
    assert len(records) == 1
    assert records[0]['dropped'] == [5]


def test_without_a_delivery_leg_the_first_robot_named_is_kept():
    plan = {
        'type': 'parallel',
        'steps': [
            {'type': 'mapf', 'robot_ids': [7], 'goals': [[-15.99, 3.65]]},
            {'type': 'mapf', 'robot_ids': [2], 'goals': [[-15.99, 3.65]]},
        ],
    }
    _, records = enforce_single_carrier(plan, TASKS)
    assert records[0]['kept'] == 7
    assert records[0]['dropped'] == [2]


def test_two_robots_in_one_leaf_are_reduced_to_one():
    plan = {'type': 'mapf', 'robot_ids': [1, 5],
            'goals': [[-15.99, 3.65], [-15.99, 3.65]]}
    guarded, records = enforce_single_carrier(plan, TASKS)
    assert records[0]['dropped'] == [5]
    assert guarded['robot_ids'] == [1]
    assert guarded['goals'] == [[-15.99, 3.65]]


def test_point_tasks_are_not_touched():
    """Two robots to one point task is normal -- only carries are single."""
    plan = {'type': 'mapf', 'robot_ids': [4, 12],
            'goals': [[-6.31, -3.66], [-6.31, -3.66]]}
    _, records = enforce_single_carrier(plan, TASKS)
    assert records == []


def test_a_task_without_a_radius_is_ignored_rather_than_guessed():
    tasks = {'c': {'type': 'carry', 'position': [0.0, 0.0],
                   'dropoff': [5.0, 5.0]}}
    plan = {'type': 'mapf', 'robot_ids': [1, 2],
            'goals': [[0.0, 0.0], [0.0, 0.0]]}
    _, records = enforce_single_carrier(plan, tasks)
    assert records == []


def test_no_tasks_means_no_opinion():
    plan = {'type': 'mapf', 'robot_ids': [1, 2], 'goals': [[0, 0], [0, 0]]}
    guarded, records = enforce_single_carrier(plan, {})
    assert records == []
    assert guarded == plan


def test_a_plan_that_is_nothing_but_duplicate_carriers_becomes_idle():
    plan = {
        'type': 'parallel',
        'steps': [
            {'type': 'mapf', 'robot_ids': [1], 'goals': [[-15.99, 3.65]]},
            {'type': 'mapf', 'robot_ids': [5], 'goals': [[-15.99, 3.65]]},
            {'type': 'mapf', 'robot_ids': [6], 'goals': [[-15.99, 3.65]]},
        ],
    }
    guarded, records = enforce_single_carrier(plan, TASKS)
    assert records[0]['dropped'] == [5, 6]
    # robot_1 survives, so the plan is not empty; the parallel collapses to
    # its one remaining leaf rather than to idle.
    assert guarded['type'] == 'parallel'
    assert [leaf['robot_ids'] for leaf in _leaves(guarded)] == [[1]]


@pytest.mark.parametrize('goals', [None, [], 'nonsense'])
def test_malformed_goals_do_not_raise(goals):
    plan = {'type': 'mapf', 'robot_ids': [1, 5], 'goals': goals}
    guarded, records = enforce_single_carrier(plan, TASKS)
    assert records == []
    assert guarded == plan


# --- shared dropoff -------------------------------------------------------

from iros_llm_orchestrator.common.carry_guard import spread_shared_dropoffs

# The two M3 carries, which ship with one dropoff between them.
M3_TASKS = {
    'carry_engine_to_reactor': {
        'type': 'carry', 'position': [-23.67, -10.86],
        'dropoff': [-29.65, -2.85], 'radius': 1.5,
    },
    'carry_shields_to_reactor': {
        'type': 'carry', 'position': [16.51, -12.19],
        'dropoff': [-29.65, -2.85], 'radius': 1.5,
    },
}


def test_two_deliveries_to_one_point_are_moved_apart_but_stay_delivered():
    """Run 20260818_104157: robot_6 parked on the dropoff, robot_8 never got there."""
    plan = {'type': 'sequence', 'steps': [
        {'type': 'mapf', 'robot_ids': [6], 'goals': [[-29.65, -2.85]]},
        {'type': 'mapf', 'robot_ids': [8], 'goals': [[-29.65, -2.85]]},
    ]}
    guarded, records = spread_shared_dropoffs(plan, M3_TASKS)
    assert len(records) == 1

    goals = [leaf['goals'][0] for leaf in _leaves(guarded)]
    apart = ((goals[0][0] - goals[1][0]) ** 2
             + (goals[0][1] - goals[1][1]) ** 2) ** 0.5
    assert apart > 1.0, 'one robot would still stand on the other goal cell'
    for g in goals:
        off = ((g[0] + 29.65) ** 2 + (g[1] + 2.85) ** 2) ** 0.5
        assert off < 1.5, 'moved outside the radius, the delivery stops counting'


def test_one_delivery_is_left_alone():
    plan = {'type': 'mapf', 'robot_ids': [6], 'goals': [[-29.65, -2.85]]}
    guarded, records = spread_shared_dropoffs(plan, M3_TASKS)
    assert records == []
    assert guarded == plan


def test_carries_with_their_own_dropoffs_are_left_alone():
    """TASKS has one carry, so nothing is shared and nothing moves."""
    plan = {'type': 'sequence', 'steps': [
        {'type': 'mapf', 'robot_ids': [1], 'goals': [[-15.99, 3.65]]},
        {'type': 'mapf', 'robot_ids': [1], 'goals': [[12.29, -5.06]]},
    ]}
    guarded, records = spread_shared_dropoffs(plan, TASKS)
    assert records == []
    assert guarded == plan
