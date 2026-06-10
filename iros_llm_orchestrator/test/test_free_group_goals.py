"""Tests for occupancy-aware ordinary MAPF group placement."""

import json
import math
import unittest

from iros_llm_orchestrator.context.group_placement import (
    find_free_group_goals_in_room,
)


def _rect_map(width=8.0, height=8.0):
    hx = width / 2.0
    hy = height / 2.0
    return {
        'name': 'test_map',
        'bounds': {
            'x_min': -20.0,
            'x_max': 20.0,
            'y_min': -20.0,
            'y_max': 20.0,
        },
        'named_locations': {
            'cafeteria': [0.0, 0.0],
        },
        'geometry': {
            'cafeteria': {
                'center': [0.0, 0.0],
                'corners': {
                    'top_left': [-hx, hy],
                    'top_right': [hx, hy],
                    'bottom_left': [-hx, -hy],
                    'bottom_right': [hx, -hy],
                },
            },
        },
    }


def _args(**extra):
    return {
        'room': 'cafeteria',
        'robot_ids': [12, 13, 14, 15],
        'avoid_existing_robots': True,
        'min_clearance_m': 0.45,
        'goal_spacing_m': 0.75,
        **extra,
    }


def _dist(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


class FreeGroupGoalsTests(unittest.TestCase):
    def test_empty_rectangular_room_returns_distinct_goals(self):
        result = find_free_group_goals_in_room(_rect_map(), _args())

        self.assertTrue(result['ok'])
        self.assertEqual(result['room'], 'cafeteria')
        self.assertEqual(result['robot_ids'], [12, 13, 14, 15])
        self.assertEqual(len(result['goals']), 4)
        self.assertEqual(len({tuple(goal) for goal in result['goals']}), 4)
        self.assertTrue(result['checks']['inside_room'])
        self.assertTrue(result['checks']['pairwise_clearance_ok'])

    def test_existing_robots_near_center_push_new_group_away(self):
        result = find_free_group_goals_in_room(
            _rect_map(),
            _args(),
            pose_snapshot={
                16: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
                17: {'x': 0.8, 'y': 0.0, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertTrue(result['ok'])
        for goal in result['goals']:
            self.assertGreaterEqual(_dist(goal, (0.0, 0.0)), 0.89)
            self.assertGreaterEqual(_dist(goal, (0.8, 0.0)), 0.89)
        self.assertTrue(result['checks']['avoids_existing_robots'])

    def test_moving_robots_do_not_block_their_own_goals(self):
        result = find_free_group_goals_in_room(
            _rect_map(),
            _args(),
            pose_snapshot={
                12: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
                13: {'x': 0.5, 'y': 0.0, 'yaw': 0.0, 'stale': False},
                14: {'x': 0.0, 'y': 0.5, 'yaw': 0.0, 'stale': False},
                15: {'x': 0.5, 'y': 0.5, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertTrue(result['ok'])
        self.assertEqual(len(result['goals']), 4)
        self.assertFalse(result['warnings'])

    def test_avoid_robot_ids_are_respected_even_when_existing_disabled(self):
        result = find_free_group_goals_in_room(
            _rect_map(),
            _args(avoid_existing_robots=False, avoid_robot_ids=[16]),
            pose_snapshot={
                16: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertTrue(result['ok'])
        for goal in result['goals']:
            self.assertGreaterEqual(_dist(goal, (0.0, 0.0)), 0.89)
        self.assertTrue(result['checks']['avoids_avoid_robot_ids'])

    def test_prefer_near_group_places_near_but_not_on_top(self):
        result = find_free_group_goals_in_room(
            _rect_map(width=10.0, height=10.0),
            _args(
                robot_ids=[12, 13],
                avoid_robot_ids=[16, 17],
                prefer_near_group=[16, 17],
                placement_mode='around_group',
            ),
            pose_snapshot={
                16: {'x': 1.0, 'y': 1.0, 'yaw': 0.0, 'stale': False},
                17: {'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertTrue(result['ok'])
        self.assertTrue(result['used_prefer_near_group'])
        anchor = result['anchor']
        for goal in result['goals']:
            self.assertLess(_dist(goal, anchor), 3.0)
            self.assertGreaterEqual(_dist(goal, (1.0, 1.0)), 0.89)
            self.assertGreaterEqual(_dist(goal, (1.0, 2.0)), 0.89)

    def test_too_small_room_returns_failure(self):
        result = find_free_group_goals_in_room(
            _rect_map(width=1.8, height=1.8),
            _args(),
        )

        self.assertFalse(result['ok'])
        self.assertEqual(result['reason'], 'not_enough_free_space')
        self.assertTrue(result['suggestions'])

    def test_output_includes_valid_mapf_leaf(self):
        result = find_free_group_goals_in_room(_rect_map(), _args())

        self.assertTrue(result['ok'])
        leaf = result['mapf_leaf']
        self.assertEqual(leaf['type'], 'mapf')
        self.assertEqual(leaf['robot_ids'], result['robot_ids'])
        self.assertEqual(leaf['goals'], result['goals'])
        self.assertNotIn('spread', leaf)

    def test_output_is_json_serializable(self):
        result = find_free_group_goals_in_room(_rect_map(), _args())

        encoded = json.dumps(result)
        self.assertIn('mapf_leaf', encoded)


if __name__ == '__main__':
    unittest.main()
