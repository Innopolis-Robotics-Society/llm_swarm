"""Tests for server-side occupancy-aware MAPF room-goal rewrites."""

import math
import unittest

from iros_llm_orchestrator.common.occupancy_rewrite import (
    rewrite_occupied_room_mapf_goals,
)


def _map():
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
            'weapons': [10.0, 0.0],
        },
        'location_aliases': {
            'caf': 'cafeteria',
        },
        'robot_groups': {
            'orange': {
                'ids': [12, 13, 14, 15],
                'aliases': ['orange'],
            },
            'yellow': {
                'ids': [16, 17, 18, 19],
                'aliases': ['yellow'],
            },
        },
        'formation_zones': [
            {'name': 'cafeteria', 'coords': [0.0, 0.0], 'radius': 5.0},
            {'name': 'weapons', 'coords': [10.0, 0.0], 'radius': 2.0},
        ],
    }


def _yellow_in_cafeteria():
    return {
        16: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
        17: {'x': 0.7, 'y': 0.0, 'yaw': 0.0, 'stale': False},
        18: {'x': 0.0, 'y': 0.7, 'yaw': 0.0, 'stale': False},
        19: {'x': 0.7, 'y': 0.7, 'yaw': 0.0, 'stale': False},
    }


def _dist(a, b):
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


class OccupancyRewriteTests(unittest.TestCase):
    def test_rewrites_spread_room_center_away_from_existing_group(self):
        plan = {
            'type': 'mapf',
            'robot_ids': [12, 13, 14, 15],
            'goals': [[0.0, 0.0]],
            'spread': True,
            'reason': 'orange to cafeteria',
        }

        rewritten, rewrites = rewrite_occupied_room_mapf_goals(
            plan,
            _map(),
            pose_snapshot=_yellow_in_cafeteria(),
            user_message='orange to cafeteria',
        )

        self.assertEqual(len(rewrites), 1)
        self.assertEqual(rewrites[0]['room'], 'cafeteria')
        self.assertNotIn('spread', rewritten)
        self.assertEqual(len(rewritten['goals']), 4)
        for goal in rewritten['goals']:
            for pose in _yellow_in_cafeteria().values():
                self.assertGreaterEqual(_dist(goal, (pose['x'], pose['y'])), 0.89)

    def test_sequence_second_group_avoids_first_group_future_goals(self):
        plan = {
            'type': 'sequence',
            'steps': [
                {
                    'type': 'mapf',
                    'robot_ids': [16, 17, 18, 19],
                    'goals': [[0.0, 0.0]],
                    'spread': True,
                    'reason': 'yellow to cafeteria',
                },
                {
                    'type': 'mapf',
                    'robot_ids': [12, 13, 14, 15],
                    'goals': [[0.0, 0.0]],
                    'spread': True,
                    'reason': 'orange to cafeteria',
                },
            ],
        }

        rewritten, rewrites = rewrite_occupied_room_mapf_goals(
            plan,
            _map(),
            pose_snapshot={},
            user_message='yellow to cafeteria then orange to cafeteria',
        )

        self.assertEqual(len(rewrites), 2)
        yellow_goals = rewritten['steps'][0]['goals']
        orange_goals = rewritten['steps'][1]['goals']
        self.assertEqual(len(yellow_goals), 4)
        self.assertEqual(len(orange_goals), 4)
        for ogoal in orange_goals:
            for ygoal in yellow_goals:
                self.assertGreaterEqual(_dist(ogoal, ygoal), 0.89)

    def test_around_group_uses_live_group_room_not_group_home_room(self):
        plan = {
            'type': 'mapf',
            'robot_ids': [12, 13, 14, 15],
            'goals': [[10.0, 0.0]],
            'spread': True,
            'reason': 'orange around yellow',
        }

        rewritten, rewrites = rewrite_occupied_room_mapf_goals(
            plan,
            _map(),
            pose_snapshot=_yellow_in_cafeteria(),
            user_message='send orange around yellow',
        )

        self.assertEqual(len(rewrites), 1)
        self.assertEqual(rewrites[0]['room'], 'cafeteria')
        self.assertEqual(rewrites[0]['mode'], 'around_group')
        for goal in rewritten['goals']:
            self.assertLess(_dist(goal, (0.35, 0.35)), 3.0)
            self.assertGreater(_dist(goal, (10.0, 0.0)), 5.0)

    def test_exact_non_conflicting_goals_are_left_unchanged(self):
        plan = {
            'type': 'mapf',
            'robot_ids': [12, 13],
            'goals': [[-5.0, -5.0], [-4.0, -5.0]],
            'reason': 'exact goals',
        }

        rewritten, rewrites = rewrite_occupied_room_mapf_goals(
            plan,
            _map(),
            pose_snapshot=_yellow_in_cafeteria(),
            user_message='send orange to exact positions',
        )

        self.assertEqual(rewrites, [])
        self.assertEqual(rewritten, plan)


if __name__ == '__main__':
    unittest.main()
