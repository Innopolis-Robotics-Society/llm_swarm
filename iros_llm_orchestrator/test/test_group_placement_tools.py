"""Tests for deterministic room-aware group placement tools."""

import json
import math
import unittest

from iros_llm_orchestrator.context.group_placement import (
    find_group_placement_in_room,
    follower_offsets_for_formation,
    formation_world_points,
)


def _rect_map(width=10.0, height=8.0):
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
            'storage': [8.0, 0.0],
        },
        'location_aliases': {
            'caf': 'cafeteria',
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


def _wedge_args(*, room='cafeteria', groups=None, **extra):
    if groups is None:
        groups = [
            {'name': 'green', 'robot_ids': [8, 9, 10, 11], 'formation': 'wedge'},
        ]
    return {
        'room': room,
        'groups': groups,
        'avoid_existing_robots': True,
        'min_clearance_m': 0.35,
        **extra,
    }


class GroupPlacementToolTests(unittest.TestCase):
    def test_single_wedge_fits_inside_simple_rectangular_room(self):
        result = find_group_placement_in_room(_rect_map(), _wedge_args())

        self.assertIs(result['ok'], True)
        self.assertEqual(result['room_boundary_source'], 'map_yaml_rect')
        placement = result['placements'][0]
        self.assertEqual(placement['leader_robot_id'], 8)
        self.assertEqual(placement['leader_goal'], [0.0, 0.0])
        self.assertEqual(placement['follower_goals'], {
            '9': [-1.0, 0.6],
            '10': [-1.0, -0.6],
            '11': [-2.0, 0.0],
        })
        self.assertTrue(all(placement['checks'].values()))

    def test_two_wedges_fit_in_same_room_without_overlap(self):
        groups = [
            {'name': 'green', 'robot_ids': [8, 9, 10, 11], 'formation': 'wedge'},
            {'name': 'yellow', 'robot_ids': [16, 17, 18, 19], 'formation': 'wedge'},
        ]
        result = find_group_placement_in_room(
            _rect_map(width=12.0, height=8.0),
            _wedge_args(groups=groups),
        )

        self.assertIs(result['ok'], True)
        self.assertEqual(len(result['placements']), 2)
        self.assertIs(
            result['placements'][0]['checks']['avoids_other_requested_groups'],
            True,
        )
        self.assertIs(
            result['placements'][1]['checks']['avoids_other_requested_groups'],
            True,
        )
        self.assertNotEqual(
            result['placements'][0]['leader_goal'],
            result['placements'][1]['leader_goal'],
        )

    def test_two_wedges_fail_if_room_is_too_small(self):
        groups = [
            {'name': 'green', 'robot_ids': [8, 9, 10, 11], 'formation': 'wedge'},
            {'name': 'yellow', 'robot_ids': [16, 17, 18, 19], 'formation': 'wedge'},
        ]
        result = find_group_placement_in_room(
            _rect_map(width=2.0, height=2.0),
            _wedge_args(groups=groups),
        )

        self.assertIs(result['ok'], False)
        self.assertEqual(result['reason'], 'not_enough_space_in_room')
        self.assertTrue(result['suggestions'])

    def test_existing_robot_blocks_one_candidate_and_tool_finds_another(self):
        result = find_group_placement_in_room(
            _rect_map(),
            _wedge_args(),
            pose_snapshot={
                99: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertIs(result['ok'], True)
        self.assertNotEqual(result['placements'][0]['leader_goal'], [0.0, 0.0])
        self.assertIs(
            result['placements'][0]['checks']['avoids_existing_robots'],
            True,
        )

    def test_existing_robot_footprints_can_make_placement_infeasible(self):
        result = find_group_placement_in_room(
            _rect_map(),
            _wedge_args(candidate_spacing_m=10.0),
            pose_snapshot={
                99: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'stale': False},
            },
        )

        self.assertIs(result['ok'], False)
        self.assertIn('avoids_existing_robots', result['failed_checks'])

    def test_generated_follower_goals_match_leader_offsets_and_heading(self):
        offsets = follower_offsets_for_formation('wedge', 3)
        points = formation_world_points((1.0, 2.0), offsets, math.pi / 2.0)
        rounded = [[round(x, 3), round(y, 3)] for x, y in points]

        self.assertEqual(rounded[0], [1.0, 2.0])
        self.assertEqual(rounded[1], [0.4, 1.0])
        self.assertEqual(rounded[2], [1.6, 1.0])
        self.assertEqual(rounded[3], [1.0, 0.0])

    def test_unknown_room_returns_clear_error_and_suggestions(self):
        result = find_group_placement_in_room(
            _rect_map(),
            _wedge_args(room='cafetaria'),
        )

        self.assertIs(result['ok'], False)
        self.assertEqual(result['reason'], 'unknown_room')
        self.assertEqual(result['failed_checks'], ['room_resolved'])
        self.assertIn('cafeteria', result['known_rooms'])
        self.assertTrue(result['suggestions'])

    def test_tool_output_is_json_serializable(self):
        result = find_group_placement_in_room(_rect_map(), _wedge_args())

        encoded = json.dumps(result)
        self.assertIn('green', encoded)


if __name__ == '__main__':
    unittest.main()
