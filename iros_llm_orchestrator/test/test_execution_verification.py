"""Unit tests for deterministic post-execution verification."""

import unittest

from iros_llm_orchestrator.context.execution_verification import (
    verify_plan_execution_state,
)


FORMATION_PLAN = {
    'type': 'formation',
    'formation_id': 'green_wedge',
    'leader_ns': 'robot_8',
    'follower_ns': ['robot_9', 'robot_10', 'robot_11'],
    'offsets_x': [-1.0, -1.0, -2.0],
    'offsets_y': [0.6, -0.6, 0.0],
}
MAPF_PLAN = {
    'type': 'mapf',
    'robot_ids': [12, 13],
    'goals': [[1.0, 1.0], [2.0, 1.0]],
    'reason': 'orange around yellow',
}


def _args(**extra):
    return {
        'original_user_request': 'green form wedge',
        'last_plan': FORMATION_PLAN,
        **extra,
    }


class ExecutionVerificationTests(unittest.TestCase):
    def test_formation_active_and_followers_within_tolerance_ok(self):
        result = verify_plan_execution_state(
            {},
            _args(),
            formations_status={
                'formations': [{
                    'formation_id': 'green_wedge',
                    'leader_ns': 'robot_8',
                    'follower_ns': ['robot_9', 'robot_10', 'robot_11'],
                    'state': 'STABLE',
                    'follower_errors_m': [0.05, 0.1, 0.03],
                }],
            },
        )

        self.assertIs(result['ok'], True)
        self.assertEqual(result['checks']['formations_active']['states']['green_wedge'], 'STABLE')
        self.assertIs(result['checks']['followers_within_tolerance']['ok'], True)

    def test_formation_missing_not_active_returns_activate_or_restage(self):
        result = verify_plan_execution_state(
            {},
            _args(),
            formations_status={'formations': []},
        )

        self.assertIs(result['ok'], False)
        self.assertEqual(result['checks']['formations_active']['missing'], ['green_wedge'])
        self.assertEqual(
            result['repair_recommendation']['type'],
            'activate_or_restage',
        )

    def test_followers_too_far_returns_restage_then_activate(self):
        result = verify_plan_execution_state(
            {},
            _args(),
            formations_status={
                'formations': [{
                    'formation_id': 'green_wedge',
                    'state': 'FORMING',
                    'follower_ns': ['robot_9', 'robot_10', 'robot_11'],
                    'follower_errors_m': [0.1, 2.67, 0.2],
                }],
            },
        )

        self.assertIs(result['ok'], False)
        failed = result['checks']['followers_within_tolerance']['failed']
        self.assertEqual(failed[0]['robot'], 'robot_10')
        self.assertEqual(
            result['repair_recommendation']['type'],
            'restage_then_activate',
        )

    def test_recent_registered_but_not_activated_error_is_captured(self):
        result = verify_plan_execution_state(
            {},
            _args(),
            formations_status={
                'formations': [{
                    'formation_id': 'green_wedge',
                    'state': 'FORMING',
                    'follower_ns': ['robot_9', 'robot_10', 'robot_11'],
                    'follower_errors_m': [0.1, 0.1, 0.1],
                }],
            },
            bt_state={
                'last_error': (
                    "Formation 'green_wedge' registered but NOT activated "
                    '— robots out of position'
                ),
            },
        )

        self.assertIs(result['ok'], False)
        self.assertIn('registered but NOT activated', result['checks']['recent_errors'][0])
        self.assertEqual(
            result['repair_recommendation']['type'],
            'restage_then_activate',
        )

    def test_missing_state_source_returns_partial_instead_of_crashing(self):
        result = verify_plan_execution_state({}, _args())

        self.assertIs(result['ok'], False)
        self.assertEqual(result['confidence'], 'partial')
        self.assertIn('/formations/status', result['missing_state'])
        self.assertEqual(
            result['repair_recommendation']['type'],
            'activate_or_restage',
        )

    def test_no_formation_execution_failure_is_repairable(self):
        result = verify_plan_execution_state(
            {},
            {
                'original_user_request': 'send orange to cafeteria',
                'last_plan': {
                    'type': 'mapf',
                    'robot_ids': [12],
                    'goals': [[1.0, 1.0]],
                },
                'last_failure': {
                    'action_status': 'FAILED',
                    'last_error': 'MAPF goal rejected',
                },
            },
        )

        self.assertIs(result['ok'], False)
        self.assertEqual(result['repair_recommendation']['type'], 'replan')
        self.assertTrue(result['repair_recommendation']['repairable'])

    def test_mapf_around_group_spacing_violation_is_repairable(self):
        result = verify_plan_execution_state(
            {},
            {
                'original_user_request': 'send orange around yellow',
                'last_plan': MAPF_PLAN,
            },
            pose_snapshot={
                12: {'x': 1.0, 'y': 1.0, 'stale': False},
                13: {'x': 2.0, 'y': 1.0, 'stale': False},
                16: {'x': 1.2, 'y': 1.0, 'stale': False},
            },
        )

        self.assertIs(result['ok'], False)
        self.assertEqual(
            result['repair_recommendation']['type'],
            'replan_mapf',
        )
        self.assertTrue(result['checks']['mapf_goals_reached']['spacing']['too_close'])


if __name__ == '__main__':
    unittest.main()
