import json
import unittest

from iros_llm_orchestrator.common.active_formation_guard import (
    active_formations_from_status,
    guard_plan_for_active_formations,
)


ACTIVE = [{
    'formation_id': 'yellow_wedge',
    'leader_ns': 'robot_16',
    'followers': ['robot_17', 'robot_18', 'robot_19'],
    'status': 'STABLE',
}]


class ActiveFormationGuardTests(unittest.TestCase):
    def test_non_formation_mapf_passes_unchanged(self):
        plan = {'type': 'mapf', 'robot_ids': [0, 1], 'goals': [[1, 1]], 'spread': True}
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNone(failure)
        self.assertEqual(guarded['robot_ids'], [0, 1])

    def test_leader_only_passes(self):
        plan = {'type': 'mapf', 'robot_ids': [16], 'goals': [[2, 3]]}
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNone(failure)
        self.assertEqual(guarded, plan)

    def test_followers_only_rejected(self):
        plan = {'type': 'mapf', 'robot_ids': [17, 18], 'goals': [[2, 3]], 'spread': True}
        _guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNotNone(failure)
        self.assertEqual(failure['reason'], 'mapf_targets_active_formation_followers')
        self.assertEqual(failure['formation_id'], 'yellow_wedge')
        self.assertIn('robot_17', failure['followers'])
        self.assertTrue(failure['repairable'])

    def test_full_active_formation_rewrites_to_leader_only(self):
        plan = {
            'type': 'mapf',
            'robot_ids': [16, 17, 18, 19],
            'goals': [[2.7, 10.1]],
            'spread': True,
        }
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNone(failure)
        self.assertEqual(guarded['robot_ids'], [16])
        self.assertEqual(guarded['goals'], [[2.7, 10.1]])
        self.assertNotIn('spread', guarded)

    def test_all_robots_with_active_formation_removes_followers(self):
        plan = {
            'type': 'mapf',
            'robot_ids': list(range(20)),
            'goals': [[2.7, 10.1]],
            'spread': True,
        }
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNone(failure)
        self.assertIn(16, guarded['robot_ids'])
        self.assertNotIn(17, guarded['robot_ids'])
        self.assertNotIn(18, guarded['robot_ids'])
        self.assertNotIn(19, guarded['robot_ids'])
        self.assertIn(0, guarded['robot_ids'])

    def test_sequence_disband_allows_individual_mapf(self):
        plan = {
            'type': 'sequence',
            'steps': [
                {'type': 'disband', 'formation_id': 'yellow_wedge'},
                {
                    'type': 'mapf',
                    'robot_ids': [16, 17, 18, 19],
                    'goals': [[2.7, 10.1]],
                    'spread': True,
                },
            ],
        }
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        self.assertIsNone(failure)
        self.assertEqual(guarded['steps'][1]['robot_ids'], [16, 17, 18, 19])

    def test_normalization_accepts_raw_like_message(self):
        class Formation:
            formation_id = 'orange_circle'
            leader_ns = 'robot_12'
            follower_ns = ['robot_13', 'robot_14', 'robot_15']
            state = 2

        class Status:
            formations = [Formation()]

        records = active_formations_from_status(Status())
        self.assertEqual(records[0]['formation_id'], 'orange_circle')
        self.assertEqual(records[0]['leader_id'], 12)

    def test_guard_output_is_json_serializable(self):
        plan = {'type': 'mapf', 'robot_ids': [17], 'goals': [[0, 0]]}
        guarded, failure = guard_plan_for_active_formations(plan, ACTIVE)
        json.dumps({'guarded': guarded, 'failure': failure}, ensure_ascii=False)


if __name__ == '__main__':
    unittest.main()
