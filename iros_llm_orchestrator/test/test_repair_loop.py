"""Unit tests for post-execution repair helpers."""

import asyncio
import json
import unittest

from iros_llm_orchestrator.common import user_prompt
from iros_llm_orchestrator.common.execution_repair import (
    should_attempt_repair,
    verification_failure_info,
)
from iros_llm_orchestrator.common.plan_executor import PlanExecutor
from iros_llm_orchestrator.common.user_prompt import build_execution_repair_prompt
from iros_llm_orchestrator.context.group_placement import follower_offsets_for_formation


REPAIRABLE_VERIFICATION = {
    'ok': False,
    'summary': 'green_wedge follower out of tolerance',
    'repair_recommendation': {
        'type': 'restage_then_activate',
        'reason': 'followers are far from required offsets',
        'repairable': True,
        'should_recompute_placement': True,
    },
}


class RepairLoopTests(unittest.TestCase):
    def test_repair_loop_stops_after_max_attempts(self):
        self.assertTrue(should_attempt_repair(
            REPAIRABLE_VERIFICATION,
            attempt=0,
            max_attempts=2,
            enabled=True,
        ))
        self.assertFalse(should_attempt_repair(
            REPAIRABLE_VERIFICATION,
            attempt=2,
            max_attempts=2,
            enabled=True,
        ))

    def test_repair_loop_does_not_retry_non_repairable(self):
        verification = {
            'ok': False,
            'summary': 'missing /formations/status',
            'repair_recommendation': {'repairable': False},
        }

        self.assertFalse(should_attempt_repair(
            verification,
            attempt=0,
            max_attempts=2,
            enabled=True,
        ))

    def test_verification_failure_info_is_prompt_safe(self):
        info = verification_failure_info(REPAIRABLE_VERIFICATION)

        self.assertEqual(info['leaf_type'], 'verification')
        self.assertEqual(info['failed_at_phase'], 'post_execution')
        self.assertTrue(info['repairable'])
        self.assertIn('green_wedge', info['last_error'])

    def test_repair_prompt_contains_verification_result(self):
        old_system = user_prompt._user_system
        old_examples = user_prompt._get_examples
        user_prompt._user_system = lambda _m: 'system'
        user_prompt._get_examples = lambda _m: []
        try:
            messages = build_execution_repair_prompt(
                'green form wedge',
                {'type': 'formation', 'formation_id': 'green_wedge'},
                REPAIRABLE_VERIFICATION,
                attempt=1,
                max_attempts=2,
                fresh_runtime_context={'source': 'test'},
                history=[],
                map_name='test',
            )
        finally:
            user_prompt._user_system = old_system
            user_prompt._get_examples = old_examples

        blob = json.dumps(messages, ensure_ascii=False)
        self.assertIn('Post-execution verification failed', blob)
        self.assertIn('green_wedge follower out of tolerance', blob)
        self.assertIn('find_group_placement_in_room', blob)
        self.assertIn('1/2', blob)

    def test_four_robot_wedge_prompt_and_tool_conventions_match(self):
        with open(
            'iros_llm_orchestrator/prompts/user_chat_system.txt',
            'r',
            encoding='utf-8',
        ) as handle:
            prompt = handle.read()

        offsets = follower_offsets_for_formation('wedge', 3)

        self.assertIn('Wedge 4 bots', prompt)
        self.assertEqual(offsets, [(-1.0, 0.6), (-1.0, -0.6), (-2.0, 0.0)])
        self.assertIn('offsets_x=[-1.0,-1.0,-2.0]', prompt)
        self.assertIn('offsets_y=[0.6,-0.6,0.0]', prompt)

    def test_prestage_hook_runs_for_formation_before_send(self):
        sent = []

        async def send_fn(node):
            sent.append(node)
            return True

        def prestage_hook(node):
            self.assertEqual(node['type'], 'formation')
            return {
                'type': 'mapf',
                'robot_ids': [9],
                'goals': [[1.0, 2.0]],
                'reason': 'stage',
            }

        executor = PlanExecutor(
            send_fn=send_fn,
            formation_prestage_hook=prestage_hook,
        )
        ok = asyncio.run(executor.run({
            'type': 'formation',
            'formation_id': 'green_wedge',
            'leader_ns': 'robot_8',
            'follower_ns': ['robot_9'],
            'offsets_x': [-1.0],
            'offsets_y': [0.6],
        }))

        self.assertTrue(ok)
        self.assertEqual([node['type'] for node in sent], ['mapf', 'formation'])


if __name__ == '__main__':
    unittest.main()
