import json
import unittest

from iros_llm_orchestrator.common.context_budget import (
    completion_budget_for_prompt,
)
from iros_llm_orchestrator.common.user_prompt import (
    build_compact_mission_context,
    build_mission_continuation_prompt,
)


class MissionContextBudgetTests(unittest.TestCase):
    def test_compact_prompt_omits_full_history(self):
        messages = build_mission_continuation_prompt(
            'send robots to main hall',
            {'type': 'mapf', 'robot_ids': [0], 'goals': [[1, 2]]},
            {'ok': False},
            {'ok': False, 'summary': 'still incomplete'},
            step=2,
            max_steps=6,
            remaining_time_sec=120.0,
            fresh_runtime_context={'source': 'test'},
            history=[{'role': 'assistant', 'content': 'SHOULD_NOT_APPEAR' * 100}],
            map_name='test',
        )
        blob = json.dumps(messages, ensure_ascii=False)
        self.assertNotIn('SHOULD_NOT_APPEAR', blob)
        self.assertIn('omitted_turns', blob)
        self.assertIn('Compact mission context JSON', blob)

    def test_large_previous_plan_is_truncated(self):
        huge_plan = {
            'type': 'sequence',
            'steps': [
                {'type': 'mapf', 'robot_ids': [i % 20], 'goals': [[i, i + 1]]}
                for i in range(300)
            ],
        }
        compact = build_compact_mission_context(
            'big mission',
            huge_plan,
            {'ok': False},
            {'ok': False, 'summary': 'too large'},
            step=1,
            max_steps=6,
            remaining_time_sec=60.0,
            fresh_runtime_context={'source': 'test'},
        )
        self.assertTrue(compact['last_plan']['truncated'])
        self.assertLess(len(compact['last_plan']['json_prefix']), 2500)

    def test_completion_budget_reduces_when_input_large(self):
        prompt = 'x' * (13200 * 4)
        decision = completion_budget_for_prompt(
            prompt,
            context_window_tokens=16384,
            default_completion_tokens=3072,
            min_completion_tokens=512,
            margin_tokens=512,
        )
        self.assertTrue(decision['ok'])
        self.assertEqual(decision['action'], 'reduce')
        self.assertLess(decision['max_completion_tokens'], 3072)

    def test_context_too_large_aborts(self):
        prompt = 'x' * (16000 * 4)
        decision = completion_budget_for_prompt(
            prompt,
            context_window_tokens=16384,
            default_completion_tokens=2048,
            min_completion_tokens=512,
            margin_tokens=512,
        )
        self.assertFalse(decision['ok'])
        self.assertEqual(decision['action'], 'abort')

    def test_previous_http_400_case_reduces_completion(self):
        prompt = 'x' * (13550 * 4)
        decision = completion_budget_for_prompt(
            prompt,
            context_window_tokens=16384,
            default_completion_tokens=3072,
            min_completion_tokens=512,
            margin_tokens=512,
        )
        self.assertTrue(decision['ok'])
        self.assertEqual(decision['action'], 'reduce')
        self.assertLessEqual(
            decision['max_completion_tokens'],
            16384 - 13550 - 512,
        )


if __name__ == '__main__':
    unittest.main()
