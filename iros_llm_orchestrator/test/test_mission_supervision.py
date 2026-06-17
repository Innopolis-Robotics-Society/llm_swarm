"""Unit tests for the pure mission supervision loop."""

import asyncio
import json
import unittest

from iros_llm_orchestrator.common import user_prompt
from iros_llm_orchestrator.common.mission_supervision import (
    MissionConfig,
    MissionContinuation,
    supervise_mission,
)
from iros_llm_orchestrator.common.user_prompt import (
    build_mission_continuation_prompt,
)


IDLE_PLAN = {'type': 'idle', 'reason': 'done'}
MAPF_PLAN = {
    'type': 'mapf',
    'robot_ids': [1],
    'goals': [[1.0, 2.0]],
    'reason': 'move',
}
REPAIR_PLAN = {
    'type': 'mapf',
    'robot_ids': [1],
    'goals': [[2.0, 2.0]],
    'reason': 'repair',
}


def _config(**extra):
    values = {
        'max_duration_sec': 30.0,
        'max_steps': 4,
        'no_progress_limit': 2,
    }
    values.update(extra)
    return MissionConfig(**values)


def _verification(ok, summary='summary', repairable=True, check='failed'):
    return {
        'ok': ok,
        'summary': summary,
        'checks': {
            check: {'ok': ok, 'failed': [] if ok else ['x']},
        },
        'repair_recommendation': {
            'type': 'replan',
            'reason': summary,
            'repairable': repairable,
        },
    }


class FakeClock:
    def __init__(self, values=None):
        self.values = list(values or [0.0])
        self.last = self.values[-1]

    def now(self):
        if self.values:
            self.last = self.values.pop(0)
        return self.last


class MissionSupervisionTests(unittest.TestCase):
    def test_initial_plan_verifies_ok_one_step_only(self):
        executed = []

        async def execute(plan, step):
            executed.append((step, plan))
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(True, 'complete', repairable=False)

        async def continuation(_ctx):
            raise AssertionError('continuation should not be called')

        outcome = asyncio.run(supervise_mission(
            original_request='simple',
            initial_plan=MAPF_PLAN,
            initial_reply='ok',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertTrue(outcome.ok)
        self.assertEqual(outcome.steps_completed, 1)
        self.assertEqual(len(executed), 1)

    def test_repair_succeeds_on_second_step(self):
        verifications = [
            _verification(False, 'not done'),
            _verification(True, 'done', repairable=False),
        ]
        plans = []

        async def execute(plan, step):
            plans.append(plan)
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return verifications.pop(0)

        async def continuation(ctx):
            self.assertEqual(ctx.step, 1)
            return MissionContinuation('repairing', REPAIR_PLAN, '{}')

        outcome = asyncio.run(supervise_mission(
            original_request='repair mission',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertTrue(outcome.ok)
        self.assertEqual(outcome.final_plan, REPAIR_PLAN)
        self.assertEqual(len(plans), 2)

    def test_verification_false_repeatedly_stops_at_max_steps(self):
        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(False, f'not done {step}', check=f'failed_{step}')

        async def continuation(ctx):
            return MissionContinuation(
                'next',
                {'type': 'idle', 'reason': f'continue {ctx.step}'},
                '{}',
            )

        outcome = asyncio.run(supervise_mission(
            original_request='never done',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(max_steps=3, min_progress_required=False),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertFalse(outcome.ok)
        self.assertEqual(outcome.status, 'exhausted')
        self.assertEqual(outcome.steps_completed, 3)

    def test_timeout_stops_mission(self):
        clock = FakeClock([0.0, 100.0])

        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(False, 'late')

        async def continuation(ctx):
            return MissionContinuation('next', REPAIR_PLAN, '{}')

        outcome = asyncio.run(supervise_mission(
            original_request='timeout',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(max_duration_sec=1.0),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
            now_fn=clock.now,
        ))

        self.assertFalse(outcome.ok)
        self.assertEqual(outcome.status, 'timeout')

    def test_non_repairable_verification_stops_immediately(self):
        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(False, 'blocked', repairable=False)

        async def continuation(_ctx):
            raise AssertionError('continuation should not be called')

        outcome = asyncio.run(supervise_mission(
            original_request='blocked',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertEqual(outcome.status, 'non_repairable')

    def test_invalid_continuation_plan_stops_safely(self):
        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(False, 'repairable')

        async def continuation(_ctx):
            return MissionContinuation('bad', {}, '{}')

        outcome = asyncio.run(supervise_mission(
            original_request='bad continuation',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertEqual(outcome.status, 'invalid_continuation')

    def test_exact_same_failed_plan_is_not_repeated(self):
        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(False, 'same')

        async def continuation(_ctx):
            return MissionContinuation('same', dict(MAPF_PLAN), '{}')

        outcome = asyncio.run(supervise_mission(
            original_request='repeat',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertEqual(outcome.status, 'repeated_plan')

    def test_continuation_prompt_contains_mission_context(self):
        old_system = user_prompt._user_system
        old_examples = user_prompt._get_examples
        user_prompt._user_system = lambda _m: 'system'
        user_prompt._get_examples = lambda _m: []
        try:
            messages = build_mission_continuation_prompt(
                'green and yellow go to cafeteria',
                MAPF_PLAN,
                {'ok': True},
                _verification(False, 'still missing'),
                step=1,
                max_steps=4,
                remaining_time_sec=42.0,
                fresh_runtime_context={'source': 'test'},
                previous_verification={'summary': 'previous'},
                history=[],
                map_name='test',
            )
        finally:
            user_prompt._user_system = old_system
            user_prompt._get_examples = old_examples

        blob = json.dumps(messages, ensure_ascii=False)
        self.assertIn('still executing the same operator mission', blob)
        self.assertIn('green and yellow go to cafeteria', blob)
        self.assertIn('still missing', blob)
        self.assertIn('42.0s', blob)
        self.assertIn('find_group_placement_in_room', blob)

    def test_simple_command_compatible_when_verification_ok(self):
        async def execute(plan, step):
            return True, None

        async def verify(plan, execution_ok, failure, previous, step):
            return _verification(True, 'simple done', repairable=False)

        async def continuation(_ctx):
            raise AssertionError('simple command should finish')

        outcome = asyncio.run(supervise_mission(
            original_request='send yellow to cafeteria',
            initial_plan=MAPF_PLAN,
            initial_reply='started',
            config=_config(),
            execute_plan=execute,
            verify_plan=verify,
            generate_continuation=continuation,
        ))

        self.assertTrue(outcome.ok)
        self.assertEqual(outcome.final_reply, 'started')


if __name__ == '__main__':
    unittest.main()
