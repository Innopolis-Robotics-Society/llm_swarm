"""
Action server for /llm/decision.

Receives LlmDecision goals from BT-nodes (MapfPlan / SetFormation / DisableFormation),
builds a few-shot prompt, queries the configured LLM backend (mock / http / local),
parses the verdict out of the raw completion and returns one of:
  "wait" | "abort" | "replan"

Every call is appended to a JSONL dataset for future SFT.

Safe defaults:
  * any failure (timeout, parse error, backend exception) falls back to "wait"
    so the BT keeps running rather than getting aborted by the supervisor.
  * an asyncio.Semaphore limits concurrent inference calls — MAPF feedback
    can fire several times per second, we must not pile up GPU load.
"""

import asyncio
import os

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmDecision

from iros_llm_orchestrator.decision_logger import DecisionLogger
from iros_llm_orchestrator.decision_parser import parse_llm_decision
from iros_llm_orchestrator.llm_client import LLMClient
from iros_llm_orchestrator.prompt_builder import build_few_shot_prompt
from iros_llm_orchestrator.scenarios import SCENARIOS


class LlmDecisionServer(Node):
    def __init__(self):
        super().__init__('llm_decision_server')

        self.declare_parameter('llm_mode', 'mock')
        self.declare_parameter('llm_endpoint', 'http://localhost:8000/v1/completions')
        self.declare_parameter('llm_model', 'nvidia/Nemotron-3-Nano-30B-A3B')
        self.declare_parameter('llm_max_tokens', 256)
        self.declare_parameter('llm_temperature', 0.2)
        self.declare_parameter('dataset_path', '~/.ros/llm_decisions')
        self.declare_parameter('max_concurrent', 1)
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter('default_on_error', 'wait')
        self.declare_parameter('log_tail', 20)

        mode = self.get_parameter('llm_mode').value
        endpoint = self.get_parameter('llm_endpoint').value
        dataset_path = self.get_parameter('dataset_path').value
        max_concurrent = int(self.get_parameter('max_concurrent').value)

        self.llm = LLMClient(
            mode=mode,
            endpoint=endpoint,
            model=self.get_parameter('llm_model').value,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        self.logger_ = DecisionLogger(path=os.path.expanduser(dataset_path))

        self.cb_group = ReentrantCallbackGroup()
        self.sem = asyncio.Semaphore(max_concurrent)

        self._action_server = ActionServer(
            self,
            LlmDecision,
            '/llm/decision',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f"LLM Decision Server up on /llm/decision "
            f"(mode={mode}, max_concurrent={max_concurrent}, timeout={self.get_parameter('timeout_sec').value}s)"
        )

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f'recv: level={request.level} event="{request.event}" log_lines={len(request.log_buffer)}'
        )

        fb = LlmDecision.Feedback()
        fb.stage = 'received'
        goal_handle.publish_feedback(fb)

        default_decision = self.get_parameter('default_on_error').value
        decision = default_decision
        prompt = ''
        raw_output = ''
        error_note = ''

        try:
            prompt = build_few_shot_prompt(
                scenarios=SCENARIOS,
                level=request.level,
                event=request.event,
                log_buffer=list(request.log_buffer),
                tail=int(self.get_parameter('log_tail').value),
            )

            async with self.sem:
                fb.stage = 'thinking'
                goal_handle.publish_feedback(fb)

                raw_output = await asyncio.wait_for(
                    self.llm.generate(prompt),
                    timeout=float(self.get_parameter('timeout_sec').value),
                )

            decision = parse_llm_decision(raw_output)
            self.get_logger().info(f'decision={decision} raw={raw_output!r}')

        except asyncio.TimeoutError:
            error_note = 'timeout'
            self.get_logger().error('LLM generate() timed out, falling back to default')
            decision = default_decision
        except Exception as exc:
            error_note = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(f'LLM pipeline failed ({error_note}), falling back to default')
            decision = default_decision

        try:
            self.logger_.log({
                'goal': {
                    'level': request.level,
                    'event': request.event,
                    'log_buffer': list(request.log_buffer),
                },
                'prompt': prompt,
                'raw_output': raw_output,
                'decision': decision,
                'error': error_note,
            })
        except Exception as exc:
            self.get_logger().error(f'decision logger failed: {exc}')

        fb.stage = 'done'
        goal_handle.publish_feedback(fb)

        goal_handle.succeed()

        result = LlmDecision.Result()
        result.decision = decision
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LlmDecisionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
