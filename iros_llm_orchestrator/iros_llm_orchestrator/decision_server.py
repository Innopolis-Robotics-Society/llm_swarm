"""Channel 1 — reactive LLM decision action server (/llm/decision).

BT nodes call this when they encounter a WARN or periodic INFO during
MapfPlan / SetFormation / DisableFormation execution.
Returns: wait | abort | replan
"""

import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmDecision

from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.parsers import parse_llm_decision
from iros_llm_orchestrator.common.decision_prompt import build_decision_prompt
from iros_llm_orchestrator.common.scenarios import DECISION_SCENARIOS
from iros_llm_orchestrator.common.logger import DecisionLogger


class LlmDecisionServer(Node):
    def __init__(self):
        super().__init__('llm_decision_server')

        self.declare_parameter('llm_mode',        'mock')
        self.declare_parameter('llm_endpoint',    '')
        self.declare_parameter('llm_model',       '')
        self.declare_parameter('llm_max_tokens',  256)
        self.declare_parameter('llm_temperature', 0.2)
        self.declare_parameter('timeout_sec',     10.0)
        self.declare_parameter('default_on_error','wait')
        self.declare_parameter('log_tail',        20)
        self.declare_parameter('max_concurrent',  1)
        self.declare_parameter('dataset_path',    '~/.ros/llm_decisions')

        mode     = self.get_parameter('llm_mode').value
        endpoint = self.get_parameter('llm_endpoint').value or None
        model    = self.get_parameter('llm_model').value

        self._llm = get_llm_client(
            mode=mode,
            endpoint=endpoint,
            model=model,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        self._timeout       = float(self.get_parameter('timeout_sec').value)
        self._default       = self.get_parameter('default_on_error').value
        self._tail          = int(self.get_parameter('log_tail').value)
        self._semaphore     = asyncio.Semaphore(int(self.get_parameter('max_concurrent').value))
        self._logger_ds     = DecisionLogger(self.get_parameter('dataset_path').value)

        self._loop = asyncio.new_event_loop()
        import threading
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        self._action_server = ActionServer(
            self,
            LlmDecision,
            '/llm/decision',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(f'LlmDecisionServer ready (mode={mode})')

    def _execute(self, goal_handle):
        import concurrent.futures
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    async def _execute_async(self, goal_handle):
        req = goal_handle.request
        prompt = build_decision_prompt(
            DECISION_SCENARIOS, req.level, req.event, list(req.log_buffer), self._tail)

        decision = self._default
        reason   = 'timeout or error'

        async with self._semaphore:
            try:
                raw = await asyncio.wait_for(
                    self._llm.generate(prompt, prompt_kind='decision'),
                    timeout=self._timeout)
                decision = parse_llm_decision(raw)
                reason   = raw[:200]
            except asyncio.TimeoutError:
                self.get_logger().warn('LLM decision timeout')
            except Exception as exc:
                self.get_logger().error(f'LLM decision error: {exc}')

        self._logger_ds.log(
            level=req.level, event=req.event,
            log_buffer=list(req.log_buffer),
            decision=decision, reason=reason)

        result = LlmDecision.Result()
        result.decision = decision
        result.reason   = reason
        goal_handle.succeed()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
