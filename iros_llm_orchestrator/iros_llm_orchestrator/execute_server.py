"""Operator-confirmed plan execution — /llm/execute_plan action server.

Used by the RViz panel's "Preview & Execute" workflow: panel previously
sent LlmChat with execute_after_planning=false and got a plan_json back.
The operator reviews the parsed plan, then presses Execute — the panel
sends that JSON here verbatim, and we replay it deterministically.

No LLM call. The plan that runs is *exactly* the plan the operator saw.
"""

import asyncio
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmExecutePlan

from iros_llm_orchestrator.common.leaf_sender import BTLeafSender
from iros_llm_orchestrator.common.plan_executor import PlanExecutor, parse_plan
from iros_llm_orchestrator.common.user_prompt import load_map_config
from iros_llm_orchestrator.user_chat_node import _postprocess_plan


class ExecuteServer(Node):
    def __init__(self):
        super().__init__('llm_execute_server')

        self.declare_parameter('step_timeout_sec', 120.0)
        self.declare_parameter('map_name',         'cave')

        self._map_name = self.get_parameter('map_name').value
        try:
            self._map_cfg = load_map_config(self._map_name)
        except Exception as exc:
            self.get_logger().warn(f'Map config load failed: {exc}')
            self._map_cfg = {}

        self._sender = BTLeafSender(
            self,
            step_timeout_sec=float(self.get_parameter('step_timeout_sec').value),
        )

        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        self._exec_lock = asyncio.Lock()

        self._action_server = ActionServer(
            self, LlmExecutePlan, '/llm/execute_plan',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info('LlmExecuteServer ready on /llm/execute_plan')

    def _execute(self, goal_handle):
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    async def _execute_async(self, goal_handle):
        async with self._exec_lock:
            return await self._execute_body(goal_handle)

    async def _execute_body(self, goal_handle):
        req = goal_handle.request
        result = LlmExecutePlan.Result()

        try:
            plan = parse_plan(req.plan_json)
        except ValueError as exc:
            return self._fail(goal_handle, result, f'parse error: {exc}')

        # Re-apply the same post-processing chat_server runs (clamp + spread
        # mapf goals). The panel may have rendered raw goals from the original
        # parsed plan — we keep behaviour identical.
        plan = _postprocess_plan(plan, self._map_cfg)

        self._publish_fb(goal_handle, stage='executing')
        executor = PlanExecutor(
            send_fn=self._sender.send,
            log_fn=lambda m: self.get_logger().info(f'executor: {m}'))
        ok = await executor.run(plan)
        if not ok:
            return self._fail(goal_handle, result, 'plan execution failed')

        self._publish_fb(goal_handle, stage='done')
        result.success = True
        result.info    = ''
        goal_handle.succeed()
        return result

    def _publish_fb(self, gh, *, stage='', detail=''):
        fb = LlmExecutePlan.Feedback()
        fb.stage  = stage
        fb.detail = detail
        gh.publish_feedback(fb)

    def _fail(self, gh, result, info):
        self._publish_fb(gh, stage='error', detail=info)
        result.success = False
        result.info    = info
        gh.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExecuteServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._loop.call_soon_threadsafe(node._loop.stop)
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
