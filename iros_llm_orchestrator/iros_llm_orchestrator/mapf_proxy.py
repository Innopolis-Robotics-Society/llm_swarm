"""MAPF action proxy with LLM WARN/ERROR decision bridge.

The behavior tree can remap its hardcoded /swarm/set_goals client to this
proxy. The proxy forwards goals to the real MAPF action server, relays
feedback, and asks /llm/decision for advisory wait/replan/abort decisions when
MAPF feedback reports trouble. It never directly controls robots.
"""

from __future__ import annotations

import asyncio
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmDecision, SetGoals


_VALID_DECISIONS = {'wait', 'replan', 'abort'}


@dataclass
class _FeedbackEvent:
    level: str
    event: str


@dataclass
class _ProxyGoalState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    log_buffer: deque[str] = field(default_factory=lambda: deque(maxlen=20))
    pending_event: _FeedbackEvent | None = None
    decision_inflight: bool = False
    decided_events: set[str] = field(default_factory=set)
    done: bool = False

    def append_log(self, line: str) -> None:
        with self.lock:
            self.log_buffer.append(line)

    def snapshot_logs(self) -> list[str]:
        with self.lock:
            return list(self.log_buffer)

    def queue_event(self, event: _FeedbackEvent) -> bool:
        key = f'{event.level}:{event.event}'
        with self.lock:
            if key in self.decided_events:
                return False
            if self.pending_event is not None or self.decision_inflight:
                return False
            self.pending_event = event
            self.decision_inflight = True
            self.decided_events.add(key)
            return True

    def take_event(self) -> _FeedbackEvent | None:
        with self.lock:
            event = self.pending_event
            self.pending_event = None
            return event

    def finish_decision(self) -> None:
        with self.lock:
            self.decision_inflight = False

    def mark_done(self) -> None:
        with self.lock:
            self.done = True

    def is_done(self) -> bool:
        with self.lock:
            return self.done


def _feedback_summary(feedback: SetGoals.Feedback) -> str:
    line = (
        f'[t={int(feedback.elapsed_ms)}ms status={feedback.status} '
        f'arrived={int(feedback.robots_arrived)} '
        f'active={int(feedback.robots_active)} '
        f'stall={int(feedback.robot_stall)} '
        f'replans={int(feedback.replans_done)}]'
    )
    if feedback.info:
        line += f' INFO: {feedback.info}'
    if feedback.warning:
        line += f' WARN: {feedback.warning}'
    return line


def _decision_event_from_feedback(
    feedback: SetGoals.Feedback,
) -> _FeedbackEvent | None:
    status = (feedback.status or '').strip()
    status_l = status.lower()
    warning = (feedback.warning or '').strip()
    info = (feedback.info or '').strip()

    if status_l in {'failed', 'error'}:
        event = warning or info or f'MAPF feedback status={status}'
        return _FeedbackEvent('ERROR', event)

    if warning:
        return _FeedbackEvent('WARN', warning)

    if int(feedback.robot_stall or 0) > 0:
        event = (
            f'robot_stall={int(feedback.robot_stall)} '
            f'status={status or "unknown"} replans={int(feedback.replans_done)}'
        )
        if info:
            event += f' info={info}'
        return _FeedbackEvent('WARN', event)

    if info:
        info_l = info.lower()
        warn_terms = (
            'abort',
            'blocked',
            'collision',
            'error',
            'fail',
            'no static path',
            'out of lives',
            'stall',
            'unplanable',
            'unplannable',
        )
        if any(term in info_l for term in warn_terms):
            level = 'ERROR' if 'error' in info_l or 'fail' in info_l else 'WARN'
            return _FeedbackEvent(level, info)

    return None


class LlmMapfProxy(Node):
    """SetGoals proxy that asks /llm/decision on MAPF WARN/ERROR feedback."""

    def __init__(self):
        super().__init__('llm_mapf_proxy')

        self.declare_parameter('proxy_action_name', '/llm/swarm/set_goals_proxy')
        self.declare_parameter('target_action_name', '/swarm/set_goals')
        self.declare_parameter('decision_action_name', '/llm/decision')
        self.declare_parameter('target_server_timeout_sec', 5.0)
        self.declare_parameter('decision_server_timeout_sec', 1.0)
        self.declare_parameter('decision_timeout_sec', 10.0)
        self.declare_parameter('log_buffer_size', 20)

        self._proxy_action_name = str(
            self.get_parameter('proxy_action_name').value)
        self._target_action_name = str(
            self.get_parameter('target_action_name').value)
        self._decision_action_name = str(
            self.get_parameter('decision_action_name').value)
        self._target_server_timeout = float(
            self.get_parameter('target_server_timeout_sec').value)
        self._decision_server_timeout = float(
            self.get_parameter('decision_server_timeout_sec').value)
        self._decision_timeout = float(
            self.get_parameter('decision_timeout_sec').value)
        self._log_buffer_size = max(
            1, int(self.get_parameter('log_buffer_size').value))

        self._mapf_client = ActionClient(
            self, SetGoals, self._target_action_name)
        self._decision_client = ActionClient(
            self, LlmDecision, self._decision_action_name)
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()
        self._action_server = ActionServer(
            self,
            SetGoals,
            self._proxy_action_name,
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            f'LlmMapfProxy ready: {self._proxy_action_name} -> '
            f'{self._target_action_name}')

    def _execute(self, goal_handle):
        future = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return future.result()

    async def _execute_async(self, goal_handle):
        self.get_logger().info('LlmMapfProxy: goal received')
        state = _ProxyGoalState(
            log_buffer=deque(maxlen=self._log_buffer_size))

        if not await self._wait_for_server(
            self._mapf_client, self._target_server_timeout):
            result = self._controlled_failure(
                'real /swarm/set_goals action server unavailable',
                SetGoals.Result.UNKNOWN,
            )
            self.get_logger().error(
                'LlmMapfProxy: real action server unavailable')
            self.get_logger().info('LlmMapfProxy: goal finished')
            state.mark_done()
            goal_handle.succeed()
            return result

        send_future = self._mapf_client.send_goal_async(
            goal_handle.request,
            feedback_callback=lambda fb_msg: self._on_real_feedback(
                goal_handle, state, fb_msg.feedback),
        )
        real_goal_handle = await send_future
        if real_goal_handle is None or not real_goal_handle.accepted:
            result = self._controlled_failure(
                'real /swarm/set_goals rejected goal',
                SetGoals.Result.UNKNOWN,
            )
            self.get_logger().error('LlmMapfProxy: real action rejected goal')
            self.get_logger().info('LlmMapfProxy: goal finished')
            state.mark_done()
            goal_handle.succeed()
            return result

        self.get_logger().info(
            'LlmMapfProxy: forwarded goal to /swarm/set_goals')
        result_future = real_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                await self._cancel_real_goal(real_goal_handle)
                result = self._controlled_failure(
                    'proxy goal canceled by client',
                    SetGoals.Result.CANCELLED,
                )
                self.get_logger().info('LlmMapfProxy: goal finished')
                state.mark_done()
                goal_handle.canceled()
                return result

            event = state.take_event()
            if event is not None:
                decision = await self._ask_decision(event, state)
                state.finish_decision()
                if result_future.done():
                    continue
                if decision == 'wait':
                    self.get_logger().info('LlmMapfProxy: applying wait')
                elif decision == 'replan':
                    self.get_logger().info('LlmMapfProxy: applying replan')
                    await self._cancel_real_goal(real_goal_handle)
                    result = self._controlled_failure(
                        f'LLM requested replan: {event.event}',
                        SetGoals.Result.CANCELLED,
                    )
                    self.get_logger().info('LlmMapfProxy: goal finished')
                    state.mark_done()
                    goal_handle.succeed()
                    return result
                elif decision == 'abort':
                    self.get_logger().info('LlmMapfProxy: applying abort')
                    await self._cancel_real_goal(real_goal_handle)
                    result = self._controlled_failure(
                        f'LLM requested abort: {event.event}',
                        SetGoals.Result.CANCELLED,
                    )
                    self.get_logger().info('LlmMapfProxy: goal finished')
                    state.mark_done()
                    goal_handle.succeed()
                    return result

            await asyncio.sleep(0.05)

        wrapped = await result_future
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            msg = f'real /swarm/set_goals finished with status {wrapped.status}'
            result = self._controlled_failure(msg, SetGoals.Result.CANCELLED)
        self.get_logger().info('LlmMapfProxy: goal finished')
        state.mark_done()
        goal_handle.succeed()
        return result

    def _on_real_feedback(
        self,
        goal_handle,
        state: _ProxyGoalState,
        feedback: SetGoals.Feedback,
    ) -> None:
        if state.is_done():
            return
        self.get_logger().info(
            'LlmMapfProxy: feedback received '
            f'status={feedback.status} warning={feedback.warning}')
        state.append_log(_feedback_summary(feedback))
        goal_handle.publish_feedback(feedback)

        event = _decision_event_from_feedback(feedback)
        if event is None:
            return
        if state.queue_event(event):
            self.get_logger().warn(
                'LlmMapfProxy: warning detected, sending /llm/decision')

    async def _ask_decision(
        self,
        event: _FeedbackEvent,
        state: _ProxyGoalState,
    ) -> str:
        if not await self._wait_for_server(
            self._decision_client, self._decision_server_timeout):
            self.get_logger().warn(
                'LlmMapfProxy: /llm/decision unavailable; defaulting to wait')
            self.get_logger().info(
                'LlmMapfProxy: decision result decision=wait')
            return 'wait'

        goal = LlmDecision.Goal()
        goal.level = event.level
        goal.event = event.event
        goal.log_buffer = state.snapshot_logs()

        try:
            send_future = self._decision_client.send_goal_async(goal)
            decision_goal_handle = await asyncio.wait_for(
                send_future, timeout=self._decision_timeout)
            if decision_goal_handle is None or not decision_goal_handle.accepted:
                self.get_logger().warn(
                    'LlmMapfProxy: /llm/decision rejected goal; '
                    'defaulting to wait')
                decision = 'wait'
            else:
                result_future = decision_goal_handle.get_result_async()
                wrapped = await asyncio.wait_for(
                    result_future, timeout=self._decision_timeout)
                decision = (wrapped.result.decision or 'wait').strip().lower()
                if decision not in _VALID_DECISIONS:
                    decision = 'wait'
        except Exception as exc:
            self.get_logger().error(
                f'LlmMapfProxy: /llm/decision error; defaulting to wait: '
                f'{type(exc).__name__}: {exc}')
            decision = 'wait'

        self.get_logger().info(
            f'LlmMapfProxy: decision result decision={decision}')
        return decision

    async def _cancel_real_goal(self, real_goal_handle: Any) -> None:
        try:
            cancel_future = real_goal_handle.cancel_goal_async()
            await asyncio.wait_for(cancel_future, timeout=2.0)
        except Exception as exc:
            self.get_logger().warn(
                f'LlmMapfProxy: failed to cancel real MAPF goal: '
                f'{type(exc).__name__}: {exc}')

    async def _wait_for_server(
        self,
        client: ActionClient,
        timeout_sec: float,
    ) -> bool:
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(
            None, lambda: client.wait_for_server(timeout_sec=timeout_sec))

    @staticmethod
    def _controlled_failure(message: str, error_code: int) -> SetGoals.Result:
        result = SetGoals.Result()
        result.success = False
        result.message = message
        result.error_code = int(error_code)
        return result

    def shutdown_async_resources(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._loop_thread.join(timeout=2.0)
        self._loop.close()


def main(args=None):
    rclpy.init(args=args)
    node = LlmMapfProxy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown_async_resources()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
