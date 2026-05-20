"""Channel 1 — reactive LLM decision action server (/llm/decision).

BT nodes call this when they encounter a WARN or periodic INFO during
MapfPlan / SetFormation / DisableFormation execution.
Returns: wait | abort | replan
"""

import asyncio
import json
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmDecision
from iros_llm_swarm_interfaces.msg import LlmEvent

from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.parsers import (
    parse_llm_decision,
    parse_llm_decision_final,
)
from iros_llm_orchestrator.common.decision_prompt import (
    DECISION_MCP_ALLOWED_TOOLS,
    build_decision_prompt,
)
from iros_llm_orchestrator.common.scenarios import DECISION_SCENARIOS
from iros_llm_orchestrator.common.logger import DecisionLogger
from iros_llm_orchestrator.context import ChatContextConfig, make_context_provider
from iros_llm_orchestrator.context.agentic_mcp import (
    AgenticMcpConfig,
    AgenticMcpError,
    McpToolBroker,
    parse_agentic_response,
)
from iros_llm_orchestrator.context.provider import safe_str


_DECISION_SUBSCRIBE_TYPES = {
    '/bt/state': 'iros_llm_swarm_interfaces/msg/BTState',
    '/formations/status': 'iros_llm_swarm_interfaces/msg/FormationsStatus',
}


class LlmDecisionServer(Node):
    def __init__(self):
        super().__init__('llm_decision_server')

        self.declare_parameter('llm_mode',        'mock')
        self.declare_parameter('llm_endpoint',    '')
        self.declare_parameter('llm_model',       '')
        self.declare_parameter('llm_max_tokens',  256)
        self.declare_parameter('llm_temperature', 0.2)
        self.declare_parameter('llm_api_key',     '')
        self.declare_parameter('llm_api_key_env', 'LLM_API_KEY')
        self.declare_parameter('llm_force_chat',  True)
        self.declare_parameter('llm_enable_stop', False)
        self.declare_parameter('timeout_sec',     10.0)
        self.declare_parameter('default_on_error','wait')
        self.declare_parameter('log_tail',        20)
        self.declare_parameter('max_concurrent',  1)
        self.declare_parameter('dataset_path',    '~/.ros/llm_decisions')
        self.declare_parameter('map_name',        'cave')
        self.declare_parameter('context_provider', 'none')
        self.declare_parameter('context_timeout_sec', 2.0)
        self.declare_parameter('context_max_chars', 6000)
        self.declare_parameter('context_include_bt_state', True)
        self.declare_parameter('context_include_formations', True)
        self.declare_parameter('context_include_map_summary', False)
        self.declare_parameter('context_include_recent_events', False)
        self.declare_parameter('context_include_robot_positions', False)
        self.declare_parameter('context_pose_stale_ms', 2000)
        self.declare_parameter('mcp_enabled', False)
        self.declare_parameter('mcp_transport', 'stdio')
        self.declare_parameter('mcp_command', 'uvx')
        self.declare_parameter('mcp_args', ['ros-mcp', '--transport=stdio'])
        self.declare_parameter(
            'mcp_tool_allowlist',
            list(DECISION_MCP_ALLOWED_TOOLS),
        )
        self.declare_parameter('mcp_decision_agentic_enabled', True)
        self.declare_parameter(
            'mcp_decision_agentic_levels',
            ['WARN', 'ERROR'],
        )
        self.declare_parameter('mcp_decision_agentic_max_rounds', 3)
        self.declare_parameter('mcp_decision_agentic_max_tools_per_round', 3)
        self.declare_parameter('mcp_decision_agentic_tool_timeout_sec', 2.0)
        self.declare_parameter('mcp_decision_agentic_max_result_chars', 6000)

        mode     = self.get_parameter('llm_mode').value
        endpoint = self.get_parameter('llm_endpoint').value or None
        model    = self.get_parameter('llm_model').value

        self._llm = get_llm_client(
            mode=mode,
            endpoint=endpoint,
            model=model,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
            api_key=self.get_parameter('llm_api_key').value,
            api_key_env=self.get_parameter('llm_api_key_env').value,
            timeout=float(self.get_parameter('timeout_sec').value),
            force_chat=bool(self.get_parameter('llm_force_chat').value),
            enable_stop=bool(self.get_parameter('llm_enable_stop').value),
        )
        self._timeout       = float(self.get_parameter('timeout_sec').value)
        self._default       = self.get_parameter('default_on_error').value
        self._tail          = int(self.get_parameter('log_tail').value)
        self._semaphore     = asyncio.Semaphore(int(self.get_parameter('max_concurrent').value))
        self._logger_ds     = DecisionLogger(self.get_parameter('dataset_path').value)
        self._event_pub     = self.create_publisher(LlmEvent, '/llm/events', 10)
        self._context_config = self._make_context_config()
        self._context_provider = make_context_provider(self, self._context_config)
        self._mcp_decision_agentic_config = (
            self._make_decision_agentic_mcp_config())
        self._mcp_decision_agentic_levels = {
            str(item).strip().upper()
            for item in self._param_string_list('mcp_decision_agentic_levels')
            if str(item).strip()
        }
        self._mcp_tool_broker = self._make_mcp_tool_broker()

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever,
            daemon=True,
        )
        self._loop_thread.start()

        self._action_server = ActionServer(
            self,
            LlmDecision,
            '/llm/decision',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(f'LlmDecisionServer ready (mode={mode})')
        status = 'enabled' if self._mcp_tool_broker is not None else 'disabled'
        self.get_logger().info(f'decision agentic MCP {status}')

    def _make_context_config(self) -> ChatContextConfig:
        return ChatContextConfig(
            provider=str(self.get_parameter('context_provider').value or 'none'),
            timeout_sec=float(self.get_parameter('context_timeout_sec').value),
            max_chars=int(self.get_parameter('context_max_chars').value),
            include_bt_state=bool(
                self.get_parameter('context_include_bt_state').value),
            include_formations=bool(
                self.get_parameter('context_include_formations').value),
            include_map_summary=bool(
                self.get_parameter('context_include_map_summary').value),
            include_recent_events=bool(
                self.get_parameter('context_include_recent_events').value),
            include_robot_positions=bool(
                self.get_parameter('context_include_robot_positions').value),
            pose_stale_ms=int(
                self.get_parameter('context_pose_stale_ms').value),
            map_name=str(self.get_parameter('map_name').value or 'cave'),
            map_config={},
            mcp_enabled=bool(self.get_parameter('mcp_enabled').value),
            mcp_transport=str(
                self.get_parameter('mcp_transport').value or 'stdio'),
            mcp_command=str(self.get_parameter('mcp_command').value or 'uvx'),
            mcp_args=self._param_string_list('mcp_args'),
            mcp_tool_allowlist=self._param_string_list('mcp_tool_allowlist'),
        )

    def _make_decision_agentic_mcp_config(self) -> AgenticMcpConfig:
        return AgenticMcpConfig(
            enabled=bool(
                self.get_parameter('mcp_decision_agentic_enabled').value),
            max_rounds=int(
                self.get_parameter('mcp_decision_agentic_max_rounds').value),
            max_tools_per_round=int(self.get_parameter(
                'mcp_decision_agentic_max_tools_per_round').value),
            tool_timeout_sec=float(self.get_parameter(
                'mcp_decision_agentic_tool_timeout_sec').value),
            max_result_chars=int(self.get_parameter(
                'mcp_decision_agentic_max_result_chars').value),
        )

    def _make_mcp_tool_broker(self) -> McpToolBroker | None:
        if not self._mcp_decision_agentic_config.enabled:
            self.get_logger().info(
                'decision agentic MCP disabled by parameter')
            return None
        if self._context_config.provider != 'mcp_readonly':
            self.get_logger().info(
                'decision agentic MCP disabled because context_provider is '
                'not mcp_readonly')
            return None
        if not self._context_config.mcp_enabled:
            self.get_logger().info(
                'decision agentic MCP disabled because mcp_enabled is false')
            return None
        runner = getattr(self._context_provider, 'execute_readonly_tool', None)
        if runner is None:
            self.get_logger().warn(
                'decision agentic MCP disabled because provider has no '
                'read-only tool runner')
            return None
        allowed = [
            name for name in self._context_config.mcp_tool_allowlist
            if name in DECISION_MCP_ALLOWED_TOOLS
        ]
        if not allowed:
            self.get_logger().warn(
                'decision agentic MCP disabled because no decision read-only '
                'tools are enabled')
            return None
        return McpToolBroker(
            allowed_tools=allowed,
            runner=runner,
            max_tools_per_round=(
                self._mcp_decision_agentic_config.max_tools_per_round),
            tool_timeout_sec=(
                self._mcp_decision_agentic_config.tool_timeout_sec),
            max_result_chars=(
                self._mcp_decision_agentic_config.max_result_chars),
            logger=self.get_logger(),
            arg_validator=self._validate_decision_tool_args,
        )

    def _param_string_list(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        return [str(item) for item in list(value)]

    def _should_use_agentic_mcp(self, level: str) -> bool:
        return (
            self._mcp_tool_broker is not None
            and self._mcp_decision_agentic_config.enabled
            and str(level).strip().upper() in self._mcp_decision_agentic_levels
        )

    @staticmethod
    def _validate_decision_tool_args(name: str, args: dict) -> tuple[bool, str]:
        if name in {'get_actions', 'get_topics'}:
            if args:
                return False, f'{name} takes no args in decision MCP mode'
            return True, ''
        if name == 'subscribe_once':
            if set(args.keys()) != {'topic', 'msg_type'}:
                return (
                    False,
                    'subscribe_once requires exactly topic and msg_type',
                )
            topic = safe_str(args.get('topic'), 240).strip()
            msg_type = safe_str(args.get('msg_type'), 240).strip()
            expected = _DECISION_SUBSCRIBE_TYPES.get(topic)
            if expected is None:
                return (
                    False,
                    'decision MCP may only subscribe to /bt/state or '
                    '/formations/status',
                )
            if msg_type != expected:
                return False, f'{topic} requires msg_type {expected}'
            return True, ''
        if name == 'get_action_status':
            if set(args.keys()) != {'action_name'}:
                return (
                    False,
                    'get_action_status requires exactly action_name',
                )
            action_name = safe_str(args.get('action_name'), 240).strip()
            if action_name != '/swarm/set_goals':
                return (
                    False,
                    'decision MCP may only inspect action_name '
                    '/swarm/set_goals',
                )
            return True, ''
        return False, 'tool is not part of the decision MCP policy'

    def _execute(self, goal_handle):
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    async def _execute_async(self, goal_handle):
        req = goal_handle.request
        self._publish_feedback(goal_handle, 'received')
        use_agentic = self._should_use_agentic_mcp(req.level)
        event_preview = safe_str(req.event, 160).replace('\n', ' ')
        self.get_logger().info(
            f'decision request level={req.level} event={event_preview} '
            f'agentic_mcp={use_agentic}')
        prompt = build_decision_prompt(
            DECISION_SCENARIOS,
            req.level,
            req.event,
            list(req.log_buffer),
            self._tail,
            agentic_enabled=use_agentic,
        )

        decision = self._default
        reason   = 'timeout or error'

        async with self._semaphore:
            try:
                self._publish_feedback(goal_handle, 'thinking')
                if use_agentic:
                    try:
                        decision, reason = await self._run_agentic_decision(
                            prompt)
                    except Exception as exc:
                        self.get_logger().warn(
                            'decision agentic MCP failed; falling back to '
                            f'normal decision prompt: {exc}')
                        fallback_prompt = build_decision_prompt(
                            DECISION_SCENARIOS,
                            req.level,
                            req.event,
                            list(req.log_buffer),
                            self._tail,
                            agentic_enabled=False,
                        )
                        decision, reason = await self._run_plain_decision(
                            fallback_prompt)
                else:
                    decision, reason = await self._run_plain_decision(prompt)
            except asyncio.TimeoutError:
                self.get_logger().warn('LLM decision timeout')
            except Exception as exc:
                self.get_logger().error(f'LLM decision error: {exc}')
        self.get_logger().info(f'LLM decision final: {decision}')

        self._logger_ds.log({
            'level': req.level,
            'event': req.event,
            'log_buffer': list(req.log_buffer),
            'decision': decision,
            'reason': reason,
        })

        ev = LlmEvent()
        ev.stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
        ev.channel  = LlmEvent.CHANNEL_DECISION
        ev.trigger  = f'[{req.level}] {req.event}'
        ev.output   = decision
        ev.reason   = reason
        self._event_pub.publish(ev)

        result = LlmDecision.Result()
        result.decision = decision
        self._publish_feedback(goal_handle, 'done')
        goal_handle.succeed()
        return result

    async def _run_plain_decision(self, prompt: str) -> tuple[str, str]:
        raw = await asyncio.wait_for(
            self._llm.generate(prompt, prompt_kind='decision'),
            timeout=self._timeout,
        )
        return parse_llm_decision(raw), raw[:200]

    async def _run_agentic_decision(self, prompt: str) -> tuple[str, str]:
        if self._mcp_tool_broker is None:
            raise AgenticMcpError('decision MCP broker is unavailable')

        conversation = [{'role': 'user', 'content': prompt}]
        rounds_used = 0
        while True:
            round_no = rounds_used + 1
            self.get_logger().info(
                f'decision agentic MCP tool round {round_no}')
            raw = await asyncio.wait_for(
                self._llm.generate(conversation, prompt_kind='decision'),
                timeout=self._timeout,
            )
            parsed = parse_agentic_response(raw)
            if parsed.mode == 'final':
                decision = parse_llm_decision_final(raw)
                self.get_logger().info(
                    f'decision agentic MCP final decision={decision}')
                return decision, raw[:200]
            if parsed.mode == 'invalid':
                raise AgenticMcpError(
                    f'invalid decision MCP response: {parsed.error}')

            if rounds_used >= max(
                0, int(self._mcp_decision_agentic_config.max_rounds)
            ):
                self.get_logger().warn(
                    'decision agentic MCP tool budget exhausted')
                conversation.append({'role': 'assistant', 'content': raw})
                conversation.append({
                    'role': 'user',
                    'content': json.dumps({
                        'mode': 'tool_result',
                        'results': [{
                            'status': 'rejected',
                            'error': 'decision MCP tool budget exhausted',
                        }],
                        'instruction': (
                            'Return mode=final now. Final decision must be '
                            'wait, replan, or abort. Do not request more '
                            'tools.'
                        ),
                    }, ensure_ascii=False),
                })
                final_raw = await asyncio.wait_for(
                    self._llm.generate(
                        conversation,
                        prompt_kind='decision',
                    ),
                    timeout=self._timeout,
                )
                final_parsed = parse_agentic_response(final_raw)
                if final_parsed.mode == 'final':
                    decision = parse_llm_decision_final(final_raw)
                    self.get_logger().info(
                        f'decision agentic MCP final decision={decision}')
                    return decision, final_raw[:200]
                raise AgenticMcpError('decision MCP tool budget exhausted')

            rounds_used += 1
            tools = (parsed.obj or {}).get('tools') or []
            names = [
                safe_str(t.get('name'), 120) if isinstance(t, dict)
                else '<invalid>'
                for t in tools
            ]
            self.get_logger().info(
                f'decision agentic MCP round {rounds_used} requested {names}')
            tool_result = await self._mcp_tool_broker.execute_tool_request(
                parsed.obj or {})
            failed, failure_reason = self._tool_result_failed(tool_result)
            if failed:
                self.get_logger().warn(
                    'decision agentic MCP unavailable or no usable '
                    f'observations: {failure_reason}')
                raise AgenticMcpError(failure_reason)
            conversation.append({'role': 'assistant', 'content': raw})
            conversation.append({
                'role': 'user',
                'content': self._format_decision_tool_result(tool_result),
            })

    @staticmethod
    def _format_decision_tool_result(tool_result: dict) -> str:
        payload = {
            **tool_result,
            'instruction': (
                'These are read-only observations. Use them only as current '
                'state. Return another mode=tool_request only if more listed '
                'read-only context is essential; otherwise return mode=final '
                'with decision wait, replan, or abort.'
            ),
        }
        return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))

    @staticmethod
    def _tool_result_failed(tool_result: dict) -> tuple[bool, str]:
        results = tool_result.get('results') if isinstance(tool_result, dict) else []
        if not isinstance(results, list) or not results:
            return False, ''
        if any(item.get('status') == 'ok' for item in results
               if isinstance(item, dict)):
            return False, ''
        errors = []
        for item in results:
            if not isinstance(item, dict):
                continue
            status = safe_str(item.get('status'), 80)
            detail = safe_str(item.get('error'), 240)
            label = safe_str(item.get('name'), 120)
            errors.append(f'{label}:{status}:{detail}')
        return True, '; '.join(errors) or 'no usable MCP tool results'

    @staticmethod
    def _publish_feedback(goal_handle, stage: str):
        fb = LlmDecision.Feedback()
        fb.stage = stage
        goal_handle.publish_feedback(fb)

    def shutdown_resources(self):
        self.get_logger().info(
            'llm_decision_server: shutting down MCP/tool resources')
        if self._loop.is_running():
            fut = asyncio.run_coroutine_threadsafe(
                self._cancel_async_tasks(),
                self._loop,
            )
            try:
                fut.result(timeout=5.0)
            except Exception as exc:
                self.get_logger().warn(
                    f'llm_decision_server: async shutdown warning: {exc}')
            self._loop.call_soon_threadsafe(self._loop.stop)
        if getattr(self, '_loop_thread', None) is not None:
            self._loop_thread.join(timeout=5.0)
            if self._loop_thread.is_alive():
                self.get_logger().warn(
                    'llm_decision_server: asyncio loop thread did not stop')
                return
        if not self._loop.is_closed():
            self._loop.close()
        self.get_logger().info('llm_decision_server: shutdown complete')

    async def _cancel_async_tasks(self):
        current = asyncio.current_task()
        tasks = [
            task for task in asyncio.all_tasks(self._loop)
            if task is not current and not task.done()
        ]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        await self._loop.shutdown_asyncgens()


def main(args=None):
    rclpy.init(args=args)
    node = LlmDecisionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_resources()
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
