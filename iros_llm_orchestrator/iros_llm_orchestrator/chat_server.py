"""Channel 3 over ROS — /llm/chat action server.

Mirrors user_chat_node's pipeline (build_user_prompt → stream → parse →
PlanExecutor) but exposes it as a ROS action so the RViz panel and other
clients can drive the LLM without owning stdin.

Two non-obvious bits the panel relies on:

* Reply streaming. Feedback chunks carry only the body of the JSON
  "reply" string, not the surrounding envelope. The same state machine
  user_chat_node uses for live printing runs here, so the panel doesn't
  display raw `{"reply":"..."}` JSON to the operator.

* Mission completion. PlanExecutor leaves are sent via BTLeafSender,
  which watches /bt/state mode transitions to know when each leaf has
  actually finished — LlmCommand.Result alone fires as soon as the goal
  is accepted, while the mission is still running.
"""

import asyncio
import json
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmChat
from iros_llm_swarm_interfaces.msg import LlmEvent

from iros_llm_orchestrator.common.leaf_sender import BTLeafSender
from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.plan_executor import PlanExecutor
from iros_llm_orchestrator.common.user_prompt import (
    build_user_prompt, build_bt_event_prompt, load_map_config)
from iros_llm_orchestrator.context import (
    DEFAULT_MCP_READ_TOOLS,
    ChatContextConfig,
    make_context_provider,
)
from iros_llm_orchestrator.context.no_execute import (
    should_skip_reply_only_execution,
)
from iros_llm_orchestrator.context.provider import bound_context, utc_now
from iros_llm_orchestrator.user_chat_node import _parse_response, _postprocess_plan

MAX_HISTORY = 8   # conversation turns kept per session


class ChatServer(Node):
    def __init__(self):
        super().__init__('llm_chat_server')

        self.declare_parameter('llm_mode',         'ollama')
        self.declare_parameter('llm_endpoint',     'http://localhost:11434/api/chat')
        self.declare_parameter('llm_model',        'qwen2.5:14b')
        self.declare_parameter('llm_max_tokens',   768)
        self.declare_parameter('llm_temperature',  0.1)
        self.declare_parameter('llm_api_key',      '')
        self.declare_parameter('llm_api_key_env',  'LLM_API_KEY')
        self.declare_parameter('llm_force_chat',   True)
        self.declare_parameter('llm_enable_stop',  False)
        self.declare_parameter('timeout_sec',      30.0)
        self.declare_parameter('step_timeout_sec', 120.0)
        self.declare_parameter('map_name',         'cave')
        self.declare_parameter('context_provider', 'none')
        self.declare_parameter('context_timeout_sec', 2.0)
        self.declare_parameter('context_max_chars', 6000)
        self.declare_parameter('context_include_bt_state', True)
        self.declare_parameter('context_include_formations', True)
        self.declare_parameter('context_include_map_summary', True)
        self.declare_parameter('context_include_recent_events', True)
        self.declare_parameter('mcp_enabled', False)
        self.declare_parameter('mcp_transport', 'stdio')
        self.declare_parameter('mcp_command', 'uvx')
        self.declare_parameter('mcp_args', ['ros-mcp', '--transport=stdio'])
        self.declare_parameter('mcp_tool_allowlist', list(DEFAULT_MCP_READ_TOOLS))

        self._timeout  = float(self.get_parameter('timeout_sec').value)
        self._map_name = self.get_parameter('map_name').value

        mode = self.get_parameter('llm_mode').value
        self._llm = get_llm_client(
            mode=mode,
            endpoint=self.get_parameter('llm_endpoint').value,
            model=self.get_parameter('llm_model').value,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
            api_key=self.get_parameter('llm_api_key').value,
            api_key_env=self.get_parameter('llm_api_key_env').value,
            timeout=self._timeout,
            force_chat=bool(self.get_parameter('llm_force_chat').value),
            enable_stop=bool(self.get_parameter('llm_enable_stop').value),
        )
        try:
            self._map_cfg = load_map_config(self._map_name)
        except Exception as exc:
            self.get_logger().warn(f'Map config load failed: {exc}')
            self._map_cfg = {}

        self._context_config = self._make_context_config()
        self._context_provider = make_context_provider(self, self._context_config)

        self._sender = BTLeafSender(
            self,
            step_timeout_sec=float(self.get_parameter('step_timeout_sec').value),
        )

        # /llm/events publisher — channel 3 emits one event per turn.
        self._event_pub = self.create_publisher(LlmEvent, '/llm/events', 10)

        # Conversation history — shared across all chat turns (serialized by
        # _chat_lock). Allows the model to resolve references like "same robots"
        # and react to BT events that were injected between user messages.
        self._history: list[dict] = []

        # Subscribe to /bt/state to detect formation WARN/ERROR events and
        # inject LLM analysis into the conversation history — same as
        # user_chat_node._handle_bt_event so both paths stay in sync.
        from iros_llm_swarm_interfaces.msg import BTState
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        bt_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        self._last_bt_status     = 'OK'
        self._bt_event_analyzing = False
        self.create_subscription(BTState, '/bt/state', self._on_bt_state, bt_qos)

        # Asyncio loop on a dedicated thread; action callbacks block on it.
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        # Serialize concurrent /llm/chat goals — the panel guards client-side
        # but a CLI caller could still race two requests.
        self._chat_lock = asyncio.Lock()

        self._action_server = ActionServer(
            self, LlmChat, '/llm/chat',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(
            f'LlmChatServer ready on /llm/chat '
            f'(mode={mode}, context={self._context_config.provider})')

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
            map_name=str(self._map_name),
            map_config=dict(self._map_cfg or {}),
            mcp_enabled=bool(self.get_parameter('mcp_enabled').value),
            mcp_transport=str(self.get_parameter('mcp_transport').value or 'stdio'),
            mcp_command=str(self.get_parameter('mcp_command').value or 'uvx'),
            mcp_args=self._param_string_list('mcp_args'),
            mcp_tool_allowlist=self._param_string_list('mcp_tool_allowlist'),
        )

    def _param_string_list(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        return [str(item) for item in list(value)]

    # ------------------------------------------------------------------
    # Action execute
    # ------------------------------------------------------------------

    def _execute(self, goal_handle):
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    async def _execute_async(self, goal_handle):
        async with self._chat_lock:
            return await self._execute_body(goal_handle)

    async def _execute_body(self, goal_handle):
        req = goal_handle.request
        result = LlmChat.Result()

        # ---- 1. Stream reply ----
        self._publish_fb(goal_handle, stage='thinking')
        runtime_context = await self._get_runtime_context()
        messages = build_user_prompt(req.user_message, history=list(self._history),
                                     map_name=self._map_name,
                                     runtime_context=runtime_context)

        try:
            full_raw = await asyncio.wait_for(
                self._stream_reply(messages, goal_handle),
                timeout=self._timeout)
        except asyncio.TimeoutError:
            return self._fail(goal_handle, result, 'LLM timeout')
        except Exception as exc:
            return self._fail(goal_handle, result, f'LLM error: {exc}')

        # ---- 2. Parse ----
        try:
            reply, plan = _parse_response(full_raw)
        except ValueError as exc:
            return self._fail(goal_handle, result, f'parse error: {exc}')

        plan = _postprocess_plan(plan, self._map_cfg)
        plan_json = json.dumps(plan, ensure_ascii=False)
        result.final_reply = reply
        result.plan_json   = plan_json

        # Save turn to shared history
        self._history.append({'role': 'user',      'content': req.user_message})
        self._history.append({'role': 'assistant', 'content': full_raw})
        self._trim_history()

        self._publish_fb(goal_handle, stage='parsed', detail=plan_json)
        self._publish_event(channel=LlmEvent.CHANNEL_USER,
                            trigger=req.user_message,
                            output=plan_json,
                            reason=reply)

        # ---- 3. Optional execute ----
        if req.execute_after_planning and should_skip_reply_only_execution(
            req.user_message,
            plan,
            runtime_context,
        ):
            self.get_logger().info(
                'Skipping PlanExecutor for reply-only context question')
        elif req.execute_after_planning:
            self._publish_fb(goal_handle, stage='executing')
            executor = PlanExecutor(
                send_fn=self._sender.send,
                log_fn=lambda m: self.get_logger().info(f'executor: {m}'))
            ok = await executor.run(plan)
            result.plan_executed = ok
            if not ok:
                return self._fail(goal_handle, result,
                                  'plan execution failed', got_plan=True)

        self._publish_fb(goal_handle, stage='done')
        result.success = True
        result.info    = ''
        goal_handle.succeed()
        return result

    async def _get_runtime_context(self) -> dict:
        try:
            context = await asyncio.wait_for(
                self._context_provider.get_context(),
                timeout=max(0.1, float(self._context_config.timeout_sec)),
            )
        except Exception as exc:
            context = {
                'timestamp': utc_now(),
                'source': self._context_config.provider,
                'warnings': [
                    f'Context provider failed; continuing without live context: {exc}',
                ],
            }
        context = bound_context(context, self._context_config.max_chars)
        source = context.get('source', 'unknown')
        if source != 'none':
            warning_count = len(context.get('warnings') or [])
            self.get_logger().info(
                f'Chat runtime context source={source} warnings={warning_count}')
        return context

    # ------------------------------------------------------------------
    # Reply streaming — emit only the JSON "reply" field via feedback
    # ------------------------------------------------------------------

    async def _stream_reply(self, messages, goal_handle) -> str:
        """Stream and forward only "reply" body characters to the panel.

        State machine identical to user_chat_node._stream_command:
          BEFORE   — scanning for `"reply":"`
          IN_REPLY — emit chars; honour JSON escapes; stop at unescaped `"`
          AFTER    — accumulate raw silently for the parser.
        """
        BEFORE, IN_REPLY, AFTER = 0, 1, 2
        state = BEFORE
        in_escape = False
        scan = ''
        MARKERS = ('"reply": "', '"reply":"')
        ESCAPES = {'n': '\n', 't': '\t', 'r': '\r',
                   'b': '\b', 'f': '\f',
                   '"': '"', '\\': '\\', '/': '/'}
        cap = max(len(m) for m in MARKERS)

        full = ''
        async for chunk in self._llm.stream(messages):
            full += chunk
            if state == AFTER:
                continue

            emit_buf = ''
            for c in chunk:
                if state == BEFORE:
                    scan += c
                    if len(scan) > cap:
                        scan = scan[-cap:]
                    if any(scan.endswith(m) for m in MARKERS):
                        state = IN_REPLY
                        scan  = ''
                        in_escape = False
                elif state == IN_REPLY:
                    if in_escape:
                        emit_buf += ESCAPES.get(c, c)
                        in_escape = False
                    elif c == '\\':
                        in_escape = True
                    elif c == '"':
                        state = AFTER
                    else:
                        emit_buf += c

            if emit_buf:
                self._publish_fb(goal_handle, stage='streaming', chunk=emit_buf)

        return full

    # ------------------------------------------------------------------
    # BT event handling — mirrors user_chat_node._handle_bt_event
    # ------------------------------------------------------------------

    def _on_bt_state(self, msg):
        if msg.action_status == self._last_bt_status:
            return
        prev = self._last_bt_status
        self._last_bt_status = msg.action_status

        if msg.action_status in ('WARN', 'ERROR'):
            if self._bt_event_analyzing:
                return
            self._bt_event_analyzing = True
            asyncio.run_coroutine_threadsafe(
                self._handle_bt_event(msg), self._loop)
        elif prev in ('WARN', 'ERROR') and msg.action_status == 'OK':
            # Status recovered — add a note to history so model knows
            self._history.append({
                'role': 'user',
                'content': f'[BT status recovered → OK]',
            })
            self._trim_history()

    async def _collect_stream(self, messages: list[dict]) -> str:
        """Collect full streamed response into a string."""
        full = ''
        async for chunk in self._llm.stream(messages):
            full += chunk
        return full

    async def _handle_bt_event(self, msg):
        try:
            messages = build_bt_event_prompt(msg, history=list(self._history))
            raw = ''
            try:
                raw = await asyncio.wait_for(
                    self._collect_stream(messages),
                    timeout=self._timeout)
            except asyncio.TimeoutError:
                self.get_logger().error('BT event LLM timeout')
                return
            except Exception as exc:
                self.get_logger().error(f'BT event LLM error: {exc}')
                return

            # Publish analysis as an LlmEvent so the RViz panel can display it
            event_text = (f'[BT {msg.action_status}] '
                          f'{msg.active_action}: {msg.last_error}')
            self._publish_event(
                channel=2,   # channel 2 = proactive
                trigger=event_text,
                output=raw,
                reason=f'formation_state={msg.formation_state}'
                       f' failure={msg.formation_failure_code}',
            )

            # Add to shared history so next user message has full context
            self._history.append({'role': 'user',      'content': event_text})
            self._history.append({'role': 'assistant', 'content': raw})
            self._trim_history()
        finally:
            self._bt_event_analyzing = False

    def _trim_history(self):
        if len(self._history) > MAX_HISTORY * 2:
            self._history = self._history[-MAX_HISTORY * 2:]

    # ------------------------------------------------------------------
    # Feedback / events / failure
    # ------------------------------------------------------------------

    def _publish_fb(self, gh, *, stage='', chunk='', detail=''):
        fb = LlmChat.Feedback()
        fb.stage  = stage
        fb.chunk  = chunk
        fb.detail = detail
        gh.publish_feedback(fb)

    def _publish_event(self, *, channel, trigger, output, reason):
        ev = LlmEvent()
        ev.stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
        ev.channel  = channel
        ev.trigger  = trigger
        ev.output   = output
        ev.reason   = reason
        self._event_pub.publish(ev)

    def _fail(self, gh, result, info, got_plan=False):
        self._publish_fb(gh, stage='error', detail=info)
        result.success = False
        result.info    = info
        if not got_plan:
            result.plan_json = ''
        gh.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ChatServer()
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
