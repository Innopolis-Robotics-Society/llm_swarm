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
from iros_llm_swarm_interfaces.srv import ListObstacles

from iros_llm_orchestrator.common.leaf_sender import BTLeafSender
from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.plan_executor import PlanExecutor, parse_plan
from iros_llm_orchestrator.common.user_prompt import (
    build_execution_repair_prompt,
    build_mission_continuation_prompt,
    build_remediation_prompt,
    build_user_prompt,
    build_bt_event_prompt,
    load_map_config,
)
from iros_llm_orchestrator.common.execution_repair import (
    append_verification_to_reply,
    should_attempt_repair,
    verification_failure_info,
    verification_summary,
)
from iros_llm_orchestrator.common.context_budget import (
    completion_budget_for_prompt,
)
from iros_llm_orchestrator.common.occupancy_rewrite import (
    rewrite_occupied_room_mapf_goals,
)
from iros_llm_orchestrator.common.active_formation_guard import (
    guard_plan_for_active_formations,
)
from iros_llm_orchestrator.common.mission_supervision import (
    MissionConfig,
    MissionContinuation,
    supervise_mission,
)
from iros_llm_orchestrator.context import (
    DEFAULT_MCP_READ_TOOLS,
    ChatContextConfig,
    RobotPoseCache,
    compute_formation_staging,
    make_context_provider,
)
from iros_llm_orchestrator.context.mcp_readonly_provider import (
    summarize_for_remediation,
)
from iros_llm_orchestrator.context.no_execute import (
    is_help_request,
    should_skip_reply_only_execution,
)
from iros_llm_orchestrator.context.provider import (
    bound_context,
    known_robot_ids,
    utc_now,
)
from iros_llm_orchestrator.common.tool_definitions import TOOL_DEFINITIONS
from iros_llm_orchestrator.common.plan_schema import PLAN_RESPONSE_SCHEMA
from iros_llm_orchestrator.common.tool_executor import ToolExecutor
from iros_llm_orchestrator.user_chat_node import (
    _parse_response, _postprocess_plan,
    _build_tool_use_assistant_message, _build_tool_result_message,
)

MAX_HISTORY = 8   # conversation turns kept per session


class _LlmStageError(RuntimeError):
    """Wraps any failure during stream/parse/postprocess so the caller
    can decide between hard-fail (initial turn) and operator escalation
    (remediation turn)."""


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
        self.declare_parameter('llm_num_ctx',      32768)
        self.declare_parameter('llm_context_window_tokens', 16384)
        self.declare_parameter('llm_context_margin_tokens', 512)
        self.declare_parameter('llm_default_max_completion_tokens', 2048)
        self.declare_parameter('llm_min_completion_tokens', 512)
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
        self.declare_parameter('context_include_robot_positions', True)
        self.declare_parameter('context_pose_stale_ms', 2000)
        # Must match formation_manager_node's position_tolerance. PlanExecutor
        # uses this to decide whether a formation leaf needs an auto-staging
        # mapf step prepended before dispatch.
        self.declare_parameter('formation_tolerance_m', 0.5)
        self.declare_parameter('mcp_enabled', False)
        self.declare_parameter('mcp_transport', 'stdio')
        self.declare_parameter('mcp_command', 'uvx')
        self.declare_parameter('mcp_args', ['ros-mcp', '--transport=stdio'])
        self.declare_parameter('mcp_tool_allowlist', list(DEFAULT_MCP_READ_TOOLS))
        self.declare_parameter('max_remediation_attempts', 2)
        self.declare_parameter('remediation_enabled', True)
        self.declare_parameter('llm_repair_enabled', True)
        self.declare_parameter('llm_max_repair_attempts', 2)
        self.declare_parameter('llm_repair_require_verification', True)
        self.declare_parameter('llm_verification_delay_sec', 0.2)
        self.declare_parameter('llm_mission_supervision_enabled', True)
        self.declare_parameter('llm_mission_max_duration_sec', 180.0)
        self.declare_parameter('llm_mission_max_steps', 6)
        self.declare_parameter('llm_mission_verify_delay_sec', 0.8)
        self.declare_parameter('llm_mission_min_progress_required', True)
        self.declare_parameter('llm_mission_allow_repair', True)
        self.declare_parameter('llm_mission_no_progress_limit', 2)
        self.declare_parameter('robot_footprint_radius', 0.22)
        self.declare_parameter('scan_timeout_sec',       3.0)
        self.declare_parameter('tool_max_iterations',    6)
        self.declare_parameter('stream_reasoning',       True)
        # Tool-calling adds latency and a prose-fallback failure mode; when the
        # prompt fits the context window the model plans directly without tools.
        # Off by default — flip on for spatial-precision experiments.
        self.declare_parameter('tool_calling_enabled',   False)
        # Structured outputs: send the plan JSON schema to the backend
        # (Ollama format / OpenAI response_format) so the model can only emit
        # schema-valid JSON. Applies to the plain (non-tool) path. Conflicts
        # with tool calling, so keep tool_calling_enabled off when using this.
        self.declare_parameter('structured_output_enabled', True)
        # Auto-spread near-coincident mapf goals into a cluster. Off: the LLM
        # must emit one goal per robot; the motion planner resolves collisions.
        self.declare_parameter('goal_spread_enabled',    False)

        self._max_remediation_attempts = max(0, int(
            self.get_parameter('max_remediation_attempts').value))
        self._remediation_enabled = bool(
            self.get_parameter('remediation_enabled').value)
        self._repair_enabled = bool(
            self.get_parameter('llm_repair_enabled').value)
        self._max_repair_attempts = max(0, int(
            self.get_parameter('llm_max_repair_attempts').value))
        self._repair_require_verification = bool(
            self.get_parameter('llm_repair_require_verification').value)
        self._verification_delay_sec = max(0.0, float(
            self.get_parameter('llm_verification_delay_sec').value))
        self._mission_supervision_enabled = bool(
            self.get_parameter('llm_mission_supervision_enabled').value)
        self._mission_config = MissionConfig(
            enabled=self._mission_supervision_enabled,
            max_duration_sec=max(
                0.1,
                float(self.get_parameter('llm_mission_max_duration_sec').value),
            ),
            max_steps=max(
                1,
                int(self.get_parameter('llm_mission_max_steps').value),
            ),
            no_progress_limit=max(
                1,
                int(self.get_parameter('llm_mission_no_progress_limit').value),
            ),
            min_progress_required=bool(
                self.get_parameter('llm_mission_min_progress_required').value),
            allow_repair=bool(
                self.get_parameter('llm_mission_allow_repair').value),
        )
        self._mission_verify_delay_sec = max(
            0.0,
            float(self.get_parameter('llm_mission_verify_delay_sec').value),
        )
        self._tool_max_iterations = max(1, int(
            self.get_parameter('tool_max_iterations').value))
        self._stream_reasoning = bool(self.get_parameter('stream_reasoning').value)
        self._tool_calling_enabled = bool(
            self.get_parameter('tool_calling_enabled').value)
        self._structured_output_enabled = bool(
            self.get_parameter('structured_output_enabled').value)
        self._goal_spread_enabled = bool(
            self.get_parameter('goal_spread_enabled').value)

        self._timeout  = float(self.get_parameter('timeout_sec').value)
        self._map_name = self.get_parameter('map_name').value
        self._llm_max_tokens = max(
            1, int(self.get_parameter('llm_max_tokens').value))
        self._llm_num_ctx = max(
            0, int(self.get_parameter('llm_num_ctx').value))
        self._llm_context_window_tokens = max(
            0, int(self.get_parameter('llm_context_window_tokens').value))
        self._llm_context_margin_tokens = max(
            0, int(self.get_parameter('llm_context_margin_tokens').value))
        self._llm_default_completion_tokens = max(
            1,
            min(
                self._llm_max_tokens,
                int(self.get_parameter(
                    'llm_default_max_completion_tokens').value),
            ),
        )
        self._llm_min_completion_tokens = max(
            1, int(self.get_parameter('llm_min_completion_tokens').value))

        mode = self.get_parameter('llm_mode').value
        self._llm = get_llm_client(
            mode=mode,
            endpoint=self.get_parameter('llm_endpoint').value,
            model=self.get_parameter('llm_model').value,
            max_tokens=self._llm_max_tokens,
            temperature=float(self.get_parameter('llm_temperature').value),
            api_key=self.get_parameter('llm_api_key').value,
            api_key_env=self.get_parameter('llm_api_key_env').value,
            timeout=self._timeout,
            force_chat=bool(self.get_parameter('llm_force_chat').value),
            enable_stop=bool(self.get_parameter('llm_enable_stop').value),
            num_ctx=self._llm_num_ctx,
        )
        self._log_llm_context_budget(mode)
        try:
            self._map_cfg = load_map_config(self._map_name)
        except Exception as exc:
            self.get_logger().warn(f'Map config load failed: {exc}')
            self._map_cfg = {}

        self._context_config = self._make_context_config()
        self._pose_cache = RobotPoseCache(
            self, known_robot_ids(self._map_cfg or {}))
        self._formation_tolerance_m = float(
            self.get_parameter('formation_tolerance_m').value)
        self._context_provider = make_context_provider(
            self, self._context_config, pose_cache=self._pose_cache)

        _footprint_r = float(self.get_parameter('robot_footprint_radius').value)
        _scan_to     = float(self.get_parameter('scan_timeout_sec').value)
        self._tool_executor = ToolExecutor(
            node=self,
            pose_cache=self._pose_cache,
            map_cfg=self._map_cfg,
            robot_footprint_radius=_footprint_r,
            scan_timeout_sec=_scan_to,
        )

        self._sender = BTLeafSender(
            self,
            step_timeout_sec=float(self.get_parameter('step_timeout_sec').value),
        )
        self._list_obstacles = self.create_client(ListObstacles, '/obstacles/list')

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
        # Cache the most recently fetched runtime context so the formation
        # prestage hook can fall back to it when pose_cache subscriptions
        # haven't received data (QoS mismatch, wrong topic, or first run).
        self._last_runtime_context: dict = {}
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
            include_robot_positions=bool(
                self.get_parameter('context_include_robot_positions').value),
            pose_stale_ms=int(
                self.get_parameter('context_pose_stale_ms').value),
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

    def _log_llm_context_budget(self, mode: str) -> None:
        mode_l = str(mode or '').strip().lower()
        recommended_ctx = 32768
        minimum_ctx = 16384
        prompt_headroom_estimate = 10000
        if self._llm_num_ctx and self._llm_num_ctx < minimum_ctx:
            self.get_logger().warning(
                'LLM context window is small for channel-3 planning: '
                f'llm_num_ctx={self._llm_num_ctx}, recommended>={recommended_ctx}. '
                'Large map prompts, tool schemas, and runtime context may be '
                'truncated.')
        if (
            self._llm_num_ctx
            and self._llm_num_ctx
            <= self._llm_max_tokens + prompt_headroom_estimate
        ):
            self.get_logger().warning(
                'LLM context/output budget is tight: '
                f'llm_num_ctx={self._llm_num_ctx}, '
                f'llm_max_tokens={self._llm_max_tokens}. '
                'Reduce llm_max_tokens or increase the model context window.')
        if mode_l == 'http':
            self.get_logger().warning(
                'OpenAI-compatible HTTP backend does not let this client set '
                f'the server context window. llm_num_ctx={self._llm_num_ctx} '
                'is only a local budget hint; configure the Qwen/vLLM server '
                'with a matching max_model_len/context window.')
        if self._tool_calling_enabled and self._structured_output_enabled:
            self.get_logger().warning(
                'tool_calling_enabled=true disables the plain structured-output '
                'path for chat turns. This is expected for tool experiments, '
                'but structured_output_enabled will not constrain those turns.')

    # ------------------------------------------------------------------
    # Action execute
    # ------------------------------------------------------------------

    def _execute(self, goal_handle):
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    def _get_obstacle_context(self) -> str:
        from iros_llm_orchestrator.common.user_prompt import build_obstacle_context_str
        if not self._list_obstacles.wait_for_service(timeout_sec=0.5):
            return ''
        try:
            resp = self._list_obstacles.call(ListObstacles.Request())
            return build_obstacle_context_str(resp.circles, resp.rectangles, resp.doors)
        except Exception:
            return ''

    async def _execute_async(self, goal_handle):
        async with self._chat_lock:
            return await self._execute_body(goal_handle)

    async def _execute_body(self, goal_handle):
        req = goal_handle.request
        result = LlmChat.Result()

        # ---- 1. Stream initial reply ----
        self._publish_fb(goal_handle, stage='thinking')
        runtime_context = await self._get_runtime_context()
        self._last_runtime_context = runtime_context
        messages = build_user_prompt(
            req.user_message,
            history=list(self._history),
            map_name=self._map_name,
            obstacle_context=self._get_obstacle_context(),
            runtime_context=runtime_context,
        )

        try:
            reply, plan, full_raw = await self._stream_and_parse(
                messages, goal_handle, user_message=req.user_message)
        except _LlmStageError as exc:
            return self._fail(goal_handle, result, str(exc))

        plan_json = json.dumps(plan, ensure_ascii=False)
        result.final_reply = reply
        result.plan_json   = plan_json
        self._history.append({'role': 'user',      'content': req.user_message})
        self._history.append({'role': 'assistant', 'content': full_raw})
        self._trim_history()
        self._publish_fb(goal_handle, stage='parsed', detail=plan_json)
        self._publish_event(channel=LlmEvent.CHANNEL_USER,
                            trigger=req.user_message,
                            output=plan_json,
                            reason=reply)

        # ---- 2. Reply-only / help-request short circuits ----
        if req.execute_after_planning and should_skip_reply_only_execution(
            req.user_message, plan, runtime_context,
        ):
            self.get_logger().info(
                'Skipping PlanExecutor for reply-only context question')
            return self._succeed(goal_handle, result)

        if is_help_request(plan):
            return self._finalize_help(
                goal_handle, result, reply, plan,
                trigger=req.user_message,
                reason='LLM emitted needs_help on initial reply')

        if not req.execute_after_planning:
            return self._succeed(goal_handle, result)

        if self._mission_supervision_enabled:
            return await self._run_supervised_mission(
                goal_handle,
                result,
                original_user_request=req.user_message,
                initial_reply=reply,
                initial_plan=plan,
                runtime_context=runtime_context,
            )

        # ---- 3. Execute ----
        self._publish_fb(goal_handle, stage='executing')
        ok, failure_info = await self._execute_plan(plan)
        result.plan_executed = ok
        if ok:
            return await self._handle_successful_execution(
                goal_handle,
                result,
                original_user_request=req.user_message,
                last_reply=reply,
                last_plan=plan,
                runtime_context=runtime_context,
            )

        if not self._remediation_enabled or self._max_remediation_attempts == 0:
            return self._fail(goal_handle, result,
                              'plan execution failed', got_plan=True)

        # ---- 4. Remediation loop ----
        attempts: list[dict] = [failure_info or {}]
        last_plan  = plan
        last_reply = reply

        for n in range(1, self._max_remediation_attempts + 1):
            failed_leaf = attempts[-1].get('leaf_type', '?')
            self._publish_fb(
                goal_handle, stage='remediating',
                detail=f'attempt={n} leaf={failed_leaf}')

            fresh_ctx = await self._get_targeted_runtime_context(
                attempts[-1], cached=runtime_context)
            slim_ctx = summarize_for_remediation(fresh_ctx)

            rem_messages = build_remediation_prompt(
                req.user_message,
                last_plan,
                attempts,
                attempts[-1],
                fresh_runtime_context=slim_ctx,
                history=list(self._history),
                map_name=self._map_name,
                obstacle_context=self._get_obstacle_context(),
            )

            try:
                r_reply, r_plan, r_raw = await self._stream_and_parse(
                    rem_messages, goal_handle, user_message=req.user_message)
            except _LlmStageError as exc:
                return self._finalize_help(
                    goal_handle, result, last_reply, last_plan,
                    trigger=req.user_message,
                    reason=f'LLM error during remediation {n}: {exc}',
                    last_failure=attempts[-1])

            self._history.append({
                'role': 'user',
                'content': (f'[remediation {n}: {failed_leaf} failed — '
                            f'{str(attempts[-1].get("last_error",""))[:160]}]'),
            })
            self._history.append({'role': 'assistant', 'content': r_raw})
            self._trim_history()

            last_plan  = r_plan
            last_reply = r_reply
            result.final_reply = r_reply
            result.plan_json   = json.dumps(r_plan, ensure_ascii=False)
            self._publish_fb(goal_handle, stage='parsed',
                             detail=result.plan_json)
            self._publish_event(channel=LlmEvent.CHANNEL_USER,
                                trigger=req.user_message,
                                output=result.plan_json,
                                reason=r_reply)

            if is_help_request(r_plan):
                return self._finalize_help(
                    goal_handle, result, r_reply, r_plan,
                    trigger=req.user_message,
                    reason=f'LLM emitted needs_help on remediation {n}',
                    last_failure=attempts[-1])

            self._publish_fb(goal_handle, stage='executing')
            ok, failure_info = await self._execute_plan(r_plan)
            result.plan_executed = ok
            if ok:
                return await self._handle_successful_execution(
                    goal_handle,
                    result,
                    original_user_request=req.user_message,
                    last_reply=r_reply,
                    last_plan=r_plan,
                    runtime_context=fresh_ctx,
                )
            attempts.append(failure_info or {})

        # Retries exhausted — escalate to operator
        return self._finalize_help(
            goal_handle, result, last_reply, last_plan,
            trigger=req.user_message,
            reason=(f'remediation budget '
                    f'{self._max_remediation_attempts} exhausted'),
            last_failure=attempts[-1])

    async def _run_supervised_mission(
        self,
        goal_handle,
        result,
        *,
        original_user_request: str,
        initial_reply: str,
        initial_plan: dict,
        runtime_context: dict,
    ):
        request_id = int(self.get_clock().now().nanoseconds / 1e6)
        self.get_logger().info(
            'LLM mission: start '
            f'request_id={request_id} '
            f'max_duration={self._mission_config.max_duration_sec:.1f} '
            f'max_steps={self._mission_config.max_steps}'
        )

        async def _execute(plan: dict, step: int) -> tuple[bool, dict | None]:
            self._publish_fb(
                goal_handle,
                stage='executing',
                detail=f'mission_step={step}',
            )
            return await self._execute_plan(plan)

        async def _verify(
            plan: dict,
            execution_ok: bool,
            failure_info: dict | None,
            previous_verification: dict | None,
            step: int,
        ) -> dict:
            return await self._verify_plan_execution_state(
                original_user_request,
                plan,
                runtime_context=runtime_context,
                last_failure=failure_info,
                delay_sec=self._mission_verify_delay_sec,
            )

        async def _continue(ctx) -> MissionContinuation:
            self._publish_fb(
                goal_handle,
                stage='repairing',
                detail=f'mission_step={ctx.step} remaining={ctx.remaining_time_sec:.1f}s',
            )
            fresh_ctx = await self._get_runtime_context()
            self._last_runtime_context = fresh_ctx
            execution_result = {
                'ok': ctx.execution_ok,
                'failure_info': ctx.failure_info or {},
            }
            messages = build_mission_continuation_prompt(
                original_user_request,
                ctx.current_plan,
                execution_result,
                ctx.verification,
                step=ctx.step,
                max_steps=self._mission_config.max_steps,
                remaining_time_sec=ctx.remaining_time_sec,
                fresh_runtime_context=fresh_ctx,
                previous_verification=ctx.previous_verification,
                history=list(self._history),
                map_name=self._map_name,
                obstacle_context=self._get_obstacle_context(),
            )
            compact_chars = len(json.dumps(
                messages,
                ensure_ascii=False,
                separators=(',', ':'),
            ))
            self.get_logger().info(
                'LLM mission context: '
                f'compacted chars={compact_chars} '
                f'est_tokens={max(1, compact_chars // 4)}'
            )
            reply, plan, raw = await self._stream_and_parse(
                messages,
                goal_handle,
                user_message=original_user_request,
            )
            self._history.append({
                'role': 'user',
                'content': (
                    f'[mission continuation step {ctx.step}: '
                    f'{verification_summary(ctx.verification)[:180]}]'
                ),
            })
            self._history.append({'role': 'assistant', 'content': raw})
            self._trim_history()
            plan_json = json.dumps(plan, ensure_ascii=False)
            self._publish_fb(goal_handle, stage='parsed', detail=plan_json)
            self._publish_event(
                channel=LlmEvent.CHANNEL_USER,
                trigger=original_user_request,
                output=plan_json,
                reason=reply,
            )
            return MissionContinuation(reply=reply, plan=plan, raw=raw)

        outcome = await supervise_mission(
            original_request=original_user_request,
            initial_plan=initial_plan,
            initial_reply=initial_reply,
            config=self._mission_config,
            execute_plan=_execute,
            verify_plan=_verify,
            generate_continuation=_continue,
            is_help_plan=is_help_request,
            log_fn=lambda msg: self.get_logger().info(msg),
        )

        result.plan_executed = bool(outcome.plan_executed)
        result.plan_json = json.dumps(outcome.final_plan, ensure_ascii=False)
        result.final_reply = append_verification_to_reply(
            outcome.final_reply,
            outcome.final_verification,
        )
        result.info = (
            f'mission {outcome.status}: '
            f'{verification_summary(outcome.final_verification)}'
        )

        if outcome.ok:
            return self._succeed(goal_handle, result)
        if outcome.status == 'needs_help':
            return self._finalize_help(
                goal_handle,
                result,
                outcome.final_reply,
                outcome.final_plan,
                trigger=original_user_request,
                reason=outcome.reason,
                last_failure=outcome.last_failure,
            )
        return self._fail(
            goal_handle,
            result,
            f'{outcome.reason}: {verification_summary(outcome.final_verification)}',
            got_plan=True,
        )

    async def _handle_successful_execution(
        self,
        goal_handle,
        result,
        *,
        original_user_request: str,
        last_reply: str,
        last_plan: dict,
        runtime_context: dict,
    ):
        verification = await self._verify_plan_execution_state(
            original_user_request,
            last_plan,
            runtime_context=runtime_context,
        )
        if verification.get('ok') or not self._repair_require_verification:
            result.final_reply = append_verification_to_reply(
                result.final_reply or last_reply,
                verification,
            )
            result.info = f'verification: {verification_summary(verification)}'
            return self._succeed(goal_handle, result)

        current_plan = last_plan
        current_reply = last_reply
        current_verification = verification
        attempt = 0

        while should_attempt_repair(
            current_verification,
            attempt=attempt,
            max_attempts=self._max_repair_attempts,
            enabled=self._repair_enabled,
        ):
            attempt += 1
            rec = current_verification.get('repair_recommendation') or {}
            reason = rec.get('reason') or verification_summary(current_verification)
            self.get_logger().info(
                f'LLM repair: attempt={attempt} reason={str(reason)[:180]}')
            self._publish_fb(
                goal_handle,
                stage='repairing',
                detail=f'attempt={attempt} reason={str(reason)[:160]}',
            )

            fresh_ctx = await self._get_runtime_context()
            self._last_runtime_context = fresh_ctx
            repair_messages = build_execution_repair_prompt(
                original_user_request,
                current_plan,
                current_verification,
                attempt=attempt,
                max_attempts=self._max_repair_attempts,
                fresh_runtime_context=fresh_ctx,
                history=list(self._history),
                map_name=self._map_name,
                obstacle_context=self._get_obstacle_context(),
            )
            try:
                r_reply, r_plan, r_raw = await self._stream_and_parse(
                    repair_messages,
                    goal_handle,
                    user_message=original_user_request,
                )
            except _LlmStageError as exc:
                self.get_logger().warning(
                    f'LLM repair: invalid repair response attempt={attempt}: {exc}')
                result.final_reply = append_verification_to_reply(
                    current_reply,
                    current_verification,
                )
                return self._fail(
                    goal_handle,
                    result,
                    f'repair LLM failed: {exc}',
                    got_plan=True,
                )

            self.get_logger().info(
                f'LLM repair: generated plan type={r_plan.get("type", "")}')
            self._history.append({
                'role': 'user',
                'content': (
                    f'[verification repair {attempt}: '
                    f'{verification_summary(current_verification)[:180]}]'
                ),
            })
            self._history.append({'role': 'assistant', 'content': r_raw})
            self._trim_history()

            current_plan = r_plan
            current_reply = r_reply
            result.final_reply = r_reply
            result.plan_json = json.dumps(r_plan, ensure_ascii=False)
            self._publish_fb(goal_handle, stage='parsed', detail=result.plan_json)
            self._publish_event(
                channel=LlmEvent.CHANNEL_USER,
                trigger=original_user_request,
                output=result.plan_json,
                reason=r_reply,
            )

            if is_help_request(r_plan):
                return self._finalize_help(
                    goal_handle,
                    result,
                    r_reply,
                    r_plan,
                    trigger=original_user_request,
                    reason=f'LLM emitted needs_help on repair {attempt}',
                    last_failure=verification_failure_info(current_verification),
                )

            self.get_logger().info(
                f'LLM repair: executing attempt={attempt}')
            self._publish_fb(goal_handle, stage='executing')
            ok, failure_info = await self._execute_plan(r_plan)
            result.plan_executed = ok
            current_verification = await self._verify_plan_execution_state(
                original_user_request,
                r_plan,
                runtime_context=fresh_ctx,
                last_failure=failure_info,
            )
            if ok and current_verification.get('ok'):
                result.final_reply = append_verification_to_reply(
                    r_reply,
                    current_verification,
                )
                result.info = (
                    f'verification: {verification_summary(current_verification)}'
                )
                return self._succeed(goal_handle, result)

        if not self._repair_enabled:
            reason = 'repair disabled'
        elif not (current_verification.get('repair_recommendation') or {}).get('repairable'):
            reason = 'verification marked failure as non-repairable'
        else:
            reason = f'repair attempts exhausted max={self._max_repair_attempts}'
            self.get_logger().info(
                f'LLM repair: exhausted attempts max={self._max_repair_attempts}')

        result.final_reply = append_verification_to_reply(
            current_reply,
            current_verification,
        )
        result.info = f'verification failed: {verification_summary(current_verification)}'
        return self._fail(
            goal_handle,
            result,
            f'{reason}: {verification_summary(current_verification)}',
            got_plan=True,
        )

    async def _verify_plan_execution_state(
        self,
        original_user_request: str,
        last_plan: dict,
        *,
        runtime_context: dict | None,
        last_failure: dict | None = None,
        delay_sec: float | None = None,
    ) -> dict:
        request_id = int(self.get_clock().now().nanoseconds / 1e6)
        self.get_logger().info(
            f'LLM verification: start request_id={request_id}')
        delay = self._verification_delay_sec if delay_sec is None else delay_sec
        if delay > 0.0:
            await asyncio.sleep(delay)
        fresh_context = await self._get_runtime_context()
        if not fresh_context:
            fresh_context = runtime_context or {}
        self._last_runtime_context = fresh_context
        args = {
            'original_user_request': original_user_request,
            'last_plan': last_plan,
            'last_failure': last_failure or {},
            'tolerance_m': self._formation_tolerance_m,
            '_formations_status': fresh_context.get('formations'),
            '_bt_state': fresh_context.get('bt_state'),
            '_recent_events': fresh_context.get('recent_events') or [],
        }
        try:
            verification = await self._tool_executor.call(
                'verify_plan_execution_state',
                args,
            )
        except Exception as exc:
            verification = {
                'ok': False,
                'confidence': 'partial',
                'missing_state': ['verify_plan_execution_state'],
                'summary': f'verification tool failed: {exc}',
                'checks': {},
                'repair_recommendation': {
                    'type': 'wait_for_state',
                    'reason': 'verification tool failed',
                    'repairable': False,
                    'should_recompute_placement': False,
                },
            }
        self.get_logger().info(
            'LLM verification: result '
            f"ok={bool(verification.get('ok'))} "
            f"summary={str(verification.get('summary') or '')[:180]}"
        )
        return verification

    async def _stream_and_parse(self, messages, goal_handle, *, user_message: str = ''):
        """Stream → parse → postprocess; raises _LlmStageError.

        Uses the tool-calling loop when ``tool_calling_enabled``; otherwise the
        plain streaming path (no tools), which is the reliable default.
        """
        if self._tool_calling_enabled:
            coro = self._stream_with_tool_loop(messages, goal_handle)
        else:
            coro = self._stream_plain(messages, goal_handle)
        try:
            full_raw = await asyncio.wait_for(coro, timeout=self._timeout)
        except asyncio.TimeoutError as exc:
            raise _LlmStageError('LLM timeout') from exc
        except Exception as exc:
            raise _LlmStageError(f'LLM error: {exc}') from exc
        try:
            reply, plan = _parse_response(full_raw)
        except ValueError as exc:
            # Diagnostic: surface exactly what the LLM returned so a parse
            # failure (e.g. tool-calling making the model answer in prose
            # instead of JSON) is debuggable from the logs.
            preview = (full_raw if len(full_raw) <= 4000
                       else full_raw[:4000] + '…[truncated]')
            self.get_logger().error(
                f'parse error: {exc} | raw LLM output ({len(full_raw)} chars): '
                f'{preview!r}')
            raise _LlmStageError(f'parse error: {exc}') from exc
        plan = _postprocess_plan(plan, self._map_cfg, self._goal_spread_enabled)
        plan = self._rewrite_occupied_room_mapf_goals(plan, user_message)
        return reply, plan, full_raw

    def _rewrite_occupied_room_mapf_goals(
        self,
        plan: dict,
        user_message: str,
    ) -> dict:
        snapshot = {}
        if self._pose_cache is not None:
            try:
                snapshot = self._pose_cache.snapshot(
                    stale_threshold_ms=int(self._context_config.pose_stale_ms))
            except Exception as exc:
                self.get_logger().warning(
                    f'occupancy_rewrite: pose snapshot unavailable: {exc}')
                snapshot = {}
        rewritten, rewrites = rewrite_occupied_room_mapf_goals(
            plan,
            self._map_cfg,
            pose_snapshot=snapshot,
            user_message=user_message,
            robot_footprint_radius=float(
                self.get_parameter('robot_footprint_radius').value),
        )
        for rewrite in rewrites:
            self.get_logger().info(
                'occupancy_rewrite: mapf goals rewritten '
                f"room={rewrite.get('room')} "
                f"robots={rewrite.get('robot_ids')} "
                f"mode={rewrite.get('mode')} "
                f"boundary={rewrite.get('room_boundary_source')}"
            )
        return rewritten

    def _prepare_llm_context_budget(
        self,
        messages,
        *,
        extra=None,
        phase: str = 'chat',
    ) -> None:
        decision = completion_budget_for_prompt(
            messages,
            context_window_tokens=self._llm_context_window_tokens,
            default_completion_tokens=self._llm_default_completion_tokens,
            min_completion_tokens=self._llm_min_completion_tokens,
            margin_tokens=self._llm_context_margin_tokens,
            extra=extra,
        )
        self.get_logger().info(
            'LLM context guard: '
            f"phase={phase} "
            f"input_est_tokens={decision['input_est_tokens']} "
            f"max_completion={decision['max_completion_tokens']} "
            f"available_completion={decision['available_completion_tokens']} "
            f"action={decision['action']}"
        )
        if not decision.get('ok'):
            self.get_logger().error(
                'LLM context guard: abort context too large '
                f"input_est_tokens={decision['input_est_tokens']} "
                f"available_completion={decision['available_completion_tokens']}"
            )
            raise RuntimeError('mission continuation context too large after compaction')
        max_completion = int(decision['max_completion_tokens'])
        old_max = int(getattr(self._llm, 'max_tokens', self._llm_max_tokens))
        if old_max != max_completion:
            if max_completion < old_max:
                self.get_logger().info(
                    'LLM context guard: reducing completion tokens from '
                    f'{old_max} to {max_completion}'
                )
            setattr(self._llm, 'max_tokens', max_completion)

    async def _stream_with_tool_loop(
        self,
        messages: list[dict],
        goal_handle,
    ) -> str:
        """Tool calling loop with streaming feedback for the RViz panel.

        When stream_reasoning=True, emits stage='thinking' chunks for reasoning
        tokens during each LLM call.  The reply field of the final response is
        always emitted as stage='streaming' via _emit_reply_streaming.
        """
        msgs = list(messages)
        for iteration in range(self._tool_max_iterations):
            full_text = ''
            terminal: dict | None = None
            self._prepare_llm_context_budget(
                msgs,
                extra=TOOL_DEFINITIONS,
                phase=f'tool_loop_{iteration + 1}',
            )

            if self._stream_reasoning:
                async for event in self._llm.stream_with_tools(msgs, TOOL_DEFINITIONS):
                    if event['type'] == 'chunk':
                        token = event.get('content', '')
                        if token:
                            full_text += token
                            self._publish_fb(goal_handle, stage='thinking', chunk=token)
                    else:
                        terminal = event
            else:
                terminal = await self._llm.generate_with_tools(msgs, TOOL_DEFINITIONS)

            if terminal is None:
                terminal = {'type': 'text', 'content': full_text}

            if terminal['type'] == 'text':
                content = terminal.get('content') or full_text
                has_json = '{' in content
                self.get_logger().info(
                    f'tool_loop: final text after {iteration + 1} iteration(s), '
                    f'{len(content)} chars, has_json={has_json}')
                if not has_json and iteration < self._tool_max_iterations - 1:
                    self.get_logger().warning(
                        'tool_loop: prose response detected, injecting JSON reminder')
                    # Only append the assistant turn when there is actual content;
                    # an empty turn confuses the model on the next iteration.
                    if content:
                        msgs.append({'role': 'assistant', 'content': content})
                    msgs.append({
                        'role': 'user',
                        'content': (
                            'Your response above is not valid JSON. '
                            'Output ONLY the JSON object now:\n'
                            '{"reply":"...","plan":{...}}\n'
                            'No prose, no explanation — just the JSON.'
                        ),
                    })
                    continue
                if has_json and iteration < self._tool_max_iterations - 1:
                    try:
                        parse_plan(content)
                    except ValueError as schema_exc:
                        self.get_logger().warning(
                            f'tool_loop: plan schema error iter={iteration + 1}: '
                            f'{schema_exc}')
                        msgs.append({'role': 'assistant', 'content': content})
                        msgs.append({
                            'role': 'user',
                            'content': (
                                f'Your JSON plan is invalid: {schema_exc}\n'
                                '\n'
                                'RULES — "plan" must be a single node with "type" at the TOP level:\n'
                                '  WRONG: {"plan":{"actions":[{"type":"..."}]}}\n'
                                '  WRONG: tool names (check_occupancy, get_positions) are not plan types\n'
                                '  Valid types: mapf | formation | disband | idle | sequence | parallel\n'
                                '\n'
                                'Correct examples:\n'
                                '  Move robots:  {"type":"mapf","robot_ids":[12,13,14,15],"goals":[[-9.5,4.0],[-9.5,4.0],[-9.5,4.0],[-9.5,4.0]],"reason":"orange to medbay"}\n'
                                '  Stop all:     {"type":"idle","reason":"operator stop"}\n'
                                '  Two steps:    {"type":"sequence","steps":[{"type":"mapf",...},{"type":"formation",...}]}\n'
                                '\n'
                                'Output the corrected complete JSON object now.'
                            ),
                        })
                        continue
                self._emit_reply_streaming(content, goal_handle)
                return content

            # Tool calls — execute and loop
            calls = terminal['calls']
            self.get_logger().info(
                f'tool_loop iter={iteration}: calling tools '
                f'{[c["name"] for c in calls]}')
            msgs.append(_build_tool_use_assistant_message(calls))

            for call in calls:
                name    = call['name']
                args    = call['arguments']
                call_id = call.get('call_id', '')
                try:
                    tool_result = await asyncio.wait_for(
                        self._tool_executor.call(name, args),
                        timeout=max(self._timeout, 10.0),
                    )
                except Exception as exc:
                    tool_result = {'error': str(exc)}
                result_str = json.dumps(tool_result, ensure_ascii=False)
                msgs.append(_build_tool_result_message(call_id, result_str))

        raise RuntimeError(
            f'tool loop exceeded {self._tool_max_iterations} iterations '
            'without producing a final text response'
        )

    async def _stream_plain(self, messages: list[dict], goal_handle) -> str:
        """Non-tool streaming path: collect the full response, surfacing
        reasoning tokens live and the parsed reply via _emit_reply_streaming.
        """
        full = ''
        schema = PLAN_RESPONSE_SCHEMA if self._structured_output_enabled else None
        self._prepare_llm_context_budget(
            messages,
            extra=schema,
            phase='plain',
        )
        async for chunk in self._llm.stream(messages, response_format=schema):
            if not chunk:
                continue
            full += chunk
            if self._stream_reasoning:
                self._publish_fb(goal_handle, stage='thinking', chunk=chunk)
        self.get_logger().info(
            f'plain stream: {len(full)} chars, has_json={"{" in full}')
        self._emit_reply_streaming(full, goal_handle)
        return full

    def _emit_reply_streaming(self, full_raw: str, goal_handle) -> None:
        """Extract the 'reply' field from full_raw and emit as stage='streaming'.

        Runs the same BEFORE/IN_REPLY/AFTER state machine as _stream_reply, but
        operates on a complete string rather than a live stream.
        """
        BEFORE, IN_REPLY, AFTER = 0, 1, 2
        state = BEFORE
        in_escape = False
        scan = ''
        MARKERS = ('"reply": "', '"reply":"')
        ESCAPES = {
            'n': '\n', 't': '\t', 'r': '\r',
            'b': '\b', 'f': '\f',
            '"': '"', '\\': '\\', '/': '/',
        }
        cap = max(len(m) for m in MARKERS)
        emit_buf = ''

        for c in full_raw:
            if state == BEFORE:
                scan += c
                if len(scan) > cap:
                    scan = scan[-cap:]
                if any(scan.endswith(m) for m in MARKERS):
                    state = IN_REPLY
                    scan = ''
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

    def _formation_prestage_hook(self, formation_node: dict) -> dict | None:
        """Auto-stage out-of-tolerance followers when the LLM emits a bare
        formation leaf.

        Primary source: RobotPoseCache (live odom subscriptions).
        Fallback: _last_runtime_context (fetched at the start of this turn)
        — used when pose_cache has no data yet (QoS mismatch, wrong topic,
        or node just started).

        Returns None only when the leader pose is genuinely unavailable or
        all followers are already within tolerance.
        """
        fid = formation_node.get('formation_id', '?')

        # ── Primary: live odom via pose_cache ──────────────────────────
        snapshot: dict = {}
        if self._pose_cache is not None:
            snapshot = self._pose_cache.snapshot(
                stale_threshold_ms=int(self._context_config.pose_stale_ms))

        leader_ns = formation_node.get('leader_ns', '')
        leader_id = int(leader_ns.split('_')[1]) if (
            leader_ns.startswith('robot_')
            and leader_ns.split('_')[1].isdigit()) else None

        if leader_id is not None and leader_id not in snapshot:
            # pose_cache has no entry → try fallback
            ctx_robots = self._last_runtime_context.get('robots') or {}
            for k, v in ctx_robots.items():
                try:
                    rid = int(k)
                except (TypeError, ValueError):
                    continue
                if rid not in snapshot:
                    snapshot[rid] = v

        if leader_id is not None and leader_id not in snapshot:
            self.get_logger().warning(
                f'formation_prestage: leader {leader_ns} not in pose snapshot '
                f'and not in runtime_context — skipping auto-stage for {fid!r}')
            return None

        result = compute_formation_staging(
            formation_node,
            snapshot,
            tolerance_m=self._formation_tolerance_m,
        )
        if result is None and leader_id is not None and leader_id in snapshot:
            # Hook ran but returned None — either stale leader or all in tolerance
            leader = snapshot[leader_id]
            if leader.get('stale'):
                self.get_logger().warning(
                    f'formation_prestage: leader {leader_ns} pose is stale '
                    f'({leader.get("stale_ms", "?")}ms) — skipping auto-stage '
                    f'for {fid!r}')
        return result

    async def _execute_plan(self, plan: dict) -> tuple[bool, dict | None]:
        """Run a plan via the shared sender; return (ok, failure_info)."""
        executor = PlanExecutor(
            send_fn=self._sender.send,
            log_fn=lambda m: self.get_logger().info(f'executor: {m}'),
            formation_prestage_hook=self._formation_prestage_hook,
            plan_guard_hook=self._formation_guard_hook,
        )
        ok = await executor.run(plan)
        if ok:
            return True, None
        failure = dict(executor.guard_failure or self._sender.last_failure() or {})
        # If sender did not record one (executor refused to send for some
        # other reason) fall back to the leaf type from the executor.
        if not failure and executor.failed_leaf:
            failure = {
                'leaf_type': str(executor.failed_leaf.get('type') or '?'),
                'last_error': 'plan execution failed (no sender detail)',
                'failed_at_phase': 'unknown',
                'action_status': '',
            }
        return False, failure

    def _formation_guard_hook(self, plan: dict) -> tuple[dict, dict | None]:
        status = None
        try:
            status = self._tool_executor.formations_status_snapshot()
        except Exception:
            status = None
        if status is None:
            status = (self._last_runtime_context or {}).get('formations')
        return guard_plan_for_active_formations(
            plan,
            status,
            log_fn=lambda m: self.get_logger().info(m),
        )

    async def _get_targeted_runtime_context(
        self,
        failure: dict,
        *,
        cached: dict,
    ) -> dict:
        """Refresh runtime context after a failure, with stale fallback."""
        get_targeted = getattr(
            self._context_provider, 'get_targeted_context', None)
        if get_targeted is None:
            # Provider doesn't support targeting (none/ros_readonly) — just
            # re-fetch the base context.
            return await self._get_runtime_context()
        try:
            context = await asyncio.wait_for(
                get_targeted(failure),
                timeout=max(0.1, float(self._context_config.timeout_sec)),
            )
        except Exception as exc:
            context = dict(cached or {})
            warnings = list(context.get('warnings') or [])
            warnings.append(
                f'targeted MCP refresh failed; using stale snapshot: {exc}')
            warnings.append('mcp_stale: true')
            context['warnings'] = warnings
            context['source'] = 'mcp_readonly_remediation_stale'
        return bound_context(context, self._context_config.max_chars)

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
            warnings = context.get('warnings') or []
            self.get_logger().info(
                f'Chat runtime context source={source} warnings={warnings}')
        return context

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

    def _succeed(self, gh, result):
        self._publish_fb(gh, stage='done')
        result.success = True
        if not result.info:
            result.info = ''
        gh.succeed()
        return result

    def _finalize_help(
        self,
        gh,
        result,
        last_reply: str,
        last_plan: dict,
        *,
        trigger: str,
        reason: str,
        last_failure: dict | None = None,
    ):
        """Deliver the LLM's question (or our synthesized fallback) to the
        operator instead of treating the turn as a hard failure.

        Sets ``success=True, plan_executed=False`` so panel callers see the
        turn as completed-with-question rather than crashed, and prefixes
        the displayed reply with a short bilingual banner whenever the LLM
        did not itself emit a ``needs_help:`` idle leaf.
        """
        failure = last_failure or {}
        leaf = failure.get('leaf_type', '?')
        err  = str(failure.get('last_error', '') or '')[:240]
        if not is_help_request(last_plan):
            banner = (f'Нужна помощь оператора: {leaf} — {err}\n'
                      f'Operator help required: {leaf} — {err}\n\n')
            last_reply = banner + (last_reply or '')

        info = f'needs_help: {reason}'
        if leaf != '?':
            info += f' (leaf={leaf})'

        result.final_reply   = last_reply
        result.plan_json     = json.dumps(last_plan, ensure_ascii=False)
        result.plan_executed = False
        result.success       = True
        result.info          = info

        self._publish_fb(gh, stage='needs_help', detail=info)
        self._publish_event(
            channel=LlmEvent.CHANNEL_USER,
            trigger=trigger,
            output='needs_help',
            reason=info,
        )
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
