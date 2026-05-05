"""Channel 3 over ROS — /llm/chat action server.

Mirrors user_chat_node's pipeline (build_user_prompt → stream → parse →
PlanExecutor) but exposes it as a ROS action so the RViz panel and other
clients can drive the LLM without owning stdin.

Two non-obvious bits the panel relies on:

* Reply streaming. Feedback chunks carry only the body of the JSON
  "reply" string, not the surrounding envelope. The same state machine
  user_chat_node uses for live printing runs here, so the panel doesn't
  display raw `{"reply":"..."}` JSON to the operator.

* Mission completion. PlanExecutor leaves are sent via /llm/command,
  but LlmCommand.Result fires as soon as ModeDispatch *accepts* the
  goal — the actual mission is still running. We wait for the BT to
  return @mode→idle by watching /bt/state, otherwise the next leaf in
  a sequence would race the previous one.
"""

import asyncio
import json
import threading
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from iros_llm_swarm_interfaces.action import LlmChat, LlmCommand
from iros_llm_swarm_interfaces.msg import BTState, LlmEvent

from iros_llm_orchestrator.common.plan_executor import PlanExecutor
from iros_llm_orchestrator.common.user_prompt import build_user_prompt, load_map_config
from iros_llm_orchestrator.local.ollama_client import OllamaClient
from iros_llm_orchestrator.user_chat_node import _parse_response, _postprocess_plan


class ChatServer(Node):
    def __init__(self):
        super().__init__('llm_chat_server')

        self.declare_parameter('llm_endpoint',     'http://localhost:11434/api/chat')
        self.declare_parameter('llm_model',        'qwen2.5:14b')
        self.declare_parameter('llm_max_tokens',   768)
        self.declare_parameter('llm_temperature',  0.1)
        self.declare_parameter('timeout_sec',      30.0)
        self.declare_parameter('step_timeout_sec', 120.0)
        self.declare_parameter('map_name',         'cave')

        self._timeout      = float(self.get_parameter('timeout_sec').value)
        self._step_timeout = float(self.get_parameter('step_timeout_sec').value)
        self._map_name     = self.get_parameter('map_name').value

        self._ollama = OllamaClient(
            endpoint=self.get_parameter('llm_endpoint').value,
            model=self.get_parameter('llm_model').value,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        try:
            self._map_cfg = load_map_config(self._map_name)
        except Exception as exc:
            self.get_logger().warn(f'Map config load failed: {exc}')
            self._map_cfg = {}

        # /llm/command client — sends individual leaves to the BT.
        self._cmd_client = ActionClient(self, LlmCommand, '/llm/command')
        self._cmd_server_ready = False

        # /bt/state — used to wait for mission completion. Same mode-seq
        # protocol as user_chat_node._send_leaf.
        self._mode_lock = threading.Lock()
        self._last_mode = ''
        self._mode_seq  = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(BTState, '/bt/state', self._on_bt_state, qos)

        # /llm/events publisher — channel 3 emits one event per turn.
        self._event_pub = self.create_publisher(LlmEvent, '/llm/events', 10)

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
        self.get_logger().info('LlmChatServer ready on /llm/chat')

    # ------------------------------------------------------------------
    # /bt/state — track mode transitions for leaf completion
    # ------------------------------------------------------------------

    def _on_bt_state(self, msg: BTState):
        # Compare-and-swap under the lock so re-entrant ROS callbacks can't
        # double-count a transition or interleave seq increments.
        with self._mode_lock:
            if msg.mode != self._last_mode:
                self._last_mode = msg.mode
                self._mode_seq += 1

    def _get_mode_state(self) -> tuple[str, int]:
        with self._mode_lock:
            return self._last_mode, self._mode_seq

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
        messages = build_user_prompt(req.user_message, history=[],
                                     map_name=self._map_name)
        self._publish_fb(goal_handle, stage='thinking')

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

        self._publish_fb(goal_handle, stage='parsed', detail=plan_json)
        self._publish_event(channel=LlmEvent.CHANNEL_USER,
                            trigger=req.user_message,
                            output=plan_json,
                            reason=reply)

        # ---- 3. Optional execute ----
        if req.execute_after_planning:
            self._publish_fb(goal_handle, stage='executing')
            executor = PlanExecutor(
                send_fn=self._send_leaf,
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

    # ------------------------------------------------------------------
    # Reply streaming — emit only the JSON "reply" field via feedback
    # ------------------------------------------------------------------

    async def _stream_reply(self, messages, goal_handle) -> str:
        """Stream and forward only "reply" body characters to the panel.

        State machine identical to user_chat_node._stream_command:
          BEFORE  — scanning for `"reply":"`
          IN_REPLY — emit chars; honour JSON escapes; stop at unescaped `"`
          AFTER   — accumulate raw silently for the parser.
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
        async for chunk in self._ollama.stream(messages):
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
    # Leaf execution — send LlmCommand, wait for BT mission completion
    # ------------------------------------------------------------------

    async def _ensure_cmd_server(self) -> bool:
        """One-shot off-loop readiness probe for /llm/command.

        wait_for_server is blocking; calling it directly from the asyncio
        loop thread freezes feedback streaming for up to 3s.
        """
        if self._cmd_server_ready:
            return True
        loop = asyncio.get_running_loop()
        ok = await loop.run_in_executor(
            None, lambda: self._cmd_client.wait_for_server(timeout_sec=3.0))
        self._cmd_server_ready = ok
        return ok

    async def _send_leaf(self, command: dict) -> bool:
        t = command.get('type') or command.get('mode', '')

        if not await self._ensure_cmd_server():
            self.get_logger().error('/llm/command unavailable')
            return False

        goal = LlmCommand.Goal()
        goal.mode         = t
        goal.reason       = command.get('reason', '')
        goal.robot_ids    = [int(r) for r in command.get('robot_ids', [])]
        goal.goals        = [Point(x=float(g[0]), y=float(g[1]), z=0.0)
                             for g in command.get('goals', [])]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns    = command.get('leader_ns', '')
        goal.follower_ns  = list(command.get('follower_ns', []))
        goal.offsets_x    = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y    = [float(o) for o in command.get('offsets_y', [])]

        # Snapshot the mode-transition counter BEFORE sending so we can tell
        # the difference between a transition triggered by THIS leaf and a
        # leftover transition from the previous step in a sequence.
        seq_at_send = self._get_mode_state()[1]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self.get_logger().warn(f'BT rejected {t!r} goal')
            return False

        cmd_result = await handle.get_result_async()
        if not cmd_result.result.success:
            self.get_logger().error(f'BT error on {t!r}: {cmd_result.result.info}')
            return False

        # idle has no mission to wait for — BT goes idle the moment we ack.
        if t == 'idle':
            return True

        # Phase 1: wait up to 1.5s for any mode transition past seq_at_send.
        # Three outcomes:
        #   * non-idle mode now → mission running, drop into Phase 2
        #   * mode flipped to idle inside the window → fast-complete leaf
        #   * no transition at all → ModeDispatch effectively no-op'd (e.g.
        #     duplicate goal). Treat as success rather than block the plan;
        #     the BT-event analyzer would surface real failures anyway.
        phase1_deadline = time.monotonic() + 1.5
        transitioned = False
        while time.monotonic() < phase1_deadline:
            mode_now, seq_now = self._get_mode_state()
            if seq_now > seq_at_send:
                if mode_now == 'idle':
                    return True
                transitioned = True
                break
            await asyncio.sleep(0.05)
        if not transitioned:
            return True

        # Phase 2: wait for mode → idle.
        deadline = time.monotonic() + self._step_timeout
        while time.monotonic() < deadline:
            if self._get_mode_state()[0] == 'idle':
                return True
            await asyncio.sleep(0.1)

        self.get_logger().error(
            f'step timeout on {t!r} after {self._step_timeout:.0f}s')
        return False

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
        # The action ran end-to-end; success flag in the result carries the
        # actual verdict. Panel reads result.success, not the goal state.
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
