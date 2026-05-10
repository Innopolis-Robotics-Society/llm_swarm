"""Channel 3 — natural-language operator chat interface."""

import asyncio
import json
import logging
import math
import os
import re
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState

from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.plan_executor import PlanExecutor, parse_plan
from iros_llm_orchestrator.common.user_prompt import (
    build_user_prompt, build_bt_event_prompt, load_map_config)

MAX_HISTORY   = 8
CLUSTER_SPACE = 1.5
MIN_GOAL_DIST = 1.0

BOT='BOT'; THK='🧠'; WRN='⚠ '; OK='✓'; ERR='✗'; ARR='→'


# ---------------------------------------------------------------------------
# Goal utilities
# ---------------------------------------------------------------------------

def _spread_goals(goals: list, spacing: float = CLUSTER_SPACE) -> list:
    n = len(goals)
    if n <= 1:
        return goals
    visited = [False] * n
    groups: list[list[int]] = []
    for i in range(n):
        if visited[i]:
            continue
        grp = [i]; visited[i] = True
        for j in range(i+1, n):
            if not visited[j]:
                dx = goals[j][0]-goals[i][0]; dy = goals[j][1]-goals[i][1]
                if math.sqrt(dx*dx+dy*dy) < MIN_GOAL_DIST:
                    grp.append(j); visited[j] = True
        groups.append(grp)
    result = [list(g) for g in goals]
    for grp in groups:
        k = len(grp)
        if k == 1:
            continue
        cx = sum(goals[i][0] for i in grp)/k
        cy = sum(goals[i][1] for i in grp)/k
        if k <= 8:
            r = spacing/(2*math.sin(math.pi/k)) if k > 2 else spacing/2
            for idx, rid in enumerate(grp):
                a = 2*math.pi*idx/k
                result[rid] = [round(cx+r*math.cos(a),2), round(cy+r*math.sin(a),2)]
        else:
            cols = max(1, round(math.sqrt(k)))
            for idx, rid in enumerate(grp):
                col=idx%cols; row=idx//cols
                result[rid] = [round(cx+(col-(cols-1)/2)*spacing,2),
                                round(cy+row*spacing,2)]
    return result


def _clamp_goals(goals: list, cfg: dict) -> list:
    b = cfg.get('bounds', {})
    mx, Mx = b.get('x_min', -1e9)+0.5, b.get('x_max', 1e9)-0.5
    my, My = b.get('y_min', -1e9)+0.5, b.get('y_max', 1e9)-0.5
    return [[round(max(mx,min(Mx,g[0])),2), round(max(my,min(My,g[1])),2)]
            for g in goals]


def _postprocess_plan(node: dict, map_cfg: dict) -> dict:
    """Recursively post-process plan nodes.

    - mapf: clamp + spread goals so robots don't pile on one point.
    - formation: warn if requested outside known formation zones.
    - sequence/parallel: recurse into steps.
    """
    t = node['type']
    if t in ('sequence', 'parallel'):
        node['steps'] = [_postprocess_plan(s, map_cfg) for s in node['steps']]
    elif t == 'mapf' and 'goals' in node:
        # Clamp first so spread happens around an in-bounds centre,
        # then clamp again as safety net for large spread radii.
        node['goals'] = _clamp_goals(
            _spread_goals(_clamp_goals(node['goals'], map_cfg)),
            map_cfg)
    elif t == 'formation':
        # Warn if the formation is requested somewhere with insufficient
        # clearance. The actual enforcement is in formation_manager_node.
        fzones = map_cfg.get('formation_zones', [])
        if fzones:
            # Check if the formation_id matches a known safe zone name
            # (operator-named formations bypass this check — they may be
            # at a custom location chosen by the LLM).
            safe_ids = {z.get('name', '') for z in fzones}
            fid = node.get('formation_id', '')
            if fid and fid not in safe_ids:
                min_radius = min((z.get('radius', 0) for z in fzones), default=0)
                node.setdefault('_hint',
                    f'Formation "{fid}" is not in a pre-approved zone '
                    f'(min clearance ~{min_radius:.1f} m). '
                    f'Approved zones: {sorted(safe_ids)}')
    return node


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

def _parse_response(raw: str) -> tuple[str, dict]:
    text = raw.strip()
    start = text.find('{')
    if start == -1:
        raise ValueError('no JSON object in LLM output')
    # Track string state so a `}` inside a JSON string (perfectly legal in
    # the `reply` field) doesn't close the outer object early.
    depth = 0; end = -1
    in_string = False; escaped = False
    for i in range(start, len(text)):
        ch = text[i]
        if in_string:
            if escaped:
                escaped = False
            elif ch == '\\':
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                end = i + 1; break
    if end == -1:
        raise ValueError('JSON object not closed')
    try:
        obj = json.loads(text[start:end])
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc}') from exc
    reply = obj.get('reply', text[:start].strip()) or text[:start].strip()
    plan  = parse_plan(obj)
    return reply, plan


# ---------------------------------------------------------------------------
# Session logger
# ---------------------------------------------------------------------------

def _resolve_log_dir(raw: str) -> str:
    """Resolve the log_dir parameter to an absolute path.

    Special value "package":
        Resolves to log/ inside the iros_llm_orchestrator source package,
        i.e. the directory that contains this very file (user_chat_node.py)
        lives at  .../iros_llm_orchestrator/iros_llm_orchestrator/user_chat_node.py
        so log/ is at  .../iros_llm_orchestrator/log/

    Any other value:
        Treated as a filesystem path with tilde expansion.
    """
    if raw.strip() == 'package':
        # __file__ = .../iros_llm_orchestrator/iros_llm_orchestrator/user_chat_node.py
        # parent   = .../iros_llm_orchestrator/iros_llm_orchestrator/
        # parent^2 = .../iros_llm_orchestrator/   ← package root
        pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg_root, 'log')
    return os.path.expanduser(raw)


def _make_session_logger(log_dir: str) -> logging.Logger:
    """Create a file logger for this chat session."""
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    ts    = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    path  = os.path.join(log_dir, f'chat_{ts}.log')
    logger = logging.getLogger(f'user_chat_{ts}')
    logger.setLevel(logging.DEBUG)
    fh = logging.FileHandler(path, encoding='utf-8', errors='replace')
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        '%(asctime)s  %(levelname)-7s  %(message)s',
        datefmt='%H:%M:%S'))
    logger.addHandler(fh)
    logger.propagate = False
    logger.info(f'Session started. Log: {path}')
    return logger


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class UserChatNode(Node):
    def __init__(self):
        super().__init__('user_chat')

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
        self.declare_parameter('map_name',         'cave')
        self.declare_parameter('log_enabled',      True)
        self.declare_parameter('log_dir',          '~/.ros/user_chat_logs')
        self.declare_parameter('step_timeout_sec', 120.0)

        self._timeout      = float(self.get_parameter('timeout_sec').value)
        self._step_timeout = float(self.get_parameter('step_timeout_sec').value)
        self._map_name     = self.get_parameter('map_name').value
        log_enabled = bool(self.get_parameter('log_enabled').value)
        log_dir     = _resolve_log_dir(self.get_parameter('log_dir').value)

        self._slog = _make_session_logger(log_dir) if log_enabled else logging.getLogger('null')
        self._slog.info(f'map={self._map_name}  '
                        f'mode={self.get_parameter("llm_mode").value}  '
                        f'model={self.get_parameter("llm_model").value}  '
                        f'log_enabled={log_enabled}')

        self._llm = get_llm_client(
            mode=self.get_parameter('llm_mode').value,
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
        except Exception as e:
            self.get_logger().warn(f'Map config load failed: {e}')
            self._map_cfg = {}

        self._cmd_client = ActionClient(self, LlmCommand, '/llm/command')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(BTState, '/bt/state', self._on_bt_state, qos)

        self._last_status        = 'OK'
        # _last_mode and _mode_seq are written from the ROS callback thread
        # and read from the asyncio loop thread — protect with a Lock. Empty
        # string is a sentinel meaning "no /bt/state seen yet" so the first
        # tick always registers as a transition. _mode_seq is a monotonic
        # transition counter; _send_leaf snapshots it before sending a goal
        # so it can detect the transition triggered by THIS leaf rather than
        # depending on a global "ever busy" flag (which races against the
        # previous step's idle->busy->idle cycle).
        self._mode_lock          = threading.Lock()
        self._last_mode          = ''
        self._mode_seq           = 0
        self._history: list[dict] = []
        self._bt_event_analyzing = False
        # /llm/command server readiness is checked once per session, off the
        # asyncio loop thread, so wait_for_server() never freezes the UI.
        self._cmd_server_ready   = False

        self._loop = asyncio.new_event_loop()
        # Plan handling is serialised on the asyncio loop: only one plan is
        # executed at a time, but a newer command preempts (cancels) the
        # currently-running plan task. _stdin_loop schedules via
        # _schedule_handle(); _handle takes the lock so the new task waits
        # for the cancelled one to unwind cleanly before starting.
        self._plan_lock          = asyncio.Lock()
        self._current_plan_task  = None  # concurrent.futures.Future
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        self._print_banner()

    # ------------------------------------------------------------------
    # Thread-safe mode accessors
    # ------------------------------------------------------------------

    def _get_mode(self) -> str:
        with self._mode_lock:
            return self._last_mode

    def _get_mode_state(self) -> tuple[str, int]:
        """Atomically read (mode, seq) so callers see a consistent pair."""
        with self._mode_lock:
            return self._last_mode, self._mode_seq

    # ------------------------------------------------------------------
    # BT feedback
    # ------------------------------------------------------------------

    def _on_bt_state(self, msg: BTState):
        # Track @mode transitions — replaces the legacy /fleet/mode topic.
        # Do the compare-and-swap under the lock so a re-entrant callback
        # cannot double-count a transition or interleave seq increments.
        with self._mode_lock:
            prev_mode = self._last_mode
            transitioned = msg.mode != prev_mode
            if transitioned:
                self._last_mode = msg.mode
                self._mode_seq += 1
                seq = self._mode_seq
        if transitioned:
            self._slog.debug(f'mode: {prev_mode!r} → {msg.mode!r} (seq={seq})')
            if prev_mode and prev_mode != 'idle' and msg.mode == 'idle':
                self._out(f'\n  {OK}  Step finished → idle')
                self._prompt()

        if msg.action_status == self._last_status:
            return
        prev = self._last_status
        self._last_status = msg.action_status

        if msg.action_status in ('WARN', 'ERROR'):
            self._out(f'\n  {WRN} [{msg.active_action}] {msg.action_status}: '
                      f'{msg.last_error}')
            self._slog.warning(
                f'BT event: [{msg.action_status}] {msg.active_action}: '
                f'{msg.last_error}')
            # Atomic check-and-set so back-to-back WARN/ERROR messages from
            # the ROS executor thread don't both pass the gate and schedule
            # two concurrent _handle_bt_event coroutines.
            with self._mode_lock:
                if self._bt_event_analyzing:
                    self._out('       (LLM analysis already in progress)')
                    self._prompt(); return
                self._bt_event_analyzing = True
            self._out(f'  {THK} Analyzing...')
            asyncio.run_coroutine_threadsafe(
                self._handle_bt_event(msg), self._loop)

        elif prev in ('WARN', 'ERROR') and msg.action_status == 'OK':
            self._out(f'\n  {OK}  Status recovered → OK')
            self._slog.info('BT status recovered to OK')
            self._prompt()

    async def _handle_bt_event(self, msg: BTState):
        try:
            messages = build_bt_event_prompt(msg, history=self._history)
            self._out(f'  {BOT} ')
            try:
                raw = await asyncio.wait_for(
                    self._stream_text(messages), timeout=self._timeout)
            except asyncio.TimeoutError:
                self._out(f'\n  {ERR}  LLM analysis timeout')
                self._slog.error('BT event LLM analysis timeout')
                return
            except Exception as exc:
                self._out(f'\n  {ERR}  {exc}')
                self._slog.error(f'BT event LLM error: {exc}')
                return
            self._out('')
            safe_raw = raw.encode('utf-8', errors='replace').decode('utf-8')
            self._slog.info(f'BT event LLM response: {safe_raw[:200]}')
            event_text = (f'[BT {msg.action_status}] '
                          f'{msg.active_action}: {msg.last_error}')
            self._history.append({'role': 'user',      'content': event_text})
            self._history.append({'role': 'assistant', 'content': safe_raw})
            self._trim_history()
        finally:
            with self._mode_lock:
                self._bt_event_analyzing = False
            self._prompt()

    # ------------------------------------------------------------------
    # Stdin
    # ------------------------------------------------------------------

    def _stdin_loop(self):
        while True:
            try:
                line = sys.stdin.readline()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                break
            text = line.strip()
            if not text:
                self._prompt(); continue
            if text.lower() in ('quit', 'exit', 'q'):
                self._out('Bye.'); rclpy.shutdown(); break
            self._schedule_handle(text)

    def _schedule_handle(self, text: str):
        """Schedule a new _handle, preempting any currently-running plan.

        Newer commands always win — typing "stop" while a plan is mid-run
        cancels the previous task, then the new one acquires _plan_lock and
        executes. This avoids interleaved /llm/command goals and history
        mutations from concurrent _handle coroutines.
        """
        prev = self._current_plan_task
        if prev is not None and not prev.done():
            self._slog.info('cancelling previous plan task — newer command arrived')
            prev.cancel()
        self._current_plan_task = asyncio.run_coroutine_threadsafe(
            self._handle(text), self._loop)

    # ------------------------------------------------------------------
    # User command pipeline
    # ------------------------------------------------------------------

    async def _handle(self, text: str):
        async with self._plan_lock:
            try:
                await self._handle_body(text)
            except asyncio.CancelledError:
                # A newer command preempted us. Don't fight it — let the new
                # _handle acquire the lock and proceed.
                self._out(f'\n  {WRN} plan cancelled — running newer command')
                self._slog.info('plan task cancelled — newer command arrived')

    async def _handle_body(self, text: str):
        safe_text = text.encode('utf-8', errors='replace').decode('utf-8')
        self._slog.info(f'USER: {safe_text}')

        self._out(f'\n  {THK} ')
        messages = build_user_prompt(text, history=self._history,
                                     map_name=self._map_name)

        try:
            full_raw = await asyncio.wait_for(
                self._stream_command(messages), timeout=self._timeout)
        except asyncio.TimeoutError:
            self._out(f'\n  {ERR}  LLM backend timeout')
            self._slog.error('LLM timeout')
            self._prompt(); return
        except Exception as exc:
            self._out(f'\n  {ERR}  LLM error: {exc}')
            self._slog.error(f'LLM error: {exc}')
            self._prompt(); return

        print()  # newline after streamed reply
        self._slog.debug(f'LLM raw ({len(full_raw)} chars):\n{full_raw}')

        try:
            reply, plan = _parse_response(full_raw)
        except ValueError as exc:
            self._out(f'  {ERR}  Parse error: {exc}')
            self._out(f'       Raw: {full_raw[:400]}')
            self._slog.error(f'Parse error: {exc}\nRaw: {full_raw}')
            self._prompt(); return

        self._slog.info(f'Plan: {json.dumps(plan, ensure_ascii=False)}')
        plan = _postprocess_plan(plan, self._map_cfg)
        self._describe_plan(plan)

        self._history.append({'role': 'user', 'content': text})
        self._history.append({'role': 'assistant', 'content': full_raw})
        self._trim_history()

        executor = PlanExecutor(
            send_fn=self._send_leaf,
            log_fn=lambda m: (print(f'  {m}', flush=True),
                              self._slog.debug(f'executor: {m}'))[0],
        )
        ok = await executor.run(plan)
        result_msg = 'Plan complete.' if ok else 'Plan failed or was interrupted.'
        self._out(f'\n  {OK if ok else ERR}  {result_msg}')
        self._slog.info(f'Plan result: {result_msg}')
        self._prompt()

    # ------------------------------------------------------------------
    # Leaf execution
    # ------------------------------------------------------------------

    async def _ensure_cmd_server(self) -> bool:
        """One-shot, off-loop readiness probe for /llm/command.

        wait_for_server is blocking; calling it from the asyncio loop thread
        freezes the chat UI. We pay the 3s probe at most once per session
        and cache the result.
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
        self._slog.debug(f'send_leaf: type={t} ids={command.get("robot_ids",[])}')

        if t == 'idle':
            return await self._send_bt_goal(mode='idle',
                                            reason=command.get('reason', ''))

        if not await self._ensure_cmd_server():
            self._out(f'  {ERR}  /llm/command not available.')
            self._slog.error('send_leaf: /llm/command server not available')
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
        # leftover transition from the previous step. Without this snapshot,
        # the second leaf in a sequence races: the BT publishes idle→<mode>
        # while we are still inside `await get_result_async`, and a stale
        # "ever busy" flag would otherwise be reset right after, leaving the
        # subsequent <mode>→idle transition undetectable (the BTState callback
        # only fires on actual mode changes).
        seq_at_send = self._get_mode_state()[1]

        # Step 1: send goal and wait for BT to accept it.
        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self._out(f'  {ERR}  BT rejected goal (say "stop" to cancel current mission).')
            self._slog.warning('send_leaf: goal rejected by BT')
            return False

        result = await handle.get_result_async()
        if not result.result.success:
            self._out(f'  {ERR}  BT error: {result.result.info}')
            self._slog.error(f'send_leaf: BT error: {result.result.info}')
            return False

        self._slog.debug(
            f'send_leaf: goal accepted and applied (seq_at_send={seq_at_send})')

        # Phase 1: wait up to 1.5s for the BT to actually pick up our goal.
        # We treat seeing _any_ transition past seq_at_send as proof the BT
        # processed our blackboard write. Three outcomes:
        #   * mode now non-idle  → mission running, drop into Phase 2.
        #   * mode flipped to idle inside the window → fast-complete leaf.
        #   * no transition at all in 1.5s → ModeDispatch effectively no-op'd
        #     this leaf (e.g. duplicate goal, validation failure inside the
        #     BT). Return success rather than block the whole plan; the
        #     BT-event analyzer surfaces real failures via WARN/ERROR.
        phase1_deadline = time.monotonic() + 1.5
        while time.monotonic() < phase1_deadline:
            mode_now, seq_now = self._get_mode_state()
            if seq_now > seq_at_send:
                if mode_now == 'idle':
                    self._slog.info(
                        f'send_leaf: {t} fast-completed (returned to idle '
                        f'in <1.5s, seq {seq_at_send}->{seq_now})')
                    return True
                self._slog.debug(
                    f'send_leaf: BT entered {mode_now!r} (seq {seq_at_send}->{seq_now})')
                break
            await asyncio.sleep(0.05)
        else:
            self._slog.info(
                f'send_leaf: BT never transitioned after {t} command '
                f'(seq stuck at {seq_at_send}) — treating as no-op success')
            return True

        # Phase 2: wait for the mission to finish (mode → idle).
        deadline = time.monotonic() + self._step_timeout
        while time.monotonic() < deadline:
            if self._get_mode() == 'idle':
                self._slog.debug('send_leaf: mission complete (mode=idle)')
                return True
            await asyncio.sleep(0.1)

        self._out(f'  {ERR}  Timed out waiting for mission to finish '
                  f'({self._step_timeout:.0f}s).')
        self._slog.error(f'send_leaf: step timeout after {self._step_timeout}s')
        return False

    async def _send_bt_goal(self, mode: str, reason: str = '') -> bool:
        if not await self._ensure_cmd_server():
            return False
        goal = LlmCommand.Goal()
        goal.mode = mode; goal.reason = reason
        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            return False
        result = await handle.get_result_async()
        return result.result.success

    # ------------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------------

    async def _stream_command(self, messages: list[dict]) -> str:
        """Stream LLM response, printing the reply text in real time.

        The model outputs: {"reply": "some text", "plan": {...}}
        We detect when we're inside the "reply" value and print those
        characters live. Once we hit the end of the reply value (closing
        quote before "plan"), we go silent and buffer the rest.

        State machine:
          BEFORE_REPLY  — scanning for :"  after "reply"
          IN_REPLY      — printing characters, watching for unescaped "
          AFTER_REPLY   — buffering silently

        Inside IN_REPLY we honour JSON string escapes so the user sees
        unescaped output (e.g. `\\"` → `"`, `\\n` → newline) instead of raw
        backslash sequences.
        """
        BEFORE_REPLY, IN_REPLY, AFTER_REPLY = 0, 1, 2
        state = BEFORE_REPLY
        # When True the next char in IN_REPLY is the body of a JSON escape.
        in_escape = False

        full  = ''
        # Buffer of chars we've seen while in BEFORE_REPLY, used to detect
        # the "reply": " sequence reliably across chunk boundaries.
        scan  = ''
        # Marker we look for to enter IN_REPLY state. Also try the no-space
        # form because some models emit compact JSON.
        MARKERS = ('"reply": "', '"reply":"')

        ESCAPES = {'n': '\n', 't': '\t', 'r': '\r',
                   'b': '\b', 'f': '\f',
                   '"': '"', '\\': '\\', '/': '/'}

        print(f'{BOT} ', end='', flush=True)

        async for chunk in self._llm.stream(messages):
            full += chunk

            if state == AFTER_REPLY:
                continue  # just accumulate, don't print

            for ch in chunk:
                if state == BEFORE_REPLY:
                    scan += ch
                    # Keep only the last len(longest_marker) chars to avoid
                    # unbounded growth.
                    cap = max(len(m) for m in MARKERS)
                    if len(scan) > cap:
                        scan = scan[-cap:]
                    if any(scan.endswith(m) for m in MARKERS):
                        state = IN_REPLY
                        scan  = ''
                        in_escape = False

                elif state == IN_REPLY:
                    if in_escape:
                        decoded = ESCAPES.get(ch, ch)
                        print(decoded, end='', flush=True)
                        in_escape = False
                    elif ch == '\\':
                        # Buffer the backslash — wait for the next char to
                        # decide whether to print an escape or a literal.
                        in_escape = True
                    elif ch == '"':
                        # Unescaped closing quote — end of reply value.
                        state = AFTER_REPLY
                    else:
                        print(ch, end='', flush=True)

        return full

    async def _stream_all(self, messages: list[dict]) -> str:
        """Buffer full response without any output."""
        full = ''
        async for chunk in self._llm.stream(messages):
            full += chunk
        return full

    async def _stream_text(self, messages: list[dict]) -> str:
        full = ''
        async for chunk in self._llm.stream(messages):
            print(chunk, end='', flush=True)
            full += chunk
        return full

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def _describe_plan(self, node: dict, depth: int = 0):
        ind = '  ' * (depth + 1)
        t = node['type']
        if t == 'sequence':
            print(f'{ind}sequence ({len(node["steps"])} steps):')
            for s in node['steps']: self._describe_plan(s, depth+1)
        elif t == 'parallel':
            print(f'{ind}parallel → will be merged/flattened:')
            for s in node['steps']: self._describe_plan(s, depth+1)
        elif t == 'mapf':
            ids, goals = node['robot_ids'], node['goals']
            print(f'{ind}{ARR} MAPF ({len(ids)}r) — {node.get("reason","")}')
            for rid, g in zip(ids, goals):
                print(f'{ind}   robot_{rid:<3}  {ARR}  ({g[0]:.2f}, {g[1]:.2f})')
        elif t == 'formation':
            print(f'{ind}{ARR} FORMATION {node.get("formation_id","")} '
                  f'leader={node.get("leader_ns","")} — {node.get("reason","")}')
        elif t == 'idle':
            print(f'{ind}{ARR} STOP ALL')

    def _out(self, msg: str):
        print(msg, flush=True)

    def _prompt(self):
        print('\n> ', end='', flush=True)

    def _trim_history(self):
        if len(self._history) > MAX_HISTORY * 2:
            self._history = self._history[-MAX_HISTORY * 2:]

    def _print_banner(self):
        desc = self._map_cfg.get('description', self._map_name)
        if isinstance(desc, str):
            desc = desc.strip().split('\n')[0][:60]
        log_enabled = bool(self.get_parameter('log_enabled').value)
        log_dir_resolved = _resolve_log_dir(self.get_parameter('log_dir').value)
        log_info = (f'logs → {log_dir_resolved}'
                    if log_enabled else 'logging disabled')
        model = getattr(self._llm, 'model', self.get_parameter('llm_model').value)
        print(f'\n  {BOT}  Swarm chat  |  model: {model}  |  map: {self._map_name}')
        print(f'       {desc}')
        print(f'       {log_info}')
        print('\n  Examples:')
        print('    yellow to east, green to west')
        print('    cyan to magenta home, magenta to cyan home')
        print('    cyan go to hub, then form a line')
        print('    stop')
        print('\n  "quit" to exit.\n')
        self._prompt()


def _inject_default_config(args: list | None) -> list:
    """Prepend --params-file <orchestrator.yaml> if not already in args.

    Looks for the config in order:
      1. Already in args — do nothing.
      2. ament share directory (installed package).
      3. Next to this file's package root (running from source).
    """
    args = list(args or [])
    if '--params-file' in args:
        return args

    candidates: list[str] = []

    # 1. Installed share
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('iros_llm_orchestrator')
        candidates.append(os.path.join(share, 'config', 'orchestrator.yaml'))
    except Exception:
        pass

    # 2. Source tree relative to this file
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidates.append(os.path.join(pkg_root, 'config', 'orchestrator.yaml'))

    for path in candidates:
        if os.path.isfile(path):
            args = ['--ros-args', '--params-file', path] + args
            print(f'  [config] {path}', flush=True)
            return args

    return args


def main(args=None):
    args = _inject_default_config(args)
    rclpy.init(args=args)
    node = UserChatNode()
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
