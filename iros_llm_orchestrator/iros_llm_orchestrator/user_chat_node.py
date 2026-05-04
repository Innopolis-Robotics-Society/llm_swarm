"""User chat interface — channel 3."""

import asyncio
import json
import math
import re
import sys
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState

from iros_llm_orchestrator.command_parser import parse_llm_command
from iros_llm_orchestrator.user_prompt_builder import build_user_prompt, build_bt_event_prompt

MAX_HISTORY_TURNS = 8
CLUSTER_SPACING   = 1.5
MIN_GOAL_DIST     = 1.0
# Stage warehouse bounds
MAP_X_MIN, MAP_X_MAX = 0.0, 30.0
MAP_Y_MIN, MAP_Y_MAX = 0.0, 30.0

BOT = '🤖'; THK = '🧠'; WRN = '⚠ '; OK = '✓'; ERR = '✗'; ARR = '→'


# ---------------------------------------------------------------------------
# Goal spreading — circular arrangement
# ---------------------------------------------------------------------------

def _spread_goals(goals: list, spacing: float = CLUSTER_SPACING) -> list:
    """Group near-coincident goals and arrange each group in a circle (or grid for large groups)."""
    if len(goals) <= 1:
        return goals

    n = len(goals)
    visited = [False] * n
    groups: list[list[int]] = []

    for i in range(n):
        if visited[i]:
            continue
        group = [i]
        visited[i] = True
        for j in range(i + 1, n):
            if not visited[j]:
                dx = goals[j][0] - goals[i][0]
                dy = goals[j][1] - goals[i][1]
                if math.sqrt(dx * dx + dy * dy) < MIN_GOAL_DIST:
                    group.append(j)
                    visited[j] = True
        groups.append(group)

    result = [list(g) for g in goals]

    for group in groups:
        k = len(group)
        if k == 1:
            continue

        cx = sum(goals[i][0] for i in group) / k
        cy = sum(goals[i][1] for i in group) / k

        if k <= 8:
            # Circular arrangement
            # radius so that adjacent robots are ~spacing apart
            radius = spacing / (2 * math.sin(math.pi / k)) if k > 2 else spacing / 2
            for idx_in_group, robot_idx in enumerate(group):
                angle = 2 * math.pi * idx_in_group / k
                result[robot_idx] = [
                    round(cx + radius * math.cos(angle), 2),
                    round(cy + radius * math.sin(angle), 2),
                ]
        else:
            # Grid for large groups
            cols = max(1, round(math.sqrt(k)))
            for idx_in_group, robot_idx in enumerate(group):
                col = idx_in_group % cols
                row = idx_in_group // cols
                result[robot_idx] = [
                    round(cx + (col - (cols - 1) / 2.0) * spacing, 2),
                    round(cy + row * spacing, 2),
                ]

    return result


def _clamp_goals(goals: list) -> list:
    """Clamp goals to map bounds with a small margin."""
    margin = 0.5
    return [
        [
            round(max(MAP_X_MIN + margin, min(MAP_X_MAX - margin, g[0])), 2),
            round(max(MAP_Y_MIN + margin, min(MAP_Y_MAX - margin, g[1])), 2),
        ]
        for g in goals
    ]


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

def _parse_response(raw: str) -> tuple[str, dict]:
    text = raw.strip()
    fenced = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', text, re.DOTALL)
    if fenced:
        text = fenced.group(1)
    start = text.find('{')
    end   = text.rfind('}')
    if start == -1 or end == -1:
        raise ValueError('no JSON object in LLM output')
    try:
        obj = json.loads(text[start:end + 1])
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc}') from exc
    if 'reply' in obj and 'command' in obj:
        return str(obj['reply']), parse_llm_command(json.dumps(obj['command'], ensure_ascii=False))
    return '', parse_llm_command(text[start:end + 1])


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class UserChatNode(Node):
    def __init__(self):
        super().__init__('user_chat')

        self.declare_parameter('llm_endpoint',    'http://localhost:11434/api/chat')
        self.declare_parameter('llm_model',       'qwen2.5:14b')
        self.declare_parameter('llm_max_tokens',  512)
        self.declare_parameter('llm_temperature', 0.1)
        self.declare_parameter('timeout_sec',     30.0)

        self._endpoint    = self.get_parameter('llm_endpoint').value
        self._model       = self.get_parameter('llm_model').value
        self._max_tokens  = int(self.get_parameter('llm_max_tokens').value)
        self._temperature = float(self.get_parameter('llm_temperature').value)
        self._timeout     = float(self.get_parameter('timeout_sec').value)

        self._cmd_client = ActionClient(self, LlmCommand, '/llm/command')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(BTState, '/bt/state', self._on_bt_state, qos)
        self.create_subscription(String, '/fleet/mode', self._on_fleet_mode, 10)

        self._last_bt_status  = 'OK'
        self._last_fleet_mode = 'idle'
        self._history: list[dict] = []

        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        threading.Thread(target=self._stdin_loop, daemon=True).start()
        self._print_banner()

    # -----------------------------------------------------------------------
    # BT feedback — show to user AND consult LLM
    # -----------------------------------------------------------------------

    def _on_bt_state(self, msg: BTState):
        if msg.action_status != self._last_bt_status:
            prev = self._last_bt_status
            self._last_bt_status = msg.action_status
            if msg.action_status in ('WARN', 'ERROR'):
                self._out(f'\n  {WRN} BT [{msg.mode}/{msg.active_action}]: {msg.last_error}')
                # Consult LLM about the event — runs in background, non-blocking
                asyncio.run_coroutine_threadsafe(
                    self._handle_bt_event(msg), self._loop
                )

    def _on_fleet_mode(self, msg: String):
        mode = msg.data
        if mode != self._last_fleet_mode:
            prev, self._last_fleet_mode = self._last_fleet_mode, mode
            if prev != 'idle' and mode == 'idle':
                self._out(f'\n  {OK}  Mission finished → idle')
                self._prompt()

    # -----------------------------------------------------------------------
    # BT event → LLM analysis (shown inline, added to history)
    # -----------------------------------------------------------------------

    async def _handle_bt_event(self, msg: BTState):
        """Ask LLM to analyse a BT warning and suggest action to the user."""
        messages = build_bt_event_prompt(msg, history=self._history)

        self._out(f'  {THK} Analyzing...')
        try:
            raw = await asyncio.wait_for(
                self._stream_ollama(messages, prefix='  💬 '),
                timeout=self._timeout,
            )
        except Exception as exc:
            self._out(f'  {ERR}  LLM analysis failed: {exc}')
            self._prompt()
            return

        # Add event + analysis to history so user can respond ("отменить", "подождём", etc.)
        event_text = f'[BT {msg.action_status}] {msg.active_action}: {msg.last_error}'
        self._history.append({'role': 'user',      'content': event_text})
        self._history.append({'role': 'assistant', 'content': raw})
        if len(self._history) > MAX_HISTORY_TURNS * 2:
            self._history = self._history[-MAX_HISTORY_TURNS * 2:]

        self._out('')
        self._prompt()

    # -----------------------------------------------------------------------
    # Stdin loop — non-blocking
    # -----------------------------------------------------------------------

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
            if text.lower() in ('quit', 'exit', 'выход', 'q'):
                self._out('Bye.')
                rclpy.shutdown(); break
            asyncio.run_coroutine_threadsafe(self._handle(text), self._loop)

    # -----------------------------------------------------------------------
    # User command pipeline
    # -----------------------------------------------------------------------

    async def _handle(self, text: str):
        self._out(f'\n  {THK} Thinking...\n  {BOT} ')
        messages = build_user_prompt(text, history=self._history)
        try:
            raw = await asyncio.wait_for(
                self._stream_ollama(messages, prefix=''),
                timeout=self._timeout,
            )
        except asyncio.TimeoutError:
            self._out(f'\n  {ERR}  Timeout. Is Ollama running?  →  ollama serve')
            self._prompt(); return
        except Exception as exc:
            self._out(f'\n  {ERR}  LLM error: {exc}')
            self._prompt(); return

        self._out('')
        try:
            reply, command = _parse_response(raw)
        except ValueError as exc:
            self._out(f'  {ERR}  Could not parse response: {exc}')
            self._out(f'       Raw: {raw[:300]}')
            self._prompt(); return

        if command.get('mode') == 'mapf' and 'goals' in command:
            command['goals'] = _clamp_goals(_spread_goals(command['goals']))

        self._describe_command(command)

        self._history.append({'role': 'user',      'content': text})
        self._history.append({'role': 'assistant', 'content': raw})
        if len(self._history) > MAX_HISTORY_TURNS * 2:
            self._history = self._history[-MAX_HISTORY_TURNS * 2:]

        await self._send_to_bt(command)

    # -----------------------------------------------------------------------
    # Ollama streaming
    # -----------------------------------------------------------------------

    async def _stream_ollama(self, messages: list[dict], prefix: str = '') -> str:
        """Stream Ollama response, printing chunks in real time. Returns full text."""
        import aiohttp

        payload = {
            'model':    self._model,
            'messages': messages,
            'stream':   True,
            'options':  {
                'temperature': self._temperature,
                'num_predict': self._max_tokens,
            },
        }

        full_text = ''
        async with aiohttp.ClientSession() as session:
            async with session.post(self._endpoint, json=payload) as resp:
                if resp.status != 200:
                    body = await resp.text()
                    raise RuntimeError(f'Ollama HTTP {resp.status}: {body[:300]}')

                if prefix:
                    print(prefix, end='', flush=True)

                async for raw_line in resp.content:
                    line = raw_line.strip()
                    if not line:
                        continue
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    chunk = data.get('message', {}).get('content', '')
                    if chunk:
                        print(chunk, end='', flush=True)
                        full_text += chunk

                    if data.get('done', False):
                        break

        return full_text

    # -----------------------------------------------------------------------
    # Send to BT
    # -----------------------------------------------------------------------

    async def _send_to_bt(self, command: dict):
        if not self._cmd_client.wait_for_server(timeout_sec=3.0):
            self._out(f'  {ERR}  /llm/command not available. Is test_bt_runner running?')
            self._prompt(); return

        goal = LlmCommand.Goal()
        goal.mode         = command['mode']
        goal.reason       = command.get('reason', '')
        goal.robot_ids    = [int(r) for r in command.get('robot_ids', [])]
        goal.goals        = [Point(x=float(g[0]), y=float(g[1]), z=0.0)
                             for g in command.get('goals', [])]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns    = command.get('leader_ns', '')
        goal.follower_ns  = list(command.get('follower_ns', []))
        goal.offsets_x    = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y    = [float(o) for o in command.get('offsets_y', [])]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self._out(f'  {ERR}  BT rejected — try "стоп" first to cancel current mission.')
            self._prompt(); return

        result = await handle.get_result_async()
        if result.result.success:
            self._out(f'  {OK}  Executing.')
        else:
            self._out(f'  {ERR}  BT error: {result.result.info}')
        self._prompt()

    # -----------------------------------------------------------------------
    # Display
    # -----------------------------------------------------------------------

    def _describe_command(self, command: dict):
        mode = command['mode']
        print()
        if mode == 'idle':
            print(f'  {ARR} STOP')
        elif mode == 'mapf':
            ids, goals = command['robot_ids'], command['goals']
            print(f'  {ARR} MAPF: {len(ids)} robot(s)')
            for rid, g in zip(ids, goals):
                print(f'       robot_{rid:<3}  {ARR}  ({g[0]:.2f}, {g[1]:.2f})')
        elif mode == 'formation':
            print(f'  {ARR} FORMATION: {command["formation_id"]}  leader: {command["leader_ns"]}')
            for f, ox, oy in zip(command.get('follower_ns', []),
                                  command.get('offsets_x', []),
                                  command.get('offsets_y', [])):
                print(f'       {f:<18} offset ({ox:+.1f}, {oy:+.1f})')
        reason = command.get('reason', '')
        if reason:
            print(f'     reason: {reason}')

    def _out(self, msg: str):
        print(msg, flush=True)

    def _prompt(self):
        print('\n> ', end='', flush=True)

    def _print_banner(self):
        print(f'\n  {BOT}  Swarm chat  |  model: {self._model}\n')
        print('  Примеры:')
        print('    роботы 1 2 3 в правый угол, остальные на месте')
        print('    все роботы в центр')
        print('    стоп')
        print('\n  "quit" для выхода.\n')
        self._prompt()


# ---------------------------------------------------------------------------

def main(args=None):
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