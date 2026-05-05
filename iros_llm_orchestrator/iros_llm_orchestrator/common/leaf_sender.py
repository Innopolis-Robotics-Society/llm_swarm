"""Shared BT-leaf sender: dispatch one PlanExecutor leaf and wait for the
behaviour tree to finish the resulting mission.

Used by both chat_server (channel 3) and execute_server (operator-confirmed
plan replay). Encapsulates the same /bt/state mode-seq protocol that
user_chat_node._send_leaf uses, so all callers behave identically:

  Phase 1 (≤1.5s): wait for any mode transition past the snapshot taken
                   before sending. If mode flips to idle inside the window
                   we treat the leaf as fast-completed; if no transition
                   happens at all, ModeDispatch effectively no-op'd this
                   leaf (e.g. duplicate goal) — return success rather than
                   block the whole plan.
  Phase 2:         wait for mode → idle within step_timeout_sec.
"""

from __future__ import annotations

import asyncio
import threading
import time

from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState


class BTLeafSender:
    """Send a single PlanExecutor leaf via /llm/command and wait for BT.

    Owns its /bt/state subscription (so callers can construct one without
    duplicating the boilerplate) but borrows the rclpy Node — does not
    spin its own executor.
    """

    def __init__(self, node: Node, *, step_timeout_sec: float = 120.0):
        self._node             = node
        self._step_timeout     = float(step_timeout_sec)
        self._cmd_client       = ActionClient(node, LlmCommand, '/llm/command')
        self._cmd_server_ready = False

        self._mode_lock = threading.Lock()
        self._last_mode = ''
        self._mode_seq  = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._sub = node.create_subscription(
            BTState, '/bt/state', self._on_bt_state, qos)

    # ------------------------------------------------------------------
    # /bt/state — track mode transitions for leaf completion
    # ------------------------------------------------------------------

    def _on_bt_state(self, msg: BTState):
        with self._mode_lock:
            if msg.mode != self._last_mode:
                self._last_mode = msg.mode
                self._mode_seq += 1

    def _get_mode_state(self) -> tuple[str, int]:
        with self._mode_lock:
            return self._last_mode, self._mode_seq

    # ------------------------------------------------------------------
    # Send one leaf
    # ------------------------------------------------------------------

    async def _ensure_cmd_server(self) -> bool:
        """Off-loop probe — wait_for_server is blocking."""
        if self._cmd_server_ready:
            return True
        loop = asyncio.get_running_loop()
        ok = await loop.run_in_executor(
            None, lambda: self._cmd_client.wait_for_server(timeout_sec=3.0))
        self._cmd_server_ready = ok
        return ok

    async def send(self, command: dict) -> bool:
        t = command.get('type') or command.get('mode', '')

        if not await self._ensure_cmd_server():
            self._node.get_logger().error('/llm/command unavailable')
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

        seq_at_send = self._get_mode_state()[1]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self._node.get_logger().warn(f'BT rejected {t!r} goal')
            return False

        cmd_result = await handle.get_result_async()
        if not cmd_result.result.success:
            self._node.get_logger().error(
                f'BT error on {t!r}: {cmd_result.result.info}')
            return False

        if t == 'idle':
            return True

        # Phase 1
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

        # Phase 2
        deadline = time.monotonic() + self._step_timeout
        while time.monotonic() < deadline:
            if self._get_mode_state()[0] == 'idle':
                return True
            await asyncio.sleep(0.1)

        self._node.get_logger().error(
            f'step timeout on {t!r} after {self._step_timeout:.0f}s')
        return False
