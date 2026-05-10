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
from iros_llm_swarm_interfaces.srv import (
    AddCircle, AddRectangle, AddDoor, RemoveObstacle, OpenDoor, CloseDoor)


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

        self._add_circle  = node.create_client(AddCircle,      '/obstacles/add_circle')
        self._add_rect    = node.create_client(AddRectangle,   '/obstacles/add_rectangle')
        self._add_door_cl = node.create_client(AddDoor,        '/obstacles/add_door')
        self._remove_obs  = node.create_client(RemoveObstacle, '/obstacles/remove')
        self._open_door   = node.create_client(OpenDoor,       '/doors/open')
        self._close_door  = node.create_client(CloseDoor,      '/doors/close')

        self._mode_lock          = threading.Lock()
        self._last_mode          = ''
        self._mode_seq           = 0
        self._last_action_status = 'OK'
        self._last_bt_error      = ''

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
            # Mirror action_status so phase 1 can detect failed missions
            self._last_action_status = msg.action_status
            self._last_bt_error      = msg.last_error

    def _get_mode_state(self) -> tuple[str, int, str, str]:
        with self._mode_lock:
            return (self._last_mode, self._mode_seq,
                    self._last_action_status, self._last_bt_error)

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

        if t == 'obstacles':
            return await self._dispatch_obstacles(command)

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

        # Phase 1 — wait for BT to acknowledge the command (mode transition).
        # We do NOT fast-complete if mode went to idle too quickly while
        # action_status is ERROR/WARN — that indicates a failed mission
        # (e.g. PBS partial plan), not a genuine instant completion.
        phase1_deadline = time.monotonic() + 1.5
        transitioned    = False
        while time.monotonic() < phase1_deadline:
            mode_now, seq_now, status_now, err_now = self._get_mode_state()
            if seq_now > seq_at_send:
                if mode_now == 'idle':
                    if status_now in ('ERROR',):
                        # Mission failed (partial plan, no path, etc.)
                        self._node.get_logger().error(
                            f'leaf {t!r}: BT returned idle with ERROR: {err_now}')
                        return False
                    # Genuine fast completion (e.g. instant formation setup)
                    return True
                transitioned = True
                break
            await asyncio.sleep(0.05)
        if not transitioned:
            # No mode transition at all — BT ignored the command (duplicate
            # goal or ModeDispatch no-op). Treat as success rather than block.
            return True

        # Phase 2 — wait for mission to finish (mode returns to idle)
        deadline = time.monotonic() + self._step_timeout
        while time.monotonic() < deadline:
            if self._get_mode_state()[0] == 'idle':
                return True
            await asyncio.sleep(0.1)

        self._node.get_logger().error(
            f'step timeout on {t!r} after {self._step_timeout:.0f}s')
        return False

    # ------------------------------------------------------------------
    # Obstacle / door dispatch (bypasses BT)
    # ------------------------------------------------------------------

    async def _call_srv(self, client, request) -> bool:
        loop = asyncio.get_running_loop()
        def _blocking():
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            return client.call(request)
        result = await loop.run_in_executor(None, _blocking)
        return result is not None and bool(result.success)

    async def _dispatch_obstacles(self, cmd: dict) -> bool:
        action = cmd.get('action', '')
        if action == 'add_circle':
            req = AddCircle.Request()
            req.id = cmd.get('id', '')
            req.position.x = float(cmd.get('x', 0.0))
            req.position.y = float(cmd.get('y', 0.0))
            req.radius = float(cmd.get('radius', 0.5))
            return await self._call_srv(self._add_circle, req)
        if action == 'add_rectangle':
            req = AddRectangle.Request()
            req.id = cmd.get('id', '')
            req.position.x = float(cmd.get('x', 0.0))
            req.position.y = float(cmd.get('y', 0.0))
            req.width  = float(cmd.get('width',  1.0))
            req.height = float(cmd.get('height', 0.5))
            return await self._call_srv(self._add_rect, req)
        if action == 'add_door':
            req = AddDoor.Request()
            req.id = cmd.get('id', '')
            req.position.x = float(cmd.get('x', 0.0))
            req.position.y = float(cmd.get('y', 0.0))
            req.width   = float(cmd.get('width',  1.5))
            req.height  = float(cmd.get('height', 0.4))
            req.is_open = bool(cmd.get('is_open', True))
            return await self._call_srv(self._add_door_cl, req)
        if action == 'remove':
            req = RemoveObstacle.Request()
            req.id = cmd.get('id', '')
            return await self._call_srv(self._remove_obs, req)
        if action == 'open_door':
            req = OpenDoor.Request()
            req.door_id = cmd.get('door_id', '')
            return await self._call_srv(self._open_door, req)
        if action == 'close_door':
            req = CloseDoor.Request()
            req.door_id = cmd.get('door_id', '')
            return await self._call_srv(self._close_door, req)
        self._node.get_logger().error(f'obstacles: unknown action {action!r}')
        return False