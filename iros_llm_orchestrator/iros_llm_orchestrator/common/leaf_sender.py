"""Shared BT-leaf sender: dispatch one PlanExecutor leaf and wait for the
behaviour tree to finish the resulting mission.

Used by both chat_server (channel 3) and execute_server (operator-confirmed
plan replay). Encapsulates the /bt/state @active_action protocol that
user_chat_node._send_leaf also uses, so all callers behave identically.

The /llm/command action result is only an ack — LlmCommandReceiver succeeds
the goal as soon as it copies the command onto the blackboard. Real mission
completion is observed on /bt/state: the refactored BT cycles @active_action
none → MapfPlan/SetFormation/DisableFormation → none. @mode no longer
returns to idle on its own (RunOnce keeps the tree alive), so completion is
keyed on @active_action, never on @mode.

  Phase 1 (≤3s): wait for @active_action to leave "none" — the BT started
                 the action node.
  Phase 2:       wait for @active_action to return to "none" within
                 step_timeout_sec; a hard failure instead raises
                 @action_status to ERROR/HALTED.
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

from iros_llm_orchestrator.common.plan_executor import coerce_robot_id


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
        self._last_active_action = 'none'

        # Failure metadata for the most recent send(). Populated whenever
        # send() returns False; cleared on each new send() call. Chat-side
        # remediation reads this to brief the LLM about which leaf failed.
        self._last_failure: dict | None = None

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
            # Mirror action_status / active_action so the send() phases can
            # detect mission start, completion and failure.
            self._last_action_status = msg.action_status
            self._last_bt_error      = msg.last_error
            self._last_active_action = msg.active_action or 'none'

    def _get_mode_state(self) -> tuple[str, int, str, str, str]:
        with self._mode_lock:
            return (self._last_mode, self._mode_seq,
                    self._last_action_status, self._last_bt_error,
                    self._last_active_action)

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

    def last_failure(self) -> dict | None:
        """Return failure metadata from the most recent send(), or None."""
        return self._last_failure

    def _record_failure(self, *, leaf_type: str, phase: str,
                        info: str = '') -> None:
        status = self._get_mode_state()[2]
        err    = self._get_mode_state()[3]
        # Prefer the explicit info string (action result / timeout reason)
        # over a stale /bt/state error if we have one.
        last_error = info or err or ''
        self._last_failure = {
            'leaf_type':       leaf_type,
            'action_status':   status,
            'last_error':      last_error[:240],
            'failed_at_phase': phase,
        }

    async def send(self, command: dict) -> bool:
        t = command.get('type') or command.get('mode', '')

        # Reset failure slot — only the latest send() is reported.
        self._last_failure = None

        if t == 'obstacles':
            ok = await self._dispatch_obstacles(command)
            if not ok:
                self._record_failure(leaf_type=t, phase='obstacles')
            return ok

        if not await self._ensure_cmd_server():
            self._node.get_logger().error('/llm/command unavailable')
            self._record_failure(
                leaf_type=t, phase='no_server',
                info='/llm/command action server unavailable')
            return False

        goal = LlmCommand.Goal()
        goal.mode         = t
        goal.reason       = command.get('reason', '')
        goal.robot_ids    = [coerce_robot_id(r)
                             for r in command.get('robot_ids', [])]
        goal.goals        = [Point(x=float(g[0]), y=float(g[1]), z=0.0)
                             for g in command.get('goals', [])]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns    = command.get('leader_ns', '')
        goal.follower_ns  = list(command.get('follower_ns', []))
        goal.offsets_x    = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y    = [float(o) for o in command.get('offsets_y', [])]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self._node.get_logger().warn(f'BT rejected {t!r} goal')
            self._record_failure(leaf_type=t, phase='rejected',
                                 info='BT rejected goal')
            return False

        cmd_result = await handle.get_result_async()
        if not cmd_result.result.success:
            info = str(cmd_result.result.info or '')
            self._node.get_logger().error(
                f'BT error on {t!r}: {info}')
            self._record_failure(leaf_type=t, phase='accept', info=info)
            return False

        if t == 'idle':
            return True

        # The /llm/command result above is only an ack ("command applied to
        # the blackboard"). Mission completion is observed on /bt/state via
        # @active_action — @mode no longer returns to idle on its own.
        #
        # Phase 1 — wait for the BT to start the action node. LlmCommandReceiver
        # bumps @command_seq, RunOnce re-ticks the wrapped node and onStart()
        # sets @active_action to MapfPlan / SetFormation / DisableFormation.
        phase1_deadline = time.monotonic() + 3.0
        started = False
        while time.monotonic() < phase1_deadline:
            _, _, status_now, err_now, action_now = self._get_mode_state()
            if action_now not in ('', 'none'):
                started = True
                break
            await asyncio.sleep(0.05)
        if not started:
            # @active_action never left "none". Surface a real failure;
            # otherwise treat as a no-op success rather than block the plan.
            _, _, status_now, err_now, _ = self._get_mode_state()
            if status_now in ('ERROR', 'HALTED'):
                self._node.get_logger().error(
                    f'leaf {t!r}: BT reported {status_now} without starting an '
                    f'action: {err_now}')
                self._record_failure(leaf_type=t, phase='phase1', info=err_now)
                return False
            self._node.get_logger().warn(
                f'leaf {t!r}: BT never started an action — treating as no-op')
            return True

        # Phase 2 — wait for the action node to finish. The BT clears
        # @active_action back to "none" on success/partial; a hard failure
        # instead raises @action_status to ERROR/HALTED.
        deadline = time.monotonic() + self._step_timeout
        while time.monotonic() < deadline:
            _, _, status_now, err_now, action_now = self._get_mode_state()
            finished = action_now in ('', 'none')
            failed   = status_now in ('ERROR', 'HALTED')
            if finished or failed:
                if failed:
                    self._node.get_logger().error(
                        f'leaf {t!r}: BT finished with {status_now}: {err_now}')
                    self._record_failure(leaf_type=t, phase='phase2',
                                         info=err_now)
                    return False
                return True
            await asyncio.sleep(0.1)

        self._node.get_logger().error(
            f'step timeout on {t!r} after {self._step_timeout:.0f}s')
        self._record_failure(
            leaf_type=t, phase='timeout',
            info=f'step timeout after {self._step_timeout:.0f}s')
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