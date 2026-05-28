#!/usr/bin/env python3
"""
demo_triangle.py — 3-robot triangle / formation / relocate demo.

Drives the REAL stack (Stage + MAPF planner + formation_manager + bt_runner)
through a three-step mission, talking only via the public BT interfaces:
  - sends LlmCommand goals on /llm/command (action)
  - listens to /bt/state (topic) to know when each step has completed

Mission:
  1. MAPF      — send 3 robots into a triangle around point A
  2. Formation — lock them into a triangle formation (leader + 2 followers)
  3. MAPF      — relocate the group to a triangle around point B

Step 3 note: formation and MAPF are mutually-exclusive modes here, so
"moving the formation to B" is done by issuing a fresh MAPF command that
plans all three robots into a triangle around B. Switching to mapf mode
drops the formation.

Edit ROBOT_IDS / POINT_A / POINT_B below to retarget.

Usage:
  ros2 run iros_llm_swarm_bt demo_triangle.py
"""

from __future__ import annotations

import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState


# Mission parameters --------------------------------------------------------
# Mission parameters --------------------------------------------------------
ROBOT_IDS: List[int] = [12, 13, 14]
POINT_A: Tuple[float, float] = (2.7,  10.1)
POINT_B: Tuple[float, float] = (0.6, -10.9)

# Follower offsets relative to the leader (leader at (0, 0)). With these the
# three robots sit at the corners of an ~equilateral triangle (side ~1.5 m).
#
# CRITICAL: this is the SINGLE source of truth for the triangle geometry. It
# is used BOTH to pre-position the robots via MAPF (steps 1 & 3) AND to define
# the formation (step 2). They must match exactly, otherwise formation_manager
# finds the followers "out of position" on /formation/set and refuses to
# activate. offsets are leader-relative.
FORMATION_OFFSETS: List[Tuple[float, float]] = [(-1.8, 1.3), (-1.8, -1.3)]


def make_point(x: float, y: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = 0.0
    return p


def formation_positions(anchor: Tuple[float, float]):
    """
    Return goals [leader, follower0, follower1] for a triangle on an anchor.

    The triangle centroid is placed on `anchor`; leader and followers keep the
    FORMATION_OFFSETS layout so /formation/set finds them already in position.
    Order matches ROBOT_IDS, i.e. goals[i] is for ROBOT_IDS[i].
    """
    ax, ay = anchor
    layout = [(0.0, 0.0)] + FORMATION_OFFSETS  # leader-relative positions
    cx = sum(p[0] for p in layout) / len(layout)
    cy = sum(p[1] for p in layout) / len(layout)
    return [(ax + px - cx, ay + py - cy) for px, py in layout]


class DemoRunner(Node):
    """
    Single-threaded mission driver using rclpy.spin_once.

    Same pattern as demo_20_robot.py: send a /llm/command goal, then tail
    /bt/state until the mission step completes. No mocks — this talks to the
    real bt_runner and downstream stack.
    """

    def __init__(self) -> None:
        super().__init__('demo_triangle')
        self._client = ActionClient(self, LlmCommand, '/llm/command')
        state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(
            BTState, '/bt/state', self._on_state, state_qos)
        self._latest_state: BTState | None = None
        self._have_new_state: bool = False

    def _on_state(self, msg: BTState) -> None:
        self._latest_state = msg
        self._have_new_state = True

    def wait_for_server(self, timeout: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout)

    def send(self, goal: LlmCommand.Goal, timeout: float = 5.0) -> bool:
        """
        Send a /llm/command goal and wait for the action to succeed.

        Success means LlmCommandReceiver applied the goal on a BT tick.
        Returns True if applied, False on rejection / timeout. Does NOT mean
        the mission step finished — use wait_for_completion for that.
        """
        future = self._client.send_goal_async(goal)
        if not self._spin_until_complete(future, timeout):
            self.get_logger().error('send_goal timed out')
            return False
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('goal rejected by LlmCommandReceiver')
            return False
        result_future = gh.get_result_async()
        if not self._spin_until_complete(result_future, timeout):
            self.get_logger().error('get_result timed out')
            return False
        wrapped = result_future.result()
        if wrapped is None:
            return False
        return bool(wrapped.result.success)

    def wait_for_completion(self, expected_mode: str, timeout: float) -> str:
        """
        Tail /bt/state until the step completes in the expected mode.

        Returns the terminal action_status ('OK', 'WARN', 'ERROR', 'HALTED')
        once we see (mode == expected_mode AND active_action == 'none'), or
        'TIMEOUT' if no such state arrives in time.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._have_new_state = False
            while not self._have_new_state:
                if time.monotonic() >= deadline:
                    return 'TIMEOUT'
                remaining = max(0.01, deadline - time.monotonic())
                rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
            state = self._latest_state
            assert state is not None
            if state.mode != expected_mode:
                continue
            if state.active_action == 'none':
                return state.action_status if state.action_status else 'OK'
        return 'TIMEOUT'

    def to_idle(self) -> bool:
        """Best-effort idle reset between mission steps."""
        g = LlmCommand.Goal()
        g.mode = 'idle'
        g.reason = 'demo reset'
        return self.send(g)

    def _spin_until_complete(self, future, timeout: float) -> bool:
        """Drain executor callbacks until ``future`` is done or timeout."""
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return True


def step1_mapf_triangle_a(runner: DemoRunner) -> bool:
    """Send the 3 robots into a triangle around point A (MAPF)."""
    runner.get_logger().info(
        f'=== STEP 1: MAPF {ROBOT_IDS} -> triangle around A {POINT_A} ===')
    g = LlmCommand.Goal()
    g.mode = 'mapf'
    g.reason = 'demo triangle: gather around A'
    g.robot_ids = list(ROBOT_IDS)
    for x, y in formation_positions(POINT_A):
        g.goals.append(make_point(x, y))

    if not runner.send(g):
        runner.get_logger().error('STEP 1: send failed')
        return False
    status = runner.wait_for_completion('mapf', 180.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 1: completion status={status}')
        return False
    runner.get_logger().info('=== STEP 1 OK ===')
    return True


def step2_triangle_formation(runner: DemoRunner) -> bool:
    """Lock the 3 robots into a triangle formation, leader = ROBOT_IDS[0]."""
    runner.get_logger().info(
        f'=== STEP 2: triangle formation, leader robot_{ROBOT_IDS[0]} ===')
    g = LlmCommand.Goal()
    g.mode = 'formation'
    g.reason = 'demo triangle: form up'
    g.formation_id = 'triangle_3'
    g.leader_ns = f'robot_{ROBOT_IDS[0]}'
    g.follower_ns = [f'robot_{ROBOT_IDS[1]}', f'robot_{ROBOT_IDS[2]}']
    # Same geometry that pre-positioned the robots in step 1, so the manager
    # finds them in position and activates instead of rejecting.
    g.offsets_x = [o[0] for o in FORMATION_OFFSETS]
    g.offsets_y = [o[1] for o in FORMATION_OFFSETS]

    if not runner.send(g):
        runner.get_logger().error('STEP 2: send failed')
        return False
    status = runner.wait_for_completion('formation', 20.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 2: completion status={status}')
        return False
    runner.get_logger().info('=== STEP 2 OK — holding formation 4s ===')
    # Let the followers physically settle into their offset positions.
    time.sleep(4.0)
    return True


def step3_mapf_triangle_b(runner: DemoRunner) -> bool:
    """Relocate the group to a triangle around point B (drops formation)."""
    runner.get_logger().info(
        f'=== STEP 3: MAPF {ROBOT_IDS} -> triangle around B {POINT_B} ===')
    g = LlmCommand.Goal()
    g.mode = 'mapf'
    g.reason = 'demo triangle: relocate to B'
    g.robot_ids = list(ROBOT_IDS)
    for x, y in formation_positions(POINT_B):
        g.goals.append(make_point(x, y))

    if not runner.send(g):
        runner.get_logger().error('STEP 3: send failed')
        return False
    status = runner.wait_for_completion('mapf', 180.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 3: completion status={status}')
        return False
    runner.get_logger().info('=== STEP 3 OK ===')
    return True


# (name, function, pause_after_seconds) — to_idle()+pause between steps
# mirrors the defensive transition pattern from demo_20_robot.py.
STEPS = [
    ('step 1', step1_mapf_triangle_a, 2.0),
    ('step 2', step2_triangle_formation, 1.0),
    ('step 3', step3_mapf_triangle_b, 0.0),
]


def main(args: List[str] | None = None) -> int:
    rclpy.init(args=args)
    runner = DemoRunner()
    log = runner.get_logger()

    try:
        log.info('demo_triangle: waiting for /llm/command action server...')
        if not runner.wait_for_server(timeout=20.0):
            log.error('demo_triangle: /llm/command never came up')
            return 1
        log.info('demo_triangle: /llm/command ready')

        # Let the rest of the stack settle before the first goal.
        time.sleep(3.0)

        for name, step_fn, pause_after in STEPS:
            if not step_fn(runner):
                log.error(f'=== DEMO FAILED at {name} ===')
                runner.to_idle()
                return 1
            runner.to_idle()
            if pause_after > 0:
                time.sleep(pause_after)

        log.info('=== STEP 4: IDLE ===')
        runner.to_idle()
        time.sleep(1.0)
        log.info('TRIANGLE DEMO COMPLETED SUCCESSFULLY')
        return 0

    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
