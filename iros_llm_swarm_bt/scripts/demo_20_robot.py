#!/usr/bin/env python3
"""
demo_20_robot.py — full 20-robot warehouse scenario, driven from outside the BT host.

Replaces the embedded `scenario:=true` mode of the old test_bt_runner.cpp.
Communicates with bt_runner only through the public ROS interfaces:
  - sends LlmCommand goals on /llm/command (action)
  - listens to /bt/state (topic) to know when each step has completed

Scenario layout (warehouse.world, 30x30m):
  Orange  robot_0..9   -- loading zone,   bottom-left  (~2-4,  2-8)
  Blue    robot_10..19 -- unloading zone, top-right    (~26-28, 22-28)

Steps:
  1. MAPF all 20 -> warehouse center (15,15)
  2. Formation WEDGE_20  -- orange squad (robot_0..9), leader robot_1
  3. Formation LINE_BLUE -- blue squad (robot_10..14), leader robot_10
  4. MAPF cross-swap     -- orange -> blue home, blue -> orange home
  5. MAPF all 20 back home
  6. Idle

Usage:
  ros2 run iros_llm_swarm_bt demo_20_robot.py
"""

from __future__ import annotations

import sys
import time
from typing import Callable, List, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def make_point(x: float, y: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = 0.0
    return p


# Fixed positions used in multiple steps -------------------------------------
ORANGE_HOME: List[Tuple[float, float]] = [
    (2.0, 2.0), (3.5, 2.0),
    (2.0, 3.5), (3.5, 3.5),
    (2.0, 5.0), (3.5, 5.0),
    (2.0, 6.5), (3.5, 6.5),
    (2.0, 8.0), (3.5, 8.0),
]

BLUE_HOME: List[Tuple[float, float]] = [
    (26.0, 22.0), (27.5, 22.0),
    (26.0, 23.5), (27.5, 23.5),
    (26.0, 25.0), (27.5, 25.0),
    (26.0, 26.5), (27.5, 26.5),
    (26.0, 28.0), (27.5, 28.0),
]


# ---------------------------------------------------------------------------
# DemoRunner — wraps /llm/command client + /bt/state subscription
# ---------------------------------------------------------------------------
class DemoRunner(Node):
    """
    Single-threaded scenario driver using rclpy.spin_once.

    Drains callbacks while sleeping, so we don't need a separate executor
    thread or condition variables — the underlying pattern is exactly the
    same as ScenarioClient in the old C++ test_bt_runner, but expressed
    naturally in Python.
    """

    def __init__(self) -> None:
        super().__init__('demo_20_robot')

        self._client = ActionClient(self, LlmCommand, '/llm/command')

        # Match BTStatePublisher's QoS (reliable, depth 20) — see swarm_bt_nodes.hpp.
        state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(
            BTState, '/bt/state', self._on_state, state_qos)

        self._latest_state: BTState | None = None
        self._have_new_state: bool = False

    # -- ROS callbacks --------------------------------------------------------

    def _on_state(self, msg: BTState) -> None:
        self._latest_state = msg
        self._have_new_state = True

    # -- Public API -----------------------------------------------------------

    def wait_for_server(self, timeout: float = 20.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout)

    def send(self, goal: LlmCommand.Goal, timeout: float = 5.0) -> bool:
        """
        Send a /llm/command goal and wait for the action to succeed.

        Success means LlmCommandReceiver applied the goal to the blackboard
        on a BT tick. Returns True if applied, False on rejection / timeout.

        This does NOT mean the mission has completed — call
        wait_for_completion(expected_mode) for that.
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

    def wait_for_completion(
        self,
        expected_mode: str,
        timeout: float,
    ) -> str:
        """
        Tail /bt/state until the mission completes in the expected mode.

        Returns the terminal action_status (typically 'OK', 'WARN', 'ERROR',
        'HALTED') once we see (mode == expected_mode AND
        active_action == 'none'), or 'TIMEOUT' on no answer in time.

        Skips states reported while mode is still the previous one — the
        BT applies LlmCommand goals on the next tick, so /bt/state may
        briefly show stale mode after send() returns.
        """
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            # Edge-triggered: only inspect a *fresh* /bt/state, not the one
            # cached from a previous iteration. Mirrors the same pattern in
            # ScenarioClient.wait_for_completion (C++).
            self._have_new_state = False
            while not self._have_new_state:
                if time.monotonic() >= deadline:
                    return 'TIMEOUT'
                remaining = max(0.01, deadline - time.monotonic())
                rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

            state = self._latest_state
            assert state is not None  # _have_new_state True implies a state was set

            if state.mode != expected_mode:
                continue  # mode change still propagating
            if state.active_action == 'none':
                return state.action_status if state.action_status else 'OK'

        return 'TIMEOUT'

    def to_idle(self) -> bool:
        """Best-effort idle reset between scenario steps."""
        g = LlmCommand.Goal()
        g.mode = 'idle'
        g.reason = 'demo reset'
        return self.send(g)

    # -- Internal -------------------------------------------------------------

    def _spin_until_complete(self, future, timeout: float) -> bool:
        """
        Drain executor callbacks until ``future`` is done or timeout expires.

        Equivalent to rclpy.spin_until_future_complete but lets us reuse the
        same Node-driven loop everywhere (we don't manage an external
        executor).
        """
        deadline = time.monotonic() + timeout
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return True


# ---------------------------------------------------------------------------
# Scenario steps
# ---------------------------------------------------------------------------

def step1_mapf_center(runner: DemoRunner) -> bool:
    """Step 1: MAPF all 20 robots -> warehouse center, 4x5 grid centered on (15,15)."""
    runner.get_logger().info('=== STEP 1: MAPF all 20 -> warehouse center (15,15) ===')
    g = LlmCommand.Goal()
    g.mode = 'mapf'
    g.reason = 'demo step 1: rendezvous'
    cols = 4
    spacing = 1.5
    for i in range(20):
        g.robot_ids.append(i)
        g.goals.append(make_point(
            15.0 + ((i % cols) - (cols - 1) / 2.0) * spacing,
            15.0 + ((i // cols) - 2.0) * spacing,
        ))

    if not runner.send(g):
        runner.get_logger().error('STEP 1: send failed')
        return False
    status = runner.wait_for_completion('mapf', 240.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 1: wait_for_completion returned {status}')
        return False
    runner.get_logger().info('=== STEP 1 OK ===')
    return True


def step2_wedge_orange(runner: DemoRunner) -> bool:
    """Step 2: Formation WEDGE_20 — orange squad (robot_0..9), leader robot_1."""
    runner.get_logger().info('=== STEP 2: Formation WEDGE_20 (orange squad) ===')
    g = LlmCommand.Goal()
    g.mode = 'formation'
    g.reason = 'demo step 2: wedge orange'
    g.formation_id = 'wedge_20'
    g.leader_ns = 'robot_1'
    g.follower_ns = [
        'robot_2', 'robot_0',
        'robot_4', 'robot_3',
        'robot_6', 'robot_5',
        'robot_8', 'robot_7',
        'robot_9',
    ]
    g.offsets_x = [-1.0, -1.0, -2.0, -2.0, -3.0, -3.0, -4.0, -4.0, -5.0]
    g.offsets_y = [0.8, -0.8, 1.6, -1.6, 2.4, -2.4, 3.2, -3.2, 0.0]

    if not runner.send(g):
        runner.get_logger().error('STEP 2: send failed')
        return False
    status = runner.wait_for_completion('formation', 15.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 2: wait_for_completion returned {status}')
        return False
    runner.get_logger().info('=== STEP 2 OK ===')
    time.sleep(3.0)
    return True


def step3_line_blue(runner: DemoRunner) -> bool:
    """Step 3: Formation LINE_BLUE — blue squad (robot_10..14), leader robot_10."""
    runner.get_logger().info('=== STEP 3: Formation LINE_BLUE (blue squad) ===')
    g = LlmCommand.Goal()
    g.mode = 'formation'
    g.reason = 'demo step 3: line blue'
    g.formation_id = 'line_blue'
    g.leader_ns = 'robot_10'
    g.follower_ns = ['robot_11', 'robot_12', 'robot_13', 'robot_14']
    g.offsets_x = [-1.5, -3.0, -4.5, -6.0]
    g.offsets_y = [0.0, 0.0, 0.0, 0.0]

    if not runner.send(g):
        runner.get_logger().error('STEP 3: send failed')
        return False
    status = runner.wait_for_completion('formation', 15.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 3: wait_for_completion returned {status}')
        return False
    runner.get_logger().info('=== STEP 3 OK ===')
    time.sleep(3.0)
    return True


def step4_cross_swap(runner: DemoRunner) -> bool:
    """Step 4: MAPF cross-swap — orange -> blue home, blue -> orange home."""
    runner.get_logger().info('=== STEP 4: MAPF cross-swap ===')
    g = LlmCommand.Goal()
    g.mode = 'mapf'
    g.reason = 'demo step 4: cross-swap'

    for i in range(10):
        g.robot_ids.append(i)
        g.goals.append(make_point(*BLUE_HOME[i]))
    for i in range(10):
        g.robot_ids.append(10 + i)
        g.goals.append(make_point(*ORANGE_HOME[i]))

    if not runner.send(g):
        runner.get_logger().error('STEP 4: send failed')
        return False
    status = runner.wait_for_completion('mapf', 300.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 4: wait_for_completion returned {status}')
        return False
    runner.get_logger().info('=== STEP 4 OK ===')
    return True


def step5_back_home(runner: DemoRunner) -> bool:
    """Step 5: MAPF all 20 back to their home zones."""
    runner.get_logger().info('=== STEP 5: MAPF all 20 -> home ===')
    g = LlmCommand.Goal()
    g.mode = 'mapf'
    g.reason = 'demo step 5: home'

    for i in range(10):
        g.robot_ids.append(i)
        g.goals.append(make_point(*ORANGE_HOME[i]))
    for i in range(10):
        g.robot_ids.append(10 + i)
        g.goals.append(make_point(*BLUE_HOME[i]))

    if not runner.send(g):
        runner.get_logger().error('STEP 5: send failed')
        return False
    status = runner.wait_for_completion('mapf', 300.0)
    if status != 'OK':
        runner.get_logger().error(f'STEP 5: wait_for_completion returned {status}')
        return False
    runner.get_logger().info('=== STEP 5 OK ===')
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

# Each step: (name, function, sleep_between_after_step)
# The to_idle()+pause between steps mirrors the defensive pattern from the
# old C++ scenario — gives /bt/state and downstream stack a chance to settle
# between MAPF / formation transitions.
STEPS: List[Tuple[str, Callable[[DemoRunner], bool], float]] = [
    ('step 1', step1_mapf_center, 2.0),
    ('step 2', step2_wedge_orange, 0.3),
    ('step 3', step3_line_blue, 0.3),
    ('step 4', step4_cross_swap, 2.0),
    ('step 5', step5_back_home, 0.0),
]


def main(args: List[str] | None = None) -> int:
    rclpy.init(args=args)
    runner = DemoRunner()
    log = runner.get_logger()

    try:
        log.info('demo: waiting for /llm/command action server...')
        if not runner.wait_for_server(timeout=20.0):
            log.error('demo: /llm/command never came up')
            return 1
        log.info('demo: /llm/command ready')

        # Give the rest of the stack (MAPF planner, formation manager, Stage)
        # time to settle before the first goal.
        time.sleep(3.0)

        for name, step_fn, pause_after in STEPS:
            if not step_fn(runner):
                log.error(f'=== DEMO FAILED at {name} ===')
                runner.to_idle()
                return 1
            runner.to_idle()
            if pause_after > 0:
                time.sleep(pause_after)

        log.info('=== STEP 6: IDLE ===')
        runner.to_idle()
        time.sleep(1.0)
        log.info('ALL 20 ROBOTS -- FULL DEMO COMPLETED SUCCESSFULLY')
        return 0

    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
