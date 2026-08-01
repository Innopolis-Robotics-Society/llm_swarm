"""
test_bt_integration_mapf.py — integration test for swarm_bt_nodes.

Launches the production BT host (bt_runner) with a *mock* /swarm/set_goals
action server defined in this file. Sends LlmCommand goals and asserts that
/bt/state shows the expected sequence of states.

This is a SINGLE example case (MAPF happy path) — the pattern (mock server +
launch_testing harness + assertions on /bt/state) is meant to be cloned for
further scenarios:
  - partial plan -> action_status == WARN, mapf_ok == false
  - num_agents_planned == 0 -> action_status == ERROR
  - mode supersession -> first command gets superseded result
  - formation success / broken / degraded (needs SetFormation + DeactivateFormation
    + /formations/status mocks; add them in test/test_bt_integration_formation.py)

Assumes the package now exposes an executable named `bt_runner` that is the
trimmed-down version of the old `test_bt_runner` (without scenario thread).

Run with:
  colcon test --packages-select iros_llm_swarm_bt
  colcon test-result --verbose
"""

from __future__ import annotations

import threading
import time
import unittest
from typing import Callable, List, Optional

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch_ros.actions import Node as LaunchNode
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import LlmCommand, SetGoals
from iros_llm_swarm_interfaces.msg import BTState


# ---------------------------------------------------------------------------
# Launch description — what gets started for the test
# ---------------------------------------------------------------------------
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bt_runner = LaunchNode(
        package='iros_llm_swarm_bt',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        # Loud BT logs help when a test fails — turn off if too noisy.
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return (
        launch.LaunchDescription([
            bt_runner,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'bt_runner': bt_runner},
    )


# ---------------------------------------------------------------------------
# MockMapfServer — configurable mock for /swarm/set_goals
# ---------------------------------------------------------------------------
class MockMapfServer:
    """
    Mock action server for /swarm/set_goals.

    Configurable response per goal, so different test cases can exercise
    different result paths in MapfPlan. Receives a goal, sends a couple of
    feedback messages, then returns a result according to set_response().
    Records every received goal for later inspection.
    """

    # Response kinds — match the branches in MapfPlan::onRunning on the
    # final result.
    SUCCESS = 'success'             # res->success=true, num_planned=N
    PARTIAL = 'partial'             # res->success=false, num_planned>0  (-> WARN)
    NO_PLANS = 'no_plans'           # num_planned=0                       (-> ERROR)
    TRANSPORT_ERROR = 'transport_error'   # goal_handle.abort()           (-> ERROR)

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._received: List[SetGoals.Goal] = []
        # Default: pretend planning succeeded for all robots in the goal.
        self._response = (self.SUCCESS, None)

        self._server = ActionServer(
            node,
            SetGoals,
            '/swarm/set_goals',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )

    def set_response(self, kind: str, num_planned: Optional[int] = None) -> None:
        """
        Configure the response for the NEXT received goal.

        num_planned defaults to len(robot_ids) for SUCCESS/PARTIAL kinds.
        """
        with self._lock:
            self._response = (kind, num_planned)

    def received_goals(self) -> List[SetGoals.Goal]:
        with self._lock:
            return list(self._received)

    def _execute(self, goal_handle):
        request: SetGoals.Goal = goal_handle.request
        with self._lock:
            self._received.append(request)
            kind, override_n = self._response

        n_robots = len(request.robot_ids)
        n_planned = override_n if override_n is not None else n_robots

        # Send a few feedback messages so MapfPlan exercises its
        # snapshot+drain code path (and so /bt/state shows non-default
        # action_summary).
        for elapsed in (50, 200, 500):
            fb = SetGoals.Feedback()
            fb.elapsed_ms = elapsed
            fb.status = 'executing'
            fb.robots_arrived = n_planned if elapsed >= 500 else 0
            fb.robots_active = 0 if elapsed >= 500 else n_planned
            fb.robot_stall = 0
            fb.replans_done = 0
            fb.info = f'mock progress at {elapsed}ms'
            fb.warning = ''
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        result = SetGoals.Result()
        if kind == self.SUCCESS:
            result.success = True
            result.message = 'mock: all agents planned'
            result.num_agents_planned = n_planned
            result.planning_time_ms = 50.0
            result.total_replans = 0
            goal_handle.succeed()
        elif kind == self.PARTIAL:
            # Partial plan — action itself completes successfully, but
            # res.success=false signals "some agents didn't get a path".
            result.success = False
            result.message = f'mock: partial, {n_planned}/{n_robots} planned'
            result.num_agents_planned = max(1, n_planned)
            result.planning_time_ms = 80.0
            result.total_replans = 0
            goal_handle.succeed()
        elif kind == self.NO_PLANS:
            result.success = False
            result.message = 'mock: all starts blocked'
            result.num_agents_planned = 0
            result.planning_time_ms = 10.0
            result.total_replans = 0
            goal_handle.succeed()
        elif kind == self.TRANSPORT_ERROR:
            goal_handle.abort()
            return SetGoals.Result()  # rclpy still wants a return value
        else:
            raise ValueError(f'unknown response kind: {kind}')

        return result


# ---------------------------------------------------------------------------
# TestBTIntegrationMapf — actual test cases
# ---------------------------------------------------------------------------
class TestBTIntegrationMapf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Node that owns: mock action server, /bt/state subscription,
        # /llm/command action client.
        self.node = rclpy.create_node('test_bt_integration_node')
        self.mock_mapf = MockMapfServer(self.node)
        self.cmd_client = ActionClient(self.node, LlmCommand, '/llm/command')

        # /bt/state buffer + condition variable for edge-triggered waits.
        # Same pattern as ScenarioClient in the old C++ test_bt_runner —
        # in Python we use threading.Condition directly.
        self._state_lock = threading.Lock()
        self._state_cv = threading.Condition(self._state_lock)
        self._states: List[BTState] = []

        state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._state_sub = self.node.create_subscription(
            BTState, '/bt/state', self._on_state, state_qos)

        # Multi-threaded executor on a side thread, so the test body can
        # use blocking patterns (action client futures, condition_variable
        # waits) without starving the executor.
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        # Wait for bt_runner's /llm/command server to come up.
        ok = self.cmd_client.wait_for_server(timeout_sec=20.0)
        self.assertTrue(ok, '/llm/command never came up — is bt_runner running?')

    def tearDown(self):
        # Best-effort: drop to idle so the next test starts clean.
        try:
            self._send_command(self._goal_idle(), timeout=2.0)
        except Exception:
            pass

        self.executor.shutdown()
        self._spin_thread.join(timeout=5.0)
        self.node.destroy_node()

    # -- /bt/state buffer ----------------------------------------------------

    def _on_state(self, msg: BTState) -> None:
        with self._state_cv:
            self._states.append(msg)
            self._state_cv.notify_all()

    def _wait_for_state(
        self,
        predicate: Callable[[BTState], bool],
        timeout: float,
    ) -> Optional[BTState]:
        """
        Wait until any /bt/state message in the buffer satisfies the predicate.

        Inspects new messages as they arrive; returns matching state or None
        on timeout.

        Edge-triggered: only checks newly-arrived states, not the entire
        history each time — but does scan the buffer once on entry so a
        state that arrived before this call can still satisfy the predicate.
        """
        deadline = time.monotonic() + timeout
        with self._state_cv:
            # Check existing buffer first.
            for state in self._states:
                if predicate(state):
                    return state
            # Then wait for new arrivals.
            last_seen = len(self._states)
            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._state_cv.wait(timeout=remaining)
                for state in self._states[last_seen:]:
                    if predicate(state):
                        return state
                last_seen = len(self._states)
        return None

    # -- /llm/command helpers ------------------------------------------------

    def _send_command(
        self,
        goal: LlmCommand.Goal,
        timeout: float = 5.0,
    ) -> LlmCommand.Result:
        """Send a goal, wait for the action result. Asserts on protocol-level failures."""
        gh_future = self.cmd_client.send_goal_async(goal)
        self._wait_future(gh_future, timeout, 'send_goal')
        gh = gh_future.result()
        self.assertIsNotNone(gh)
        self.assertTrue(gh.accepted, 'goal not accepted by LlmCommandReceiver')

        result_future = gh.get_result_async()
        self._wait_future(result_future, timeout, 'get_result')
        wrapped = result_future.result()
        self.assertIsNotNone(wrapped)
        return wrapped.result

    def _wait_future(self, future, timeout: float, what: str) -> None:
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() >= deadline:
                self.fail(f'{what} timed out')
            time.sleep(0.02)

    @staticmethod
    def _goal_idle() -> LlmCommand.Goal:
        g = LlmCommand.Goal()
        g.mode = 'idle'
        g.reason = 'test teardown'
        return g

    @staticmethod
    def _goal_mapf(robot_ids: List[int], xy: List[tuple]) -> LlmCommand.Goal:
        assert len(robot_ids) == len(xy)
        g = LlmCommand.Goal()
        g.mode = 'mapf'
        g.reason = 'integration test'
        g.robot_ids = robot_ids
        for x, y in xy:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            g.goals.append(p)
        return g

    # =======================================================================
    # Actual test case
    # =======================================================================

    def test_mapf_happy_path(self):
        """
        MAPF command with a successful mock response.

        Expected sequence on /bt/state:
          1. mode='idle', active_action='none' (initial state)
          2. mode='mapf', active_action='MapfPlan' (action running)
          3. mode='mapf', active_action='none', action_status='OK' (completed)
        """
        # Confirm the initial idle state.
        idle = self._wait_for_state(lambda s: s.mode == 'idle', timeout=10.0)
        self.assertIsNotNone(idle, 'never observed initial idle state')

        # Configure the mock to return full success.
        self.mock_mapf.set_response(MockMapfServer.SUCCESS, num_planned=3)

        # Send the MAPF goal.
        goal = self._goal_mapf(
            robot_ids=[1, 2, 3],
            xy=[(5.0, 5.0), (5.5, 5.5), (6.0, 6.0)],
        )
        result = self._send_command(goal, timeout=5.0)
        self.assertTrue(
            result.success,
            f'LlmCommand did not apply: {result.info!r}',
        )

        # The mock should have received exactly one goal with our robot_ids.
        # Allow a short window for the BT thread to forward the goal.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self.mock_mapf.received_goals():
            time.sleep(0.05)
        received = self.mock_mapf.received_goals()
        self.assertEqual(len(received), 1, 'mock did not receive any goal')
        self.assertEqual(list(received[0].robot_ids), [1, 2, 3])

        # /bt/state should at some point report mode=mapf and an active action.
        running = self._wait_for_state(
            lambda s: s.mode == 'mapf' and s.active_action == 'MapfPlan',
            timeout=5.0,
        )
        self.assertIsNotNone(running, '/bt/state never showed MapfPlan running')

        # Then it should complete with action_status=OK and active_action=none.
        done = self._wait_for_state(
            lambda s: s.mode == 'mapf' and s.active_action == 'none',
            timeout=10.0,
        )
        self.assertIsNotNone(done, 'MapfPlan never completed')
        self.assertEqual(
            done.action_status, 'OK',
            f'expected OK, got {done.action_status!r}: {done.last_error!r}',
        )
