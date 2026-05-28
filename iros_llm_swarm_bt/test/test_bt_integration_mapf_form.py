"""
test_bt_integration_formation.py — 3-robot triangle / formation flow.

Exercises a three-step mission against the production BT host (bt_runner)
with mock downstream servers:

  1. MAPF      — send 3 robots into a triangle around point A
  2. Formation — lock them into a triangle formation (leader + 2 followers)
  3. MAPF      — relocate the group to a triangle around point B

Architectural note on step 3: formation and MAPF are mutually-exclusive
modes in this system (a formation does not "drive" via MAPF). To relocate a
formed group we therefore issue a fresh MAPF command planning all three
members to the new triangle around B. If you later add a leader-nav channel
that moves a live formation, that would be a different (layered) flow not
modeled here.

Mocks live in this file (self-contained):
  - MockMapfServer       — action server for /swarm/set_goals
  - MockFormationService — services /formation/set, /formation/deactivate,
                           and publishes STABLE /formations/status

Assumed interface names (adjust the imports below if your .srv / .msg names
differ):
  - iros_llm_swarm_interfaces/srv/SetFormation
  - iros_llm_swarm_interfaces/srv/DeactivateFormation
  - iros_llm_swarm_interfaces/msg/FormationsStatus  (array of FormationStatus)
  - iros_llm_swarm_interfaces/msg/FormationStatus

Wire it into colcon with, in CMakeLists.txt:
    add_launch_test(test/test_bt_integration_formation.py)
"""

from __future__ import annotations

import math
import threading
import time
import unittest
from typing import Callable, List, Optional, Tuple

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch_ros.actions import Node as LaunchNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import LlmCommand, SetGoals
from iros_llm_swarm_interfaces.msg import BTState, FormationsStatus, FormationStatus
from iros_llm_swarm_interfaces.srv import DeactivateFormation, SetFormation


# Mission parameters --------------------------------------------------------
ROBOT_IDS: List[int] = [3, 7, 12]
POINT_A: Tuple[float, float] = (5.0, 5.0)
POINT_B: Tuple[float, float] = (20.0, 20.0)
TRIANGLE_RADIUS = 1.5


def triangle_around(cx: float, cy: float, radius: float = TRIANGLE_RADIUS):
    """
    Return the 3 vertices of an equilateral triangle centered on (cx, cy).

    The first vertex points "up" (+y); the others are spaced 120 degrees apart.
    """
    pts = []
    for k in range(3):
        ang = math.pi / 2.0 + k * 2.0 * math.pi / 3.0
        pts.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang)))
    return pts


# Launch description --------------------------------------------------------
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bt_runner = LaunchNode(
        package='iros_llm_swarm_bt',
        executable='bt_runner',
        name='bt_runner',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )
    return (
        launch.LaunchDescription([
            bt_runner,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'bt_runner': bt_runner},
    )


# Mock /swarm/set_goals -----------------------------------------------------
class MockMapfServer:
    """
    Mock action server for /swarm/set_goals.

    Accepts a goal, streams a few feedback messages, then returns success
    with num_agents_planned == len(robot_ids). Records every received goal.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._received: List[SetGoals.Goal] = []
        self._server = ActionServer(
            node,
            SetGoals,
            '/swarm/set_goals',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
        )

    def received_goals(self) -> List[SetGoals.Goal]:
        with self._lock:
            return list(self._received)

    def _execute(self, goal_handle):
        request: SetGoals.Goal = goal_handle.request
        with self._lock:
            self._received.append(request)
        n = len(request.robot_ids)

        for elapsed in (50, 200, 400):
            fb = SetGoals.Feedback()
            fb.elapsed_ms = elapsed
            fb.status = 'executing'
            fb.robots_arrived = n if elapsed >= 400 else 0
            fb.robots_active = 0 if elapsed >= 400 else n
            fb.robot_stall = 0
            fb.replans_done = 0
            fb.info = f'mock progress {elapsed}ms'
            fb.warning = ''
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        result = SetGoals.Result()
        result.success = True
        result.message = 'mock: all agents planned'
        result.num_agents_planned = n
        result.planning_time_ms = 40.0
        result.total_replans = 0
        goal_handle.succeed()
        return result


# Mock /formation/* ---------------------------------------------------------
class MockFormationService:
    """
    Mock for the formation subsystem.

    Serves /formation/set and /formation/deactivate, and publishes a STABLE
    /formations/status for whichever formation is currently set so the BT's
    FormationHealthMonitor populates formation_state in /bt/state.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._set_requests: List[SetFormation.Request] = []
        self._deactivate_requests: List[DeactivateFormation.Request] = []
        self._active_formation_id: Optional[str] = None

        self._set_srv = node.create_service(
            SetFormation, '/formation/set', self._on_set)
        self._deactivate_srv = node.create_service(
            DeactivateFormation, '/formation/deactivate', self._on_deactivate)

        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._status_pub = node.create_publisher(
            FormationsStatus, '/formations/status', status_qos)
        self._status_timer = node.create_timer(0.2, self._publish_status)

    def set_requests(self) -> List[SetFormation.Request]:
        with self._lock:
            return list(self._set_requests)

    def _on_set(self, request, response):
        with self._lock:
            self._set_requests.append(request)
            self._active_formation_id = request.formation_id
        response.success = True
        response.message = 'mock: formation set'
        return response

    def _on_deactivate(self, request, response):
        with self._lock:
            self._deactivate_requests.append(request)
            self._active_formation_id = None
        response.success = True
        response.message = 'mock: formation deactivated'
        return response

    def _publish_status(self):
        with self._lock:
            fid = self._active_formation_id
        if not fid:
            return
        fs = FormationStatus()
        fs.formation_id = fid
        fs.state = FormationStatus.STATE_STABLE
        fs.failure_code = FormationStatus.FAILURE_NONE
        fs.failure_reason = ''
        fs.max_error_m = 0.05
        fs.mean_error_m = 0.02
        msg = FormationsStatus()
        msg.formations = [fs]
        self._status_pub.publish(msg)


# Test ----------------------------------------------------------------------
class TestBTFormationTriangle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_bt_formation_node')
        self.mock_mapf = MockMapfServer(self.node)
        self.mock_formation = MockFormationService(self.node)
        self.cmd_client = ActionClient(self.node, LlmCommand, '/llm/command')

        self._state_lock = threading.Lock()
        self._state_cv = threading.Condition(self._state_lock)
        self._states: List[BTState] = []
        state_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._state_sub = self.node.create_subscription(
            BTState, '/bt/state', self._on_state, state_qos)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        ok = self.cmd_client.wait_for_server(timeout_sec=20.0)
        self.assertTrue(ok, '/llm/command never came up — is bt_runner running?')

    def tearDown(self):
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

    def _mark(self) -> int:
        with self._state_cv:
            return len(self._states)

    def _wait_for_state(
        self,
        predicate: Callable[[BTState], bool],
        timeout: float,
        since: int = 0,
    ) -> Optional[BTState]:
        """
        Wait for a /bt/state at or after index `since` matching `predicate`.

        Using `since` avoids matching a stale completion state from an earlier
        step (e.g. both step 1 and step 3 end in mode=mapf, active_action=none).
        """
        deadline = time.monotonic() + timeout
        with self._state_cv:
            idx = since
            while True:
                while idx < len(self._states):
                    if predicate(self._states[idx]):
                        return self._states[idx]
                    idx += 1
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._state_cv.wait(timeout=remaining)

    # -- /llm/command helpers ------------------------------------------------

    def _send_command(
        self,
        goal: LlmCommand.Goal,
        timeout: float = 5.0,
    ) -> LlmCommand.Result:
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
    def _goal_mapf(robot_ids: List[int], xy) -> LlmCommand.Goal:
        g = LlmCommand.Goal()
        g.mode = 'mapf'
        g.reason = 'triangle mapf'
        g.robot_ids = list(robot_ids)
        for x, y in xy:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            g.goals.append(p)
        return g

    @staticmethod
    def _goal_formation_triangle() -> LlmCommand.Goal:
        g = LlmCommand.Goal()
        g.mode = 'formation'
        g.reason = 'triangle formation'
        g.formation_id = 'triangle_3'
        g.leader_ns = f'robot_{ROBOT_IDS[0]}'
        g.follower_ns = [f'robot_{ROBOT_IDS[1]}', f'robot_{ROBOT_IDS[2]}']
        # Two followers trailing the leader, forming a triangle.
        g.offsets_x = [-1.0, -1.0]
        g.offsets_y = [0.8, -0.8]
        return g

    # =======================================================================
    # Mission test
    # =======================================================================

    def test_triangle_mapf_formation_move(self):
        """
        MAPF triangle around A, lock formation, then MAPF to triangle around B.

        Verifies the BT walks idle -> mapf(OK) -> formation(OK) -> mapf(OK),
        and that each mock received the expected request.
        """
        idle = self._wait_for_state(lambda s: s.mode == 'idle', timeout=10.0)
        self.assertIsNotNone(idle, 'never observed initial idle state')

        # --- Step 1: MAPF 3 robots into a triangle around A -----------------
        mark = self._mark()
        goal1 = self._goal_mapf(ROBOT_IDS, triangle_around(*POINT_A))
        r1 = self._send_command(goal1)
        self.assertTrue(r1.success, f'step 1 not applied: {r1.info!r}')

        done1 = self._wait_for_state(
            lambda s: s.mode == 'mapf' and s.active_action == 'none',
            timeout=10.0, since=mark)
        self.assertIsNotNone(done1, 'step 1 (MAPF around A) never completed')
        self.assertEqual(done1.action_status, 'OK', done1.last_error)

        recv = self.mock_mapf.received_goals()
        self.assertGreaterEqual(len(recv), 1, 'mock MAPF got no goal in step 1')
        self.assertEqual(list(recv[-1].robot_ids), ROBOT_IDS)
        self.assertEqual(len(recv[-1].goals), 3)

        # --- Step 2: triangle formation, leader = ROBOT_IDS[0] --------------
        mark = self._mark()
        goal2 = self._goal_formation_triangle()
        r2 = self._send_command(goal2)
        self.assertTrue(r2.success, f'step 2 not applied: {r2.info!r}')

        done2 = self._wait_for_state(
            lambda s: s.mode == 'formation' and s.active_action == 'none',
            timeout=10.0, since=mark)
        self.assertIsNotNone(done2, 'step 2 (formation) never completed')
        self.assertEqual(done2.action_status, 'OK', done2.last_error)

        freq = self.mock_formation.set_requests()
        self.assertGreaterEqual(len(freq), 1, '/formation/set never called')
        self.assertEqual(freq[-1].formation_id, 'triangle_3')
        self.assertEqual(freq[-1].leader_ns, f'robot_{ROBOT_IDS[0]}')
        self.assertEqual(len(freq[-1].follower_ns), 2)

        # The monitor should have picked up the STABLE status by now.
        stable = self._wait_for_state(
            lambda s: s.mode == 'formation'
            and s.formation_state == FormationStatus.STATE_STABLE,
            timeout=5.0, since=mark)
        self.assertIsNotNone(stable, 'formation never reported STABLE')

        # --- Step 3: relocate the group to a triangle around B (MAPF) -------
        mark = self._mark()
        goal3 = self._goal_mapf(ROBOT_IDS, triangle_around(*POINT_B))
        r3 = self._send_command(goal3)
        self.assertTrue(r3.success, f'step 3 not applied: {r3.info!r}')

        done3 = self._wait_for_state(
            lambda s: s.mode == 'mapf' and s.active_action == 'none',
            timeout=10.0, since=mark)
        self.assertIsNotNone(done3, 'step 3 (MAPF around B) never completed')
        self.assertEqual(done3.action_status, 'OK', done3.last_error)

        recv2 = self.mock_mapf.received_goals()
        self.assertGreaterEqual(len(recv2), 2, 'mock MAPF got no goal in step 3')
        self.assertEqual(list(recv2[-1].robot_ids), ROBOT_IDS)
        # Goals should now be the triangle around B, not A.
        b_pts = triangle_around(*POINT_B)
        for got, (bx, by) in zip(recv2[-1].goals, b_pts):
            self.assertAlmostEqual(got.x, bx, places=3)
            self.assertAlmostEqual(got.y, by, places=3)
