"""
Integration test for the LlmDecisionServer.

Spins the server in a background thread with llm_mode=mock, fires a few
action-client requests and checks that:

  * the action returns a non-empty decision,
  * each mock keyword maps to the expected decision,
  * a JSONL dataset file is written for every call,
  * the action feedback reaches the "done" stage.
"""

import os
import tempfile
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from iros_llm_swarm_interfaces.action import LlmDecision  # noqa: E402
from iros_llm_orchestrator.decision_server import LlmDecisionServer  # noqa: E402


class _ActionClientNode(Node):
    def __init__(self):
        super().__init__('test_llm_decision_client')
        self._client = ActionClient(self, LlmDecision, '/llm/decision')
        self.stages_seen = []

    def wait_for_server(self, timeout_sec=5.0):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send(self, level, event, log_buffer, timeout_sec=5.0):
        goal = LlmDecision.Goal()
        goal.level = level
        goal.event = event
        goal.log_buffer = log_buffer

        self.stages_seen = []

        def on_feedback(fb_msg):
            self.stages_seen.append(fb_msg.feedback.stage)

        send_goal_future = self._client.send_goal_async(goal, feedback_callback=on_feedback)
        self._spin(send_goal_future, timeout_sec)
        goal_handle = send_goal_future.result()
        assert goal_handle is not None and goal_handle.accepted

        result_future = goal_handle.get_result_async()
        self._spin(result_future, timeout_sec)
        return result_future.result().result

    def _spin(self, future, timeout_sec):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                raise TimeoutError('action client timed out')


def _spin_server(executor, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


@pytest.fixture
def running_server():
    rclpy.init()
    tmp_dir = tempfile.mkdtemp(prefix='llm_decisions_')

    server = LlmDecisionServer()
    server._logger_ds.path = tmp_dir
    os.makedirs(tmp_dir, exist_ok=True)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server)

    stop_event = threading.Event()
    thread = threading.Thread(target=_spin_server, args=(executor, stop_event), daemon=True)
    thread.start()

    yield server, tmp_dir

    stop_event.set()
    thread.join(timeout=2.0)
    executor.shutdown()
    executor.remove_node(server)
    server.destroy_node()
    rclpy.shutdown()


def test_mock_replan_on_stall(running_server):
    _, tmp_dir = running_server

    client = _ActionClientNode()
    assert client.wait_for_server(timeout_sec=5.0)

    result = client.send(
        level='WARN',
        event='robot_3 stalled for 5s',
        log_buffer=['[t=1200ms status=executing] WARN: robot_3 stalled'],
    )

    assert result.decision == 'replan'
    assert 'done' in client.stages_seen
    assert 'received' in client.stages_seen

    files = [f for f in os.listdir(tmp_dir) if f.endswith('.jsonl')]
    assert files, f'no JSONL dataset file in {tmp_dir}'

    client.destroy_node()


def test_mock_abort_on_collision(running_server):
    _, _ = running_server
    client = _ActionClientNode()
    assert client.wait_for_server(timeout_sec=5.0)

    result = client.send(
        level='WARN',
        event='fatal collision near checkpoint C',
        log_buffer=['[t=2200ms status=failed] WARN: fatal collision near checkpoint C'],
    )

    assert result.decision == 'abort'
    client.destroy_node()


def test_mock_wait_on_healthy_heartbeat(running_server):
    _, _ = running_server
    client = _ActionClientNode()
    assert client.wait_for_server(timeout_sec=5.0)

    result = client.send(
        level='INFO',
        event='planner progressing normally',
        log_buffer=['[t=2500ms status=executing arrived=8 active=12] INFO: planner progressing normally'],
    )

    assert result.decision == 'wait'
    client.destroy_node()
