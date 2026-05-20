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
import asyncio
import tempfile
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from iros_llm_swarm_interfaces.action import LlmDecision  # noqa: E402
from iros_llm_orchestrator.decision_server import LlmDecisionServer  # noqa: E402
from iros_llm_orchestrator.context.agentic_mcp import AgenticMcpError  # noqa: E402


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


class _FakeGoalHandle:
    def __init__(self, level, event, log_buffer):
        self.request = SimpleNamespace(
            level=level,
            event=event,
            log_buffer=log_buffer,
        )
        self.stages_seen = []
        self.succeeded = False

    def publish_feedback(self, feedback):
        self.stages_seen.append(feedback.stage)

    def succeed(self):
        self.succeeded = True


def _run_on_server_loop(server, coro, timeout_sec=5.0):
    future = asyncio.run_coroutine_threadsafe(coro, server._loop)
    return future.result(timeout=timeout_sec)


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


def test_info_does_not_use_agentic_mcp_gate(running_server):
    server, _ = running_server
    server._mcp_tool_broker = object()
    server._mcp_decision_agentic_config.enabled = True
    server._mcp_decision_agentic_levels = {'WARN', 'ERROR'}

    assert server._should_use_agentic_mcp('WARN')
    assert server._should_use_agentic_mcp('ERROR')
    assert not server._should_use_agentic_mcp('INFO')


def test_decision_tool_policy_rejects_cmd_vel():
    ok, reason = LlmDecisionServer._validate_decision_tool_args(
        'subscribe_once',
        {
            'topic': '/cmd_vel',
            'msg_type': 'geometry_msgs/msg/Twist',
        },
    )

    assert not ok
    assert '/bt/state' in reason


def test_warn_uses_agentic_mcp_when_enabled(running_server, monkeypatch):
    server, _ = running_server
    called = {}

    async def agentic(prompt):
        called['agentic_prompt'] = prompt
        return 'replan', 'agentic ok'

    async def plain(_prompt):
        raise AssertionError('plain decision path should not run')

    monkeypatch.setattr(server, '_should_use_agentic_mcp', lambda _level: True)
    monkeypatch.setattr(server, '_run_agentic_decision', agentic)
    monkeypatch.setattr(server, '_run_plain_decision', plain)

    goal_handle = _FakeGoalHandle(
        level='WARN',
        event='robot_3 stalled',
        log_buffer=['WARN: robot_3 stalled'],
    )
    result = _run_on_server_loop(server, server._execute_async(goal_handle))

    assert result.decision == 'replan'
    assert goal_handle.succeeded
    assert 'done' in goal_handle.stages_seen
    assert 'read-only MCP observations' in called['agentic_prompt']


def test_mcp_failure_falls_back_safely(running_server, monkeypatch):
    server, _ = running_server
    called = {'fallback': False}

    async def agentic(_prompt):
        raise AgenticMcpError('MCP unavailable')

    async def plain(prompt):
        called['fallback'] = True
        assert 'read-only MCP observations' not in prompt
        return 'wait', 'fallback ok'

    monkeypatch.setattr(server, '_should_use_agentic_mcp', lambda _level: True)
    monkeypatch.setattr(server, '_run_agentic_decision', agentic)
    monkeypatch.setattr(server, '_run_plain_decision', plain)

    goal_handle = _FakeGoalHandle(
        level='WARN',
        event='robot_3 stalled',
        log_buffer=['WARN: robot_3 stalled'],
    )
    result = _run_on_server_loop(server, server._execute_async(goal_handle))

    assert result.decision == 'wait'
    assert called['fallback']
    assert goal_handle.succeeded
