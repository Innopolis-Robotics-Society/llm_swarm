"""Integration test for PassiveObserver.

Spins the observer with llm_mode=mock, stands up a fake /llm/command
action server, publishes a WARN BTState and checks that:

  * the observer sent a goal to /llm/command,
  * the dataset file is written,
  * a rapid second WARN within the cooldown window does not trigger a
    second goal.
"""

import os
import tempfile
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from iros_llm_swarm_interfaces.action import LlmCommand  # noqa: E402
from iros_llm_swarm_interfaces.msg import BTState  # noqa: E402

from iros_llm_orchestrator.passive_observer import PassiveObserver  # noqa: E402


class _FakeCommandServer(Node):
    def __init__(self):
        super().__init__('fake_llm_command_server')
        self.received_goals = []
        self._server = ActionServer(
            self, LlmCommand, '/llm/command', self._execute,
        )

    def _execute(self, goal_handle):
        self.received_goals.append(goal_handle.request)
        fb = LlmCommand.Feedback()
        fb.stage = 'applied'
        goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        result = LlmCommand.Result()
        result.success = True
        result.info = 'ok'
        return result


def _spin(executor, stop_event):
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


@pytest.fixture
def running_stack():
    rclpy.init()
    tmp_dir = tempfile.mkdtemp(prefix='llm_commands_')

    observer = PassiveObserver()
    observer._logger_ds.path = tmp_dir
    os.makedirs(tmp_dir, exist_ok=True)
    # Shorter cooldown so the second-trigger test runs inside the window.
    observer.set_parameters([
        rclpy.parameter.Parameter(
            'cooldown_sec', rclpy.Parameter.Type.DOUBLE, 5.0),
        rclpy.parameter.Parameter(
            'enabled', rclpy.Parameter.Type.BOOL, True),
    ])

    fake_server = _FakeCommandServer()

    publisher_node = rclpy.create_node('test_bt_state_publisher')
    pub = publisher_node.create_publisher(BTState, '/bt/state', 10)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(observer)
    executor.add_node(fake_server)
    executor.add_node(publisher_node)

    stop_event = threading.Event()
    thread = threading.Thread(target=_spin, args=(executor, stop_event), daemon=True)
    thread.start()

    yield observer, fake_server, pub, tmp_dir

    stop_event.set()
    thread.join(timeout=2.0)
    executor.shutdown()
    executor.remove_node(observer)
    executor.remove_node(fake_server)
    executor.remove_node(publisher_node)
    observer.destroy_node()
    fake_server.destroy_node()
    publisher_node.destroy_node()
    rclpy.shutdown()


def _make_state(status='WARN', action='MapfPlan', err='robot_3 stalled'):
    msg = BTState()
    msg.mode = 'mapf'
    msg.action_status = status
    msg.active_action = action
    msg.action_summary = err
    msg.last_error = err
    msg.stamp_ms = int(time.time() * 1000)
    return msg


def test_observer_triggers_on_warn(running_stack):
    _, fake_server, pub, tmp_dir = running_stack

    # Give publisher time to discover the subscription.
    time.sleep(0.5)

    pub.publish(_make_state())

    deadline = time.time() + 8.0
    while time.time() < deadline and not fake_server.received_goals:
        time.sleep(0.1)

    assert fake_server.received_goals, 'observer did not send a goal on WARN'
    goal = fake_server.received_goals[0]
    assert goal.mode in ('idle', 'mapf', 'formation')

    files = [f for f in os.listdir(tmp_dir) if f.endswith('.jsonl')]
    assert files, f'no JSONL dataset file written in {tmp_dir}'


def test_cooldown_suppresses_second_trigger(running_stack):
    _, fake_server, pub, _ = running_stack

    time.sleep(0.5)

    pub.publish(_make_state())
    deadline = time.time() + 8.0
    while time.time() < deadline and not fake_server.received_goals:
        time.sleep(0.1)
    assert len(fake_server.received_goals) == 1

    # Second WARN within cooldown: must be ignored.
    pub.publish(_make_state(err='robot_5 also stalled'))
    time.sleep(1.5)
    assert len(fake_server.received_goals) == 1, (
        'cooldown failed — observer triggered twice in the window'
    )
