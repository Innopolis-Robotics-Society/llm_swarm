"""Channel 2 — proactive passive observer (/bt/state watcher → /llm/command).

Watches BT state stream. When action_status transitions to WARN/ERROR
and channel 1 is not already active (llm_thinking=False), asks the LLM
for a full command and sends it to the BT via /llm/command action.

Disabled by default (enabled=false). Enable via:
  ros2 param set /llm_passive_observer enabled true
  or launch arg: enable_passive_observer:=true
"""

import asyncio
import threading
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState

from iros_llm_orchestrator.common.llm_factory import get_llm_client
from iros_llm_orchestrator.common.parsers import parse_llm_command
from iros_llm_orchestrator.common.command_prompt import build_command_prompt
from iros_llm_orchestrator.common.scenarios import COMMAND_SCENARIOS
from iros_llm_orchestrator.common.logger import DecisionLogger


class PassiveObserver(Node):
    TRIGGER_STATUSES = ('WARN', 'ERROR')

    def __init__(self):
        super().__init__('llm_passive_observer')

        self.declare_parameter('llm_mode',        'mock')
        self.declare_parameter('llm_endpoint',    '')
        self.declare_parameter('llm_model',       '')
        self.declare_parameter('llm_max_tokens',  512)
        self.declare_parameter('llm_temperature', 0.3)
        self.declare_parameter('timeout_sec',     15.0)
        self.declare_parameter('history_size',    20)
        self.declare_parameter('cooldown_sec',    10.0)
        self.declare_parameter('dataset_path',    '~/.ros/llm_commands')
        self.declare_parameter('enabled',         False)

        mode     = self.get_parameter('llm_mode').value
        endpoint = self.get_parameter('llm_endpoint').value or None
        model    = self.get_parameter('llm_model').value

        self._llm = get_llm_client(
            mode=mode,
            endpoint=endpoint,
            model=model,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        self._timeout     = float(self.get_parameter('timeout_sec').value)
        self._cooldown    = float(self.get_parameter('cooldown_sec').value)
        self._history     = deque(maxlen=int(self.get_parameter('history_size').value))
        self._logger_ds   = DecisionLogger(self.get_parameter('dataset_path').value)

        self._last_action_time = 0.0
        self._is_thinking      = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(BTState, '/bt/state', self._on_state, qos)
        self._cmd_client = ActionClient(self, LlmCommand, '/llm/command')

        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        self.get_logger().info(f'PassiveObserver ready (mode={mode}, '
                               f'enabled={self.get_parameter("enabled").value})')

    def _on_state(self, msg: BTState):
        if not self.get_parameter('enabled').value:
            return
        self._history.append(msg)

        if msg.action_status not in self.TRIGGER_STATUSES:
            return

        # Don't double-trigger if channel 1 is already consulting LLM
        if msg.llm_thinking:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_action_time < self._cooldown:
            return
        if self._is_thinking:
            return

        self._is_thinking      = True
        self._last_action_time = now

        self.get_logger().warn(
            f'Triggered by status={msg.action_status} '
            f'action={msg.active_action} error={msg.last_error!r}')

        # asyncio.run_coroutine_threadsafe is correct here:
        # self._loop is a dedicated event loop started in __init__,
        # not the ROS executor loop.
        asyncio.run_coroutine_threadsafe(
            self._think_and_command(msg), self._loop)

    async def _think_and_command(self, trigger: BTState):
        try:
            prompt = build_command_prompt(
                COMMAND_SCENARIOS, list(self._history), trigger)
            raw = await asyncio.wait_for(
                self._llm.generate(prompt, prompt_kind='command'),
                timeout=self._timeout)
            command = parse_llm_command(raw)
        except asyncio.TimeoutError:
            self.get_logger().warn('PassiveObserver: LLM timeout')
            self._is_thinking = False
            return
        except Exception as exc:
            self.get_logger().error(f'PassiveObserver: error: {exc}')
            self._is_thinking = False
            return

        self._logger_ds.log(
            level=trigger.action_status,
            event=f'{trigger.active_action}: {trigger.last_error}',
            log_buffer=[],
            decision=command.get('mode', 'idle'),
            reason=command.get('reason', ''))

        await self._send_command(command)
        self._is_thinking = False

    async def _send_command(self, command: dict):
        from geometry_msgs.msg import Point

        if not self._cmd_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('PassiveObserver: /llm/command not available')
            return

        goal = LlmCommand.Goal()
        goal.mode         = command['mode']
        goal.reason       = command.get('reason', '')
        goal.robot_ids    = [int(r) for r in command.get('robot_ids', [])]
        goal.goals        = [Point(x=float(g[0]), y=float(g[1]), z=0.0)
                             for g in command.get('goals', [])]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns    = command.get('leader_ns', '')
        goal.follower_ns  = list(command.get('follower_ns', []))
        goal.offsets_x    = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y    = [float(o) for o in command.get('offsets_y', [])]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            self.get_logger().warn('PassiveObserver: goal rejected by BT')
            return
        result = await handle.get_result_async()
        self.get_logger().info(
            f'PassiveObserver: command applied success={result.result.success}')


def main(args=None):
    rclpy.init(args=args)
    node = PassiveObserver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
