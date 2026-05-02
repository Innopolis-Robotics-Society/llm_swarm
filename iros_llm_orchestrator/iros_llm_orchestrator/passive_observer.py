"""PassiveObserver — channel 2 of the LLM pipeline.

Subscribes to /bt/state, and whenever action_status transitions to WARN or
ERROR (respecting a cooldown), asks the LLM to emit a full command that
gets applied via the /llm/command action. Runs independently from the
reactive /llm/decision server.
"""
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import asyncio
import os
from collections import deque
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from iros_llm_swarm_interfaces.action import LlmCommand
from iros_llm_swarm_interfaces.msg import BTState

from iros_llm_orchestrator.command_parser import parse_llm_command
from iros_llm_orchestrator.command_prompt_builder import build_command_prompt
from iros_llm_orchestrator.command_scenarios import COMMAND_SCENARIOS
from iros_llm_orchestrator.decision_logger import DecisionLogger
from iros_llm_orchestrator.llm_client import LLMClient


class PassiveObserver(Node):
    TRIGGER_STATUSES = {'WARN', 'ERROR'}

    def __init__(self):
        super().__init__('llm_passive_observer')
        

        self.declare_parameter('llm_mode', 'mock')
        self.declare_parameter('llm_endpoint', 'http://localhost:8000/v1/completions')
        self.declare_parameter('llm_model', 'nvidia/Nemotron-3-Nano-30B-A3B')
        self.declare_parameter('llm_max_tokens', 512)
        self.declare_parameter('llm_temperature', 0.3)
        self.declare_parameter('history_size', 20)
        self.declare_parameter('cooldown_sec', 10.0)
        self.declare_parameter('timeout_sec', 15.0)
        self.declare_parameter('dataset_path', '~/.ros/llm_commands')
        self.declare_parameter('enabled', False)

        self.history = deque(maxlen=int(self.get_parameter('history_size').value))
        self.last_action_time = 0.0
        self.is_thinking = False

        self.llm = LLMClient(
            mode=self.get_parameter('llm_mode').value,
            endpoint=self.get_parameter('llm_endpoint').value,
            model=self.get_parameter('llm_model').value,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        self.dataset_logger = DecisionLogger(
            path=os.path.expanduser(self.get_parameter('dataset_path').value),
        )

        self.cb_group = ReentrantCallbackGroup()
        

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,   
        )

        self.state_sub = self.create_subscription(
            BTState, '/bt/state', self.on_state, qos,
            callback_group=self.cb_group,
        )
        self.cmd_client = ActionClient(
            self, LlmCommand, '/llm/command',
            callback_group=self.cb_group,
        )

        self.get_logger().info(
            f'PassiveObserver up: mode={self.get_parameter("llm_mode").value}, '
            f'cooldown={self.get_parameter("cooldown_sec").value}s, '
            f'trigger=action_status in {sorted(self.TRIGGER_STATUSES)}'
        )

        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True)
        self._loop_thread.start()

    def on_state(self, msg: BTState):
        if not self.get_parameter('enabled').value:
            return
        self.history.append(msg)

        if msg.action_status not in self.TRIGGER_STATUSES:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        cooldown = float(self.get_parameter('cooldown_sec').value)
        if now - self.last_action_time < cooldown:
            return
        if self.is_thinking:
            return

        self.is_thinking = True
        self.last_action_time = now

        self.get_logger().warn(
            f'Triggered by status={msg.action_status} '
            f'action={msg.active_action} error={msg.last_error!r}'
        )

        self.executor.create_task(self.think_and_command(msg))

    async def think_and_command(self, trigger_msg: BTState):
        prompt = ''
        raw_output = ''
        command = None
        error_note = ''

        try:
            prompt = build_command_prompt(
                scenarios=COMMAND_SCENARIOS,
                history=list(self.history),
                trigger=trigger_msg,
            )

            raw_output = await asyncio.wait_for(
                self.llm.generate(prompt, prompt_kind='command'),
                timeout=float(self.get_parameter('timeout_sec').value),
            )

            command = parse_llm_command(raw_output)
            self.get_logger().info(f'LLM command: {command}')

            if not self.cmd_client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError('/llm/command server unavailable')

            goal = self._build_goal(command)
            handle = await self.cmd_client.send_goal_async(goal)

            if not handle.accepted:
                error_note = 'goal rejected by LlmCommandReceiver'
                self.get_logger().error(error_note)
            else:
                result = await handle.get_result_async()
                self.get_logger().info(
                    f'Applied: success={result.result.success} info={result.result.info}'
                )

        except asyncio.TimeoutError:
            error_note = 'LLM timeout'
            self.get_logger().error('LLM timed out')
        except Exception as exc:  # noqa: BLE001
            error_note = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(f'pipeline failed: {error_note}')

        try:
            self.dataset_logger.log({
                'timestamp': datetime.utcnow().isoformat(),
                'trigger': {
                    'action_status': trigger_msg.action_status,
                    'active_action': trigger_msg.active_action,
                    'last_error': trigger_msg.last_error,
                    'action_summary': trigger_msg.action_summary,
                },
                'history_snapshot': [self._state_to_dict(s) for s in self.history],
                'prompt': prompt,
                'raw_output': raw_output,
                'command': command,
                'error': error_note,
            })
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'dataset logger failed: {exc}')

        self.is_thinking = False

    def _build_goal(self, command: dict) -> LlmCommand.Goal:
        goal = LlmCommand.Goal()
        goal.mode = command['mode']
        goal.reason = command.get('reason', '')
        goal.robot_ids = [int(r) for r in command.get('robot_ids', [])]
        goal.goals = [
            Point(x=float(g[0]), y=float(g[1]), z=0.0)
            for g in command.get('goals', [])
        ]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns = command.get('leader_ns', '')
        goal.follower_ns = list(command.get('follower_ns', []))
        goal.offsets_x = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y = [float(o) for o in command.get('offsets_y', [])]
        return goal

    @staticmethod
    def _state_to_dict(msg: BTState) -> dict:
        return {
            'mode': msg.mode,
            'action_status': msg.action_status,
            'active_action': msg.active_action,
            'action_summary': msg.action_summary,
            'last_error': msg.last_error,
            'stamp_ms': msg.stamp_ms,
        }


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
        node._loop.call_soon_threadsafe(node._loop.stop)
        node._loop_thread.join(timeout=2.0)
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
