#!/usr/bin/env python3
"""
Test script: sends goals to the /swarm/set_goals action for MAPF planning
and execution. The action stays alive until all robots arrive at their
goals (long-lived action), so this script keeps running and printing
feedback until the mission completes or is cancelled with Ctrl+C.

Usage:
  # Send all robots to a single point (spread on a 1.0m grid)
  ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0

  # Send each robot to a random point around map centre
  ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

  # Set goals from a JSON file: {"goals": [{"id": 0, "gx": ..., "gy": ...}, ...]}
  ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json

  # Custom timeout (default 600s = 10 min)
  ros2 run iros_llm_swarm_mapf test_send_goals --random --timeout 300
"""

import argparse
import json
import math
import random
import signal
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import SetGoals


class GoalSender(Node):
    def __init__(self, args):
        super().__init__('test_send_goals')
        self.args = args
        self.client = ActionClient(self, SetGoals, '/swarm/set_goals')
        self.goal_handle = None

    def send(self):
        self.get_logger().info('Waiting for /swarm/set_goals action server...')
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        goal_msg = self._build_goal()
        self.get_logger().info(
            f'Sending goal for {len(goal_msg.robot_ids)} robots...')

        send_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)

        if send_future.result() is None:
            self.get_logger().error('Goal send timed out')
            return False

        self.goal_handle = send_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self.get_logger().info('Goal accepted — mission in progress...')
        return True

    def wait_for_result(self, timeout_sec=600.0):
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=timeout_sec)

        if result_future.result() is None:
            self.get_logger().error(
                f'Result timed out after {timeout_sec:.0f} s')
            return False

        res = result_future.result().result
        self._print_result(res)
        return res.success

    def cancel(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future,
                                             timeout_sec=5.0)

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        s = fb.status

        if s in ('validating', 'planning', 'publishing', 'failed'):
            # Planning phase — simple status line
            self.get_logger().info(
                f'  [{fb.elapsed_ms:>6} ms] {s}')

        elif s == 'executing':
            # Execution phase — arrival progress
            total = fb.robots_arrived + fb.robots_active
            self.get_logger().info(
                f'  [{fb.elapsed_ms:>6} ms] executing: '
                f'{fb.robots_arrived}/{total} arrived, '
                f'{fb.replans_done} replans')

        elif s == 'replanning':
            self.get_logger().warn(
                f'  [{fb.elapsed_ms:>6} ms] replanning: '
                f'{fb.robots_deviated} deviated, '
                f'{fb.replans_done} replans so far')

        else:
            self.get_logger().info(
                f'  [{fb.elapsed_ms:>6} ms] {s}')

    # ------------------------------------------------------------------
    # Result
    # ------------------------------------------------------------------

    def _print_result(self, res):
        if res.success:
            self.get_logger().info(
                f'=== MISSION COMPLETE ===\n'
                f'  {res.num_agents_planned} agents, '
                f'{res.planning_time_ms:.1f} ms planning\n'
                f'  PBS: {res.pbs_expansions} exp, '
                f'A*: {res.astar_ok_count} ok '
                f'(avg {res.astar_avg_exp} / max {res.astar_max_exp}), '
                f'{res.astar_fail_count} failed\n'
                f'  max path {res.max_path_length} steps\n'
                f'  {res.total_replans} replans, '
                f'{res.total_execution_sec:.1f} s execution')
            if res.message not in ('All robots arrived', 'OK'):
                self.get_logger().warn(res.message)
        else:
            self.get_logger().error(
                f'Mission failed (error_code={res.error_code}): '
                f'{res.message}')

    # ------------------------------------------------------------------
    # Goal building (unchanged logic)
    # ------------------------------------------------------------------

    def _build_goal(self):
        goal_msg = SetGoals.Goal()
        a = self.args

        if a.json_file:
            with open(a.json_file) as f:
                payload = json.load(f)
            for entry in payload['goals']:
                goal_msg.robot_ids.append(int(entry['id']))
                goal_msg.goals.append(
                    Point(x=float(entry['gx']), y=float(entry['gy']), z=0.0))
        else:
            for i in range(a.num):
                if a.random:
                    ang = random.uniform(0.0, 2 * math.pi)
                    r   = a.radius * math.sqrt(random.random())
                    cx, cy = 15.0, 15.0
                    gx = cx + r * math.cos(ang)
                    gy = cy + r * math.sin(ang)
                elif a.goal_x is not None and a.goal_y is not None:
                    step   = 1.0
                    cols_n = math.ceil(math.sqrt(a.num))
                    row_i  = i // cols_n
                    col_i  = i % cols_n
                    gx = a.goal_x + (col_i - (cols_n - 1) / 2.0) * step
                    gy = a.goal_y + (row_i - (cols_n - 1) / 2.0) * step
                else:
                    print('Specify --goal-x/--goal-y or --random')
                    sys.exit(1)

                goal_msg.robot_ids.append(i)
                goal_msg.goals.append(Point(x=gx, y=gy, z=0.0))

        return goal_msg


def main():
    parser = argparse.ArgumentParser(
        description='Send swarm goals via /swarm/set_goals action')
    parser.add_argument('--num',       type=int,   default=20)
    parser.add_argument('--goal-x',    type=float, default=None)
    parser.add_argument('--goal-y',    type=float, default=None)
    parser.add_argument('--random',    action='store_true')
    parser.add_argument('--radius',    type=float, default=5.0)
    parser.add_argument('--json-file', type=str,   default=None)
    parser.add_argument('--timeout',   type=float, default=600.0,
                        help='Max seconds to wait for mission completion')
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender(args)

    # Ctrl+C cancels the goal gracefully, then exits
    def sigint_handler(sig, frame):
        node.cancel()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)

    if not node.send():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    success = node.wait_for_result(timeout_sec=args.timeout)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()