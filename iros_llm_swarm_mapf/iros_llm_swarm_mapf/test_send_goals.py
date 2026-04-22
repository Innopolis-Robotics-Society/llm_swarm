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
from visualization_msgs.msg import Marker, MarkerArray

from iros_llm_swarm_interfaces.action import SetGoals


class GoalSender(Node):
    def __init__(self, args):
        super().__init__('test_send_goals')
        self.args = args
        self.client = ActionClient(self, SetGoals, '/swarm/set_goals')
        self.goal_handle = None

        # RViz publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self._last_markers = None

    def send(self):
        self.get_logger().info('Waiting for /swarm/set_goals action server...')
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        goal_msg = self._build_goal()

        # publish markers
        self._publish_markers(goal_msg.goals)

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
    # RViz markers
    # ------------------------------------------------------------------

    def _publish_markers(self, goals):
        markers = MarkerArray()

        for i, p in enumerate(goals):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = "goals"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = p.x
            m.pose.position.y = p.y
            m.pose.position.z = 0.0

            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3

            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 1.0

            markers.markers.append(m)

        self._last_markers = markers
        self.marker_pub.publish(markers)

        # повторная публикация (чтобы RViz не терял маркеры)
        self.create_timer(1.0, self._republish_markers)

    def _republish_markers(self):
        if self._last_markers is not None:
            self.marker_pub.publish(self._last_markers)

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------
    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        s = fb.status

        # Базовая информация (всегда показываем)
        base = f'[{fb.elapsed_ms:>6} ms] {s}'

        # Основная статистика выполнения
        if s == 'executing':
            total = fb.robots_arrived + fb.robots_active
            self.get_logger().info(
                f'{base} | '
                f'arrived: {fb.robots_arrived}/{total} | '
                f'active: {fb.robots_active} | '
                f'replans: {fb.replans_done}'
            )

        elif s == 'replanning':
            self.get_logger().warn(
                f'{base} | '
                f'deviated: {fb.robots_deviated} | '
                f'replans: {fb.replans_done} | '
                f'stalled: {fb.robot_stall}'
            )

        elif s in ('validating', 'planning', 'publishing', 'failed'):
            self.get_logger().info(base)

        else:
            self.get_logger().info(base)

        # Дополнительные поля (показываем всегда, если они не пустые)
        if fb.info:
            self.get_logger().info(f'    INFO: {fb.info}')

        if fb.warning:
            self.get_logger().warn(f'    WARNING: {fb.warning}')

        # Полная статистика (выводим в executing/replanning и при наличии stall)
        if fb.robots_arrived or fb.robots_active or fb.robots_deviated or fb.robot_stall:
            self.get_logger().debug(
                f'    → arrived={fb.robots_arrived} | '
                f'active={fb.robots_active} | '
                f'deviated={fb.robots_deviated} | '
                f'stalled={fb.robot_stall} | '
                f'replans={fb.replans_done}'
            )

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
    # Goal building
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
    parser.add_argument('--timeout',   type=float, default=600.0)
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender(args)

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