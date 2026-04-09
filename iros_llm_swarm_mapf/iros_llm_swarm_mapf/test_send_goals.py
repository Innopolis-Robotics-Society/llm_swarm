#!/usr/bin/env python3
"""
Тестовый скрипт: вызывает сервис /swarm/set_goals для планирования.
Старты берутся из одометрии планировщиком автоматически.

Использование:
  # Отправить всех роботов в одну точку (раскидывает по сетке 0.4м)
  ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0

  # Отправить каждого в случайную точку вокруг центра карты
  ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

  # Задать цели из JSON-файла: {"goals": [{"id": 0, "gx": ..., "gy": ...}, ...]}
  ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json
"""

import argparse
import json
import math
import random
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.srv import SetGoals


def main():
    parser = argparse.ArgumentParser(description='Send swarm goals via /swarm/set_goals service')
    parser.add_argument('--num',       type=int,   default=20)
    parser.add_argument('--goal-x',    type=float, default=None)
    parser.add_argument('--goal-y',    type=float, default=None)
    parser.add_argument('--random',    action='store_true')
    parser.add_argument('--radius',    type=float, default=5.0)
    parser.add_argument('--json-file', type=str,   default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node('test_send_goals')
    client = node.create_client(SetGoals, '/swarm/set_goals')

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service /swarm/set_goals not available')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    req = SetGoals.Request()

    if args.json_file:
        with open(args.json_file) as f:
            payload = json.load(f)
        for entry in payload['goals']:
            req.robot_ids.append(int(entry['id']))
            req.goals.append(Point(x=float(entry['gx']), y=float(entry['gy']), z=0.0))

    else:
        for i in range(args.num):
            if args.random:
                ang = random.uniform(0.0, 2 * math.pi)
                r = args.radius * math.sqrt(random.random())
                # Центр карты как база для случайных целей
                cx, cy = 15.0, 15.0
                gx = cx + r * math.cos(ang)
                gy = cy + r * math.sin(ang)
            elif args.goal_x is not None and args.goal_y is not None:
                # Раскидываем роботов по сетке вокруг целевой точки.
                # Шаг должен быть > 2 * robot_radius (hard boundary),
                # чтобы роботы физически поместились.  Мягкая зона
                # (inflation) обрабатывается градиентом в PBS.
                step = 1.0
                cols_n = math.ceil(math.sqrt(args.num))
                row_i = i // cols_n
                col_i = i % cols_n
                offset_x = (col_i - (cols_n - 1) / 2.0) * step
                offset_y = (row_i - (cols_n - 1) / 2.0) * step
                gx = args.goal_x + offset_x
                gy = args.goal_y + offset_y
            else:
                print("Укажи --goal-x/--goal-y или --random")
                sys.exit(1)

            req.robot_ids.append(i)
            req.goals.append(Point(x=gx, y=gy, z=0.0))

    node.get_logger().info(
        f"Calling /swarm/set_goals for {len(req.robot_ids)} robots...")

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)

    if future.result() is None:
        node.get_logger().error('Service call timed out or failed')
    else:
        res = future.result()
        if res.success:
            node.get_logger().info(
                f"{res.num_agents_planned} agents, "
                f"{res.planning_time_ms:.1f} ms, "
                f"{res.pbs_expansions} PBS exp, "
                f"A*: {res.astar_ok_count} ok "
                f"(avg {res.astar_avg_exp} / max {res.astar_max_exp} exp), "
                f"{res.astar_fail_count} failed, "
                f"max path {res.max_path_length} steps")
            if res.message != "OK":
                node.get_logger().warn(res.message)
        else:
            node.get_logger().error(f"Planning failed: {res.message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
