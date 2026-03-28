#!/usr/bin/env python3
"""
Тестовый скрипт: отправляет /swarm_goals для всех роботов.

Использование:
  # Отправить всех роботов в одну точку
  ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0

  # Отправить каждого в свою случайную точку вокруг центра
  ros2 run iros_llm_swarm_mapf test_send_goals --random --radius 5.0

  # Задать свои старты/цели явно через JSON-файл
  ros2 run iros_llm_swarm_mapf test_send_goals --json-file /path/to/goals.json
"""

import argparse
import json
import math
import random
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Стартовые позиции роботов из warehouse.world
DEFAULT_STARTS = [
    (2.0, 2.0), (3.5, 2.0), (2.0, 3.5), (3.5, 3.5),
    (2.0, 5.0), (3.5, 5.0), (2.0, 6.5), (3.5, 6.5),
    (2.0, 8.0), (3.5, 8.0),
    (26.0, 22.0), (27.5, 22.0), (26.0, 23.5), (27.5, 23.5),
    (26.0, 25.0), (27.5, 25.0), (26.0, 26.5), (27.5, 26.5),
    (26.0, 28.0), (27.5, 28.0),
]


def main():
    parser = argparse.ArgumentParser(description='Send swarm goals to mapf_planner')
    parser.add_argument('--num',      type=int,   default=20)
    parser.add_argument('--goal-x',   type=float, default=None)
    parser.add_argument('--goal-y',   type=float, default=None)
    parser.add_argument('--random',   action='store_true')
    parser.add_argument('--radius',   type=float, default=5.0)
    parser.add_argument('--json-file', type=str,  default=None)
    args = parser.parse_args()

    rclpy.init()
    node = Node('test_send_goals')
    pub  = node.create_publisher(String, '/swarm_goals', 10)

    # Даём время на установку соединения
    import time; time.sleep(0.5)

    goals = []

    if args.json_file:
        with open(args.json_file) as f:
            payload = json.load(f)
        goals = payload['goals']

    else:
        starts = DEFAULT_STARTS[:args.num]
        for i, (sx, sy) in enumerate(starts):
            if args.random:
                ang = random.uniform(0.0, 2 * math.pi)
                r   = args.radius * math.sqrt(random.random())
                # Берём центр карты как базу для случайных целей
                cx, cy = 15.0, 15.0
                gx = cx + r * math.cos(ang)
                gy = cy + r * math.sin(ang)
            elif args.goal_x is not None and args.goal_y is not None:
                # Раскидываем роботов по сетке вокруг целевой точки.
                # PBS требует уникальных финальных клеток.
                # Шаг 0.4м = 2 клетки PBS-сетки (resolution=0.2м).
                step = 0.4
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

            goals.append({'id': i, 'sx': sx, 'sy': sy, 'gx': gx, 'gy': gy})

    payload_str = json.dumps({'goals': goals}, indent=2)
    print("Отправляю в /swarm_goals:")
    print(payload_str)

    msg = String()
    msg.data = payload_str
    pub.publish(msg)

    # Даём время на доставку сообщения
    time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()