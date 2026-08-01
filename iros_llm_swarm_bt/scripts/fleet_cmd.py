#!/usr/bin/env python3
"""CLI utility to send LlmCommand goals to /llm/command for bt_runner."""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point
from iros_llm_swarm_interfaces.action import LlmCommand


SCENARIOS = {
    'idle': dict(mode='idle'),
    'simple': dict(
        mode='mapf',
        robot_ids=[0, 1, 2, 3],
        goals=[(15.0, 15.0), (16.5, 15.0), (15.0, 16.5), (16.5, 16.5)],
    ),
    'stress': dict(
        mode='mapf',
        robot_ids=list(range(20)),
        goals=[
            (26.0, 22.0), (27.5, 22.0), (26.0, 23.5), (27.5, 23.5),
            (26.0, 25.0), (27.5, 25.0), (26.0, 26.5), (27.5, 26.5),
            (26.0, 28.0), (27.5, 28.0),
            (2.0, 2.0),  (3.5, 2.0),  (2.0, 3.5),  (3.5, 3.5),
            (2.0, 5.0),  (3.5, 5.0),  (2.0, 6.5),  (3.5, 6.5),
            (2.0, 8.0),  (3.5, 8.0),
        ],
    ),
    'unreachable': dict(
        mode='mapf',
        robot_ids=[0, 1, 2, 3],
        goals=[(0.5, 0.5)] * 4,
    ),
    'formation_simple': dict(
        mode='formation',
        formation_id='line_demo',
        leader_ns='robot_0',
        follower_ns=['robot_1', 'robot_2', 'robot_3'],
        offsets_x=[-1.0, -2.0, -3.0],
        offsets_y=[0.0, 0.0, 0.0],
    ),
}


def parse_goals_csv(s):
    """Parse 'x1,y1;x2,y2;...' or 'x1,y1,x2,y2,...' into [(x1,y1),(x2,y2),...]."""
    if ';' in s:
        pairs = []
        for chunk in s.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            x, y = chunk.split(',')
            pairs.append((float(x), float(y)))
        return pairs
    parts = [float(p.strip()) for p in s.split(',') if p.strip()]
    if len(parts) % 2 != 0:
        raise ValueError('--goals must have an even number of values')
    return [(parts[i], parts[i + 1]) for i in range(0, len(parts), 2)]


def parse_floats_csv(s):
    return [float(x.strip()) for x in s.split(',') if x.strip()]


def parse_ints_csv(s):
    return [int(x.strip()) for x in s.split(',') if x.strip()]


def _set_goals(g, goals):
    for x, y in goals:
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        g.goals.append(p)


def build_goal_from_args(args):
    g = LlmCommand.Goal()
    g.mode = args.mode
    g.reason = args.reason or f'fleet_cmd: {args.mode}'
    if args.robots:
        g.robot_ids = parse_ints_csv(args.robots)
    if args.goals:
        _set_goals(g, parse_goals_csv(args.goals))
    if args.formation_id:
        g.formation_id = args.formation_id
    if args.leader:
        g.leader_ns = args.leader
    if args.followers:
        g.follower_ns = [s.strip() for s in args.followers.split(',') if s.strip()]
    if args.offsets_x:
        g.offsets_x = parse_floats_csv(args.offsets_x)
    if args.offsets_y:
        g.offsets_y = parse_floats_csv(args.offsets_y)
    return g


def build_goal_from_scenario(name):
    scn = SCENARIOS[name]
    g = LlmCommand.Goal()
    g.mode = scn.get('mode', 'idle')
    g.reason = f'fleet_cmd scenario: {name}'
    if 'robot_ids' in scn:
        g.robot_ids = list(scn['robot_ids'])
    if 'goals' in scn:
        _set_goals(g, scn['goals'])
    if 'formation_id' in scn:
        g.formation_id = scn['formation_id']
    if 'leader_ns' in scn:
        g.leader_ns = scn['leader_ns']
    if 'follower_ns' in scn:
        g.follower_ns = list(scn['follower_ns'])
    if 'offsets_x' in scn:
        g.offsets_x = list(scn['offsets_x'])
    if 'offsets_y' in scn:
        g.offsets_y = list(scn['offsets_y'])
    return g


def main():
    parser = argparse.ArgumentParser(
        description='Send a single LlmCommand goal to /llm/command.',
    )
    parser.add_argument('--scenario', choices=sorted(SCENARIOS.keys()),
                        help='Predefined command from the SCENARIOS table.')
    parser.add_argument('--mode', choices=['idle', 'mapf', 'formation'],
                        help='BT mode for the command.')
    parser.add_argument('--reason',
                        help='Free-form reason string (echoed into /bt/state).')
    parser.add_argument('--robots', help='Comma-separated robot ids: 0,1,2,3')
    parser.add_argument('--goals',
                        help='Goals "x1,y1;x2,y2;..." or flat "x1,y1,x2,y2,...".')
    parser.add_argument('--formation-id', dest='formation_id')
    parser.add_argument('--leader')
    parser.add_argument('--followers', help='Comma-separated follower ns')
    parser.add_argument('--offsets-x', dest='offsets_x')
    parser.add_argument('--offsets-y', dest='offsets_y')
    parser.add_argument('--server-timeout', type=float, default=10.0,
                        help='Seconds to wait for /llm/command server to come up.')
    args = parser.parse_args()

    if args.scenario:
        goal = build_goal_from_scenario(args.scenario)
    elif args.mode:
        goal = build_goal_from_args(args)
    else:
        parser.error('Specify --scenario or --mode (and the fields it needs).')

    rclpy.init()
    node = Node('fleet_cmd')
    client = ActionClient(node, LlmCommand, '/llm/command')

    try:
        node.get_logger().info('[fleet_cmd] waiting for /llm/command server...')
        if not client.wait_for_server(timeout_sec=args.server_timeout):
            node.get_logger().error(
                f'[fleet_cmd] /llm/command server not available after '
                f'{args.server_timeout:.1f}s')
            return 2

        node.get_logger().info(
            f'[fleet_cmd] sending goal: mode={goal.mode} reason={goal.reason!r}')
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            node.get_logger().error('[fleet_cmd] goal rejected by /llm/command')
            return 3

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        result = wrapped.result if wrapped else None
        if result is None:
            node.get_logger().error('[fleet_cmd] no result received')
            return 4
        if result.success:
            node.get_logger().info(f'[fleet_cmd] applied: {result.info}')
            return 0
        node.get_logger().error(f'[fleet_cmd] command failed: {result.info}')
        return 5
    except KeyboardInterrupt:
        node.get_logger().info('[fleet_cmd] interrupted')
        return 130
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    sys.exit(main())
