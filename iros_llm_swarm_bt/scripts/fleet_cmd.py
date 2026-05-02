#!/usr/bin/env python3
"""CLI utility to publish key=value commands on /fleet/cmd for test_bt_runner."""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String


SCENARIOS = {
    'idle': [
        ('mode', 'idle'),
    ],
    'simple': [
        ('robot_ids', '0,1,2,3'),
        ('goals', '15.0,15.0,16.5,15.0,15.0,16.5,16.5,16.5'),
        ('mode', 'mapf'),
    ],
    'stress': [
        ('robot_ids', ','.join(str(i) for i in range(20))),
        ('goals',
            '26.0,22.0,27.5,22.0,26.0,23.5,27.5,23.5,26.0,25.0,'
            '27.5,25.0,26.0,26.5,27.5,26.5,26.0,28.0,27.5,28.0,'
            '2.0,2.0,3.5,2.0,2.0,3.5,3.5,3.5,2.0,5.0,'
            '3.5,5.0,2.0,6.5,3.5,6.5,2.0,8.0,3.5,8.0'
        ),
        ('mode', 'mapf'),
    ],
    'unreachable': [
        ('robot_ids', '0,1,2,3'),
        ('goals', '0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5'),
        ('mode', 'mapf'),
    ],
    'formation_simple': [
        ('formation_id', 'line_demo'),
        ('leader_ns', 'robot_0'),
        ('follower_ns', 'robot_1,robot_2,robot_3'),
        ('offsets_x', '-1.0,-2.0,-3.0'),
        ('offsets_y', '0.0,0.0,0.0'),
        ('mode', 'formation'),
    ],
}


def goals_semicolon_to_flat(s: str) -> str:
    """Convert "x1,y1;x2,y2;..." -> "x1,y1,x2,y2,...".  Already flat strings pass through."""
    if ';' not in s:
        return s
    parts = []
    for pair in s.split(';'):
        pair = pair.strip()
        if not pair:
            continue
        parts.append(pair)
    return ','.join(parts)


def build_commands_from_args(args) -> list:
    cmds = []
    if args.robots:
        cmds.append(('robot_ids', args.robots))
    if args.goals:
        cmds.append(('goals', goals_semicolon_to_flat(args.goals)))
    if args.formation_id:
        cmds.append(('formation_id', args.formation_id))
    if args.leader:
        cmds.append(('leader_ns', args.leader))
    if args.followers:
        cmds.append(('follower_ns', args.followers))
    if args.offsets_x:
        cmds.append(('offsets_x', args.offsets_x))
    if args.offsets_y:
        cmds.append(('offsets_y', args.offsets_y))
    if args.mode:
        cmds.append(('mode', args.mode))
    return cmds


def main():
    parser = argparse.ArgumentParser(
        description='Send key=value commands to /fleet/cmd for test_bt_runner.'
    )
    parser.add_argument('--scenario', choices=sorted(SCENARIOS.keys()),
                        help='Predefined command sequence.')
    parser.add_argument('--mode', choices=['idle', 'mapf', 'formation'],
                        help='BT mode (sent last).')
    parser.add_argument('--robots', help='Comma-separated robot ids: 0,1,2,3')
    parser.add_argument('--goals',
                        help='Goals "x1,y1;x2,y2;..." or flat "x1,y1,x2,y2,...".')
    parser.add_argument('--formation-id', dest='formation_id')
    parser.add_argument('--leader')
    parser.add_argument('--followers', help='Comma-separated follower ns')
    parser.add_argument('--offsets-x', dest='offsets_x')
    parser.add_argument('--offsets-y', dest='offsets_y')
    parser.add_argument('--topic', default='/fleet/cmd')
    parser.add_argument('--wait-timeout', type=float, default=10.0,
                        help='Seconds to wait for a subscriber on the topic.')
    parser.add_argument('--inter-delay', type=float, default=0.5,
                        help='Seconds between successive commands.')
    args = parser.parse_args()

    if args.scenario:
        cmds = list(SCENARIOS[args.scenario])
    else:
        cmds = build_commands_from_args(args)

    if not cmds:
        parser.error('No commands to send. Use --scenario or --mode (and friends).')

    rclpy.init()
    node = Node('fleet_cmd_publisher')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(String, args.topic, qos)

    try:
        node.get_logger().info(f'[fleet_cmd] Waiting for {args.topic} subscriber...')
        deadline = time.monotonic() + args.wait_timeout
        while pub.get_subscription_count() < 1:
            if time.monotonic() > deadline:
                node.get_logger().error(
                    f'[fleet_cmd] No subscriber on {args.topic} after '
                    f'{args.wait_timeout:.1f}s — aborting.'
                )
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)

        n = pub.get_subscription_count()
        node.get_logger().info(f'[fleet_cmd] Found {n} subscriber(s), sending commands...')

        for key, value in cmds:
            msg = String()
            msg.data = f'{key}={value}'
            pub.publish(msg)
            node.get_logger().info(f'[fleet_cmd] Sent: {msg.data}')
            end = time.monotonic() + args.inter_delay
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info('[fleet_cmd] Done.')
        return 0
    except KeyboardInterrupt:
        node.get_logger().info('[fleet_cmd] Interrupted.')
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
