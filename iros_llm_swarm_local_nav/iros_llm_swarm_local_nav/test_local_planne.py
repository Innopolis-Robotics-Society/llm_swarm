#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OneRobot:
    def __init__(self, node: Node, ns: str):
        self.node = node
        self.ns = ns
        self.pose = None          # (x, y)
        self.frame_id = None      # odom frame name from Odometry header
        self.sent = False
        self.done = False

        self.sub = node.create_subscription(Odometry, f"/{ns}/odom", self._odom_cb, 10)
        self.ac = ActionClient(node, FollowPath, f"/{ns}/follow_path")

    def _odom_cb(self, msg: Odometry):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.frame_id = msg.header.frame_id

    def _make_path(self, x0, y0, x1, y1, frame_id: str, steps: int = 10) -> Path:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.node.get_clock().now().to_msg()

        yaw = math.atan2(y1 - y0, x1 - x0)
        q = yaw_to_quat(yaw)

        for k in range(steps + 1):
            t = k / steps
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = path.header.stamp
            ps.pose.position.x = float(x0 + t * (x1 - x0))
            ps.pose.position.y = float(y0 + t * (y1 - y0))
            ps.pose.position.z = 0.0
            ps.pose.orientation = q
            path.poses.append(ps)

        return path

    def try_send(self, radius: float):
        if self.sent:
            return
        if self.pose is None or self.frame_id is None:
            return
        if not self.ac.wait_for_server(timeout_sec=0.0):
            return

        x0, y0 = self.pose
        ang = random.uniform(0.0, 2.0 * math.pi)
        r = radius * math.sqrt(random.uniform(0.0, 1.0))
        x1 = x0 + r * math.cos(ang)
        y1 = y0 + r * math.sin(ang)

        goal = FollowPath.Goal()
        goal.path = self._make_path(x0, y0, x1, y1, self.frame_id)

        self.node.get_logger().info(f"[{self.ns}] goal -> ({x1:.2f}, {y1:.2f}) frame={self.frame_id}")

        self.sent = True
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._on_goal)

    def _on_goal(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error(f"[{self.ns}] goal rejected")
            self.done = True
            return
        rf = gh.get_result_async()
        rf.add_done_callback(self._on_result)

    def _on_result(self, fut):
        res = fut.result()
        status = getattr(res, "status", None)
        self.node.get_logger().info(f"[{self.ns}] done status={status}")
        self.done = True


class RandomGoals(Node):
    def __init__(self, num: int, radius: float):
        super().__init__("random_goals")
        self.robots = [OneRobot(self, f"robot_{i}") for i in range(num)]
        self.radius = radius
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        for r in self.robots:
            r.try_send(self.radius)

        if all(r.sent for r in self.robots) and all(r.done for r in self.robots):
            self.get_logger().info("All done.")
            rclpy.shutdown()


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--num", type=int, default=20)
    p.add_argument("--radius", type=float, default=10)
    args = p.parse_args()

    rclpy.init()
    node = RandomGoals(args.num, args.radius)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()