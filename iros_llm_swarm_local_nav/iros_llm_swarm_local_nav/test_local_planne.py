#!/usr/bin/env python3
import math
import random
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OneRobot:
    def __init__(self, node: Node, tf_buffer: Buffer, ns: str, global_frame: str = "map"):
        self.node = node
        self.tf_buffer = tf_buffer
        self.ns = ns
        self.global_frame = global_frame  # обычно "map"

        self.pose_odom: Optional[Tuple[float, float]] = None
        self.odom_frame: Optional[str] = None  # из Odometry.header.frame_id

        self.sent = False
        self.done = False

        self.sub = node.create_subscription(Odometry, f"/{ns}/odom", self._odom_cb, 10)
        self.ac = ActionClient(node, FollowPath, f"/{ns}/follow_path")

    def _odom_cb(self, msg: Odometry):
        self.pose_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_frame = msg.header.frame_id  # ожидаем robot_i/odom

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

    def _map_xy_to_odom_xy(self, x_map: float, y_map: float) -> Optional[Tuple[float, float]]:
        """
        Перевод (x,y) из global_frame (обычно map) -> odom робота (robot_i/odom) через TF.
        Реализовано вручную (без tf2_geometry_msgs), поэтому не падает на PoseStamped.
        """
        if self.odom_frame is None:
            return None

        try:
            # transform data from source(global_frame) -> target(odom_frame)
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,       # target
                self.global_frame,     # source
                Time(),                # latest
                timeout=Duration(seconds=0.2),
            )

            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = quat_to_yaw(q)
            c = math.cos(yaw)
            s = math.sin(yaw)

            # p_target = R * p_source + t
            x_odom = c * x_map - s * y_map + t.x
            y_odom = s * x_map + c * y_map + t.y
            return (x_odom, y_odom)

        except Exception as e:
            self.node.get_logger().warn(
                f"[{self.ns}] TF {self.global_frame}->{self.odom_frame} not ready: {e}"
            )
            return None

    def try_send_random(self, radius: float, use_map: bool):
        if self.sent:
            return
        if self.pose_odom is None or self.odom_frame is None:
            return
        if not self.ac.wait_for_server(timeout_sec=0.0):
            return

        x0_odom, y0_odom = self.pose_odom

        ang = random.uniform(0.0, 2.0 * math.pi)
        r = radius * math.sqrt(random.uniform(0.0, 1.0))

        if use_map:
            # цель задаём в map: берём текущую позицию в map = (map<-odom)^-1 * odom,
            # но проще: случайную цель строим вокруг ТЕКУЩЕГО odom, а потом переводим её в map нельзя без обратного TF.
            # Поэтому делаем так: строим цель в map вокруг текущей оценки в map.
            # Для этого сначала найдём текущую позицию odom->map (т.е. map в odom) и инвертируем точку через обратное преобразование.
            # Чтобы не усложнять: строим цель в map вокруг (0,0) нельзя.
            # Практично: используем goal_map напрямую из аргументов, а random делаем в odom.
            # => если нужно именно random в map, лучше не включать use_map.
            self.node.get_logger().warn(f"[{self.ns}] random in map not supported in this minimal version; sending in odom")
            use_map = False

        x1_odom = x0_odom + r * math.cos(ang)
        y1_odom = y0_odom + r * math.sin(ang)

        goal = FollowPath.Goal()
        goal.path = self._make_path(x0_odom, y0_odom, x1_odom, y1_odom, self.odom_frame)

        self.node.get_logger().info(f"[{self.ns}] random goal -> ({x1_odom:.2f}, {y1_odom:.2f}) frame={self.odom_frame}")
        self.sent = True
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._on_goal)

    def try_send_fixed(self, x: float, y: float, frame: str):
        """
        Отправить цель (x,y) в frame: "map" или "odom".
        Если frame=="map" — конвертируем в odom и шлём путь в odom (FollowPath обычно ожидает local frame).
        """
        if self.sent:
            return
        if self.pose_odom is None or self.odom_frame is None:
            return
        if not self.ac.wait_for_server(timeout_sec=0.0):
            return

        x0_odom, y0_odom = self.pose_odom

        if frame == self.odom_frame or frame.endswith("/odom") or frame == "odom":
            x1_odom, y1_odom = x, y
        else:
            xy = self._map_xy_to_odom_xy(x, y)
            if xy is None:
                return
            x1_odom, y1_odom = xy

        goal = FollowPath.Goal()
        goal.path = self._make_path(x0_odom, y0_odom, x1_odom, y1_odom, self.odom_frame)

        self.node.get_logger().info(f"[{self.ns}] fixed goal -> ({x1_odom:.2f}, {y1_odom:.2f}) path_frame={self.odom_frame} from {frame}")
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


class GoalsNode(Node):
    def __init__(self, num: int, radius: float, goal_map: Optional[Tuple[float, float]], goal_frame: str):
        super().__init__("random_goals")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robots = [OneRobot(self, self.tf_buffer, f"robot_{i}", global_frame="map") for i in range(num)]

        self.radius = radius
        self.goal_map = goal_map
        self.goal_frame = goal_frame

        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        for r in self.robots:
            if self.goal_map is None:
                r.try_send_random(self.radius, use_map=False)
            else:
                gx, gy = self.goal_map
                r.try_send_fixed(gx, gy, frame=self.goal_frame)

        if all(r.sent for r in self.robots) and all(r.done for r in self.robots):
            self.get_logger().info("All done.")
            rclpy.shutdown()


def main():
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--num", type=int, default=20)
    p.add_argument("--radius", type=float, default=2.0, help="random radius in odom")
    p.add_argument("--goal-x", type=float, default=None, help="fixed goal x")
    p.add_argument("--goal-y", type=float, default=None, help="fixed goal y")
    p.add_argument("--goal-frame", type=str, default="map", help='frame for fixed goal: "map" or "odom"')
    args = p.parse_args()

    goal = None
    if args.goal_x is not None and args.goal_y is not None:
        goal = (args.goal_x, args.goal_y)

    rclpy.init()
    node = GoalsNode(args.num, args.radius, goal, args.goal_frame)
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()