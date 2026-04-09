#!/usr/bin/env python3
"""
formation_manager_node
======================
Topic
-----
  /formations/config  (FormationConfig, latched QoS)
      Published on every create / update / disband / remove.

Services
--------
  /formation/set     (SetFormation)      — create or update a formation
  /formation/disband (DisbandFormation)  — deactivate

Parameters
----------
  config_file      : str   — path to formations.yaml  (default: package share)
  auto_activate    : bool  — activate formations from file immediately (default: false)
  footprint_padding: float — extra radius [m] for MAPF bounding circle (default: 0.2)
  robot_radius     : float — single robot radius [m] (default: 0.3)
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point, Polygon, Point32
from std_msgs.msg import Header

from iros_llm_swarm_interfaces.msg import FormationConfig
from iros_llm_swarm_interfaces.srv import SetFormation, DisbandFormation

FORMATIONS_TOPIC = "/formations/config"


# Helpers

def _circle_polygon(radius: float, n_pts: int = 16) -> Polygon:
    poly = Polygon()
    for i in range(n_pts):
        a = 2.0 * math.pi * i / n_pts
        p = Point32(x=float(radius * math.cos(a)),
                    y=float(radius * math.sin(a)),
                    z=0.0)
        poly.points.append(p)
    return poly


def _bounding_radius(offsets: list[tuple[float, float]], robot_radius: float) -> float:
    if not offsets:
        return robot_radius
    return max(math.hypot(dx, dy) for dx, dy in offsets) + robot_radius


# Internal formation record

class _Formation:
    __slots__ = ("formation_id", "leader_ns", "follower_ns", "offsets", "active")

    def __init__(self, formation_id: str, leader_ns: str,
                 follower_ns: list[str], offsets: list[tuple[float, float]],
                 active: bool = False):
        self.formation_id = formation_id
        self.leader_ns    = leader_ns
        self.follower_ns  = follower_ns
        self.offsets      = offsets       # [(dx, dy), ...]
        self.active       = active

    def to_msg(self, stamp, padding: float, robot_radius: float) -> FormationConfig:
        msg = FormationConfig()
        msg.header       = Header(stamp=stamp, frame_id="")
        msg.formation_id = self.formation_id
        msg.leader_ns    = self.leader_ns
        msg.follower_ns  = list(self.follower_ns)
        msg.active       = self.active

        for dx, dy in self.offsets:
            msg.offsets.append(Point(x=float(dx), y=float(dy), z=0.0))

        radius = _bounding_radius(self.offsets, robot_radius) + padding
        msg.footprint = _circle_polygon(radius)
        return msg


# Node

class FormationManagerNode(Node):

    def __init__(self):
        super().__init__("formation_manager")

        self.declare_parameter("config_file",       "")
        self.declare_parameter("auto_activate",     False)
        self.declare_parameter("footprint_padding", 0.2)
        self.declare_parameter("robot_radius",      0.3)

        self._padding      = self.get_parameter("footprint_padding").value
        self._robot_r      = self.get_parameter("robot_radius").value
        self._auto_act     = self.get_parameter("auto_activate").value

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(FormationConfig, FORMATIONS_TOPIC, latched)

        # Formation registry
        self._registry: dict[str, _Formation] = {}

        # Services
        self.create_service(SetFormation,     "/formation/set",     self._on_set)
        self.create_service(DisbandFormation, "/formation/disband", self._on_disband)

        # Load initial config
        cfg = self.get_parameter("config_file").value
        if cfg:
            self._load_yaml(cfg)

        self.get_logger().info(
            f"formation_manager ready  topic={FORMATIONS_TOPIC}  "
            f"formations={list(self._registry.keys())}"
        )


    # Services

    def _on_set(self, req: SetFormation.Request,
                res: SetFormation.Response):
        fid = req.formation_id.strip()
        if not fid:
            res.success = False
            res.message = "formation_id must not be empty"
            return res

        n_followers = len(req.follower_ns)
        if len(req.offsets_x) != n_followers or len(req.offsets_y) != n_followers:
            res.success = False
            res.message = (
                f"follower_ns length ({n_followers}) must match "
                f"offsets_x ({len(req.offsets_x)}) / offsets_y ({len(req.offsets_y)})"
            )
            return res

        offsets = list(zip(req.offsets_x, req.offsets_y))
        formation = _Formation(
            formation_id=fid,
            leader_ns=req.leader_ns,
            follower_ns=list(req.follower_ns),
            offsets=offsets,
            active=req.activate,
        )
        self._registry[fid] = formation
        self._publish(formation)

        verb = "activated" if req.activate else "registered"
        res.success = True
        res.message = f"Formation '{fid}' {verb}"
        self.get_logger().info(res.message)
        return res

    def _on_disband(self, req: DisbandFormation.Request,
                    res: DisbandFormation.Response):
        fid = req.formation_id.strip()
        if fid not in self._registry:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        f = self._registry[fid]
        f.active = False
        self._publish(f)

        res.success = True
        res.message = f"Formation '{fid}' disbanded"
        self.get_logger().info(res.message)
        return res


    # Internal

    def _publish(self, f: _Formation):
        msg = f.to_msg(
            stamp=self.get_clock().now().to_msg(),
            padding=self._padding,
            robot_radius=self._robot_r,
        )
        self._pub.publish(msg)
        self.get_logger().debug(
            f"published formation '{f.formation_id}'  "
            f"active={f.active}"
            f"leader={f.leader_ns}  followers={f.follower_ns}"
        )

    def _load_yaml(self, path: str):
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Cannot load '{path}': {e}")
            return

        for entry in data.get("formations", []):
            try:
                fid      = str(entry["id"])
                leader   = str(entry["leader"])
                fws      = entry.get("followers", [])
                ns_list  = [str(fw["ns"]) for fw in fws]
                offsets  = [(float(fw["dx"]), float(fw["dy"])) for fw in fws]
                formation = _Formation(
                    formation_id=fid,
                    leader_ns=leader,
                    follower_ns=ns_list,
                    offsets=offsets,
                    active=self._auto_act,
                )
                self._registry[fid] = formation
                self._publish(formation)
                self.get_logger().info(
                    f"  loaded '{fid}': leader={leader} "
                    f"followers={ns_list} active={self._auto_act}"
                )
            except KeyError as e:
                self.get_logger().error(f"Malformed formation entry, missing {e}: {entry}")


def main(args=None):
    rclpy.init(args=args)
    node = FormationManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()