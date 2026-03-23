#!/usr/bin/env python3
"""
formation_manager_node
======================
Maintains a registry of formations and publishes FormationConfig messages.

Services
--------
/formation/set      (SetFormation)     — create / update a formation
/formation/disband  (DisbandFormation) — dissolve a formation

Published topics
----------------
/formations/<id>/config  (FormationConfig, latched QoS)  — one per formation

Parameters
----------
config_file : str   — path to formations.yaml (default: package config)
auto_activate : bool — activate formations loaded from file immediately (default: false)
footprint_padding : float — extra radius [m] added to bounding circle for MAPF (default: 0.3)
robot_radius : float      — single robot radius [m] used in footprint computation (default: 0.2)
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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_circle_polygon(radius: float, n_pts: int = 16) -> Polygon:
    """Approximate a circle as a polygon for the dummy MAPF footprint."""
    poly = Polygon()
    for i in range(n_pts):
        angle = 2.0 * math.pi * i / n_pts
        p = Point32()
        p.x = float(radius * math.cos(angle))
        p.y = float(radius * math.sin(angle))
        p.z = 0.0
        poly.points.append(p)
    return poly


def _bounding_radius(offsets: list[tuple[float, float]], robot_radius: float) -> float:
    """
    Minimum enclosing circle radius for the formation.
    = max distance from origin to any follower offset + robot_radius
    """
    if not offsets:
        return robot_radius
    max_r = max(math.hypot(dx, dy) for dx, dy in offsets)
    return max_r + robot_radius


# ---------------------------------------------------------------------------
# Formation data class
# ---------------------------------------------------------------------------

class Formation:
    def __init__(self, formation_id: str, leader_ns: str,
                 follower_ns: list[str], offsets: list[tuple[float, float]],
                 active: bool = False):
        self.formation_id = formation_id
        self.leader_ns = leader_ns
        self.follower_ns = follower_ns        # list of str
        self.offsets = offsets                # list of (dx, dy)
        self.active = active

    def to_msg(self, stamp, footprint_padding: float, robot_radius: float) -> FormationConfig:
        msg = FormationConfig()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = ""

        msg.formation_id = self.formation_id
        msg.leader_ns = self.leader_ns
        msg.follower_ns = list(self.follower_ns)
        msg.active = self.active

        for dx, dy in self.offsets:
            p = Point()
            p.x = float(dx)
            p.y = float(dy)
            p.z = 0.0
            msg.offsets.append(p)

        # Dummy footprint: bounding circle
        radius = _bounding_radius(self.offsets, robot_radius) + footprint_padding
        msg.footprint = _make_circle_polygon(radius)

        return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FormationManagerNode(Node):

    def __init__(self):
        super().__init__("formation_manager")

        # Parameters
        self.declare_parameter("config_file", "")
        self.declare_parameter("auto_activate", False)
        self.declare_parameter("footprint_padding", 0.3)
        self.declare_parameter("robot_radius", 0.2)

        self._padding = self.get_parameter("footprint_padding").get_parameter_value().double_value
        self._robot_r = self.get_parameter("robot_radius").get_parameter_value().double_value
        self._auto_activate = self.get_parameter("auto_activate").get_parameter_value().bool_value

        # Latched QoS — new subscribers always get the last message
        self._latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Registry:  formation_id -> (Formation, publisher)
        self._formations: dict[str, tuple[Formation, object]] = {}

        # Services
        self._set_srv = self.create_service(
            SetFormation, "/formation/set", self._on_set_formation)
        self._disband_srv = self.create_service(
            DisbandFormation, "/formation/disband", self._on_disband_formation)

        # Load initial formations from YAML
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        if config_file:
            self._load_yaml(config_file)

        self.get_logger().info(
            f"formation_manager ready | formations loaded: {list(self._formations.keys())}"
        )

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _on_set_formation(self, request: SetFormation.Request,
                          response: SetFormation.Response):
        fid = request.formation_id.strip()
        if not fid:
            response.success = False
            response.message = "formation_id must not be empty"
            return response

        if len(request.follower_ns) != len(request.offsets_x) or \
           len(request.follower_ns) != len(request.offsets_y):
            response.success = False
            response.message = (
                f"follower_ns length ({len(request.follower_ns)}) must match "
                f"offsets_x ({len(request.offsets_x)}) and offsets_y ({len(request.offsets_y)})"
            )
            return response

        offsets = list(zip(request.offsets_x, request.offsets_y))
        formation = Formation(
            formation_id=fid,
            leader_ns=request.leader_ns,
            follower_ns=list(request.follower_ns),
            offsets=offsets,
            active=request.activate,
        )

        self._upsert_formation(formation)

        response.success = True
        response.message = f"Formation '{fid}' {'activated' if request.activate else 'registered'}"
        self.get_logger().info(response.message)
        return response

    def _on_disband_formation(self, request: DisbandFormation.Request,
                               response: DisbandFormation.Response):
        fid = request.formation_id.strip()
        if fid not in self._formations:
            response.success = False
            response.message = f"Formation '{fid}' not found"
            return response

        formation, pub = self._formations[fid]
        formation.active = False
        self._publish(formation, pub)

        response.success = True
        response.message = f"Formation '{fid}' disbanded"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _upsert_formation(self, formation: Formation):
        """Insert or update a formation, creating publisher if needed."""
        fid = formation.formation_id

        if fid in self._formations:
            _, pub = self._formations[fid]
        else:
            topic = f"/formations/{fid}/config"
            pub = self.create_publisher(FormationConfig, topic, self._latched_qos)
            self.get_logger().info(f"  publisher created: {topic}")

        self._formations[fid] = (formation, pub)
        self._publish(formation, pub)

    def _publish(self, formation: Formation, pub):
        msg = formation.to_msg(
            stamp=self.get_clock().now().to_msg(),
            footprint_padding=self._padding,
            robot_radius=self._robot_r,
        )
        pub.publish(msg)

    def _load_yaml(self, path: str):
        """Load formations.yaml and register all entries."""
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load config file '{path}': {e}")
            return

        entries = data.get("formations", [])
        for entry in entries:
            try:
                fid = str(entry["id"])
                leader = str(entry["leader"])
                followers = entry.get("followers", [])
                ns_list = [str(fw["ns"]) for fw in followers]
                offsets = [(float(fw["dx"]), float(fw["dy"])) for fw in followers]

                formation = Formation(
                    formation_id=fid,
                    leader_ns=leader,
                    follower_ns=ns_list,
                    offsets=offsets,
                    active=self._auto_activate,
                )
                self._upsert_formation(formation)
                self.get_logger().info(
                    f"  loaded '{fid}': leader={leader}, "
                    f"{len(ns_list)} follower(s), active={self._auto_activate}"
                )
            except KeyError as e:
                self.get_logger().error(f"Malformed formation entry — missing key {e}: {entry}")


# ---------------------------------------------------------------------------

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