#!/usr/bin/env python3
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point, Polygon, Point32
from std_msgs.msg import Header

from iros_llm_swarm_interfaces.msg import FormationConfig, FormationsConfig
from iros_llm_swarm_interfaces.srv import (
    ActivateFormation,
    DeactivateFormation,
    GetFormation,
    ListFormations,
    LoadFormations,
    RemoveFormation,
    SaveFormations,
    SetFormation
)

FORMATIONS_TOPIC = "/formations/config"


# Helpers

def _bounding_circle(offsets, robot_radius):
    if not offsets:
        return (0.0, 0.0, robot_radius)

    cx = sum(dx for dx, _ in offsets) / len(offsets)
    cy = sum(dy for _, dy in offsets) / len(offsets)

    radius = max(math.hypot(dx - cx, dy - cy) for dx, dy in offsets)
    return cx, cy, radius + robot_radius

def _circle_polygon(radius: float, cx: float = 0.0, cy: float = 0.0, n_pts: int = 16) -> Polygon:
    poly = Polygon()
    for i in range(n_pts):
        a = 2.0 * math.pi * i / n_pts
        p = Point32(
            x=float(cx + radius * math.cos(a)),
            y=float(cy + radius * math.sin(a)),
            z=0.0
        )
        poly.points.append(p)
    return poly


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

        cx, cy, radius = _bounding_circle(self.offsets, robot_radius)
        msg.footprint = _circle_polygon(radius + padding, cx, cy)
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
        self._pub = self.create_publisher(FormationsConfig, FORMATIONS_TOPIC, latched)

        # Formation registry
        self._registry: dict[str, _Formation] = {}

        # Services
        self.create_service(ActivateFormation,      "/formation/activate",     self._on_activate)
        self.create_service(DeactivateFormation,    "/formation/deactivate",   self._on_deactivate)
        self.create_service(GetFormation,           "/formation/get",          self._on_get)
        self.create_service(ListFormations,         "/formation/list",         self._on_list)
        self.create_service(LoadFormations,         "/formation/load",         self._on_load)
        self.create_service(RemoveFormation,        "/formation/remove",       self._on_remove)
        self.create_service(SaveFormations,         "/formation/save",         self._on_save)
        self.create_service(SetFormation,           "/formation/set",          self._on_set)

        # Load initial config
        cfg = self.get_parameter("config_file").value
        if cfg:
            self._load_yaml(cfg)

        self.get_logger().info(
            f"formation_manager ready  topic={FORMATIONS_TOPIC}  "
            f"formations={list(self._registry.keys())}"
        )

        self._publish_all()


    # Services

    def _on_activate(self, req: ActivateFormation.Request,
                     res: ActivateFormation.Response):
        fid = req.formation_id.strip()
        try:
            f = self._registry[fid]
        except KeyError:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res
        
        if f.active: 
            res.success = True
            res.message = f"Formation '{fid}' already active"
            return res

        f.active = True
        self._publish_all()

        res.success = True
        res.message = f"Formation '{fid}' activated"
        self.get_logger().info(res.message)
        return res
    
    def _on_deactivate(self, req: DeactivateFormation.Request,
                       res: DeactivateFormation.Response):
        fid = req.formation_id.strip()

        if fid not in self._registry:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        f = self._registry[fid]

        if not f.active:
            res.success = True
            res.message = f"Formation '{fid}' already inactive"
            return res

        f.active = False
        self._publish_all()

        res.success = True
        res.message = f"Formation '{fid}' deactivated"
        self.get_logger().info(res.message)
        return res

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
        self._publish_all()

        verb = "activated" if req.activate else "registered"
        res.success = True
        res.message = f"Formation '{fid}' {verb}"
        self.get_logger().info(res.message)
        return res
    
    def _on_remove(self, req: RemoveFormation.Request,
                   res: RemoveFormation.Response):
        fid = req.formation_id.strip()

        try:
            f = self._registry[fid]
        except KeyError:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        if f.active:
            res.success = False
            res.message = (
                f"Formation '{fid}' is active. "
                f"Disband it before removing."
            )
            return res

        del self._registry[fid]

        self._publish_all()

        res.success = True
        res.message = f"Formation '{fid}' removed"
        self.get_logger().info(res.message)
        return res
    
    def _on_get(self, req: GetFormation.Request,
                res: GetFormation.Response):
        fid = req.formation_id.strip()

        if fid not in self._registry:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        now = self.get_clock().now().to_msg()
        f = self._registry[fid]

        res.formation = f.to_msg(
            stamp=now,
            padding=self._padding,
            robot_radius=self._robot_r,
        )
        res.success = True
        res.message = f"Formation '{fid}' returned"
        return res
    
    def _on_list(self, req: ListFormations.Request,
                 res: ListFormations.Response):
        now = self.get_clock().now().to_msg()

        res.formations = [
            f.to_msg(
                stamp=now,
                padding=self._padding,
                robot_radius=self._robot_r,
            )
            for f in self._registry.values()
        ]

        return res
    
    def _on_load(self, req, res):
        path = req.file_path.strip()

        if not path:
            res.success = False
            res.message = "file_path is empty"
            return res

        try:
            with open(path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            res.success = False
            res.message = f"Failed to load file: {e}"
            return res

        if req.clear_existing:
            self._registry.clear()

        loaded = 0

        for entry in data.get("formations", []):
            try:
                fid = str(entry["id"])
                leader = str(entry["leader"])
                fws = entry.get("followers", [])

                ns_list = [str(fw["ns"]) for fw in fws]
                offsets = [(float(fw["dx"]), float(fw["dy"])) for fw in fws]

                formation = _Formation(
                    formation_id=fid,
                    leader_ns=leader,
                    follower_ns=ns_list,
                    offsets=offsets,
                    active=req.activate,
                )

                self._registry[fid] = formation
                loaded += 1

            except Exception as e:
                self.get_logger().error(f"Bad entry skipped: {entry} ({e})")

        self._publish_all()

        res.success = True
        res.message = f"Loaded {loaded} formations"
        res.loaded_count = loaded
        return res
    
    def _on_save(self, req, res):
        path = req.file_path.strip()

        if not path:
            res.success = False
            res.message = "file_path is empty"
            return res

        data = {"formations": []}
        saved = 0

        for f in self._registry.values():
            if req.only_active and not f.active:
                continue

            entry = {
                "id": f.formation_id,
                "leader": f.leader_ns,
                "followers": [],
            }

            for ns, (dx, dy) in zip(f.follower_ns, f.offsets):
                entry["followers"].append({
                    "ns": ns,
                    "dx": float(dx),
                    "dy": float(dy),
                })

            data["formations"].append(entry)
            saved += 1

        try:
            with open(path, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
        except Exception as e:
            res.success = False
            res.message = f"Failed to save file: {e}"
            return res

        res.success = True
        res.message = f"Saved {saved} formations"
        res.saved_count = saved
        return res


    # Internal

    def _publish_all(self):
        msg = FormationsConfig()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = ""

        for f in self._registry.values():
            msg.formations.append(
                f.to_msg(
                    stamp=now,
                    padding=self._padding,
                    robot_radius=self._robot_r,
                )
            )

        self._pub.publish(msg)

        self.get_logger().debug(
            f"Published state: {[f.formation_id for f in self._registry.values()]}"
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
                self.get_logger().info(
                    f"  loaded '{fid}': leader={leader} "
                    f"followers={ns_list} active={self._auto_act}"
                )
            except KeyError as e:
                self.get_logger().error(f"Malformed formation entry, missing {e}: {entry}")

        self._publish_all()


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