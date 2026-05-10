#!/usr/bin/env python3
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point, Polygon, Point32
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from iros_llm_swarm_interfaces.msg import FormationConfig, FormationsConfig
from iros_llm_swarm_interfaces.srv import (
    ActivateFormation,
    DeactivateFormation,
    GetFormation,
    ListFormations,
    LoadFormations,
    RemoveFormation,
    SaveFormations,
    SetFormation,
)

FORMATIONS_TOPIC = "/formations/config"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _bounding_circle(offsets, robot_radius):
    if not offsets:
        return (0.0, 0.0, robot_radius)
    cx = sum(dx for dx, _ in offsets) / len(offsets)
    cy = sum(dy for _, dy in offsets) / len(offsets)
    radius = max(math.hypot(dx - cx, dy - cy) for dx, dy in offsets)
    return cx, cy, radius + robot_radius


def _circle_polygon(radius: float, cx: float = 0.0, cy: float = 0.0,
                    n_pts: int = 16) -> Polygon:
    poly = Polygon()
    for i in range(n_pts):
        a = 2.0 * math.pi * i / n_pts
        poly.points.append(Point32(
            x=float(cx + radius * math.cos(a)),
            y=float(cy + radius * math.sin(a)),
            z=0.0,
        ))
    return poly


def _quat_to_yaw(ox, oy, oz, ow) -> float:
    return math.atan2(2.0 * (ow * oz + ox * oy),
                      1.0 - 2.0 * (oy * oy + oz * oz))


# ---------------------------------------------------------------------------
# Per-robot live pose (updated from odom subscriptions)
# ---------------------------------------------------------------------------

class _RobotPose:
    """Latest known pose for one robot namespace."""
    __slots__ = ("ns", "x", "y", "yaw", "received")

    def __init__(self, ns: str):
        self.ns       = ns
        self.x        = 0.0
        self.y        = 0.0
        self.yaw      = 0.0
        self.received = False   # True once at least one odom message arrived


# ---------------------------------------------------------------------------
# Internal formation record
# ---------------------------------------------------------------------------

class _Formation:
    __slots__ = ("formation_id", "leader_ns", "follower_ns", "offsets", "active")

    def __init__(self, formation_id: str, leader_ns: str,
                 follower_ns: list[str], offsets: list[tuple[float, float]],
                 active: bool = False):
        self.formation_id = formation_id
        self.leader_ns    = leader_ns
        self.follower_ns  = follower_ns
        self.offsets      = offsets
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


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FormationManagerNode(Node):

    def __init__(self):
        super().__init__("formation_manager")

        self.declare_parameter("config_file",        "")
        self.declare_parameter("auto_activate",      False)
        self.declare_parameter("footprint_padding",  0.2)
        self.declare_parameter("robot_radius",       0.3)
        self.declare_parameter("position_tolerance", 0.5)

        self._padding  = self.get_parameter("footprint_padding").value
        self._robot_r  = self.get_parameter("robot_radius").value
        self._auto_act = self.get_parameter("auto_activate").value
        self._pos_tol  = self.get_parameter("position_tolerance").value

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(FormationsConfig, FORMATIONS_TOPIC, latched)

        # formation_id → _Formation
        self._registry: dict[str, _Formation] = {}

        # ns → _RobotPose  (shared — one pose per robot regardless of how many
        #                    formations reference it)
        self._poses: dict[str, _RobotPose] = {}

        # ns → odom subscription
        self._odom_subs: dict[str, object] = {}

        # Services
        self.create_service(ActivateFormation,   "/formation/activate",   self._on_activate)
        self.create_service(DeactivateFormation, "/formation/deactivate", self._on_deactivate)
        self.create_service(GetFormation,        "/formation/get",        self._on_get)
        self.create_service(ListFormations,      "/formation/list",       self._on_list)
        self.create_service(LoadFormations,      "/formation/load",       self._on_load)
        self.create_service(RemoveFormation,     "/formation/remove",     self._on_remove)
        self.create_service(SaveFormations,      "/formation/save",       self._on_save)
        self.create_service(SetFormation,        "/formation/set",        self._on_set)

        cfg = self.get_parameter("config_file").value
        if cfg:
            self._load_yaml(cfg)

        self.get_logger().info(
            f"formation_manager ready  topic={FORMATIONS_TOPIC}  "
            f"formations={list(self._registry.keys())}  "
            f"position_tolerance={self._pos_tol}m"
        )
        self._publish_all()

    # ------------------------------------------------------------------
    # Odometry subscription management
    # ------------------------------------------------------------------

    def _ensure_odom_sub(self, ns: str):
        """Subscribe to /<ns>/odom if not already subscribed."""
        if ns in self._odom_subs:
            return
        if ns not in self._poses:
            self._poses[ns] = _RobotPose(ns)
        self._odom_subs[ns] = self.create_subscription(
            Odometry, f"/{ns}/odom",
            lambda msg, _ns=ns: self._on_odom(msg, _ns),
            10,
        )
        self.get_logger().debug(f"  subscribed to /{ns}/odom")

    def _release_odom_sub(self, ns: str):
        """Unsubscribe from /<ns>/odom if no remaining formation needs it."""
        still_needed = any(
            ns in ([f.leader_ns] + list(f.follower_ns))
            for f in self._registry.values()
        )
        if not still_needed:
            sub = self._odom_subs.pop(ns, None)
            if sub is not None:
                self.destroy_subscription(sub)
            self._poses.pop(ns, None)
            self.get_logger().debug(f"  unsubscribed from /{ns}/odom")

    def _on_odom(self, msg: Odometry, ns: str):
        pose = self._poses.get(ns)
        if pose is None:
            return
        pose.x   = msg.pose.pose.position.x
        pose.y   = msg.pose.pose.position.y
        pose.yaw = _quat_to_yaw(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w,
        )
        pose.received = True

    def _subscribe_formation(self, f: _Formation):
        self._ensure_odom_sub(f.leader_ns)
        for ns in f.follower_ns:
            self._ensure_odom_sub(ns)

    def _release_formation(self, f: _Formation):
        for ns in [f.leader_ns] + list(f.follower_ns):
            self._release_odom_sub(ns)

    # ------------------------------------------------------------------
    # Position check
    # ------------------------------------------------------------------

    def _check_positions(self, f: _Formation) -> tuple[bool, str]:
        """
        Check whether all followers are within position_tolerance of their
        assigned offsets relative to the current leader pose.

        Returns (ok, detail_message).
        Called synchronously from service handlers — uses latest stored odom.
        """
        leader = self._poses.get(f.leader_ns)

        if leader is None or not leader.received:
            return False, (
                f"No odometry received from leader '{f.leader_ns}'. "
                f"Cannot verify positions."
            )

        c, s = math.cos(leader.yaw), math.sin(leader.yaw)
        violations: list[str] = []

        for ns, (ox, oy) in zip(f.follower_ns, f.offsets):
            fw = self._poses.get(ns)

            if fw is None or not fw.received:
                violations.append(f"  {ns}: no odometry received")
                continue

            # Expected world-frame position: leader_pos + R(leader_yaw) * offset
            target_x = leader.x + c * ox - s * oy
            target_y = leader.y + s * ox + c * oy

            error = math.hypot(fw.x - target_x, fw.y - target_y)

            if error > self._pos_tol:
                violations.append(
                    f"  {ns}: {error:.2f}m away from target "
                    f"(tolerance {self._pos_tol:.2f}m, "
                    f"offset=({ox:.2f}, {oy:.2f}))"
                )

        if violations:
            return False, (
                f"Formation '{f.formation_id}': "
                f"{len(violations)}/{len(f.follower_ns)} follower(s) "
                f"out of position:\n" + "\n".join(violations)
            )

        return True, ""

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _on_activate(self, req: ActivateFormation.Request,
                     res: ActivateFormation.Response):
        fid = req.formation_id.strip()
        if fid not in self._registry:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        f = self._registry[fid]

        if f.active:
            res.success = True
            res.message = f"Formation '{fid}' already active"
            return res

        ok, detail = self._check_positions(f)
        if not ok:
            res.success = False
            res.message = (
                f"Formation '{fid}' NOT activated — robots out of position.\n"
                f"{detail}\n"
                f"Move robots into position and retry /formation/activate."
            )
            self.get_logger().warn(res.message)
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

        n = len(req.follower_ns)
        if len(req.offsets_x) != n or len(req.offsets_y) != n:
            res.success = False
            res.message = (
                f"follower_ns length ({n}) must match "
                f"offsets_x ({len(req.offsets_x)}) / offsets_y ({len(req.offsets_y)})"
            )
            return res

        # Release old subscriptions if overwriting
        if fid in self._registry:
            self._release_formation(self._registry[fid])

        f = _Formation(
            formation_id=fid,
            leader_ns=req.leader_ns,
            follower_ns=list(req.follower_ns),
            offsets=list(zip(req.offsets_x, req.offsets_y)),
            active=False,   # always register inactive; activate below if requested
        )
        self._registry[fid] = f
        self._subscribe_formation(f)

        if req.activate:
            ok, detail = self._check_positions(f)
            if ok:
                f.active = True
                self._publish_all()
                res.success = True
                res.message = f"Formation '{fid}' registered and activated"
            else:
                # Register as inactive — caller must explicitly activate later
                self._publish_all()
                res.success = False
                res.message = (
                    f"Formation '{fid}' registered but NOT activated — "
                    f"robots out of position.\n{detail}\n"
                    f"Call /formation/activate once robots are in position."
                )
                self.get_logger().warn(res.message)
        else:
            self._publish_all()
            res.success = True
            res.message = f"Formation '{fid}' registered (inactive)"

        self.get_logger().info(res.message.splitlines()[0])
        return res

    def _on_remove(self, req: RemoveFormation.Request,
                   res: RemoveFormation.Response):
        fid = req.formation_id.strip()
        if fid not in self._registry:
            res.success = False
            res.message = f"Formation '{fid}' not found"
            return res

        f = self._registry[fid]
        if f.active:
            res.success = False
            res.message = (
                f"Formation '{fid}' is active. Deactivate it before removing."
            )
            return res

        self._release_formation(f)
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
        res.formation = self._registry[fid].to_msg(
            stamp=now, padding=self._padding, robot_radius=self._robot_r)
        res.success = True
        res.message = f"Formation '{fid}' returned"
        return res

    def _on_list(self, req: ListFormations.Request,
                 res: ListFormations.Response):
        now = self.get_clock().now().to_msg()
        res.formations = [
            f.to_msg(stamp=now, padding=self._padding, robot_radius=self._robot_r)
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
            with open(path) as fh:
                data = yaml.safe_load(fh)
        except Exception as e:
            res.success = False
            res.message = f"Failed to load file: {e}"
            return res

        if req.clear_existing:
            for f in list(self._registry.values()):
                self._release_formation(f)
            self._registry.clear()

        loaded = 0
        not_activated: list[str] = []

        for entry in data.get("formations", []):
            try:
                fid    = str(entry["id"])
                leader = str(entry["leader"])
                fws    = entry.get("followers", [])

                if fid in self._registry:
                    self._release_formation(self._registry[fid])

                f = _Formation(
                    formation_id=fid,
                    leader_ns=leader,
                    follower_ns=[str(fw["ns"]) for fw in fws],
                    offsets=[(float(fw["dx"]), float(fw["dy"])) for fw in fws],
                    active=False,
                )
                self._registry[fid] = f
                self._subscribe_formation(f)

                if req.activate:
                    ok, detail = self._check_positions(f)
                    if ok:
                        f.active = True
                    else:
                        not_activated.append(fid)
                        self.get_logger().warn(
                            f"'{fid}' loaded but not activated: {detail}")

                loaded += 1
            except Exception as e:
                self.get_logger().error(f"Bad entry skipped: {entry} ({e})")

        self._publish_all()

        res.success = True
        res.loaded_count = loaded
        if not_activated:
            res.message = (
                f"Loaded {loaded} formations. "
                f"Not activated (out of position): {not_activated}"
            )
        else:
            res.message = f"Loaded {loaded} formations"
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
            data["formations"].append({
                "id":     f.formation_id,
                "leader": f.leader_ns,
                "followers": [
                    {"ns": ns, "dx": float(dx), "dy": float(dy)}
                    for ns, (dx, dy) in zip(f.follower_ns, f.offsets)
                ],
            })
            saved += 1

        try:
            with open(path, "w") as fh:
                yaml.safe_dump(data, fh, sort_keys=False)
        except Exception as e:
            res.success = False
            res.message = f"Failed to save file: {e}"
            return res

        res.success = True
        res.message = f"Saved {saved} formations"
        res.saved_count = saved
        return res

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _publish_all(self):
        msg = FormationsConfig()
        now = self.get_clock().now().to_msg()
        msg.header.stamp    = now
        msg.header.frame_id = ""
        for f in self._registry.values():
            msg.formations.append(
                f.to_msg(stamp=now, padding=self._padding,
                         robot_radius=self._robot_r))
        self._pub.publish(msg)
        self.get_logger().debug(
            f"Published: {[f.formation_id for f in self._registry.values()]}")

    def _load_yaml(self, path: str):
        try:
            with open(path) as fh:
                data = yaml.safe_load(fh)
        except Exception as e:
            self.get_logger().error(f"Cannot load '{path}': {e}")
            return

        for entry in data.get("formations", []):
            try:
                fid    = str(entry["id"])
                leader = str(entry["leader"])
                fws    = entry.get("followers", [])
                f = _Formation(
                    formation_id=fid,
                    leader_ns=leader,
                    follower_ns=[str(fw["ns"]) for fw in fws],
                    offsets=[(float(fw["dx"]), float(fw["dy"])) for fw in fws],
                    active=self._auto_act,
                )
                self._registry[fid] = f
                self._subscribe_formation(f)
                self.get_logger().info(
                    f"  loaded '{fid}': leader={leader} "
                    f"followers={f.follower_ns} active={self._auto_act}"
                )
            except KeyError as e:
                self.get_logger().error(
                    f"Malformed formation entry, missing {e}: {entry}")


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