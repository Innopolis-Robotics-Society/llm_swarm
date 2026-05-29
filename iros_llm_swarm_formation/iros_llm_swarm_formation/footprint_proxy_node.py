#!/usr/bin/env python3
"""
footprint_proxy_node
====================
Relays each robot's Nav2 footprint onto a separate topic that the LNS2 planner
subscribes to, substituting the formation footprint for active-formation leaders.

The LNS2 planner models every robot as a circular agent and reads that circle's
radius live from a footprint topic. Plain robots should pass their own Nav2
footprint through unchanged. A robot that leads an active formation must instead
advertise the formation's bounding footprint so the planner leaves room for the
whole group along the leader's path.

Per robot it forwards the latest Nav2 PolygonStamped verbatim, except:
  * leader of an active formation -> the polygon is replaced by that formation's
    footprint until the formation is deactivated;
  * follower of an active formation -> an *empty* polygon is published. The
    planner reads an empty footprint as "no body, not an obstacle" and excludes
    the robot from its grid, so the whole formation's collision volume is carried
    by the leader's enlarged footprint and the followers don't block the leader.
The substituted polygon keeps the incoming Nav2 header — the planner only uses
the polygon's radius, so no frame transform is needed.

Subscriptions
-------------
  /formations/config                          (FormationsConfig, latched)
  /<ns>/local_costmap/published_footprint      (PolygonStamped, per robot)

Publications
------------
  /<ns>/lns/footprint                          (PolygonStamped, per robot)

Parameters
----------
  num_robots            : int    — fleet size (default: 20)
  robot_ns_prefix       : str    — namespace prefix (default: "robot_")
  input_topic_template  : str    — {ns} → robot namespace (default Nav2 footprint)
  output_topic_template  : str    — {ns} → robot namespace (default /<ns>/lns/footprint)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Polygon, PolygonStamped

from iros_llm_swarm_interfaces.msg import FormationsConfig


class FootprintProxyNode(Node):

    def __init__(self):
        super().__init__("footprint_proxy")

        self.declare_parameter("num_robots",           20)
        self.declare_parameter("robot_ns_prefix",      "robot_")
        self.declare_parameter("input_topic_template",
                               "/{ns}/local_costmap/published_footprint")
        self.declare_parameter("output_topic_template", "/{ns}/lns/footprint")

        num_robots = self.get_parameter("num_robots").value
        prefix     = self.get_parameter("robot_ns_prefix").value
        in_tmpl    = self.get_parameter("input_topic_template").value
        out_tmpl   = self.get_parameter("output_topic_template").value

        # leader_ns → formation footprint (only formations with active == True)
        self._leader_footprints: dict[str, Polygon] = {}
        # follower namespaces of active formations — published as empty footprint
        self._follower_set: set[str] = set()

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._config_sub = self.create_subscription(
            FormationsConfig, "/formations/config", self._on_config, latched)

        # depth-1 reliable volatile — matches the Nav2 publisher and the LNS
        # subscriber (rclcpp::QoS(1)) this relay sits between.
        relay_qos = QoSProfile(depth=1)

        self._pubs: list = []
        self._subs: list = []
        for i in range(num_robots):
            ns = f"{prefix}{i}"
            pub = self.create_publisher(
                PolygonStamped, out_tmpl.format(ns=ns), relay_qos)
            sub = self.create_subscription(
                PolygonStamped, in_tmpl.format(ns=ns),
                lambda msg, ns=ns, pub=pub: self._on_footprint(msg, ns, pub),
                relay_qos)
            self._pubs.append(pub)
            self._subs.append(sub)

        self.get_logger().info(
            f"footprint_proxy relaying {num_robots} robots "
            f"({in_tmpl} -> {out_tmpl})")

    def _on_config(self, msg: FormationsConfig) -> None:
        self._leader_footprints = {
            f.leader_ns: f.footprint for f in msg.formations if f.active
        }
        self._follower_set = {
            ns for f in msg.formations if f.active for ns in f.follower_ns
        }

    def _on_footprint(self, msg: PolygonStamped, ns: str, pub) -> None:
        out = PolygonStamped()
        leader_fp = self._leader_footprints.get(ns)
        if leader_fp is not None:
            # The formation footprint is expressed in the leader BODY frame
            # (centered near the leader). Stamp it in base_link so TF places it
            # on the robot; copying the Nav2 odom-frame header would draw the
            # body-frame points at the odom origin instead.
            out.header.stamp = msg.header.stamp
            out.header.frame_id = f"{ns}/base_link"
            out.polygon = leader_fp
        elif ns in self._follower_set:
            out.header = msg.header
            out.polygon = Polygon()  # empty → planner excludes this robot
        else:
            out.header = msg.header
            out.polygon = msg.polygon
        pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintProxyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
