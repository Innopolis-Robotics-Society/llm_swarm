#!/usr/bin/env python3
"""
formation_monitor_node
======================
Monitors all active formations and publishes per-formation health status.

For each formation it tracks:
  - Leader odometry (to detect leader loss)
  - Per-follower odometry (to detect follower loss / stuck)
  - Per-follower position error vs. assigned offset in leader body frame

State machine per formation:
  INACTIVE  → (formation activated)          → FORMING
  FORMING   → (all errors < stable_thresh)   → STABLE
  STABLE    → (any error > degraded_thresh)  → DEGRADED
  DEGRADED  → (all errors < stable_thresh)   → STABLE
  DEGRADED  → (stuck or lost detected)       → BROKEN
  BROKEN    → (formation disbanded)          → INACTIVE
  any       → (formation disbanded)          → INACTIVE

Subscriptions
-------------
  /formations/config          (FormationsConfig, latched)   — registry snapshot
  /<ns>/odom                  (nav_msgs/Odometry)          — per robot, dynamic

Publications
------------
  /formations/status          (FormationsStatus, 10 Hz)    — full snapshot

Parameters
----------
  monitor_hz        : float  — publish rate (default: 10.0)
  stable_thresh_m   : float  — all errors below this → STABLE (default: 0.15)
  degraded_thresh_m : float  — any error above this → DEGRADED (default: 0.35)
  odom_timeout_s    : float  — no odom for this long → LOST (default: 1.0)
  stuck_window_s    : float  — error not improving over this window → STUCK (default: 3.0)
  stuck_delta_m     : float  — min improvement required to not be stuck (default: 0.05)
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from iros_llm_swarm_interfaces.msg import (
    FormationConfig,
    FormationsConfig,
    FormationStatus,
    FormationsStatus,
)

# Helper

def _yaw_from_quat(q):
    # q = geometry_msgs.msg.Quaternion
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Internal per-robot state

@dataclass
class _RobotState:
    ns: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    last_odom_t: float = -1.0          # wall-clock time of last odom msg
    error_m: float = -1.0              # current position error vs offset
    # For stuck detection: (wall_time, error_m) sampled every stuck_window_s
    stuck_sample_t: float = -1.0
    stuck_sample_error: float = -1.0


@dataclass
class _FormationState:
    config: FormationConfig
    leader: _RobotState = field(default_factory=lambda: _RobotState(""))
    followers: List[_RobotState] = field(default_factory=list)
    state: int = FormationStatus.STATE_INACTIVE
    failure_code: int = FormationStatus.FAILURE_NONE
    failure_reason: str = ""


# Node
class FormationMonitorNode(Node):

    def __init__(self):
        super().__init__("formation_monitor")

        self.declare_parameter("monitor_hz",        10.0)
        self.declare_parameter("stable_thresh_m",    0.15)
        self.declare_parameter("degraded_thresh_m",  0.35)
        self.declare_parameter("odom_timeout_s",     1.0)
        self.declare_parameter("stuck_window_s",     3.0)
        self.declare_parameter("stuck_delta_m",      0.05)

        self._hz             = self.get_parameter("monitor_hz").value
        self._stable_thr     = self.get_parameter("stable_thresh_m").value
        self._degraded_thr   = self.get_parameter("degraded_thresh_m").value
        self._odom_timeout   = self.get_parameter("odom_timeout_s").value
        self._stuck_window   = self.get_parameter("stuck_window_s").value
        self._stuck_delta    = self.get_parameter("stuck_delta_m").value

        # formation_id → _FormationState
        self._formations: Dict[str, _FormationState] = {}

        # ns → list of formation_ids that robot belongs to (leader or follower)
        # Used to route incoming odom to the right formation(s)
        self._ns_to_formations: Dict[str, List[str]] = {}

        # Active odom subscriptions: ns → subscription
        self._odom_subs: Dict[str, object] = {}

        # Publication
        self._status_pub = self.create_publisher(
            FormationsStatus, "/formations/status", 10)

        # Registry subscription (latched)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._registry_sub = self.create_subscription(
            FormationsConfig, "/formations/config", self._on_registry, latched)

        # Publish timer
        period = 1.0 / self._hz
        self._timer = self.create_timer(period, self._publish_status)

        self.get_logger().info(
            f"formation_monitor ready  hz={self._hz}  "
            f"stable={self._stable_thr}m  degraded={self._degraded_thr}m"
        )


    # Registry callback — rebuild formation tracking state

    def _on_registry(self, msg: FormationsConfig):
        incoming_ids = {f.formation_id for f in msg.formations}

        # Remove formations that have disappeared from the registry
        for fid in list(self._formations.keys()):
            if fid not in incoming_ids:
                self._remove_formation(fid)

        # Add or update formations
        for f in msg.formations:
            fid = f.formation_id
            if fid in self._formations:
                self._update_formation(self._formations[fid], f)
            else:
                self._add_formation(f)

    def _add_formation(self, cfg: FormationConfig):
        fid = cfg.formation_id
        fs = _FormationState(config=cfg)
        fs.leader = _RobotState(ns=cfg.leader_ns)
        fs.followers = [_RobotState(ns=ns) for ns in cfg.follower_ns]
        fs.state = (FormationStatus.STATE_FORMING if cfg.active
                    else FormationStatus.STATE_INACTIVE)
        self._formations[fid] = fs

        # Subscribe to leader + follower odom
        self._ensure_odom_sub(cfg.leader_ns)
        for ns in cfg.follower_ns:
            self._ensure_odom_sub(ns)

        # Build reverse index
        for ns in [cfg.leader_ns] + list(cfg.follower_ns):
            self._ns_to_formations.setdefault(ns, [])
            if fid not in self._ns_to_formations[ns]:
                self._ns_to_formations[ns].append(fid)

        self.get_logger().info(
            f"[monitor] tracking formation '{fid}'  "
            f"leader={cfg.leader_ns}  followers={list(cfg.follower_ns)}"
        )

    def _update_formation(self, fs: _FormationState, cfg: FormationConfig):
        fid = cfg.formation_id

        # Handle active flag change
        if not cfg.active and fs.state != FormationStatus.STATE_INACTIVE:
            fs.state = FormationStatus.STATE_INACTIVE
            fs.failure_code = FormationStatus.FAILURE_NONE
            fs.failure_reason = ""

        elif cfg.active and fs.state == FormationStatus.STATE_INACTIVE:
            fs.state = FormationStatus.STATE_FORMING

        # Subscribe to any newly added robots
        new_ns = set([cfg.leader_ns] + list(cfg.follower_ns))
        old_ns = set([fs.config.leader_ns] + list(fs.config.follower_ns))

        for ns in new_ns - old_ns:
            self._ensure_odom_sub(ns)
            self._ns_to_formations.setdefault(ns, [])
            if fid not in self._ns_to_formations[ns]:
                self._ns_to_formations[ns].append(fid)

        # Rebuild follower list if membership changed
        if (cfg.leader_ns != fs.config.leader_ns or
                list(cfg.follower_ns) != list(fs.config.follower_ns)):
            fs.leader = _RobotState(ns=cfg.leader_ns)
            # Preserve existing follower state for robots that remain
            old_map = {fw.ns: fw for fw in fs.followers}
            fs.followers = [
                old_map.get(ns, _RobotState(ns=ns))
                for ns in cfg.follower_ns
            ]

        fs.config = cfg

    def _remove_formation(self, fid: str):
        fs = self._formations.pop(fid, None)
        if fs is None:
            return
        for ns in [fs.config.leader_ns] + list(fs.config.follower_ns):
            fids = self._ns_to_formations.get(ns, [])
            if fid in fids:
                fids.remove(fid)
            # Unsubscribe if robot no longer belongs to any formation
            if not fids:
                self._ns_to_formations.pop(ns, None)
                self._odom_subs.pop(ns, None)
        self.get_logger().info(f"[monitor] stopped tracking formation '{fid}'")


    # Odometry subscriptions

    def _ensure_odom_sub(self, ns: str):
        if ns in self._odom_subs:
            return
        sub = self.create_subscription(
            Odometry,
            f"/{ns}/odom",
            lambda msg, _ns=ns: self._on_odom(msg, _ns),
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10),
        )
        self._odom_subs[ns] = sub

    def _on_odom(self, msg: Odometry, ns: str):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        t = time.monotonic()

        for fid in self._ns_to_formations.get(ns, []):
            fs = self._formations.get(fid)
            if fs is None:
                continue

            if ns == fs.config.leader_ns:
                fs.leader.x = x
                fs.leader.y = y
                fs.leader.yaw = yaw
                fs.leader.last_odom_t = t
            else:
                for fw in fs.followers:
                    if fw.ns == ns:
                        fw.x = x
                        fw.y = y
                        fw.yaw = yaw
                        fw.last_odom_t = t
                        break


    # Publish timer — compute errors and update states

    def _publish_status(self):
        now_t = time.monotonic()
        out = FormationsStatus()
        out.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="")

        for fid, fs in self._formations.items():
            status = self._compute_status(fs, now_t)
            out.formations.append(status)

        self._status_pub.publish(out)

    def _compute_status(self, fs: _FormationState,
                        now_t: float) -> FormationStatus:
        msg = FormationStatus()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="")
        msg.formation_id = fs.config.formation_id
        msg.leader_ns    = fs.config.leader_ns
        msg.follower_ns  = list(fs.config.follower_ns)
        msg.failure_code = FormationStatus.FAILURE_NONE
        msg.failure_reason = ""

        # INACTIVE fast-path
        if not fs.config.active:
            msg.state = FormationStatus.STATE_INACTIVE
            msg.follower_errors_m = [-1.0] * len(fs.followers)
            msg.max_error_m  = -1.0
            msg.mean_error_m = -1.0
            fs.state = FormationStatus.STATE_INACTIVE
            return msg

        # Check leader odom timeout
        if (fs.leader.last_odom_t < 0 or
                now_t - fs.leader.last_odom_t > self._odom_timeout):
            msg.state        = FormationStatus.STATE_BROKEN
            msg.failure_code = FormationStatus.FAILURE_LEADER_LOST
            msg.failure_reason = (
                f"Leader {fs.config.leader_ns} odom timeout "
                f"({now_t - fs.leader.last_odom_t:.1f}s)"
                if fs.leader.last_odom_t >= 0
                else f"Leader {fs.config.leader_ns} odom never received"
            )
            msg.follower_errors_m = [-1.0] * len(fs.followers)
            msg.max_error_m  = -1.0
            msg.mean_error_m = -1.0
            fs.state         = FormationStatus.STATE_BROKEN
            fs.failure_code  = msg.failure_code
            fs.failure_reason = msg.failure_reason
            return msg

        # Compute per-follower errors
        leader_x   = fs.leader.x
        leader_y   = fs.leader.y
        # Euclidean distance to the expected world-frame target
        errors: List[float] = []
        lost_followers: List[str] = []
        stuck_followers: List[str] = []

        for i, fw in enumerate(fs.followers):
            # Lost check
            if (fw.last_odom_t < 0 or
                    now_t - fw.last_odom_t > self._odom_timeout):
                errors.append(-1.0)
                lost_followers.append(fw.ns)
                continue

            # Offset comes from FormationConfig.offsets[i]
            if i < len(fs.config.offsets):
                ox = fs.config.offsets[i].x
                oy = fs.config.offsets[i].y
            else:
                errors.append(-1.0)
                continue

            # Expected world-frame position (distance-preserving)
            cy = math.cos(fs.leader.yaw)
            sy = math.sin(fs.leader.yaw)

            wx = leader_x + (cy * ox - sy * oy)
            wy = leader_y + (sy * ox + cy * oy)

            err = math.hypot(wx - fw.x, wy - fw.y)
            errors.append(err)
            fw.error_m = err

            # Stuck detection: sample error every stuck_window_s
            if fw.stuck_sample_t < 0:
                fw.stuck_sample_t     = now_t
                fw.stuck_sample_error = err
            elif now_t - fw.stuck_sample_t >= self._stuck_window:
                improvement = fw.stuck_sample_error - err
                if improvement < self._stuck_delta and err > self._degraded_thr:
                    stuck_followers.append(fw.ns)
                # Reset sample
                fw.stuck_sample_t     = now_t
                fw.stuck_sample_error = err

        msg.follower_errors_m = [float(e) for e in errors]

        valid_errors = [e for e in errors if e >= 0.0]
        msg.max_error_m  = float(max(valid_errors))  if valid_errors else -1.0
        msg.mean_error_m = float(sum(valid_errors) / len(valid_errors)) if valid_errors else -1.0

        # Determine state
        if lost_followers:
            new_state     = FormationStatus.STATE_BROKEN
            failure_code  = FormationStatus.FAILURE_FOLLOWER_LOST
            failure_reason = f"Follower(s) odom lost: {lost_followers}"
        elif stuck_followers:
            new_state     = FormationStatus.STATE_BROKEN
            failure_code  = FormationStatus.FAILURE_FOLLOWER_STUCK
            failure_reason = f"Follower(s) stuck: {stuck_followers}"
        elif valid_errors and max(valid_errors) > self._degraded_thr:
            new_state     = FormationStatus.STATE_DEGRADED
            failure_code  = FormationStatus.FAILURE_NONE
            failure_reason = ""
        elif valid_errors and max(valid_errors) <= self._stable_thr:
            new_state     = FormationStatus.STATE_STABLE
            failure_code  = FormationStatus.FAILURE_NONE
            failure_reason = ""
        else:
            # Errors exist but between thresholds — keep current state
            # to avoid oscillating between STABLE and DEGRADED
            new_state     = fs.state if fs.state not in (
                FormationStatus.STATE_INACTIVE,
                FormationStatus.STATE_BROKEN,
            ) else FormationStatus.STATE_FORMING
            failure_code  = FormationStatus.FAILURE_NONE
            failure_reason = ""

        # Never go from BROKEN back to active state without disbanding
        if fs.state == FormationStatus.STATE_BROKEN and \
                new_state not in (FormationStatus.STATE_BROKEN,
                                  FormationStatus.STATE_INACTIVE):
            new_state     = FormationStatus.STATE_BROKEN
            failure_code  = fs.failure_code
            failure_reason = fs.failure_reason

        fs.state         = new_state
        fs.failure_code  = failure_code
        fs.failure_reason = failure_reason

        msg.state          = new_state
        msg.failure_code   = failure_code
        msg.failure_reason = failure_reason
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FormationMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()