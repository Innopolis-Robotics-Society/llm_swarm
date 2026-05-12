"""Live robot-pose cache for /llm/chat runtime context.

Owns one subscription per known robot to ``/robot_{id}/odom``. ``snapshot()``
returns a per-robot dict of ``(x, y, yaw, stale_ms)`` consumed by both
context providers, so the LLM can reason from actual positions instead of
inventing coordinates.

Local subscriptions are deliberate: 20 robots × ~20 Hz over rosbridge would
blow ``context_timeout_sec`` budgets on every chat turn. In-process subs
keep reads O(1).
"""

from __future__ import annotations

import math
import threading
from typing import Any, Iterable


class RobotPoseCache:
    """Latest /robot_N/odom per robot, accessed by snapshot()."""

    def __init__(self, node, robot_ids: Iterable[int]):
        self._node = node
        self._lock = threading.Lock()
        # rid -> (x, y, yaw, stamp_ns)
        self._latest: dict[int, tuple[float, float, float, int]] = {}
        self._subs: list = []
        self._enabled = self._create_subscriptions(robot_ids)

    @property
    def enabled(self) -> bool:
        return self._enabled

    def _create_subscriptions(self, robot_ids: Iterable[int]) -> bool:
        try:
            from nav_msgs.msg import Odometry
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        except Exception as exc:
            if hasattr(self._node, 'get_logger'):
                self._node.get_logger().warn(
                    f'RobotPoseCache disabled (imports failed): {exc}')
            return False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        for rid in robot_ids:
            try:
                rid_int = int(rid)
            except (TypeError, ValueError):
                continue
            topic = f'/robot_{rid_int}/odom'
            self._subs.append(self._node.create_subscription(
                Odometry,
                topic,
                lambda msg, _id=rid_int: self._on_odom(_id, msg),
                qos,
            ))
        return True

    def _on_odom(self, rid: int, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from quaternion (Z component only — diff-drive on a plane)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp_ns = self._node.get_clock().now().nanoseconds
        with self._lock:
            self._latest[rid] = (float(p.x), float(p.y), float(yaw), stamp_ns)

    @staticmethod
    def _robot_id_from_ns(ns: str) -> int | None:
        if not isinstance(ns, str):
            return None
        suffix = ns[len('robot_'):] if ns.startswith('robot_') else ns
        try:
            return int(suffix)
        except (TypeError, ValueError):
            return None

    def snapshot(self, stale_threshold_ms: int = 2000) -> dict[int, dict]:
        """Return a copy of cached poses keyed by integer robot id.

        Each entry carries ``x``, ``y``, ``yaw``, ``stale_ms`` (age of the
        last odom message), and a boolean ``stale`` flag set when
        ``stale_ms`` exceeds ``stale_threshold_ms``. Robots that have
        never produced odom are omitted — callers infer "missing pose" by
        absence.
        """
        now_ns = self._node.get_clock().now().nanoseconds
        out: dict[int, dict] = {}
        with self._lock:
            items = list(self._latest.items())
        for rid, (x, y, yaw, stamp_ns) in items:
            stale_ms = max(0, int((now_ns - stamp_ns) / 1_000_000))
            out[rid] = {
                'x': round(x, 3),
                'y': round(y, 3),
                'yaw': round(yaw, 3),
                'stale_ms': stale_ms,
                'stale': stale_ms > stale_threshold_ms,
            }
        return out


def compute_formation_staging(
    formation_node: dict,
    snapshot: dict[int, dict],
    *,
    tolerance_m: float = 0.5,
) -> dict | None:
    """Return a mapf leaf that moves out-of-tolerance followers to their
    leader-relative targets, or None if no staging is required.

    The math mirrors ``formation_manager_node._check_positions``:
        target = leader_xy + R(leader_yaw) @ (offset_x, offset_y)

    If the leader pose is missing or stale, returns None — staging without a
    known leader pose would be guessing. The formation_manager will then
    reject the leaf with its own clear "no odometry" / "out of position"
    message, which feeds the LLM remediation prompt.
    """
    if formation_node.get('type') != 'formation':
        return None
    leader_ns = formation_node.get('leader_ns', '')
    leader_id = RobotPoseCache._robot_id_from_ns(leader_ns)
    if leader_id is None:
        return None
    leader = snapshot.get(leader_id)
    if leader is None or leader.get('stale'):
        return None

    follower_ns = list(formation_node.get('follower_ns', []) or [])
    offsets_x = list(formation_node.get('offsets_x', []) or [])
    offsets_y = list(formation_node.get('offsets_y', []) or [])
    n = min(len(follower_ns), len(offsets_x), len(offsets_y))
    if n == 0:
        return None

    c = math.cos(float(leader['yaw']))
    s = math.sin(float(leader['yaw']))
    lx = float(leader['x'])
    ly = float(leader['y'])

    out_ids: list[int] = []
    out_goals: list[list[float]] = []
    for i in range(n):
        fid = RobotPoseCache._robot_id_from_ns(follower_ns[i])
        if fid is None:
            continue
        fpos = snapshot.get(fid)
        if fpos is None or fpos.get('stale'):
            # No live pose for this follower — let formation_manager
            # surface the precise error rather than guessing a target.
            continue
        ox = float(offsets_x[i])
        oy = float(offsets_y[i])
        tx = lx + c * ox - s * oy
        ty = ly + s * ox + c * oy
        dist = math.hypot(float(fpos['x']) - tx, float(fpos['y']) - ty)
        if dist > tolerance_m:
            out_ids.append(fid)
            out_goals.append([round(tx, 3), round(ty, 3)])

    if not out_ids:
        return None
    return {
        'type': 'mapf',
        'robot_ids': out_ids,
        'goals': out_goals,
        'reason': (
            f'auto-stage {len(out_ids)} follower(s) for '
            f"{formation_node.get('formation_id','')}"
        ),
    }
