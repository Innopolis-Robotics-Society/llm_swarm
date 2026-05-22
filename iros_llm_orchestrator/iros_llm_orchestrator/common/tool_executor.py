"""ToolExecutor — three tool implementations for user chat (channel 3).

Tools:
  get_robot_position(robot_id) — current pose from RobotPoseCache
  get_positions(room, qualifier) — named geometry from map_cfg["geometry"]
  check_occupancy(robot_id)   — one-shot LaserScan → free/occupied cell grid
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import Any


class ToolExecutor:
    """Dispatch tool calls requested by the LLM in user_chat_node."""

    def __init__(
        self,
        node: Any,
        pose_cache: Any,
        map_cfg: dict,
        robot_footprint_radius: float = 0.22,
        scan_timeout_sec: float = 3.0,
    ) -> None:
        self._node = node
        self._pose_cache = pose_cache
        self._map_cfg = map_cfg
        self._footprint_radius = robot_footprint_radius
        self._scan_timeout = scan_timeout_sec

    async def call(self, name: str, arguments: dict) -> dict:
        """Dispatch a tool call by name, always returns a JSON-serialisable dict."""
        if name == "check_occupancy":
            return await self._check_occupancy(arguments.get("robot_id", ""))
        if name == "get_positions":
            return self._get_positions(
                arguments.get("room", ""), arguments.get("qualifier", "")
            )
        if name == "get_robot_position":
            return self._get_robot_position(arguments.get("robot_id", ""))
        return {"error": f"unknown tool '{name}'"}

    # ------------------------------------------------------------------
    # Tool: get_robot_position
    # ------------------------------------------------------------------

    def _get_robot_position(self, robot_id: str) -> dict:
        if self._pose_cache is None:
            return {"error": "pose cache not available"}
        rid = _parse_robot_id(robot_id)
        if rid is None:
            return {"error": f"invalid robot_id '{robot_id}'"}
        snapshot = self._pose_cache.snapshot()
        entry = snapshot.get(rid)
        if entry is None:
            return {"error": f"{robot_id} not in pose cache"}
        return {
            "robot_id": robot_id,
            "x": entry["x"],
            "y": entry["y"],
            "yaw": entry["yaw"],
            "stale": entry["stale"],
            "stale_ms": entry["stale_ms"],
        }

    # ------------------------------------------------------------------
    # Tool: get_positions
    # ------------------------------------------------------------------

    def _get_positions(self, room: str, qualifier: str) -> dict:
        geometry = self._map_cfg.get("geometry", {})
        room_data = geometry.get(room.lower())
        if room_data is None:
            return {"error": f"unknown room '{room}' or qualifier '{qualifier}'"}
        node: Any = room_data
        for key in qualifier.split("."):
            if not isinstance(node, dict) or key not in node:
                return {"error": f"unknown room '{room}' or qualifier '{qualifier}'"}
            node = node[key]
        if not isinstance(node, (list, tuple)) or len(node) < 2:
            return {
                "error": (
                    f"qualifier '{qualifier}' in room '{room}' "
                    "is not a [x, y] coordinate"
                )
            }
        return {
            "room": room,
            "qualifier": qualifier,
            "position": [float(node[0]), float(node[1])],
        }

    # ------------------------------------------------------------------
    # Tool: check_occupancy
    # ------------------------------------------------------------------

    async def _check_occupancy(self, robot_id: str) -> dict:
        if self._pose_cache is None:
            return {"error": "pose cache not available"}
        rid = _parse_robot_id(robot_id)
        if rid is None:
            return {"error": f"invalid robot_id '{robot_id}'"}

        snapshot = self._pose_cache.snapshot(stale_threshold_ms=2000)
        pose = snapshot.get(rid)
        if pose is None:
            return {"error": f"{robot_id} pose unavailable"}
        if pose.get("stale"):
            return {"error": f"{robot_id} pose stale"}

        rx, ry, ryaw = float(pose["x"]), float(pose["y"]), float(pose["yaw"])

        scan = await self._get_laser_scan(robot_id)
        if scan is None:
            return {"error": f"{robot_id} laser timeout"}

        cell_size = (self._footprint_radius * 2.0) * 1.1
        free_cells, occupied_cells = _build_occupancy_grid(
            scan, rx, ry, ryaw, cell_size
        )

        return {
            "robot_id": robot_id,
            "cell_size_m": round(cell_size, 4),
            "robot_pose": {
                "x": round(rx, 3),
                "y": round(ry, 3),
                "yaw": round(ryaw, 3),
            },
            "free_cells": [[round(x, 3), round(y, 3)] for x, y in free_cells],
            "occupied_cells": [[round(x, 3), round(y, 3)] for x, y in occupied_cells],
        }

    async def _get_laser_scan(self, robot_id: str) -> Any | None:
        """Subscribe once to /robot_{id}/scan; return first message within timeout."""
        try:
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        except ImportError:
            return None

        topic = f"/{robot_id}/scan"
        arrived = threading.Event()
        result: list[Any] = [None]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        def _cb(msg: Any) -> None:
            result[0] = msg
            arrived.set()

        sub = self._node.create_subscription(LaserScan, topic, _cb, qos)
        try:
            loop = asyncio.get_running_loop()
            # arrived.wait() is blocking — offload to thread pool so the
            # asyncio loop stays responsive during the wait.
            got = await loop.run_in_executor(
                None, lambda: arrived.wait(self._scan_timeout)
            )
        finally:
            self._node.destroy_subscription(sub)

        return result[0] if got else None


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _parse_robot_id(robot_id: str) -> int | None:
    if not isinstance(robot_id, str):
        return None
    s = robot_id.strip()
    if s.startswith("robot_"):
        s = s[len("robot_"):]
    try:
        return int(s)
    except (ValueError, TypeError):
        return None


def _build_occupancy_grid(
    scan: Any,
    rx: float,
    ry: float,
    ryaw: float,
    cell_size: float,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Transform LaserScan to map frame and classify cells as free or occupied."""
    ranges: list[float] = list(scan.ranges)
    angle_min: float = float(scan.angle_min)
    angle_inc: float = float(scan.angle_increment)
    range_max: float = float(scan.range_max)
    range_min: float = float(scan.range_min)

    cos_yaw = math.cos(ryaw)
    sin_yaw = math.sin(ryaw)

    # Hit points in map frame
    hit_points: list[tuple[float, float]] = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r >= range_max:
            continue
        angle = angle_min + i * angle_inc
        lx = r * math.cos(angle)
        ly = r * math.sin(angle)
        hit_points.append((
            rx + cos_yaw * lx - sin_yaw * ly,
            ry + sin_yaw * lx + cos_yaw * ly,
        ))

    # Candidate cell centres within laser range
    half = cell_size / 2.0
    n_cells = int(range_max / cell_size) + 1
    free_cells: list[tuple[float, float]] = []
    occupied_cells: list[tuple[float, float]] = []

    for row in range(-n_cells, n_cells + 1):
        for col in range(-n_cells, n_cells + 1):
            cx = rx + col * cell_size
            cy = ry + row * cell_size
            if math.hypot(cx - rx, cy - ry) > range_max:
                continue
            occ = any(
                abs(hx - cx) <= half and abs(hy - cy) <= half
                for hx, hy in hit_points
            )
            if occ:
                occupied_cells.append((cx, cy))
            else:
                free_cells.append((cx, cy))

    return free_cells, occupied_cells
