"""ToolExecutor — read-only tool implementations for user chat (channel 3).

Tools:
  get_robot_position(robot_id) — current pose from RobotPoseCache
  get_positions(room, qualifier) — named geometry from map_cfg["geometry"]
  check_occupancy(robot_id)   — one-shot LaserScan → free/occupied cell grid
  find_free_group_goals_in_room(...) — occupancy-aware normal MAPF room goals
  find_group_placement_in_room(...) — deterministic room/formation placement
  verify_plan_execution_state(...) — deterministic post-execution verification
"""

from __future__ import annotations

import asyncio
import math
import threading
from collections import deque
from typing import Any

from iros_llm_orchestrator.context.execution_verification import (
    verify_plan_execution_state,
)
from iros_llm_orchestrator.context.group_placement import (
    find_free_group_goals_in_room,
    find_group_placement_in_room,
)


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
        self._latest_formations_status: Any | None = None
        self._latest_bt_state: Any | None = None
        self._recent_events = deque(maxlen=12)
        self._state_subscriptions: list[Any] = []
        self._create_verification_state_subscriptions()
        self._list_tasks_client: Any | None = None
        self._reset_task_client: Any | None = None
        self._create_task_clients()

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
        if name == "find_group_placement_in_room":
            result = self._find_group_placement_in_room(arguments)
            self._log_tool_result(name, result)
            return result
        if name == "find_free_group_goals_in_room":
            self._log_tool_start(name, arguments)
            result = self._find_free_group_goals_in_room(arguments)
            self._log_tool_result(name, result)
            return result
        if name == "verify_plan_execution_state":
            result = self._verify_plan_execution_state(arguments)
            self._log_tool_result(name, result)
            return result
        if name == "list_tasks":
            return self._call_list_tasks()
        if name == "reset_task":
            return self._call_reset_task(arguments.get("task_id", ""))
        return {"error": f"unknown tool '{name}'"}

    def _create_verification_state_subscriptions(self) -> None:
        if self._node is None or not hasattr(self._node, "create_subscription"):
            return
        try:
            from iros_llm_swarm_interfaces.msg import (
                BTState,
                FormationsStatus,
                LlmEvent,
            )
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        except Exception:
            return
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        try:
            self._state_subscriptions.append(
                self._node.create_subscription(
                    FormationsStatus,
                    "/formations/status",
                    self._on_formations_status,
                    qos,
                )
            )
            self._state_subscriptions.append(
                self._node.create_subscription(
                    BTState,
                    "/bt/state",
                    self._on_bt_state,
                    qos,
                )
            )
            self._state_subscriptions.append(
                self._node.create_subscription(
                    LlmEvent,
                    "/llm/events",
                    self._on_llm_event,
                    reliable_qos,
                )
            )
        except Exception:
            self._state_subscriptions = []

    def _on_formations_status(self, msg: Any) -> None:
        self._latest_formations_status = msg

    def _on_bt_state(self, msg: Any) -> None:
        self._latest_bt_state = msg

    def _on_llm_event(self, msg: Any) -> None:
        self._recent_events.append({
            "trigger": str(getattr(msg, "trigger", "") or ""),
            "output": str(getattr(msg, "output", "") or ""),
            "reason": str(getattr(msg, "reason", "") or ""),
        })

    def formations_status_snapshot(self) -> Any | None:
        """Return the latest raw /formations/status message, if available."""
        return self._latest_formations_status

    def _log_tool_result(self, name: str, result: dict) -> None:
        logger = self._node.get_logger() if hasattr(self._node, "get_logger") else None
        if logger is None:
            return
        try:
            if name == "find_group_placement_in_room":
                logger.info(
                    "LLM tool: find_group_placement_in_room result "
                    f"ok={bool(result.get('ok'))} "
                    f"room={result.get('room', '')} "
                    f"placements={len(result.get('placements') or [])}"
                )
            elif name == "find_free_group_goals_in_room":
                if bool(result.get('ok')):
                    logger.info(
                        "LLM tool: find_free_group_goals_in_room result "
                        f"ok=true room={result.get('room', '')} "
                        f"goals={len(result.get('goals') or [])}"
                    )
                else:
                    logger.info(
                        "LLM tool: find_free_group_goals_in_room result "
                        f"ok=false reason={result.get('reason', '')}"
                    )
            elif name == "verify_plan_execution_state":
                logger.info(
                    "LLM tool: verify_plan_execution_state result "
                    f"ok={bool(result.get('ok'))} "
                    f"summary={str(result.get('summary') or '')[:160]}"
                )
        except Exception:
            return

    def _log_tool_start(self, name: str, arguments: dict) -> None:
        logger = self._node.get_logger() if hasattr(self._node, "get_logger") else None
        if logger is None:
            return
        try:
            if name == "find_free_group_goals_in_room":
                logger.info(
                    "LLM tool: find_free_group_goals_in_room start "
                    f"room={(arguments or {}).get('room', '')} "
                    f"robots={(arguments or {}).get('robot_ids', [])}"
                )
        except Exception:
            return

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
    # Tool: find_group_placement_in_room
    # ------------------------------------------------------------------

    def _find_group_placement_in_room(self, arguments: dict) -> dict:
        snapshot = None
        if self._pose_cache is not None:
            try:
                snapshot = self._pose_cache.snapshot()
            except Exception:
                snapshot = None
        return find_group_placement_in_room(
            self._map_cfg,
            arguments or {},
            pose_snapshot=snapshot,
            robot_footprint_radius=self._footprint_radius,
        )

    # ------------------------------------------------------------------
    # Tool: find_free_group_goals_in_room
    # ------------------------------------------------------------------

    def _find_free_group_goals_in_room(self, arguments: dict) -> dict:
        snapshot = None
        if self._pose_cache is not None:
            try:
                snapshot = self._pose_cache.snapshot()
            except Exception:
                snapshot = None
        return find_free_group_goals_in_room(
            self._map_cfg,
            arguments or {},
            pose_snapshot=snapshot,
            robot_footprint_radius=self._footprint_radius,
        )

    # ------------------------------------------------------------------
    # Tool: verify_plan_execution_state
    # ------------------------------------------------------------------

    def _verify_plan_execution_state(self, arguments: dict) -> dict:
        snapshot = None
        if self._pose_cache is not None:
            try:
                snapshot = self._pose_cache.snapshot()
            except Exception:
                snapshot = None
        return verify_plan_execution_state(
            self._map_cfg,
            arguments or {},
            pose_snapshot=snapshot,
            formations_status=(
                (arguments or {}).get('_formations_status')
                if (arguments or {}).get('_formations_status') is not None
                else self._latest_formations_status
            ),
            bt_state=(
                (arguments or {}).get('_bt_state')
                if (arguments or {}).get('_bt_state') is not None
                else self._latest_bt_state
            ),
            recent_events=(
                list((arguments or {}).get('_recent_events') or [])
                or list(self._recent_events)
            ),
        )

    # ------------------------------------------------------------------
    # Task service clients
    # ------------------------------------------------------------------

    def _create_task_clients(self) -> None:
        if self._node is None or not hasattr(self._node, 'create_client'):
            return
        try:
            from iros_llm_swarm_interfaces.srv import ListTasks, ResetTask
            self._list_tasks_client = self._node.create_client(ListTasks, '/tasks/list')
            self._reset_task_client = self._node.create_client(ResetTask, '/tasks/reset')
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Tool: list_tasks
    # ------------------------------------------------------------------

    def _call_list_tasks(self) -> dict:
        if self._list_tasks_client is None:
            return {"error": "task service client not initialised"}
        try:
            from iros_llm_swarm_interfaces.srv import ListTasks
        except ImportError:
            return {"error": "iros_llm_swarm_interfaces not available"}
        if not self._list_tasks_client.wait_for_service(timeout_sec=0.3):
            return {"error": "/tasks/list service not available"}
        try:
            resp = self._list_tasks_client.call(ListTasks.Request())
            tasks: dict = {}
            for state in resp.states:
                t = state.task
                entry: dict = {
                    'type': t.type,
                    'label': t.label,
                    'status': state.status,
                    'position': [round(t.position[0], 2), round(t.position[1], 2)],
                    'assigned': list(state.assigned_robot_ids),
                }
                if t.type == 'carry':
                    entry['dropoff'] = [round(t.dropoff[0], 2), round(t.dropoff[1], 2)]
                tasks[t.id] = entry
            return {"tasks": tasks}
        except Exception as exc:
            return {"error": str(exc)}

    # ------------------------------------------------------------------
    # Tool: reset_task
    # ------------------------------------------------------------------

    def _call_reset_task(self, task_id: str) -> dict:
        if not task_id:
            return {"error": "task_id is required"}
        if self._reset_task_client is None:
            return {"error": "task service client not initialised"}
        try:
            from iros_llm_swarm_interfaces.srv import ResetTask
        except ImportError:
            return {"error": "iros_llm_swarm_interfaces not available"}
        if not self._reset_task_client.wait_for_service(timeout_sec=0.3):
            return {"error": "/tasks/reset service not available"}
        try:
            req = ResetTask.Request()
            req.id = task_id
            resp = self._reset_task_client.call(req)
            return {"success": resp.success, "message": resp.message}
        except Exception as exc:
            return {"error": str(exc)}

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
