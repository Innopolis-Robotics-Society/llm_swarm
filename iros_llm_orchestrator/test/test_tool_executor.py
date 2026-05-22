"""Tests for ToolExecutor — get_robot_position, get_positions, check_occupancy."""

import asyncio
import math
import types
from unittest.mock import MagicMock, patch

import pytest


def _run(coro):
    """Run a coroutine synchronously (avoids pytest-asyncio dependency)."""
    return asyncio.new_event_loop().run_until_complete(coro)

from iros_llm_orchestrator.common.tool_executor import (
    ToolExecutor,
    _build_occupancy_grid,
    _parse_robot_id,
)


# ---------------------------------------------------------------------------
# _parse_robot_id helper
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
    ("robot_3", 3),
    ("robot_0", 0),
    ("3",       3),
    ("15",      15),
    ("robot_",  None),
    ("abc",     None),
    ("",        None),
])
def test_parse_robot_id(raw, expected):
    assert _parse_robot_id(raw) == expected


# ---------------------------------------------------------------------------
# Fixture: ToolExecutor with mock pose cache
# ---------------------------------------------------------------------------

MAP_CFG = {
    "geometry": {
        "nw_hall": {
            "center": [-27.0, 14.0],
            "corners": {
                "top_left":     [-33.0, 19.0],
                "top_right":    [-21.0, 19.0],
                "bottom_left":  [-33.0,  9.0],
                "bottom_right": [-21.0,  9.0],
            },
            "doorways": {
                "south": [-27.0, 9.0],
                "east":  [-21.0, 14.0],
            },
        }
    }
}


def _make_executor(poses: dict) -> ToolExecutor:
    node = MagicMock()
    pose_cache = MagicMock()
    pose_cache.snapshot.return_value = poses
    return ToolExecutor(
        node=node,
        pose_cache=pose_cache,
        map_cfg=MAP_CFG,
        robot_footprint_radius=0.22,
        scan_timeout_sec=3.0,
    )


# ---------------------------------------------------------------------------
# get_robot_position
# ---------------------------------------------------------------------------

def test_get_robot_position_found():
    ex = _make_executor({3: {"x": 5.2, "y": -3.1, "yaw": 1.57, "stale": False, "stale_ms": 45}})
    result = ex._get_robot_position("robot_3")
    assert result["x"] == 5.2
    assert result["y"] == -3.1
    assert result["yaw"] == 1.57
    assert result["stale"] is False
    assert result["stale_ms"] == 45
    assert result["robot_id"] == "robot_3"


def test_get_robot_position_not_in_cache():
    ex = _make_executor({})
    result = ex._get_robot_position("robot_3")
    assert "error" in result
    assert "robot_3" in result["error"]


def test_get_robot_position_invalid_id():
    ex = _make_executor({})
    result = ex._get_robot_position("not_a_robot")
    assert "error" in result


def test_get_robot_position_no_pose_cache():
    node = MagicMock()
    ex = ToolExecutor(node=node, pose_cache=None, map_cfg=MAP_CFG)
    result = ex._get_robot_position("robot_3")
    assert "error" in result


# ---------------------------------------------------------------------------
# get_positions
# ---------------------------------------------------------------------------

def test_get_positions_center():
    ex = _make_executor({})
    result = ex._get_positions("nw_hall", "center")
    assert result["position"] == [-27.0, 14.0]
    assert result["room"] == "nw_hall"
    assert result["qualifier"] == "center"


def test_get_positions_nested():
    ex = _make_executor({})
    result = ex._get_positions("nw_hall", "corners.top_right")
    assert result["position"] == [-21.0, 19.0]


def test_get_positions_doorway():
    ex = _make_executor({})
    result = ex._get_positions("nw_hall", "doorways.east")
    assert result["position"] == [-21.0, 14.0]


def test_get_positions_unknown_room():
    ex = _make_executor({})
    result = ex._get_positions("no_such_room", "center")
    assert "error" in result
    assert "no_such_room" in result["error"]


def test_get_positions_unknown_qualifier():
    ex = _make_executor({})
    result = ex._get_positions("nw_hall", "corners.middle")
    assert "error" in result


def test_get_positions_case_insensitive_room():
    ex = _make_executor({})
    result = ex._get_positions("NW_HALL", "center")
    assert result["position"] == [-27.0, 14.0]


# ---------------------------------------------------------------------------
# check_occupancy (mocked laser scan)
# ---------------------------------------------------------------------------

def _make_scan(
    angle_min: float = -math.pi,
    angle_max: float = math.pi,
    n_rays: int = 36,
    range_max: float = 5.0,
    range_min: float = 0.1,
    hit_range: float = 3.0,
    hit_at_ray: int = 5,
) -> MagicMock:
    angle_inc = (angle_max - angle_min) / n_rays
    ranges = [range_max] * n_rays      # all clear except one hit
    if 0 <= hit_at_ray < n_rays:
        ranges[hit_at_ray] = hit_range
    scan = MagicMock()
    scan.ranges = ranges
    scan.angle_min = angle_min
    scan.angle_increment = angle_inc
    scan.range_max = range_max
    scan.range_min = range_min
    return scan


def test_check_occupancy_returns_structure():
    scan = _make_scan()
    poses = {3: {"x": 0.0, "y": 0.0, "yaw": 0.0, "stale": False, "stale_ms": 10}}
    node = MagicMock()
    pose_cache = MagicMock()
    pose_cache.snapshot.return_value = poses

    ex = ToolExecutor(node=node, pose_cache=pose_cache, map_cfg=MAP_CFG,
                      robot_footprint_radius=0.22, scan_timeout_sec=0.1)

    async def _fake_scan(robot_id):
        return scan
    ex._get_laser_scan = _fake_scan

    result = _run(ex._check_occupancy("robot_3"))
    assert "error" not in result
    assert result["robot_id"] == "robot_3"
    assert "free_cells" in result
    assert "occupied_cells" in result
    assert result["cell_size_m"] == pytest.approx(0.484, rel=1e-3)
    assert isinstance(result["free_cells"], list)
    assert isinstance(result["occupied_cells"], list)
    assert len(result["free_cells"]) > 0


def test_check_occupancy_stale_pose():
    poses = {3: {"x": 0.0, "y": 0.0, "yaw": 0.0, "stale": True, "stale_ms": 5000}}
    ex = _make_executor(poses)
    result = _run(ex._check_occupancy("robot_3"))
    assert "error" in result
    assert "stale" in result["error"]


def test_check_occupancy_missing_pose():
    ex = _make_executor({})
    result = _run(ex._check_occupancy("robot_3"))
    assert "error" in result


def test_check_occupancy_laser_timeout():
    poses = {3: {"x": 0.0, "y": 0.0, "yaw": 0.0, "stale": False, "stale_ms": 10}}
    node = MagicMock()
    pose_cache = MagicMock()
    pose_cache.snapshot.return_value = poses
    ex = ToolExecutor(node=node, pose_cache=pose_cache, map_cfg=MAP_CFG,
                      robot_footprint_radius=0.22, scan_timeout_sec=0.01)

    async def _timeout_scan(robot_id):
        return None
    ex._get_laser_scan = _timeout_scan

    result = _run(ex._check_occupancy("robot_3"))
    assert "error" in result
    assert "timeout" in result["error"]


# ---------------------------------------------------------------------------
# _build_occupancy_grid pure function
# ---------------------------------------------------------------------------

def test_build_occupancy_grid_all_clear():
    scan = _make_scan(n_rays=36, range_max=2.0, hit_at_ray=-1)  # no hits
    # All rays return range_max (out of range) → no occupied cells
    free, occ = _build_occupancy_grid(scan, 0.0, 0.0, 0.0, cell_size=0.5)
    assert len(occ) == 0
    assert len(free) > 0


def test_build_occupancy_grid_has_hit():
    scan = _make_scan(n_rays=360, range_max=5.0, hit_range=1.0, hit_at_ray=0)
    free, occ = _build_occupancy_grid(scan, 0.0, 0.0, 0.0, cell_size=0.5)
    # Ray 0 at angle_min=-pi hits at 1.0 m: roughly (-1, 0) → at least one occupied cell
    assert len(occ) >= 1


def test_build_occupancy_grid_robot_pose_applied():
    scan = _make_scan(n_rays=8, angle_min=0.0, angle_max=math.pi * 2,
                      range_max=3.0, hit_range=1.5, hit_at_ray=0)
    # Robot at (10, 10)
    free1, occ1 = _build_occupancy_grid(scan, 10.0, 10.0, 0.0, cell_size=0.5)
    free2, occ2 = _build_occupancy_grid(scan,  0.0,  0.0, 0.0, cell_size=0.5)
    # Both should produce the same count; cells are shifted by offset
    assert len(free1) == len(free2)
    # All occupied cell x-coords should be near 10 (not near 0)
    if occ1:
        assert all(abs(c[0] - 10.0) < 5.0 for c in occ1)
