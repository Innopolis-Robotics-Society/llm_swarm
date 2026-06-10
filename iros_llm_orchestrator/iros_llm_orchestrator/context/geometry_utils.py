"""Small deterministic geometry helpers for semantic planning tools."""

from __future__ import annotations

import math
from typing import Any


def euclidean_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Return d(a,b) = sqrt((ax-bx)^2 + (ay-by)^2)."""
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def distance_2d(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Alias for Euclidean distance in the map plane."""
    return euclidean_distance(a, b)


def squared_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Return squared Euclidean distance, useful for argmin."""
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return dx * dx + dy * dy


def centroid(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    """Return c = (1/n) * sum_i p_i, or None for an empty set."""
    if not points:
        return None
    return (
        sum(float(p[0]) for p in points) / len(points),
        sum(float(p[1]) for p in points) / len(points),
    )


def rms_spread(
    points: list[tuple[float, float]],
    center: tuple[float, float] | None = None,
) -> float:
    """Return sqrt((1/n) * sum_i ||p_i - c||^2)."""
    if not points:
        return 0.0
    c = center if center is not None else centroid(points)
    if c is None:
        return 0.0
    return math.sqrt(sum(squared_distance(p, c) for p in points) / len(points))


def max_radius(
    points: list[tuple[float, float]],
    center: tuple[float, float] | None = None,
) -> float:
    """Return max_i ||p_i - c||."""
    if not points:
        return 0.0
    c = center if center is not None else centroid(points)
    if c is None:
        return 0.0
    return max(euclidean_distance(p, c) for p in points)


def nearest_point_index(
    point: tuple[float, float],
    candidates: list[tuple[float, float]],
) -> tuple[int, float] | None:
    """Return (argmin_i, distance(point, candidates[i])) or None."""
    if not candidates:
        return None
    best_idx = 0
    best_sq = squared_distance(point, candidates[0])
    for idx, candidate in enumerate(candidates[1:], start=1):
        dist_sq = squared_distance(point, candidate)
        if dist_sq < best_sq:
            best_idx = idx
            best_sq = dist_sq
    return best_idx, math.sqrt(best_sq)


def point_to_segment_distance(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    """Distance from p to segment ab using clamped projection."""
    px, py = float(p[0]), float(p[1])
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 0.0:
        return euclidean_distance((px, py), (ax, ay))
    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = max(0.0, min(1.0, t))
    qx = ax + t * vx
    qy = ay + t * vy
    return euclidean_distance((px, py), (qx, qy))


def point_in_circle(
    p: tuple[float, float],
    center: tuple[float, float],
    radius: float,
) -> bool:
    return squared_distance(p, center) <= float(radius) * float(radius)


def point_in_rect(
    p: tuple[float, float],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> bool:
    """Return True when p is inside or on an axis-aligned rectangle."""
    x, y = float(p[0]), float(p[1])
    return float(min_x) <= x <= float(max_x) and float(min_y) <= y <= float(max_y)


def point_in_polygon(
    p: tuple[float, float],
    polygon: list[tuple[float, float]],
) -> bool:
    """Return True when p is inside or on the boundary of a polygon."""
    if len(polygon) < 3:
        return False
    x, y = float(p[0]), float(p[1])
    inside = False
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        if point_to_segment_distance((x, y), a, b) <= 1e-9:
            return True
        ax, ay = float(a[0]), float(a[1])
        bx, by = float(b[0]), float(b[1])
        if (ay > y) != (by > y):
            x_cross = (bx - ax) * (y - ay) / (by - ay) + ax
            if x < x_cross:
                inside = not inside
    return inside


def polygon_bbox(polygon: list[tuple[float, float]]) -> dict[str, float | None]:
    if not polygon:
        return {'min_x': None, 'max_x': None, 'min_y': None, 'max_y': None}
    xs = [float(p[0]) for p in polygon]
    ys = [float(p[1]) for p in polygon]
    return {
        'min_x': min(xs),
        'max_x': max(xs),
        'min_y': min(ys),
        'max_y': max(ys),
    }


def rect_polygon(
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
) -> list[tuple[float, float]]:
    """Return a counter-clockwise polygon for an axis-aligned rectangle."""
    return [
        (float(min_x), float(min_y)),
        (float(max_x), float(min_y)),
        (float(max_x), float(max_y)),
        (float(min_x), float(max_y)),
    ]


def point_to_polygon_boundary_distance(
    p: tuple[float, float],
    polygon: list[tuple[float, float]],
) -> float:
    """Shortest distance from a point to any polygon edge."""
    if len(polygon) < 2:
        return 0.0
    return min(
        point_to_segment_distance(p, polygon[i], polygon[(i + 1) % len(polygon)])
        for i in range(len(polygon))
    )


def min_pairwise_clearance(
    points: list[tuple[float, float]],
    radius_m: float,
) -> float | None:
    """Minimum free space between equal-radius robot discs."""
    if len(points) < 2:
        return None
    best: float | None = None
    diameter = 2.0 * float(radius_m)
    for i, point in enumerate(points):
        for other in points[i + 1:]:
            clearance = euclidean_distance(point, other) - diameter
            if best is None or clearance < best:
                best = clearance
    return best


def min_clearance_to_points(
    points: list[tuple[float, float]],
    obstacles: list[tuple[float, float]],
    point_radius_m: float,
    obstacle_radius_m: float,
) -> float | None:
    """Minimum free space between two sets of circular footprints."""
    if not points or not obstacles:
        return None
    best: float | None = None
    combined_radius = float(point_radius_m) + float(obstacle_radius_m)
    for point in points:
        for obstacle in obstacles:
            clearance = euclidean_distance(point, obstacle) - combined_radius
            if best is None or clearance < best:
                best = clearance
    return best


def rotate_point(offset: tuple[float, float], theta_rad: float) -> tuple[float, float]:
    """Return R(theta) @ offset."""
    c = math.cos(float(theta_rad))
    s = math.sin(float(theta_rad))
    ox, oy = float(offset[0]), float(offset[1])
    return c * ox - s * oy, s * ox + c * oy


def apply_pose(
    offset: tuple[float, float],
    origin: tuple[float, float],
    theta_rad: float,
) -> tuple[float, float]:
    """Return origin + R(theta) @ offset."""
    rx, ry = rotate_point(offset, theta_rad)
    return float(origin[0]) + rx, float(origin[1]) + ry


def formation_points_from_offsets(
    leader_xy: tuple[float, float],
    offsets_x: list[float],
    offsets_y: list[float],
    heading_rad: float,
) -> list[tuple[float, float]]:
    """Return [leader, follower...] world points for leader-frame offsets."""
    points = [(float(leader_xy[0]), float(leader_xy[1]))]
    for ox, oy in zip(offsets_x, offsets_y):
        points.append(apply_pose((float(ox), float(oy)), leader_xy, heading_rad))
    return points


def world_to_grid(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[int, int]:
    gx = math.floor((float(x) - float(origin_x)) / float(resolution))
    gy = math.floor((float(y) - float(origin_y)) / float(resolution))
    return int(gx), int(gy)


def grid_to_world(
    gx: int,
    gy: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[float, float]:
    x = float(origin_x) + (int(gx) + 0.5) * float(resolution)
    y = float(origin_y) + (int(gy) + 0.5) * float(resolution)
    return x, y


def coerce_point(value: Any) -> tuple[float, float] | None:
    """Internal convenience for callers that consume YAML-ish coordinates."""
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            return None
    return None
