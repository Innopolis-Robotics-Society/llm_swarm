"""Deterministic room-aware formation placement helpers.

This module is intentionally pure: it does not call ROS services, planners, or
controllers.  Callers provide static map config, an optional pose snapshot, and
tool arguments; the result is a JSON-serialisable feasibility report.
"""

from __future__ import annotations

import math
import re
from difflib import get_close_matches
from typing import Any

from iros_llm_orchestrator.context.geometry_utils import (
    apply_pose,
    coerce_point,
    euclidean_distance,
    formation_points_from_offsets,
    min_clearance_to_points,
    min_pairwise_clearance,
    point_in_circle,
    point_in_polygon,
    point_to_polygon_boundary_distance,
    polygon_bbox,
    rect_polygon,
)


DEFAULT_MIN_CLEARANCE_M = 0.35
DEFAULT_FOOTPRINT_RADIUS_M = 0.22
DEFAULT_ROOM_HALF_EXTENT_M = 3.0
DEFAULT_CANDIDATE_SPACING_M = 0.5
DEFAULT_LINE_SPACING_M = 1.5
DEFAULT_WEDGE_DEPTH_M = 1.0
DEFAULT_WEDGE_LATERAL_M = 0.6
MAX_CANDIDATES_PER_GROUP = 240


def find_group_placement_in_room(
    map_cfg: dict,
    args: dict,
    *,
    pose_snapshot: dict | None = None,
    robot_footprint_radius: float = DEFAULT_FOOTPRINT_RADIUS_M,
) -> dict:
    """Find non-overlapping formation placements for groups inside a room."""
    args = args or {}
    room_query = _safe_str(args.get('room') or args.get('location') or '').strip()
    groups = args.get('groups') or []
    warnings: list[str] = []
    if not isinstance(groups, list) or not groups:
        return _failure(
            room_query,
            'no_groups',
            ['groups'],
            ['provide at least one group with robot_ids and formation'],
            warnings,
        )

    min_clearance = _positive_float(
        args.get('min_clearance_m'),
        DEFAULT_MIN_CLEARANCE_M,
    )
    footprint_radius = _positive_float(
        args.get('footprint_radius_m'),
        robot_footprint_radius,
    )
    candidate_spacing = _positive_float(
        args.get('candidate_spacing_m'),
        DEFAULT_CANDIDATE_SPACING_M,
    )
    max_candidates = int(_positive_float(
        args.get('max_candidates_per_group'),
        MAX_CANDIDATES_PER_GROUP,
    ))
    max_candidates = max(1, min(max_candidates, 1000))
    avoid_existing = bool(args.get('avoid_existing_robots', True))

    room_boundary = resolve_room_boundary(map_cfg or {}, room_query, args=args)
    if room_boundary is None:
        known = _known_room_names(map_cfg or {})
        suggestions = get_close_matches(
            _norm(room_query),
            [_norm(name) for name in known],
            n=5,
            cutoff=0.55,
        )
        reverse = {_norm(name): name for name in known}
        return {
            'ok': False,
            'room': room_query,
            'reason': 'unknown_room',
            'failed_checks': ['room_resolved'],
            'known_rooms': known,
            'suggestions': [
                reverse.get(item, item) for item in suggestions
            ] or ['use a named location from the map context'],
            'warnings': warnings,
        }

    parsed_groups = _parse_groups(groups, args)
    if parsed_groups['errors']:
        return _failure(
            room_boundary['canonical'],
            'invalid_groups',
            parsed_groups['errors'],
            ['use robot_ids with at least one leader and one follower'],
            warnings,
            boundary=room_boundary,
        )
    requested_ids = {
        rid for group in parsed_groups['groups'] for rid in group['robot_ids']
    }
    existing_positions, pose_warnings = _existing_robot_positions(
        pose_snapshot or {},
        requested_ids=requested_ids,
        avoid_existing=avoid_existing,
    )
    warnings.extend(pose_warnings)
    if avoid_existing and pose_snapshot is None:
        warnings.append('no pose snapshot supplied; existing robots were not checked')

    bounds = _map_bounds(map_cfg or {})
    heading_candidates = _heading_candidates(args)
    all_candidates: list[list[dict]] = []
    failed_checks: set[str] = set()

    for group in parsed_groups['groups']:
        candidates = _placement_candidates_for_group(
            group,
            room_boundary,
            bounds,
            existing_positions,
            footprint_radius=footprint_radius,
            min_clearance=min_clearance,
            candidate_spacing=candidate_spacing,
            heading_candidates=heading_candidates,
            avoid_existing=avoid_existing,
            max_candidates=max_candidates,
        )
        if not candidates:
            failed_checks.update(group.get('_failed_checks') or [])
            failed_checks.add('inside_room')
        all_candidates.append(candidates)

    if any(not candidates for candidates in all_candidates):
        return _failure(
            room_boundary['canonical'],
            'not_enough_space_in_room',
            sorted(failed_checks) or ['candidate_generation'],
            _space_suggestions(),
            warnings,
            boundary=room_boundary,
        )

    selected = _select_non_overlapping(
        all_candidates,
        footprint_radius=footprint_radius,
        min_clearance=min_clearance,
    )
    if selected is None:
        return _failure(
            room_boundary['canonical'],
            'not_enough_space_in_room',
            ['avoids_other_requested_groups'],
            _space_suggestions(),
            warnings,
            boundary=room_boundary,
        )

    placements = _finalise_placements(
        selected,
        footprint_radius=footprint_radius,
        min_clearance=min_clearance,
    )
    return {
        'ok': True,
        'room': room_boundary['canonical'],
        'room_boundary_source': room_boundary['source'],
        'room_boundary': _json_boundary(room_boundary),
        'placements': placements,
        'warnings': warnings,
    }


def resolve_room_boundary(
    map_cfg: dict,
    room: str,
    *,
    args: dict | None = None,
) -> dict | None:
    """Resolve a named room/location to a deterministic boundary model."""
    args = args or {}
    query_norm = _norm(room)
    if not query_norm:
        return None

    geometry = map_cfg.get('geometry') or {}
    geom_name, geom_data = _lookup_named_mapping(geometry, query_norm)
    if isinstance(geom_data, dict):
        boundary = _boundary_from_geometry(geom_name, geom_data)
        if boundary is not None:
            return boundary

    named_name, center = _resolve_named_location(map_cfg, query_norm)
    zone = _resolve_formation_zone(map_cfg, query_norm, center)
    if center is None and zone is not None:
        center = coerce_point(zone.get('coords') or zone.get('center'))
        named_name = _safe_str(zone.get('name') or room)
    if center is None:
        return None

    half_extent = _fallback_half_extent(args, zone)
    min_x = center[0] - half_extent
    max_x = center[0] + half_extent
    min_y = center[1] - half_extent
    max_y = center[1] + half_extent
    bounds = _map_bounds(map_cfg)
    if bounds is not None:
        min_x = max(min_x, bounds['min_x'])
        max_x = min(max_x, bounds['max_x'])
        min_y = max(min_y, bounds['min_y'])
        max_y = min(max_y, bounds['max_y'])
    polygon = rect_polygon(min_x, max_x, min_y, max_y)
    canonical = named_name or _safe_str(room)
    return {
        'canonical': canonical,
        'type': 'rect',
        'source': 'fallback_center_box',
        'center': center,
        'half_extent_m': half_extent,
        'polygon': polygon,
        'bbox': {
            'min_x': min_x,
            'max_x': max_x,
            'min_y': min_y,
            'max_y': max_y,
        },
    }


def follower_offsets_for_formation(
    formation: str,
    follower_count: int,
    *,
    spacing_m: float | None = None,
) -> list[tuple[float, float]]:
    """Return leader-frame follower offsets for supported formation names."""
    n = max(0, int(follower_count))
    if n == 0:
        return []
    name = _norm(formation)
    line_spacing = _positive_float(spacing_m, DEFAULT_LINE_SPACING_M)

    if name == 'wedge':
        offsets: list[tuple[float, float]] = []
        template = [
            (-DEFAULT_WEDGE_DEPTH_M, DEFAULT_WEDGE_LATERAL_M),
            (-DEFAULT_WEDGE_DEPTH_M, -DEFAULT_WEDGE_LATERAL_M),
            (-2.0 * DEFAULT_WEDGE_DEPTH_M, 0.0),
        ]
        offsets.extend(template[:n])
        extra = n - len(offsets)
        layer = 2
        while extra > 0:
            offsets.append((
                -layer * DEFAULT_WEDGE_DEPTH_M,
                layer * DEFAULT_WEDGE_LATERAL_M,
            ))
            extra -= 1
            if extra <= 0:
                break
            offsets.append((
                -layer * DEFAULT_WEDGE_DEPTH_M,
                -layer * DEFAULT_WEDGE_LATERAL_M,
            ))
            extra -= 1
            layer += 1
        return offsets

    if name == 'line':
        return [(-line_spacing * (i + 1), 0.0) for i in range(n)]

    if name == 'column':
        offsets = []
        for i in range(n):
            distance = line_spacing * ((i // 2) + 1)
            side = 1.0 if i % 2 == 0 else -1.0
            offsets.append((0.0, side * distance))
        return offsets

    return []


def formation_world_points(
    leader_xy: tuple[float, float],
    offsets: list[tuple[float, float]],
    heading_rad: float,
) -> list[tuple[float, float]]:
    return formation_points_from_offsets(
        leader_xy,
        [point[0] for point in offsets],
        [point[1] for point in offsets],
        heading_rad,
    )


def _boundary_from_geometry(name: str, data: dict) -> dict | None:
    raw_polygon = data.get('polygon') or data.get('boundary')
    polygon = _coerce_polygon(raw_polygon)
    if polygon:
        bbox = polygon_bbox(polygon)
        center = _polygon_center(polygon)
        return {
            'canonical': name,
            'type': 'polygon',
            'source': 'map_yaml_polygon',
            'center': center,
            'polygon': polygon,
            'bbox': bbox,
        }

    corners = data.get('corners')
    if isinstance(corners, dict):
        corner_points = [
            coerce_point(corners.get(key))
            for key in ('bottom_left', 'bottom_right', 'top_right', 'top_left')
        ]
        if all(point is not None for point in corner_points):
            polygon = [point for point in corner_points if point is not None]
            bbox = polygon_bbox(polygon)
            return {
                'canonical': name,
                'type': 'rect',
                'source': 'map_yaml_rect',
                'center': coerce_point(data.get('center')) or _polygon_center(polygon),
                'polygon': polygon,
                'bbox': bbox,
            }

    bounds = data.get('bounds')
    if isinstance(bounds, dict):
        rect = _rect_from_bounds(bounds)
        if rect is not None:
            polygon = rect_polygon(
                rect['min_x'], rect['max_x'], rect['min_y'], rect['max_y'])
            return {
                'canonical': name,
                'type': 'rect',
                'source': 'map_yaml_rect',
                'center': coerce_point(data.get('center')) or _polygon_center(polygon),
                'polygon': polygon,
                'bbox': rect,
            }

    center = coerce_point(data.get('center'))
    width = _float_or_none(data.get('width_m') or data.get('width'))
    height = _float_or_none(data.get('height_m') or data.get('height'))
    if center is not None and width is not None and height is not None:
        min_x = center[0] - width / 2.0
        max_x = center[0] + width / 2.0
        min_y = center[1] - height / 2.0
        max_y = center[1] + height / 2.0
        polygon = rect_polygon(min_x, max_x, min_y, max_y)
        return {
            'canonical': name,
            'type': 'rect',
            'source': 'map_yaml_rect',
            'center': center,
            'polygon': polygon,
            'bbox': {
                'min_x': min_x,
                'max_x': max_x,
                'min_y': min_y,
                'max_y': max_y,
            },
        }
    return None


def _parse_groups(groups: list, args: dict) -> dict:
    parsed = []
    errors = []
    for idx, raw in enumerate(groups):
        if not isinstance(raw, dict):
            errors.append(f'groups[{idx}] must be an object')
            continue
        robot_ids = _int_list(raw.get('robot_ids') or [])
        formation = _safe_str(raw.get('formation') or '').strip()
        name = _safe_str(raw.get('name') or f'group_{idx + 1}').strip()
        if len(robot_ids) < 2:
            errors.append(f'{name}: at least 2 robot_ids required')
            continue
        offsets = follower_offsets_for_formation(
            formation,
            len(robot_ids) - 1,
            spacing_m=raw.get('spacing_m') or args.get('spacing_m'),
        )
        if not offsets:
            errors.append(f'{name}: unsupported formation {formation!r}')
            continue
        parsed.append({
            'name': name,
            'robot_ids': robot_ids,
            'leader_robot_id': robot_ids[0],
            'follower_robot_ids': robot_ids[1:],
            'formation': formation,
            'offsets': offsets,
            '_failed_checks': [],
        })
    return {'groups': parsed, 'errors': errors}


def _placement_candidates_for_group(
    group: dict,
    boundary: dict,
    bounds: dict | None,
    existing_positions: list[tuple[float, float]],
    *,
    footprint_radius: float,
    min_clearance: float,
    candidate_spacing: float,
    heading_candidates: list[float],
    avoid_existing: bool,
    max_candidates: int,
) -> list[dict]:
    candidates = []
    failed_checks: set[str] = set()
    for leader_xy in _candidate_leader_points(boundary, candidate_spacing):
        for heading in heading_candidates:
            candidate = _evaluate_candidate(
                group,
                leader_xy,
                heading,
                boundary,
                bounds,
                existing_positions,
                footprint_radius=footprint_radius,
                min_clearance=min_clearance,
                avoid_existing=avoid_existing,
            )
            if candidate['candidate_ok']:
                candidates.append(candidate)
            else:
                failed_checks.update(candidate['failed_checks'])
    group['_failed_checks'] = sorted(failed_checks)
    center = boundary.get('center') or (0.0, 0.0)
    candidates.sort(
        key=lambda item: (
            round(euclidean_distance(item['leader_xy'], center), 6),
            round(item['leader_xy'][1], 6),
            round(item['leader_xy'][0], 6),
            round(item['heading_rad'], 6),
        )
    )
    return candidates[:max_candidates]


def _evaluate_candidate(
    group: dict,
    leader_xy: tuple[float, float],
    heading: float,
    boundary: dict,
    bounds: dict | None,
    existing_positions: list[tuple[float, float]],
    *,
    footprint_radius: float,
    min_clearance: float,
    avoid_existing: bool,
) -> dict:
    points = formation_world_points(leader_xy, group['offsets'], heading)
    boundary_distance = _min_boundary_clearance(points, boundary)
    boundary_clearance = (
        boundary_distance - footprint_radius
        if boundary_distance is not None else None
    )
    inside_room = (
        boundary_clearance is not None
        and boundary_clearance >= min_clearance
    )
    inside_map_bounds = _points_inside_bounds(points, bounds, footprint_radius)
    pairwise_clearance = min_pairwise_clearance(points, footprint_radius)
    pairwise_ok = (
        pairwise_clearance is None
        or pairwise_clearance >= min_clearance
    )
    existing_clearance = min_clearance_to_points(
        points,
        existing_positions,
        footprint_radius,
        footprint_radius,
    )
    avoids_existing = (
        not avoid_existing
        or existing_clearance is None
        or existing_clearance >= min_clearance
    )
    failed = []
    if not inside_room:
        failed.append('inside_room')
    if not inside_map_bounds:
        failed.append('inside_map_bounds')
    if not pairwise_ok:
        failed.append('pairwise_clearance_ok')
    if not avoids_existing:
        failed.append('avoids_existing_robots')
    clearances = [
        value for value in (
            boundary_clearance,
            pairwise_clearance,
            existing_clearance,
        )
        if value is not None
    ]
    return {
        'candidate_ok': not failed,
        'failed_checks': failed,
        'group': group,
        'leader_xy': leader_xy,
        'heading_rad': float(heading),
        'points': points,
        'boundary_clearance_m': boundary_clearance,
        'pairwise_clearance_m': pairwise_clearance,
        'existing_clearance_m': existing_clearance,
        'clearance_min_m': min(clearances) if clearances else None,
        'checks': {
            'inside_room': inside_room,
            'inside_map_bounds': inside_map_bounds,
            'pairwise_clearance_ok': pairwise_ok,
            'avoids_existing_robots': avoids_existing,
        },
    }


def _select_non_overlapping(
    candidate_groups: list[list[dict]],
    *,
    footprint_radius: float,
    min_clearance: float,
) -> list[dict] | None:
    selected: list[dict] = []

    def _search(index: int) -> list[dict] | None:
        if index >= len(candidate_groups):
            return list(selected)
        for candidate in candidate_groups[index]:
            if _candidate_avoids_selected(
                candidate,
                selected,
                footprint_radius=footprint_radius,
                min_clearance=min_clearance,
            ):
                selected.append(candidate)
                result = _search(index + 1)
                if result is not None:
                    return result
                selected.pop()
        return None

    return _search(0)


def _candidate_avoids_selected(
    candidate: dict,
    selected: list[dict],
    *,
    footprint_radius: float,
    min_clearance: float,
) -> bool:
    for other in selected:
        clearance = min_clearance_to_points(
            candidate['points'],
            other['points'],
            footprint_radius,
            footprint_radius,
        )
        if clearance is not None and clearance < min_clearance:
            return False
    return True


def _finalise_placements(
    selected: list[dict],
    *,
    footprint_radius: float,
    min_clearance: float,
) -> list[dict]:
    placements = []
    for idx, candidate in enumerate(selected):
        group = candidate['group']
        other_points = [
            point
            for j, other in enumerate(selected)
            if j != idx
            for point in other['points']
        ]
        other_clearance = min_clearance_to_points(
            candidate['points'],
            other_points,
            footprint_radius,
            footprint_radius,
        )
        avoids_other = (
            other_clearance is None
            or other_clearance >= min_clearance
        )
        clearances = [
            value for value in (
                candidate.get('clearance_min_m'),
                other_clearance,
            )
            if value is not None
        ]
        clearance_min = min(clearances) if clearances else None
        checks = dict(candidate['checks'])
        checks['avoids_other_requested_groups'] = avoids_other
        follower_goals = {}
        for rid, point in zip(group['follower_robot_ids'], candidate['points'][1:]):
            follower_goals[str(rid)] = _round_point(point)
        offsets_x = [round(float(offset[0]), 3) for offset in group['offsets']]
        offsets_y = [round(float(offset[1]), 3) for offset in group['offsets']]
        placements.append({
            'group': group['name'],
            'leader_robot_id': group['leader_robot_id'],
            'leader_ns': f"robot_{group['leader_robot_id']}",
            'follower_ns': [
                f'robot_{rid}' for rid in group['follower_robot_ids']
            ],
            'leader_goal': _round_point(candidate['points'][0]),
            'heading_rad': round(candidate['heading_rad'], 3),
            'formation': group['formation'],
            'follower_goals': follower_goals,
            'offsets_x': offsets_x,
            'offsets_y': offsets_y,
            'mapf_robot_ids': group['robot_ids'],
            'mapf_goals': [_round_point(point) for point in candidate['points']],
            'footprint_radius_m': round(footprint_radius, 3),
            'clearance_min_m': (
                round(clearance_min, 3) if clearance_min is not None else None
            ),
            'checks': checks,
        })
    return placements


def _candidate_leader_points(boundary: dict, spacing: float) -> list[tuple[float, float]]:
    bbox = boundary.get('bbox') or polygon_bbox(boundary.get('polygon') or [])
    center = boundary.get('center') or _polygon_center(boundary.get('polygon') or [])
    if center is None:
        min_x = float(bbox.get('min_x') or 0.0)
        max_x = float(bbox.get('max_x') or 0.0)
        min_y = float(bbox.get('min_y') or 0.0)
        max_y = float(bbox.get('max_y') or 0.0)
        center = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
    xs = _axis_samples(
        float(bbox['min_x']),
        float(bbox['max_x']),
        center[0],
        spacing,
    )
    ys = _axis_samples(
        float(bbox['min_y']),
        float(bbox['max_y']),
        center[1],
        spacing,
    )
    points = [(x, y) for y in ys for x in xs]
    points.sort(
        key=lambda point: (
            round(euclidean_distance(point, center), 6),
            round(point[1], 6),
            round(point[0], 6),
        )
    )
    return points


def _axis_samples(min_value: float, max_value: float, center: float, spacing: float) -> list[float]:
    spacing = max(0.1, float(spacing))
    values = [center]
    step = 1
    while center - step * spacing >= min_value:
        values.append(center - step * spacing)
        step += 1
    step = 1
    while center + step * spacing <= max_value:
        values.append(center + step * spacing)
        step += 1
    return sorted({round(value, 6) for value in values})


def _min_boundary_clearance(
    points: list[tuple[float, float]],
    boundary: dict,
) -> float | None:
    polygon = boundary.get('polygon') or []
    if boundary.get('type') in {'polygon', 'rect'} and polygon:
        clearances = []
        for point in points:
            if not point_in_polygon(point, polygon):
                return -1.0
            clearances.append(point_to_polygon_boundary_distance(point, polygon))
        return min(clearances) if clearances else None
    if boundary.get('type') == 'circle':
        center = boundary.get('center')
        radius = boundary.get('radius')
        if center is None or radius is None:
            return None
        clearances = []
        for point in points:
            if not point_in_circle(point, center, radius):
                return -1.0
            clearances.append(float(radius) - euclidean_distance(point, center))
        return min(clearances) if clearances else None
    return None


def _points_inside_bounds(
    points: list[tuple[float, float]],
    bounds: dict | None,
    margin: float,
) -> bool:
    if bounds is None:
        return True
    for x, y in points:
        if x - margin < bounds['min_x'] or x + margin > bounds['max_x']:
            return False
        if y - margin < bounds['min_y'] or y + margin > bounds['max_y']:
            return False
    return True


def _existing_robot_positions(
    snapshot: dict,
    *,
    requested_ids: set[int],
    avoid_existing: bool,
) -> tuple[list[tuple[float, float]], list[str]]:
    if not avoid_existing:
        return [], []
    positions = []
    warnings = []
    for raw_id, pose in (snapshot or {}).items():
        rid = _int_or_none(raw_id)
        if rid is None:
            continue
        if rid in requested_ids:
            continue
        if not isinstance(pose, dict):
            continue
        if bool(pose.get('stale')):
            warnings.append(f'ignored stale pose for robot_{rid}')
            continue
        point = coerce_point([pose.get('x'), pose.get('y')])
        if point is None:
            continue
        positions.append(point)
    return positions, warnings


def _heading_candidates(args: dict) -> list[float]:
    raw = args.get('heading_candidates_rad')
    if isinstance(raw, list):
        values = []
        for item in raw:
            value = _float_or_none(item)
            if value is not None:
                values.append(float(value))
        if values:
            return values
    return [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]


def _resolve_named_location(
    map_cfg: dict,
    query_norm: str,
) -> tuple[str | None, tuple[float, float] | None]:
    named = map_cfg.get('named_locations') or {}
    aliases = map_cfg.get('location_aliases') or {}
    alias_target = None
    for alias, target in aliases.items():
        if _norm(alias) == query_norm:
            alias_target = _safe_str(target)
            break
    for name, value in named.items():
        if _norm(name) == query_norm or (alias_target and str(name) == alias_target):
            point = coerce_point(value)
            if point is not None:
                return _safe_str(name), point
    return None, None


def _resolve_formation_zone(
    map_cfg: dict,
    query_norm: str,
    center: tuple[float, float] | None,
) -> dict | None:
    for zone in map_cfg.get('formation_zones') or []:
        if not isinstance(zone, dict):
            continue
        if _norm(zone.get('name')) == query_norm:
            return zone
    if center is None:
        return None
    for zone in map_cfg.get('formation_zones') or []:
        zcenter = coerce_point(zone.get('coords') or zone.get('center'))
        radius = _float_or_none(zone.get('radius'))
        if zcenter is not None and radius is not None:
            if euclidean_distance(center, zcenter) <= max(0.25, radius):
                return zone
    return None


def _fallback_half_extent(args: dict, zone: dict | None) -> float:
    explicit = _float_or_none(args.get('room_half_extent_m'))
    if explicit is not None and explicit > 0.0:
        return explicit
    radius = _float_or_none((zone or {}).get('radius'))
    if radius is not None and radius > 0.0:
        return max(1.0, radius / math.sqrt(2.0))
    return DEFAULT_ROOM_HALF_EXTENT_M


def _lookup_named_mapping(mapping: dict, query_norm: str) -> tuple[str, Any]:
    for name, value in mapping.items():
        if _norm(name) == query_norm:
            return _safe_str(name), value
    return '', None


def _known_room_names(map_cfg: dict) -> list[str]:
    names = []
    for section in ('geometry', 'named_locations'):
        for name in (map_cfg.get(section) or {}).keys():
            names.append(_safe_str(name))
    for zone in map_cfg.get('formation_zones') or []:
        if isinstance(zone, dict) and zone.get('name'):
            names.append(_safe_str(zone.get('name')))
    return sorted(dict.fromkeys(name for name in names if name))


def _json_boundary(boundary: dict) -> dict:
    out = {
        'type': boundary.get('type'),
        'source': boundary.get('source'),
        'center': _round_point(boundary.get('center')),
    }
    if boundary.get('bbox'):
        out['bbox'] = {
            key: round(float(value), 3)
            for key, value in boundary['bbox'].items()
            if value is not None
        }
    if boundary.get('half_extent_m') is not None:
        out['half_extent_m'] = round(float(boundary['half_extent_m']), 3)
    if boundary.get('polygon'):
        out['polygon'] = [_round_point(point) for point in boundary['polygon']]
    return out


def _failure(
    room: str,
    reason: str,
    failed_checks: list[str],
    suggestions: list[str],
    warnings: list[str],
    *,
    boundary: dict | None = None,
) -> dict:
    result = {
        'ok': False,
        'room': room,
        'reason': reason,
        'failed_checks': failed_checks,
        'suggestions': suggestions,
        'warnings': warnings,
    }
    if boundary is not None:
        result['room_boundary_source'] = boundary.get('source')
        result['room_boundary'] = _json_boundary(boundary)
    return result


def _space_suggestions() -> list[str]:
    return [
        'try smaller formation',
        'move existing group out of the room',
        'use line formation instead of wedge',
    ]


def _map_bounds(map_cfg: dict) -> dict | None:
    bounds = map_cfg.get('bounds') or {}
    rect = _rect_from_bounds(bounds)
    return rect


def _rect_from_bounds(bounds: dict) -> dict | None:
    min_x = _float_or_none(bounds.get('min_x', bounds.get('x_min')))
    max_x = _float_or_none(bounds.get('max_x', bounds.get('x_max')))
    min_y = _float_or_none(bounds.get('min_y', bounds.get('y_min')))
    max_y = _float_or_none(bounds.get('max_y', bounds.get('y_max')))
    if None in (min_x, max_x, min_y, max_y):
        return None
    if max_x <= min_x or max_y <= min_y:
        return None
    return {
        'min_x': float(min_x),
        'max_x': float(max_x),
        'min_y': float(min_y),
        'max_y': float(max_y),
    }


def _coerce_polygon(value: Any) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        return []
    polygon = []
    for item in value:
        point = coerce_point(item)
        if point is None:
            return []
        polygon.append(point)
    return polygon if len(polygon) >= 3 else []


def _polygon_center(polygon: list[tuple[float, float]]) -> tuple[float, float] | None:
    if not polygon:
        return None
    return (
        sum(point[0] for point in polygon) / len(polygon),
        sum(point[1] for point in polygon) / len(polygon),
    )


def _round_point(point: Any) -> list[float] | None:
    xy = coerce_point(point)
    if xy is None:
        return None
    return [round(float(xy[0]), 3), round(float(xy[1]), 3)]


def _int_list(value: Any) -> list[int]:
    if not isinstance(value, list):
        return []
    out = []
    for item in value:
        parsed = _int_or_none(item)
        if parsed is not None:
            out.append(parsed)
    return out


def _int_or_none(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        text = value.strip()
        if text.startswith('robot_'):
            text = text[len('robot_'):]
        if text.isdigit():
            return int(text)
    return None


def _positive_float(value: Any, default: float) -> float:
    parsed = _float_or_none(value)
    if parsed is None or parsed <= 0.0:
        return float(default)
    return float(parsed)


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _safe_str(value: Any) -> str:
    return '' if value is None else str(value)


def _norm(value: Any) -> str:
    text = _safe_str(value).strip().lower()
    text = text.replace('_', ' ').replace('-', ' ')
    return re.sub(r'\s+', ' ', text)
