"""Server-side occupancy-aware rewrites for room MAPF goals.

This is a safety net for cases where the LLM emits the old style
``spread:true`` room-center MAPF plan instead of calling the placement tool.
It is intentionally pure and read-only: callers provide the parsed plan, map
config, and a pose snapshot; the helper returns a rewritten plan plus a short
rewrite report.
"""

from __future__ import annotations

import copy
import math
import re
from typing import Any

from iros_llm_orchestrator.context.geometry_utils import (
    coerce_point,
    euclidean_distance,
)
from iros_llm_orchestrator.context.group_placement import (
    find_free_group_goals_in_room,
)


DEFAULT_REWRITE_CLEARANCE_M = 0.45
DEFAULT_REWRITE_GOAL_SPACING_M = 0.75
DEFAULT_ROOM_MATCH_RADIUS_M = 1.5
RELATION_WORDS = (
    'around',
    'near',
    'next to',
    'beside',
    'close to',
    'рядом',
    'вокруг',
    'около',
    'возле',
)


def rewrite_occupied_room_mapf_goals(
    plan: dict,
    map_cfg: dict,
    *,
    pose_snapshot: dict | None = None,
    user_message: str = '',
    robot_footprint_radius: float = 0.22,
    min_clearance_m: float = DEFAULT_REWRITE_CLEARANCE_M,
    goal_spacing_m: float = DEFAULT_REWRITE_GOAL_SPACING_M,
) -> tuple[dict, list[dict]]:
    """Rewrite room-center MAPF leaves into exact occupancy-aware room goals.

    The helper handles both already-occupied rooms and sequence-local future
    occupancy. For a sequence like ``yellow to cafeteria, then orange to
    cafeteria``, the first rewritten MAPF goals are added to a virtual pose
    snapshot before the second step is processed.
    """
    if not isinstance(plan, dict):
        return plan, []
    map_cfg = map_cfg or {}
    base_snapshot = _usable_snapshot(pose_snapshot or {})
    virtual_snapshot = copy.deepcopy(base_snapshot)
    explicit_rooms = _explicit_rooms_in_text(user_message, map_cfg)
    mentioned_groups = _mentioned_groups(user_message, map_cfg)
    relation_requested = _has_relation_word(user_message)
    rewrites: list[dict] = []
    new_plan = copy.deepcopy(plan)

    def _visit(node: Any) -> None:
        if not isinstance(node, dict):
            return
        node_type = node.get('type')
        if node_type == 'sequence':
            for child in list(node.get('steps') or []):
                _visit(child)
            return
        if node_type == 'parallel':
            # Deterministic left-to-right processing makes later siblings avoid
            # earlier siblings' planned destinations too. That is conservative
            # for endpoint selection and still leaves path coordination to MAPF.
            for child in list(node.get('steps') or []):
                _visit(child)
            return
        if node_type != 'mapf':
            return

        robot_ids = _int_list(node.get('robot_ids') or [])
        goals = _goal_points(node.get('goals') or [])
        if not robot_ids or not goals:
            return

        rewrite_args = _rewrite_args_for_leaf(
            node,
            robot_ids,
            goals,
            map_cfg,
            virtual_snapshot,
            explicit_rooms=explicit_rooms,
            mentioned_groups=mentioned_groups,
            relation_requested=relation_requested,
            robot_footprint_radius=robot_footprint_radius,
            min_clearance_m=min_clearance_m,
            goal_spacing_m=goal_spacing_m,
        )
        if rewrite_args is not None:
            result = find_free_group_goals_in_room(
                map_cfg,
                rewrite_args,
                pose_snapshot=virtual_snapshot,
                robot_footprint_radius=robot_footprint_radius,
            )
            if result.get('ok') and len(result.get('goals') or []) == len(robot_ids):
                old_goals = [list(point) for point in goals]
                node['goals'] = result['goals']
                node.pop('spread', None)
                node['reason'] = (
                    f"{node.get('reason') or 'mapf'}; "
                    f"occupancy-aware room placement in {result.get('room')}"
                )
                rewrites.append({
                    'type': 'mapf_room_goal_rewrite',
                    'room': result.get('room'),
                    'robot_ids': robot_ids,
                    'old_goals': old_goals,
                    'new_goals': result.get('goals'),
                    'mode': rewrite_args.get('placement_mode', 'cluster'),
                    'room_boundary_source': result.get('room_boundary_source'),
                })

        _update_virtual_positions(virtual_snapshot, robot_ids, _goal_points(node.get('goals') or []))

    _visit(new_plan)
    return new_plan, rewrites


def _rewrite_args_for_leaf(
    node: dict,
    robot_ids: list[int],
    goals: list[tuple[float, float]],
    map_cfg: dict,
    snapshot: dict[int, dict],
    *,
    explicit_rooms: list[str],
    mentioned_groups: list[str],
    relation_requested: bool,
    robot_footprint_radius: float,
    min_clearance_m: float,
    goal_spacing_m: float,
) -> dict | None:
    groups = _robot_groups(map_cfg)
    moving_group = _group_for_robot_ids(robot_ids, groups)
    if relation_requested:
        target_group = _relation_target_group(
            moving_group,
            mentioned_groups,
            groups,
        )
        if target_group:
            target_ids = _int_list(groups[target_group].get('ids') or [])
            target_center = _centroid_for_robot_ids(snapshot, target_ids)
            room = (
                explicit_rooms[0]
                if len(explicit_rooms) == 1
                else _nearest_room_name(target_center, map_cfg, loose=True)
            )
            if room:
                return {
                    'room': room,
                    'robot_ids': robot_ids,
                    'avoid_robot_ids': target_ids,
                    'prefer_near_group': target_ids,
                    'placement_mode': 'around_group',
                    'avoid_existing_robots': True,
                    'min_clearance_m': min_clearance_m,
                    'goal_spacing_m': goal_spacing_m,
                    'reason': node.get('reason') or f'{moving_group or robot_ids} around {target_group}',
                }

    anchor = _goal_anchor(goals)
    room = explicit_rooms[0] if len(explicit_rooms) == 1 else None
    if room is None:
        room = _nearest_room_name(anchor, map_cfg, loose=False)
    if not room:
        return None

    old_style_room_goal = (
        bool(node.get('spread'))
        or (len(goals) == 1 and len(robot_ids) > 1)
    )
    conflicting = _goals_conflict_with_snapshot(
        goals,
        snapshot,
        requested_ids=set(robot_ids),
        robot_footprint_radius=robot_footprint_radius,
        min_clearance_m=min_clearance_m,
    )
    if not old_style_room_goal and not conflicting:
        return None

    return {
        'room': room,
        'robot_ids': robot_ids,
        'placement_mode': 'cluster',
        'avoid_existing_robots': True,
        'min_clearance_m': min_clearance_m,
        'goal_spacing_m': goal_spacing_m,
        'reason': node.get('reason') or f'occupancy-aware placement in {room}',
    }


def _usable_snapshot(snapshot: dict) -> dict[int, dict]:
    out: dict[int, dict] = {}
    for raw_id, pose in (snapshot or {}).items():
        rid = _int_or_none(raw_id)
        if rid is None or not isinstance(pose, dict):
            continue
        if pose.get('stale'):
            continue
        point = coerce_point([pose.get('x'), pose.get('y')])
        if point is None:
            continue
        out[rid] = {
            **pose,
            'x': float(point[0]),
            'y': float(point[1]),
            'stale': False,
        }
    return out


def _update_virtual_positions(
    snapshot: dict[int, dict],
    robot_ids: list[int],
    goals: list[tuple[float, float]],
) -> None:
    for rid, goal in zip(robot_ids, goals):
        prev = snapshot.get(rid, {})
        snapshot[rid] = {
            'x': float(goal[0]),
            'y': float(goal[1]),
            'yaw': float(prev.get('yaw', 0.0) or 0.0),
            'stale_ms': 0,
            'stale': False,
        }


def _goal_points(raw_goals: list) -> list[tuple[float, float]]:
    points = []
    for raw in raw_goals:
        point = coerce_point(raw)
        if point is not None:
            points.append(point)
    return points


def _goal_anchor(goals: list[tuple[float, float]]) -> tuple[float, float] | None:
    if not goals:
        return None
    return (
        sum(point[0] for point in goals) / len(goals),
        sum(point[1] for point in goals) / len(goals),
    )


def _goals_conflict_with_snapshot(
    goals: list[tuple[float, float]],
    snapshot: dict[int, dict],
    *,
    requested_ids: set[int],
    robot_footprint_radius: float,
    min_clearance_m: float,
) -> bool:
    threshold = 2.0 * robot_footprint_radius + min_clearance_m
    for raw_id, pose in snapshot.items():
        rid = _int_or_none(raw_id)
        if rid is None or rid in requested_ids:
            continue
        point = coerce_point([pose.get('x'), pose.get('y')])
        if point is None:
            continue
        for goal in goals:
            if euclidean_distance(goal, point) < threshold:
                return True
    return False


def _room_candidates(map_cfg: dict) -> list[dict]:
    candidates: list[dict] = []
    zone_radius: dict[str, float] = {}
    for zone in map_cfg.get('formation_zones') or []:
        if not isinstance(zone, dict):
            continue
        name = str(zone.get('name') or '').strip()
        point = coerce_point(zone.get('coords') or zone.get('center'))
        if not name or point is None:
            continue
        radius = _float_or_none(zone.get('radius'))
        if radius is not None:
            zone_radius[_norm(name)] = float(radius)
        candidates.append({
            'name': name,
            'point': point,
            'match_radius': max(DEFAULT_ROOM_MATCH_RADIUS_M, float(radius or 0.0)),
        })
    for name, raw in (map_cfg.get('named_locations') or {}).items():
        point = coerce_point(raw)
        if point is None:
            continue
        radius = zone_radius.get(_norm(name), DEFAULT_ROOM_MATCH_RADIUS_M)
        candidates.append({
            'name': str(name),
            'point': point,
            'match_radius': max(DEFAULT_ROOM_MATCH_RADIUS_M, radius),
        })
    for name, data in (map_cfg.get('geometry') or {}).items():
        if not isinstance(data, dict):
            continue
        point = coerce_point(data.get('center'))
        if point is None:
            continue
        candidates.append({
            'name': str(name),
            'point': point,
            'match_radius': DEFAULT_ROOM_MATCH_RADIUS_M,
        })
    deduped: dict[str, dict] = {}
    for candidate in candidates:
        deduped.setdefault(_norm(candidate['name']), candidate)
    return list(deduped.values())


def _nearest_room_name(
    point: tuple[float, float] | None,
    map_cfg: dict,
    *,
    loose: bool,
) -> str | None:
    if point is None:
        return None
    best: tuple[float, dict] | None = None
    for candidate in _room_candidates(map_cfg):
        dist = euclidean_distance(point, candidate['point'])
        if best is None or dist < best[0]:
            best = (dist, candidate)
    if best is None:
        return None
    dist, candidate = best
    radius = float(candidate.get('match_radius') or DEFAULT_ROOM_MATCH_RADIUS_M)
    if loose or dist <= radius + 0.75:
        return str(candidate['name'])
    return None


def _explicit_rooms_in_text(text: str, map_cfg: dict) -> list[str]:
    norm_text = _text_norm(text)
    if not norm_text:
        return []
    names: dict[str, str] = {}
    for candidate in _room_candidates(map_cfg):
        names[_norm(candidate['name'])] = str(candidate['name'])
    for alias, target in (map_cfg.get('location_aliases') or {}).items():
        target_s = str(target)
        if _norm(target_s) in names:
            names[_norm(alias)] = names[_norm(target_s)]
    found = []
    for term, canonical in sorted(names.items(), key=lambda item: -len(item[0])):
        if _contains_term(norm_text, term) and canonical not in found:
            found.append(canonical)
    return found


def _mentioned_groups(text: str, map_cfg: dict) -> list[str]:
    norm_text = _text_norm(text)
    groups = _robot_groups(map_cfg)
    found = []
    for name, group in groups.items():
        terms = [_norm(name)]
        terms.extend(_norm(alias) for alias in (group.get('aliases') or []))
        terms.extend(f'{term}s' for term in list(terms) if term)
        if any(_contains_term(norm_text, term) for term in terms if term):
            found.append(name)
    return found


def _has_relation_word(text: str) -> bool:
    norm_text = _text_norm(text)
    return any(_contains_term(norm_text, _norm(word)) for word in RELATION_WORDS)


def _relation_target_group(
    moving_group: str | None,
    mentioned_groups: list[str],
    groups: dict[str, dict],
) -> str | None:
    for group in mentioned_groups:
        if group in groups and group != moving_group:
            return group
    return None


def _group_for_robot_ids(robot_ids: list[int], groups: dict[str, dict]) -> str | None:
    ids = set(robot_ids)
    for name, group in groups.items():
        group_ids = set(_int_list(group.get('ids') or []))
        if ids and ids.issubset(group_ids):
            return name
    return None


def _robot_groups(map_cfg: dict) -> dict[str, dict]:
    return {
        str(name): value
        for name, value in (map_cfg.get('robot_groups') or {}).items()
        if isinstance(value, dict)
    }


def _centroid_for_robot_ids(
    snapshot: dict[int, dict],
    robot_ids: list[int],
) -> tuple[float, float] | None:
    points = []
    for rid in robot_ids:
        pose = snapshot.get(rid)
        if not isinstance(pose, dict):
            continue
        point = coerce_point([pose.get('x'), pose.get('y')])
        if point is not None:
            points.append(point)
    if not points:
        return None
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _int_list(value: Any) -> list[int]:
    if not isinstance(value, list):
        return []
    out = []
    for item in value:
        number = _int_or_none(item)
        if number is not None:
            out.append(number)
    return out


def _int_or_none(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        match = re.search(r'-?\d+', value)
        if match:
            try:
                return int(match.group(0))
            except ValueError:
                return None
    return None


def _float_or_none(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _text_norm(text: str) -> str:
    return re.sub(r'\s+', ' ', str(text or '').replace('_', ' ').lower()).strip()


def _norm(text: Any) -> str:
    return _text_norm(str(text or ''))


def _contains_term(norm_text: str, norm_term: str) -> bool:
    if not norm_text or not norm_term:
        return False
    pattern = r'(?<![\w])' + re.escape(norm_term) + r'(?![\w])'
    return re.search(pattern, norm_text, flags=re.IGNORECASE) is not None
