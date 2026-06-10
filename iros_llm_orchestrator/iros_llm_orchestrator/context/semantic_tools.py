"""High-level read-only semantic tools for /llm/chat.

The tools in this module intentionally sit above raw ROS/MCP inspection.
They summarize the map, group, route, formation, and recovery context that a
supervisor LLM needs for planning, without exposing any write/control surface.
"""

from __future__ import annotations

import asyncio
import heapq
import json
import math
import re
from difflib import get_close_matches
from typing import Any, Callable

from iros_llm_orchestrator.common import plan_executor
from iros_llm_orchestrator.context.provider import (
    BLOCKED_MCP_TOOLS,
    ChatContextConfig,
    DEFAULT_MCP_READ_TOOLS,
    safe_str,
    to_jsonable,
)
from iros_llm_orchestrator.context.geometry_utils import (
    apply_pose,
    centroid,
    coerce_point,
    euclidean_distance,
    max_radius,
    point_in_circle,
    rms_spread,
)
from iros_llm_orchestrator.context.execution_verification import (
    verify_plan_execution_state as _verify_plan_execution_state,
)
from iros_llm_orchestrator.context.group_placement import (
    find_group_placement_in_room as _find_group_placement_in_room,
)


DEFAULT_FORMATION_SPACING_M = 1.0
MIN_PAIRWISE_SPACING_FACTOR = 0.75
GROUP_SPREAD_OUT_THRESHOLD_M = 3.0
BOTTLENECK_PENALTY_M = 4.0
DOOR_PENALTY_M = 2.0
DEAD_END_PENALTY_M = 5.0
CONGESTION_PENALTY_M = 3.0


DEFAULT_SEMANTIC_READ_TOOLS = (
    'semantic_get_system_state',
    'semantic_get_group_state',
    'semantic_resolve_location',
    'semantic_get_route_context',
    'semantic_check_goal_feasibility',
    'semantic_check_formation_feasibility',
    'semantic_find_group_placement_in_room',
    'semantic_verify_plan_execution_state',
    'semantic_get_allowed_action_schema',
    'semantic_get_recovery_options',
)


SEMANTIC_TOOL_DESCRIPTIONS = {
    'semantic_get_system_state':
        'compact BT/action/formation/event/map status for the current turn',
    'semantic_get_group_state':
        'robot ids, live pose quality, center/spread, and nearest map areas',
    'semantic_resolve_location':
        'map location alias resolution, coordinates, and formation-zone info',
    'semantic_get_route_context':
        'high-level route distance, topology notes, bottlenecks, and staging hints',
    'semantic_check_goal_feasibility':
        'bounds/group/heuristic feasibility check for a navigation target',
    'semantic_check_formation_feasibility':
        'formation-zone and group-size check without calling formation services',
    'semantic_find_group_placement_in_room':
        'room-aware non-overlapping placement candidates for group formations',
    'semantic_verify_plan_execution_state':
        'post-execution formation/pose verification with repair recommendation',
    'semantic_get_allowed_action_schema':
        'the final plan leaf/container types accepted by PlanExecutor',
    'semantic_get_recovery_options':
        'safe wait/replan/abort choices for WARN/ERROR or failed execution events',
}


class SemanticToolProvider:
    """Read-only semantic context provider independent of MCP."""

    def __init__(
        self,
        node: Any,
        config: ChatContextConfig,
        pose_cache: Any | None = None,
        get_cached_context: Callable[[], dict] | None = None,
        get_obstacle_context: Callable[[], str] | None = None,
    ):
        self._node = node
        self.config = config
        self._pose_cache = pose_cache
        self._get_cached_context = get_cached_context or (lambda: {})
        self._get_obstacle_context = get_obstacle_context or (lambda: '')
        self._map_cfg = dict(config.map_config or {})
        self._map_name = str(config.map_name or self._map_cfg.get('name') or '')
        self._spatial_model = self._build_spatial_model()

    async def execute_tool(self, name: str, args: dict) -> Any:
        """Execute a semantic read-only tool by stable name."""
        args = args or {}
        if name == 'semantic_get_system_state':
            return self.semantic_get_system_state()
        if name == 'semantic_resolve_location':
            return self.semantic_resolve_location(args)
        if name == 'semantic_get_group_state':
            return self.semantic_get_group_state(args)
        if name == 'semantic_get_route_context':
            return self.semantic_get_route_context(args)
        if name == 'semantic_check_goal_feasibility':
            return self.semantic_check_goal_feasibility(args)
        if name == 'semantic_check_formation_feasibility':
            return self.semantic_check_formation_feasibility(args)
        if name == 'semantic_find_group_placement_in_room':
            return self.semantic_find_group_placement_in_room(args)
        if name == 'semantic_verify_plan_execution_state':
            return self.semantic_verify_plan_execution_state(args)
        if name == 'semantic_get_allowed_action_schema':
            return self.semantic_get_allowed_action_schema()
        if name == 'semantic_get_recovery_options':
            return self.semantic_get_recovery_options(args)
        raise ValueError(f'unknown semantic tool: {safe_str(name, 120)}')

    def semantic_get_system_state(self) -> dict:
        context = self._cached_context()
        warnings = list(context.get('warnings') or [])
        bt = _extract_bt_state(context) or {}
        if not bt:
            warnings.append('BT state unavailable from cached runtime context')
        mode = safe_str(bt.get('mode') or 'unknown', 80)
        action_status = safe_str(bt.get('action_status') or 'unknown', 80)
        active_action = safe_str(bt.get('active_action') or 'unknown', 120)
        last_error = safe_str(bt.get('last_error') or '', 240)
        active_robot_ids = _int_list(bt.get('robot_ids') or [])
        if not active_robot_ids:
            assignment = context.get('robot_assignment') or {}
            active_robot_ids = _int_list(assignment.get('active') or [])
        busy = _looks_busy(mode, action_status, active_action)
        recent_events = [
            safe_str(item, 240)
            for item in list(context.get('recent_events') or [])[-8:]
        ]
        formations = context.get('formations') or []
        formation_summary = _summarize_formations(formations)
        return {
            'busy': busy,
            'bt': {
                'mode': mode,
                'action_status': action_status,
                'active_action': active_action,
                'last_error': last_error,
            },
            'active_robot_ids': active_robot_ids,
            'recent_events': recent_events,
            'formation_summary': formation_summary,
            'map_name': self._map_cfg.get('name') or self._map_name or 'unknown',
            'warnings': warnings,
        }

    def semantic_resolve_location(self, args: dict) -> dict:
        query = safe_str(args.get('query') or args.get('location') or '', 200).strip()
        detail = self._resolve_location_detail(query)
        if detail is None:
            return {
                'query': query,
                'resolved': False,
                'known_locations': sorted(self._named_locations().keys()),
                'suggestions': self._location_suggestions(query),
                'warnings': [f'Unknown location: {query}'] if query else [
                    'No location query supplied'
                ],
                'calculation_summary': {},
            }
        canonical = detail['canonical']
        coords = detail['coords']
        zone = detail.get('zone')
        return {
            'query': query,
            'resolved': True,
            'canonical': canonical,
            'coords': [round(coords[0], 3), round(coords[1], 3)],
            'match_type': detail['match_type'],
            'aliases': self._aliases_for_location(canonical),
            'distance_to_known_locations': self._distances_to_known_locations(coords),
            'is_formation_zone': zone is not None,
            'formation_radius': (
                round(float(zone.get('radius', 0.0)), 3) if zone else None
            ),
            'notes': self._location_notes(canonical),
            'warnings': [],
            'calculation_summary': {},
        }

    def semantic_get_group_state(self, args: dict) -> dict:
        group_query = safe_str(args.get('group') or '', 120).strip()
        resolved = self._resolve_group(group_query)
        if resolved is None:
            return {
                'group': group_query,
                'resolved': False,
                'known_groups': sorted(self._robot_groups().keys()),
                'warnings': [f'Unknown robot group: {group_query}'],
                'calculation_summary': {},
            }
        group_name, group_cfg = resolved
        robot_ids = _int_list(group_cfg.get('ids') or [])
        snapshot = self._pose_snapshot()
        poses: dict[str, dict] = {}
        nearest_legacy: dict[str, str] = {}
        robot_locations: dict[str, dict] = {}
        warnings: list[str] = []
        known_xy: list[tuple[float, float]] = []
        missing_ids: list[int] = []
        stale_ms_values: list[int] = []
        stale_count = 0
        for rid in robot_ids:
            pose = _snapshot_get(snapshot, rid)
            if pose is None:
                warnings.append(f'No live pose for robot_{rid}')
                missing_ids.append(rid)
                continue
            stale = bool(pose.get('stale'))
            if stale:
                stale_count += 1
            stale_ms = int(pose.get('stale_ms') or 0)
            stale_ms_values.append(stale_ms)
            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            point = (x, y)
            known_xy.append(point)
            nearest = self._nearest_location(point)
            poses[str(rid)] = {
                'x': round(x, 3),
                'y': round(y, 3),
                'stale': stale,
            }
            robot_locations[str(rid)] = {
                'pose': [round(x, 3), round(y, 3)],
                'nearest': nearest['name'] if nearest else 'unknown',
                'distance_m': (
                    nearest['distance_m'] if nearest else None
                ),
                'stale': stale,
                'stale_ms': stale_ms,
            }
            nearest_legacy[str(rid)] = (
                nearest['name'] if nearest else 'unknown'
            )
        if not poses:
            warnings.append('No current poses for this group')
        if stale_count:
            warnings.append(f'{stale_count} robot pose(s) are stale')
        center = centroid(known_xy)
        spread_rms = (
            rms_spread(known_xy, center) if center is not None else None
        )
        spread_max = (
            max_radius(known_xy, center) if center is not None else None
        )
        group_nearest = self._nearest_location(center) if center else None
        runtime = self._cached_context()
        state_guess = self._group_state_guess(
            robot_ids,
            runtime,
            have_any_pose=bool(poses),
            missing_count=max(0, len(robot_ids) - len(poses)),
            stale_count=stale_count,
            spread_max=spread_max,
        )
        return {
            'group': group_name,
            'resolved': True,
            'robot_ids': robot_ids,
            'known_pose_count': len(poses),
            'missing_pose_count': max(0, len(robot_ids) - len(poses)),
            'missing_pose_ids': missing_ids,
            'home': _coord_list(group_cfg.get('home')),
            'aliases': self._aliases_for_group(group_name, group_cfg),
            'poses': poses,
            'nearest_locations': nearest_legacy,
            'robot_locations': robot_locations,
            'group_center': (
                [round(center[0], 3), round(center[1], 3)]
                if center is not None else None
            ),
            'nearest_location': group_nearest or {
                'name': 'unknown',
                'distance_m': None,
            },
            'spread': {
                'rms_m': round(spread_rms, 3) if spread_rms is not None else None,
                'max_radius_m': (
                    round(spread_max, 3) if spread_max is not None else None
                ),
            },
            'spread_m': round(spread_max, 3) if spread_max is not None else None,
            'pose_staleness': {
                'stale_count': stale_count,
                'max_stale_ms': max(stale_ms_values) if stale_ms_values else None,
            },
            'state_guess': state_guess,
            'warnings': warnings,
            'calculation_summary': {
                'centroid_formula': 'center = (1/n) * sum(robot_xy)',
                'spread_rms_formula': 'sqrt(mean(||robot_xy - center||^2))',
                'pose_count': len(known_xy),
            },
        }

    def semantic_get_route_context(self, args: dict) -> dict:
        group_query = safe_str(args.get('group') or '', 120).strip()
        target_query = safe_str(args.get('target') or '', 200).strip()
        group = self._resolve_group(group_query)
        target = self._resolve_location_detail(target_query)
        group_state = self.semantic_get_group_state({'group': group_query})
        target_resolved = target is not None
        target_name = target['canonical'] if target else target_query
        target_xy = target['coords'] if target else None
        center = _coords(group_state.get('group_center'))
        warnings = []
        if group is None:
            warnings.append(f'Unknown robot group: {group_query}')
        if not target_resolved:
            warnings.append(f'Unknown target location: {target_query}')
        if center is None:
            warnings.append('Group center unavailable because no live poses were known')
        start_nearest = self._nearest_location(center) if center else None
        start_area = start_nearest['name'] if start_nearest else 'unknown'
        distance = (
            euclidean_distance(center, target_xy)
            if center is not None and target_xy is not None else None
        )
        notes = self._heuristic_lines()
        bottlenecks = [
            line for line in notes
            if _line_has_any(line, ('bottleneck', 'narrow', 'corridor',
                                    'contention', 'deadlock', 'stall'))
        ][:6]
        dead_end = self._target_has_dead_end_hint(target_name)
        group_size = len(group[1].get('ids') or []) if group else 0
        corridorish = any(
            _line_has_any(line, ('corridor', 'narrow', 'bottleneck'))
            for line in notes
        )
        staging = bool(group_size > 4 or dead_end or (corridorish and group_size >= 4))
        if (group_state.get('spread') or {}).get('max_radius_m'):
            staging = staging or float(
                (group_state.get('spread') or {}).get('max_radius_m') or 0.0
            ) > GROUP_SPREAD_OUT_THRESHOLD_M
        obstacle_note = safe_str(self._get_obstacle_context(), 800)
        if obstacle_note and obstacle_note not in '\n'.join(notes):
            notes = [*notes[:5], obstacle_note]
        route = self._route_between_named_areas(
            start_area,
            target_name if target_resolved else '',
            center=center,
            target_xy=target_xy,
            straight_line_distance=distance,
        )
        if route['source'] != 'route_graph':
            warnings.append('Exact traversability was not checked; using geometry fallback')
        reason_bits = []
        if not target_resolved:
            reason_bits.append('target is not a known named location')
        if distance is not None:
            reason_bits.append(f'estimated straight-line distance {distance:.1f} m')
        if staging:
            reason_bits.append('staging is recommended by map/group heuristics')
        if not reason_bits:
            reason_bits.append('heuristic route context only; no route graph available')
        return {
            'group': group[0] if group else group_query,
            'target': target_name,
            'target_resolved': target_resolved,
            'start_area_guess': start_area or 'unknown',
            'target_area': target_name if target_resolved else 'unknown',
            'straight_line_distance_m': (
                round(distance, 3) if distance is not None else None
            ),
            'distance_estimate_m': (
                round(distance, 3) if distance is not None else None
            ),
            'route': route,
            'map_topology_notes': notes[:8],
            'likely_bottlenecks': bottlenecks,
            'dead_end_warning': dead_end,
            'staging_recommended': staging,
            'reason': '; '.join(reason_bits),
            'calculation_summary': {
                'formula': 'distance = sqrt((x2-x1)^2 + (y2-y1)^2)',
                'group_center': (
                    [round(center[0], 3), round(center[1], 3)]
                    if center is not None else None
                ),
                'target': (
                    [round(target_xy[0], 3), round(target_xy[1], 3)]
                    if target_xy is not None else None
                ),
            },
            'warnings': warnings,
        }

    def semantic_check_goal_feasibility(self, args: dict) -> dict:
        group_query = safe_str(args.get('group') or '', 120).strip()
        target_query = safe_str(args.get('target') or '', 200).strip()
        group = self._resolve_group(group_query)
        target = self._resolve_location_detail(target_query)
        warnings = ['Occupancy-grid collision check was not run']
        target_inside: bool | None = None
        target_xy = target['coords'] if target else None
        if target is not None:
            target_inside = self._inside_bounds_known(target_xy)
            if target_inside is False:
                warnings.append('Target coordinates are outside map bounds')
        else:
            warnings.append(f'Unknown target location: {target_query}')
        if group is None:
            warnings.append(f'Unknown robot group: {group_query}')
        group_size = len(group[1].get('ids') or []) if group else 0
        group_state = (
            self.semantic_get_group_state({'group': group_query})
            if group is not None else {}
        )
        center = _coords(group_state.get('group_center'))
        distance = (
            euclidean_distance(center, target_xy)
            if center is not None and target_xy is not None else None
        )
        if center is None and group is not None:
            warnings.append('Group center unavailable because no live poses were known')
        target_name = target['canonical'] if target else target_query
        zone = target.get('zone') if target else None
        capacity_check = 'unknown'
        if zone is not None:
            radius = float(zone.get('radius') or 0.0)
            capacity = self._formation_zone_capacity(radius)
            capacity_check = 'pass' if group_size <= capacity else 'fail'
            if capacity_check == 'fail':
                warnings.append(
                    f'Formation zone radius {radius:.1f} m may be crowded for '
                    f'{group_size} robots'
                )
        route_hints = self._route_risk_hints(target_name)
        route_has_bottleneck = bool(route_hints['bottlenecks'])
        dead_end = self._target_has_dead_end_hint(target_name)
        spread_max = (group_state.get('spread') or {}).get('max_radius_m')
        spread_high = (
            spread_max is not None
            and float(spread_max) > GROUP_SPREAD_OUT_THRESHOLD_M
        )
        checks = {
            'bounds': self._check_label(target_inside),
            'occupancy': 'not_checked',
            'clearance': 'not_checked',
            'capacity': capacity_check,
        }
        risk_score = 0
        if target is None:
            risk_score += 2
        if target_inside is False:
            risk_score += 2
        risk_score += 1  # occupancy is intentionally not checked here
        if route_has_bottleneck:
            risk_score += 1
        if spread_high:
            risk_score += 1
        if dead_end and group_size > 3:
            risk_score += 2
        if capacity_check == 'fail':
            risk_score += 2
        essential_missing = target is None or group is None
        risk = 'unknown' if essential_missing else self._risk_from_score(risk_score)
        likely_valid = bool(
            target is not None
            and group is not None
            and target_inside is not False
            and capacity_check != 'fail'
        )
        if target is None or group is None:
            recommendation = 'ask_clarification'
        elif target_inside is False:
            recommendation = 'hold'
        elif capacity_check == 'fail' or risk == 'high':
            recommendation = 'choose_staging'
        elif risk == 'medium':
            recommendation = 'choose_staging'
        else:
            recommendation = 'move_group'
        if dead_end and group_size > 1:
            warnings.append('Target has dead-end/spur hints; avoid unnecessary traffic')
        return {
            'target_resolved': target is not None,
            'target_inside_bounds': target_inside,
            'group_known': group is not None,
            'group_size': group_size,
            'distance_from_group_center_m': (
                round(distance, 3) if distance is not None else None
            ),
            'is_formation_zone': zone is not None,
            'likely_valid': likely_valid,
            'risk': risk,
            'risk_score': risk_score,
            'checks': checks,
            'recommendation': recommendation,
            'route_risk_hints': route_hints,
            'warnings': warnings,
            'calculation_summary': {
                'risk_formula': (
                    '+2 unresolved/outside/capacity-fail/dead-end-large, '
                    '+1 occupancy-not-checked/bottleneck/high-spread'
                ),
                'group_center': (
                    [round(center[0], 3), round(center[1], 3)]
                    if center is not None else None
                ),
                'target': (
                    [round(target_xy[0], 3), round(target_xy[1], 3)]
                    if target_xy is not None else None
                ),
            },
        }

    def semantic_check_formation_feasibility(self, args: dict) -> dict:
        group_query = safe_str(args.get('group') or '', 120).strip()
        formation = safe_str(args.get('formation') or '', 120).strip()
        location_query = safe_str(args.get('location') or args.get('target') or '', 200).strip()
        group = self._resolve_group(group_query)
        location = self._resolve_location_detail(location_query)
        warnings: list[str] = []
        group_size = len(group[1].get('ids') or []) if group else 0
        if group is None:
            warnings.append(f'Unknown robot group: {group_query}')
        if location is None:
            warnings.append(f'Unknown formation location: {location_query}')
        loc_name = location['canonical'] if location else location_query
        loc_xy = location['coords'] if location else None
        zone = location.get('zone') if location else None
        formations_anywhere = any(
            'formations work anywhere' in line.lower()
            for line in self._heuristic_lines()
        )
        spacing = self._formation_spacing(args)
        offsets = _formation_offsets(formation, group_size, spacing)
        if group_size and not offsets:
            warnings.append(f'Unsupported formation: {formation}')
        supported_formations = _supported_formations_for_zone(zone)
        formation_supported = (
            not supported_formations or _norm(formation) in supported_formations
        )
        if not formation_supported:
            warnings.append(
                f'Formation {formation} is not listed for zone {loc_name}'
            )
        if location and zone is None and not formations_anywhere:
            warnings.append('Location is not a configured formation zone')
        group_state = (
            self.semantic_get_group_state({'group': group_query})
            if group is not None else {}
        )
        center = _coords(group_state.get('group_center'))
        footprint_eval = self._best_formation_footprint(
            offsets,
            loc_xy,
            zone=zone,
        )
        footprint = footprint_eval['footprint']
        checks = footprint_eval['checks']
        need_move = True
        group_distance = None
        if center is not None and loc_xy is not None:
            group_distance = euclidean_distance(center, loc_xy)
            move_threshold = max(
                1.0,
                float((zone or {}).get('radius') or 0.0),
                float(footprint.get('radius_m') or 0.0),
            )
            need_move = group_distance > move_threshold
        if center is None and group is not None:
            warnings.append('Group center unavailable because no live poses were known')
        leader = None
        if group is not None:
            ids = _int_list(group[1].get('ids') or [])
            if ids:
                leader = f'robot_{ids[0]}'
        can_form = bool(
            group
            and location
            and offsets
            and formation_supported
            and (zone is not None or formations_anywhere)
            and checks['zone_fit'] != 'fail'
            and checks['bounds'] != 'fail'
            and checks['pairwise_spacing'] == 'pass'
        )
        recommended = loc_name if can_form else self._nearest_formation_zone(loc_xy)
        if checks['bounds'] == 'fail':
            warnings.append('At least one formation footprint point is outside map bounds')
        if checks['zone_fit'] == 'fail':
            warnings.append('Formation footprint does not fit inside the formation zone')
        if checks['pairwise_spacing'] == 'fail':
            warnings.append('Formation footprint violates minimum pairwise spacing')
        reason = self._formation_reason(can_form, checks, zone, formations_anywhere)
        return {
            'group': group[0] if group else group_query,
            'formation': formation,
            'location': loc_name,
            'can_form_here': can_form,
            'recommended_location': recommended,
            'need_move_before_formation': bool(need_move),
            'leader_suggestion': leader,
            'footprint': footprint,
            'checks': checks,
            'group_distance_to_location_m': (
                round(group_distance, 3) if group_distance is not None else None
            ),
            'warnings': warnings,
            'reason': reason,
            'calculation_summary': {
                'offsets': (
                    'line/column centered on origin; wedge leader at origin; '
                    'circle radius = spacing/(2*sin(pi/n))'
                ),
                'heading_candidates_rad': [0.0, round(math.pi / 2, 3),
                                           round(math.pi, 3),
                                           round(3 * math.pi / 2, 3)],
            },
        }

    def semantic_find_group_placement_in_room(self, args: dict) -> dict:
        return _find_group_placement_in_room(
            self._map_cfg,
            args,
            pose_snapshot=self._pose_snapshot(),
            robot_footprint_radius=0.22,
        )

    def semantic_verify_plan_execution_state(self, args: dict) -> dict:
        context = self._cached_context()
        return _verify_plan_execution_state(
            self._map_cfg,
            args,
            pose_snapshot=self._pose_snapshot(),
            formations_status=context.get('formations'),
            bt_state=_extract_bt_state(context),
            recent_events=list(context.get('recent_events') or []),
            tolerance_m=0.5,
        )

    def semantic_get_allowed_action_schema(self) -> dict:
        leaf_types = _ordered_supported(
            getattr(plan_executor, '_LEAF_TYPES', set()),
            ['mapf', 'formation', 'idle'],
        )
        container_types = _ordered_supported(
            getattr(plan_executor, '_CONTAINER_TYPES', set()),
            ['sequence', 'parallel'],
        )
        return {
            'leaf_types': leaf_types,
            'container_types': container_types,
            'notes': [
                'Do not output raw ROS service calls.',
                'Do not output MCP tools as final actions.',
                'Use sequence for move-then-form.',
            ],
        }

    def semantic_get_recovery_options(self, args: dict) -> dict:
        event = safe_str(args.get('event') or '', 300).strip()
        group = safe_str(args.get('group') or '', 120).strip()
        text = event.lower()
        warnings: list[str] = []
        if group and self._resolve_group(group) is None:
            warnings.append(f'Unknown robot group: {group}')
        if not event:
            warnings.append('No event text supplied; recovery bias is unknown')
        safe_options = [
            {
                'decision': 'wait',
                'when': 'WARN is transient, progress continues, or replans are still within normal map heuristics',
            },
            {
                'decision': 'replan',
                'when': 'robots are stalled/deadlocked, a corridor conflict persists, or a previous route became blocked',
            },
            {
                'decision': 'abort',
                'when': 'goal is unreachable/out of bounds, planner reports no valid agents/path, or repeated replans make no progress',
            },
        ]
        recommended = 'unknown'
        if _line_has_any(text, ('out of bounds', 'unreachable', 'no valid',
                                'pbs_failed', 'invalid', 'aborted')):
            recommended = 'abort'
        elif _line_has_any(text, ('stall', 'deadlock', 'deviation', 'blocked',
                                  'replan', 'timeout')):
            recommended = 'replan'
        elif _line_has_any(text, ('warn', 'degraded', 'planning', 'executing',
                                  'temporary')):
            recommended = 'wait'
        return {
            'event': event,
            'safe_options': safe_options,
            'recommended_bias': recommended,
            'warnings': warnings,
        }

    def _build_spatial_model(self) -> dict:
        named_locations: dict[str, dict] = {}
        raw_named = self._map_cfg.get('named_locations') or {}
        for name, value in raw_named.items():
            coords = _coords(value)
            if coords is None:
                continue
            canonical = str(name)
            named_locations[canonical] = {
                'coords': [coords[0], coords[1]],
                'aliases': self._location_aliases_from_config(canonical),
                'notes': self._location_notes_from_heuristics(canonical),
            }

        robot_groups: dict[str, dict] = {}
        for name, value in (self._map_cfg.get('robot_groups') or {}).items():
            if not isinstance(value, dict):
                continue
            cfg = dict(value or {})
            robot_groups[str(name)] = {
                'ids': _int_list(cfg.get('ids') or []),
                'home': _coord_list(cfg.get('home')),
                'aliases': self._aliases_for_group(str(name), cfg),
                'description': safe_str(cfg.get('description') or '', 240),
            }

        formation_zones: dict[str, dict] = {}
        for item in self._map_cfg.get('formation_zones') or []:
            if not isinstance(item, dict):
                continue
            name = safe_str(item.get('name') or '', 120).strip()
            coords = _coords(item.get('coords'))
            if not name or coords is None:
                continue
            formation_zones[name] = {
                'center': [coords[0], coords[1]],
                'radius': float(item.get('radius') or 0.0),
                'supported_formations': _string_list(
                    item.get('supported_formations')
                    or item.get('formations')
                    or item.get('supported')
                    or []
                ),
                'note': safe_str(item.get('note') or '', 240),
            }

        route_graph = self._extract_yaml_route_graph(named_locations)
        if route_graph is None:
            route_graph = self._geometric_fallback_graph(named_locations)

        return {
            'named_locations': named_locations,
            'robot_groups': robot_groups,
            'formation_zones': formation_zones,
            'map_bounds': self._normalized_bounds(),
            'navigation_heuristics': self._heuristic_lines(),
            'route_graph': route_graph,
        }

    def _location_aliases_from_config(self, canonical: str) -> list[str]:
        aliases = [canonical]
        pretty = canonical.replace('_', ' ')
        if pretty != canonical:
            aliases.append(pretty)
        for alias, target in (self._map_cfg.get('location_aliases') or {}).items():
            if str(target) == canonical:
                aliases.append(str(alias))
        return sorted(dict.fromkeys(aliases), key=lambda x: (x != canonical, x))

    def _location_notes_from_heuristics(self, canonical: str) -> list[str]:
        notes = []
        for line in self._heuristic_lines():
            if _heuristic_mentions(line, canonical):
                notes.append(line)
        return notes[:4]

    def _normalized_bounds(self) -> dict:
        bounds = self._map_cfg.get('bounds') or {}
        values = {
            'min_x': bounds.get('min_x', bounds.get('x_min')),
            'max_x': bounds.get('max_x', bounds.get('x_max')),
            'min_y': bounds.get('min_y', bounds.get('y_min')),
            'max_y': bounds.get('max_y', bounds.get('y_max')),
        }
        out = {}
        for key, value in values.items():
            try:
                out[key] = float(value)
            except (TypeError, ValueError):
                out[key] = None
        return out

    def _extract_yaml_route_graph(self, named_locations: dict[str, dict]) -> dict | None:
        raw = (
            self._map_cfg.get('route_graph')
            or self._map_cfg.get('navigation_graph')
            or self._map_cfg.get('graph')
        )
        if not isinstance(raw, dict):
            return None
        nodes: dict[str, list[float]] = {
            name: list(info['coords'])
            for name, info in named_locations.items()
            if _coords(info.get('coords')) is not None
        }
        raw_nodes = raw.get('nodes') or {}
        if isinstance(raw_nodes, dict):
            for name, value in raw_nodes.items():
                coords = _coords(value.get('coords') if isinstance(value, dict) else value)
                if coords is not None:
                    nodes[str(name)] = [coords[0], coords[1]]
        elif isinstance(raw_nodes, list):
            for item in raw_nodes:
                if not isinstance(item, dict):
                    continue
                name = safe_str(item.get('name') or item.get('id') or '', 120)
                coords = _coords(item.get('coords') or item.get('xy'))
                if name and coords is not None:
                    nodes[name] = [coords[0], coords[1]]

        edges = []
        for item in raw.get('edges') or []:
            edge = self._normalize_route_edge(item, nodes)
            if edge is not None:
                edges.append(edge)
        if not nodes or not edges:
            return None
        return {'source': 'yaml_route_graph', 'nodes': nodes, 'edges': edges}

    def _normalize_route_edge(
        self,
        item: Any,
        nodes: dict[str, list[float]],
    ) -> dict | None:
        if isinstance(item, dict):
            src = safe_str(item.get('from') or item.get('src') or item.get('a') or '', 120)
            dst = safe_str(item.get('to') or item.get('dst') or item.get('b') or '', 120)
            tags = _string_list(item.get('tags') or [])
            label = safe_str(item.get('label') or item.get('type') or '', 120)
            if label:
                tags.append(label)
            directed = bool(item.get('directed') or False)
            length = _float_or_none(
                item.get('length_m', item.get('length', item.get('cost')))
            )
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            src = safe_str(item[0], 120)
            dst = safe_str(item[1], 120)
            tags = []
            directed = False
            length = _float_or_none(item[2]) if len(item) >= 3 else None
        else:
            return None
        if not src or not dst or src not in nodes or dst not in nodes:
            return None
        if length is None:
            length = euclidean_distance(tuple(nodes[src]), tuple(nodes[dst]))
        return {
            'from': src,
            'to': dst,
            'length_m': round(float(length), 3),
            'tags': sorted(dict.fromkeys(tags)),
            'directed': directed,
        }

    def _geometric_fallback_graph(self, named_locations: dict[str, dict]) -> dict:
        nodes = {
            name: list(info['coords'])
            for name, info in named_locations.items()
            if _coords(info.get('coords')) is not None
        }
        edges = []
        seen: set[tuple[str, str]] = set()
        for name, coords in nodes.items():
            candidates = []
            for other, other_coords in nodes.items():
                if other == name:
                    continue
                candidates.append((
                    euclidean_distance(tuple(coords), tuple(other_coords)),
                    other,
                ))
            for length, other in sorted(candidates)[:3]:
                key = tuple(sorted((name, other)))
                if key in seen:
                    continue
                seen.add(key)
                edges.append({
                    'from': name,
                    'to': other,
                    'length_m': round(length, 3),
                    'tags': ['geometric_fallback'],
                    'directed': False,
                })
        return {'source': 'geometric_fallback', 'nodes': nodes, 'edges': edges}

    def _resolve_location_detail(self, query: str) -> dict | None:
        query = safe_str(query, 200).strip()
        if not query:
            return None
        named = self._spatial_model.get('named_locations') or {}

        if query in named:
            return self._location_detail(query, 'canonical')

        for canonical, info in named.items():
            aliases = [a for a in info.get('aliases') or [] if a != canonical]
            if query in aliases:
                return self._location_detail(canonical, 'alias')

        qnorm = _norm(query)
        for canonical, info in named.items():
            if qnorm == _norm(canonical):
                return self._location_detail(canonical, 'normalized')
            for alias in info.get('aliases') or []:
                if qnorm == _norm(alias):
                    match_type = 'alias' if alias != canonical else 'normalized'
                    return self._location_detail(canonical, match_type)

        partial_matches: set[str] = set()
        for canonical, info in named.items():
            candidates = [canonical, *(info.get('aliases') or [])]
            for candidate in candidates:
                cnorm = _norm(candidate)
                if qnorm and (qnorm in cnorm or cnorm in qnorm):
                    partial_matches.add(canonical)
        if len(partial_matches) == 1:
            return self._location_detail(next(iter(partial_matches)), 'partial')
        return None

    def _location_detail(self, canonical: str, match_type: str) -> dict | None:
        named = self._spatial_model.get('named_locations') or {}
        info = named.get(canonical)
        if not info:
            return None
        coords = _coords(info.get('coords'))
        if coords is None:
            return None
        zone = self._formation_zone_for(canonical, coords)
        return {
            'canonical': canonical,
            'coords': coords,
            'match_type': match_type,
            'zone': zone,
        }

    def _location_suggestions(self, query: str) -> list[str]:
        named = self._spatial_model.get('named_locations') or {}
        if not query:
            return sorted(named.keys())[:6]
        qnorm = _norm(query)
        lookup: dict[str, str] = {}
        for canonical, info in named.items():
            lookup[_norm(canonical)] = canonical
            for alias in info.get('aliases') or []:
                lookup[_norm(alias)] = canonical
        substring = [
            canonical for normed, canonical in lookup.items()
            if qnorm and (qnorm in normed or normed in qnorm)
        ]
        close = [
            lookup[item]
            for item in get_close_matches(qnorm, sorted(lookup.keys()), n=6, cutoff=0.65)
        ]
        return list(dict.fromkeys([*substring, *close]))[:6]

    def _distances_to_known_locations(
        self,
        point: tuple[float, float],
        *,
        limit: int = 6,
    ) -> list[dict]:
        items = []
        for name, coords in self._named_locations().items():
            loc = _coords(coords)
            if loc is None:
                continue
            items.append({
                'name': name,
                'distance_m': round(euclidean_distance(point, loc), 3),
            })
        return sorted(items, key=lambda item: (item['distance_m'], item['name']))[:limit]

    def _nearest_location(
        self,
        coords: tuple[float, float] | list[float] | None,
    ) -> dict | None:
        point = _coords(coords)
        if point is None:
            return None
        distances = self._distances_to_known_locations(point, limit=1)
        if not distances:
            return None
        return distances[0]

    def _route_between_named_areas(
        self,
        start_area: str,
        target_area: str,
        *,
        center: tuple[float, float] | None,
        target_xy: tuple[float, float] | None,
        straight_line_distance: float | None,
    ) -> dict:
        graph = self._spatial_model.get('route_graph') or {}
        if (
            graph.get('source') == 'yaml_route_graph'
            and start_area in (graph.get('nodes') or {})
            and target_area in (graph.get('nodes') or {})
        ):
            routed = self._dijkstra_route(graph, start_area, target_area)
            if routed is not None:
                start_coords = _coords((graph.get('nodes') or {}).get(start_area))
                start_offset = (
                    euclidean_distance(center, start_coords)
                    if center is not None and start_coords is not None else 0.0
                )
                total = routed['total_cost'] + start_offset
                return {
                    'source': 'route_graph',
                    'path': routed['path'],
                    'total_cost': round(total, 3),
                    'confidence': 'high',
                }
        path = [
            item for item in (start_area, target_area)
            if item and item != 'unknown'
        ]
        return {
            'source': 'straight_line_fallback',
            'path': path,
            'total_cost': (
                round(straight_line_distance, 3)
                if straight_line_distance is not None else None
            ),
            'confidence': 'low',
        }

    def _dijkstra_route(self, graph: dict, start: str, target: str) -> dict | None:
        adjacency: dict[str, list[tuple[str, float]]] = {}
        for edge in graph.get('edges') or []:
            src = edge.get('from')
            dst = edge.get('to')
            if not src or not dst:
                continue
            cost = self._edge_cost(edge)
            adjacency.setdefault(src, []).append((dst, cost))
            if not edge.get('directed'):
                adjacency.setdefault(dst, []).append((src, cost))
        queue = [(0.0, start, [start])]
        best = {start: 0.0}
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node == target:
                return {'path': path, 'total_cost': cost}
            if cost > best.get(node, float('inf')):
                continue
            for nxt, edge_cost in adjacency.get(node, []):
                new_cost = cost + edge_cost
                if new_cost < best.get(nxt, float('inf')):
                    best[nxt] = new_cost
                    heapq.heappush(queue, (new_cost, nxt, [*path, nxt]))
        return None

    def _edge_cost(self, edge: dict) -> float:
        tags = ' '.join(str(tag).lower() for tag in edge.get('tags') or [])
        cost = float(edge.get('length_m') or 0.0)
        if _line_has_any(tags, ('bottleneck', 'narrow', 'corridor')):
            cost += BOTTLENECK_PENALTY_M
        if 'door' in tags:
            cost += DOOR_PENALTY_M
        if _line_has_any(tags, ('dead_end', 'dead-end', 'dead end', 'spur')):
            cost += DEAD_END_PENALTY_M
        if _line_has_any(tags, ('congestion', 'contention', 'deadlock', 'stall')):
            cost += CONGESTION_PENALTY_M
        return cost

    def _route_risk_hints(self, target_name: str) -> dict:
        lines = self._heuristic_lines()
        bottlenecks = [
            line for line in lines
            if _line_has_any(line, ('bottleneck', 'narrow', 'corridor',
                                    'contention', 'deadlock', 'stall'))
        ][:4]
        target_notes = [
            line for line in lines
            if target_name and _heuristic_mentions(line, target_name)
        ][:3]
        return {
            'bottlenecks': bottlenecks,
            'target_notes': target_notes,
            'dead_end': self._target_has_dead_end_hint(target_name),
        }

    def _check_label(self, value: bool | None) -> str:
        if value is True:
            return 'pass'
        if value is False:
            return 'fail'
        return 'unknown'

    def _risk_from_score(self, score: int) -> str:
        if score <= 1:
            return 'low'
        if score <= 3:
            return 'medium'
        return 'high'

    def _formation_zone_capacity(self, radius_m: float) -> int:
        return max(1, int(float(radius_m) * 2.0))

    def _formation_spacing(self, args: dict) -> float:
        raw = (
            args.get('spacing_m')
            or self._map_cfg.get('formation_spacing_m')
            or DEFAULT_FORMATION_SPACING_M
        )
        try:
            return max(0.1, float(raw))
        except (TypeError, ValueError):
            return DEFAULT_FORMATION_SPACING_M

    def _best_formation_footprint(
        self,
        offsets: list[tuple[float, float]],
        location: tuple[float, float] | None,
        *,
        zone: dict | None,
    ) -> dict:
        empty = {
            'footprint': {
                'spacing_m': DEFAULT_FORMATION_SPACING_M,
                'heading_rad': 0.0,
                'radius_m': 0.0,
                'points': [],
                'fit_score': 0.0,
            },
            'checks': {
                'zone_fit': 'unknown',
                'bounds': 'unknown',
                'pairwise_spacing': 'fail' if offsets else 'pass',
                'occupancy': 'not_checked',
            },
        }
        if location is None or not offsets:
            return empty
        best: dict | None = None
        for heading in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2):
            points = [apply_pose(offset, location, heading) for offset in offsets]
            evaluated = self._evaluate_formation_points(
                points,
                offsets,
                heading,
                zone=zone,
            )
            if best is None or evaluated['footprint']['fit_score'] > best['footprint']['fit_score']:
                best = evaluated
        return best or empty

    def _evaluate_formation_points(
        self,
        points: list[tuple[float, float]],
        offsets: list[tuple[float, float]],
        heading: float,
        *,
        zone: dict | None,
    ) -> dict:
        zone_center = _coords((zone or {}).get('coords'))
        zone_radius = float((zone or {}).get('radius') or 0.0)
        if zone_center is None and zone is not None:
            zone_center = _coords((zone or {}).get('center'))
        if zone_center is not None and zone_radius > 0.0:
            zone_passes = [
                point_in_circle(point, zone_center, zone_radius)
                for point in points
            ]
            zone_fit = 'pass' if all(zone_passes) else 'fail'
            zone_ratio = sum(1 for ok in zone_passes if ok) / len(zone_passes)
        else:
            zone_fit = 'unknown'
            zone_ratio = 1.0

        bounds_values = [self._inside_bounds_known(point) for point in points]
        if any(value is False for value in bounds_values):
            bounds = 'fail'
        elif all(value is True for value in bounds_values):
            bounds = 'pass'
        else:
            bounds = 'unknown'
        known_bounds = [value for value in bounds_values if value is not None]
        bounds_ratio = (
            sum(1 for ok in known_bounds if ok) / len(known_bounds)
            if known_bounds else 1.0
        )

        min_pairwise = _min_pairwise_distance(points)
        spacing = _offset_spacing_estimate(offsets)
        min_allowed = spacing * MIN_PAIRWISE_SPACING_FACTOR
        spacing_ok = min_pairwise is None or min_pairwise >= min_allowed
        pairwise = 'pass' if spacing_ok else 'fail'
        spacing_score = 1.0 if spacing_ok else max(0.0, min_pairwise or 0.0) / min_allowed
        radius = max_radius(offsets, (0.0, 0.0))
        fit_score = max(0.0, min(1.0, (zone_ratio + bounds_ratio + spacing_score) / 3.0))
        return {
            'footprint': {
                'spacing_m': round(spacing, 3),
                'heading_rad': round(float(heading), 3),
                'radius_m': round(radius, 3),
                'points': [[round(x, 3), round(y, 3)] for x, y in points],
                'fit_score': round(fit_score, 3),
            },
            'checks': {
                'zone_fit': zone_fit,
                'bounds': bounds,
                'pairwise_spacing': pairwise,
                'occupancy': 'not_checked',
            },
        }

    def _formation_reason(
        self,
        can_form: bool,
        checks: dict,
        zone: dict | None,
        formations_anywhere: bool,
    ) -> str:
        if can_form:
            if zone is not None:
                return 'formation footprint fits the configured formation zone'
            if formations_anywhere:
                return 'map heuristics allow formations anywhere and footprint checks pass'
        if checks.get('zone_fit') == 'fail':
            return 'formation footprint exceeds the configured formation zone'
        if checks.get('bounds') == 'fail':
            return 'formation footprint leaves map bounds'
        if checks.get('pairwise_spacing') == 'fail':
            return 'formation footprint violates pairwise spacing'
        return 'formation should use a configured formation zone'

    def _cached_context(self) -> dict:
        try:
            context = self._get_cached_context()
        except Exception as exc:
            return {
                'source': 'semantic_cached_context_error',
                'warnings': [f'cached context unavailable: {safe_str(exc, 200)}'],
            }
        return context if isinstance(context, dict) else {}

    def _named_locations(self) -> dict[str, list[float]]:
        out = {}
        for name, value in (self._map_cfg.get('named_locations') or {}).items():
            coord = _coords(value)
            if coord is not None:
                out[str(name)] = [coord[0], coord[1]]
        return out

    def _robot_groups(self) -> dict[str, dict]:
        return {
            str(name): dict(value or {})
            for name, value in (self._map_cfg.get('robot_groups') or {}).items()
            if isinstance(value, dict)
        }

    def _resolve_location(self, query: str) -> tuple[str, tuple[float, float]] | None:
        detail = self._resolve_location_detail(query)
        if detail is None:
            return None
        return detail['canonical'], detail['coords']

    def _resolve_group(self, query: str) -> tuple[str, dict] | None:
        if not query:
            return None
        groups = self._robot_groups()
        aliases: dict[str, str] = {}
        for name, cfg in groups.items():
            aliases[_norm(name)] = name
            color = cfg.get('color')
            if color:
                aliases[_norm(color)] = name
            for alias in cfg.get('aliases') or []:
                aliases[_norm(alias)] = name
        group_name = aliases.get(_norm(query))
        if group_name is None:
            return None
        return group_name, groups[group_name]

    def _formation_zone_for(
        self,
        canonical: str,
        coords: tuple[float, float] | list[float] | None,
    ) -> dict | None:
        zones = self._map_cfg.get('formation_zones') or []
        canonical_norm = _norm(canonical)
        for zone in zones:
            if _norm(zone.get('name')) == canonical_norm:
                return zone
        point = _coords(coords)
        if point is None:
            return None
        for zone in zones:
            zcoords = _coords(zone.get('coords'))
            radius = float(zone.get('radius') or 0.0)
            if zcoords is not None and radius > 0.0 and _dist(point, zcoords) <= radius:
                return zone
        return None

    def _nearest_formation_zone(
        self,
        coords: tuple[float, float] | list[float] | None,
    ) -> str | None:
        zones = self._map_cfg.get('formation_zones') or []
        if not zones:
            return None
        point = _coords(coords)
        if point is None:
            return safe_str(zones[0].get('name') or '', 120) or None
        best_name = None
        best_dist = None
        for zone in zones:
            zcoords = _coords(zone.get('coords'))
            if zcoords is None:
                continue
            d = _dist(point, zcoords)
            if best_dist is None or d < best_dist:
                best_dist = d
                best_name = safe_str(zone.get('name') or '', 120)
        return best_name

    def _aliases_for_location(self, canonical: str) -> list[str]:
        aliases = []
        for alias, target in (self._map_cfg.get('location_aliases') or {}).items():
            if str(target) == canonical:
                aliases.append(str(alias))
        if canonical not in aliases:
            aliases.insert(0, canonical)
        return sorted(dict.fromkeys(aliases), key=lambda x: (x != canonical, x))

    def _aliases_for_group(self, group_name: str, group_cfg: dict) -> list[str]:
        aliases = [group_name]
        color = group_cfg.get('color')
        if color:
            aliases.append(str(color))
        aliases.extend(str(alias) for alias in group_cfg.get('aliases') or [])
        return sorted(dict.fromkeys(aliases), key=lambda x: (x != group_name, x))

    def _location_notes(self, canonical: str) -> list[str]:
        notes = []
        for line in self._heuristic_lines():
            if _heuristic_mentions(line, canonical):
                notes.append(line)
        return notes[:4]

    def _heuristic_lines(self) -> list[str]:
        text = safe_str(self._map_cfg.get('heuristics') or '', 4000)
        lines = []
        for raw in text.splitlines():
            line = raw.strip()
            line = re.sub(r'^[-*]\s*', '', line)
            if line:
                lines.append(line)
        return lines

    def _pose_snapshot(self) -> dict:
        if self._pose_cache is None:
            return {}
        try:
            return self._pose_cache.snapshot(
                stale_threshold_ms=int(self.config.pose_stale_ms))
        except Exception:
            return {}

    def _nearest_location_name(
        self,
        coords: tuple[float, float] | list[float] | None,
    ) -> str | None:
        nearest = self._nearest_location(coords)
        return nearest['name'] if nearest else None

    def _group_state_guess(
        self,
        robot_ids: list[int],
        runtime: dict,
        *,
        have_any_pose: bool,
        missing_count: int,
        stale_count: int,
        spread_max: float | None,
    ) -> str:
        if not have_any_pose:
            return 'unknown'
        if missing_count or stale_count:
            return 'partially_unknown'
        if spread_max is not None and spread_max > GROUP_SPREAD_OUT_THRESHOLD_M:
            return 'spread_out'
        bt = _extract_bt_state(runtime) or {}
        mode = str(bt.get('mode') or '').lower()
        active = set(_int_list(bt.get('robot_ids') or []))
        if active.intersection(robot_ids) and mode in {'mapf', 'formation'}:
            return 'moving'
        if mode == 'idle':
            return 'idle'
        return 'unknown'

    def _inside_bounds(self, coords: tuple[float, float]) -> bool:
        return self._inside_bounds_known(coords) is not False

    def _inside_bounds_known(
        self,
        coords: tuple[float, float] | list[float] | None,
    ) -> bool | None:
        point = _coords(coords)
        if point is None:
            return None
        bounds = self._spatial_model.get('map_bounds') or self._normalized_bounds()
        min_x = bounds.get('min_x')
        max_x = bounds.get('max_x')
        min_y = bounds.get('min_y')
        max_y = bounds.get('max_y')
        if None in (min_x, max_x, min_y, max_y):
            return None
        x, y = point
        return float(min_x) <= x <= float(max_x) and float(min_y) <= y <= float(max_y)

    def _target_has_dead_end_hint(self, canonical: str) -> bool:
        for line in self._heuristic_lines():
            if _line_has_any(line, ('dead-end', 'dead end', 'spur')) and (
                _heuristic_mentions(line, canonical)
            ):
                return True
        return False


class CombinedReadOnlyToolBroker:
    """Dispatch semantic tools and optional MCP tools through one safe broker."""

    def __init__(
        self,
        *,
        semantic_provider: SemanticToolProvider | None,
        semantic_allowlist: list[str] | tuple[str, ...],
        mcp_broker: Any | None = None,
        max_tools_per_round: int,
        tool_timeout_sec: float,
        max_result_chars: int,
        logger: Any | None = None,
    ):
        self.semantic_provider = semantic_provider
        self.mcp_broker = mcp_broker
        self.max_tools_per_round = max(0, int(max_tools_per_round))
        self.tool_timeout_sec = max(0.1, float(tool_timeout_sec))
        self.max_result_chars = max(200, int(max_result_chars))
        self.logger = logger
        self.semantic_allowlist = self._safe_semantic_allowlist(semantic_allowlist)
        allowed = list(self.semantic_allowlist)
        if mcp_broker is not None:
            for name in list(getattr(mcp_broker, 'allowed_tools', []) or []):
                if name not in allowed:
                    allowed.append(name)
        self.allowed_tools = allowed

    async def execute_tool_request(self, request: dict) -> dict:
        tools = request.get('tools')
        if not isinstance(tools, list):
            tools = []
        if len(tools) > self.max_tools_per_round:
            self._warn(
                f'agentic read-only requested {len(tools)} tools; '
                f'limiting to {self.max_tools_per_round}')
        selected = tools[:self.max_tools_per_round]
        names = [
            safe_str(t.get('name'), 120) if isinstance(t, dict) else '<invalid>'
            for t in selected
        ]
        self._info(f'agentic read-only requested tools: {names}')
        results = []
        for item in selected:
            results.append(await self._execute_one(item))
        ok_names = [
            result.get('name')
            for result in results
            if result.get('status') == 'ok'
        ]
        if ok_names:
            self._info(f'agentic read-only successful tools: {ok_names}')
        return {'mode': 'tool_result', 'results': results}

    async def _execute_one(self, item: Any) -> dict:
        if not isinstance(item, dict):
            return {
                'name': '<invalid>',
                'status': 'rejected',
                'error': 'tool entry must be an object',
            }
        name = safe_str(item.get('name'), 120).strip()
        raw_args = item.get('args') or {}
        if not name:
            return {'name': '<invalid>', 'status': 'rejected',
                    'error': 'tool name is required'}
        if name in BLOCKED_MCP_TOOLS:
            self._warn(f'agentic read-only rejected blocked tool {name}')
            return {'name': name, 'status': 'rejected',
                    'error': 'tool is blocked because it can write/control ROS'}
        if name.startswith('semantic_'):
            return await self._execute_semantic(name, raw_args)
        if name in DEFAULT_MCP_READ_TOOLS:
            if self.mcp_broker is None:
                return {'name': name, 'status': 'rejected',
                        'error': 'MCP tools are not enabled'}
            delegated = await self.mcp_broker.execute_tool_request({'tools': [item]})
            results = delegated.get('results') or []
            if results:
                return results[0]
            return {'name': name, 'status': 'error',
                    'error': 'MCP broker returned no result'}
        return {
            'name': name,
            'status': 'rejected',
            'error': 'tool is not in the read-only semantic or MCP allowlist',
        }

    async def _execute_semantic(self, name: str, raw_args: Any) -> dict:
        if name not in DEFAULT_SEMANTIC_READ_TOOLS:
            return {'name': name, 'status': 'rejected',
                    'error': 'semantic tool is not known'}
        if name not in self.semantic_allowlist:
            return {'name': name, 'status': 'rejected',
                    'error': 'semantic tool is not enabled by semantic_tool_allowlist'}
        if self.semantic_provider is None:
            return {'name': name, 'status': 'rejected',
                    'error': 'semantic tools are not enabled'}
        if not isinstance(raw_args, dict):
            return {'name': name, 'status': 'rejected',
                    'error': 'tool args must be an object'}
        args = to_jsonable(raw_args)
        ok, reason = _semantic_args_are_safe(args)
        if not ok:
            return {'name': name, 'args': args, 'status': 'rejected',
                    'error': reason}
        try:
            raw = await asyncio.wait_for(
                self.semantic_provider.execute_tool(name, args),
                timeout=self.tool_timeout_sec,
            )
            return {
                'name': name,
                'args': args,
                'status': 'ok',
                'result': self._bound_result(raw),
            }
        except asyncio.TimeoutError:
            return {
                'name': name,
                'args': args,
                'status': 'timeout',
                'error': f'tool timed out after {self.tool_timeout_sec:.2f}s',
            }
        except Exception as exc:
            detail = f'{type(exc).__name__}: {safe_str(exc, 500)}'
            self._warn(f'agentic read-only semantic tool error: {name}: {detail}')
            return {'name': name, 'args': args, 'status': 'error',
                    'error': detail}

    def _bound_result(self, value: Any) -> Any:
        data = to_jsonable(value)
        text = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        if len(text) <= self.max_result_chars:
            return data
        preview_len = max(0, self.max_result_chars - 120)
        return {
            'truncated': True,
            'max_chars': self.max_result_chars,
            'preview': text[:preview_len],
        }

    @staticmethod
    def _safe_semantic_allowlist(raw_tools: list[str] | tuple[str, ...]) -> list[str]:
        allowed = []
        for tool in raw_tools or []:
            name = safe_str(tool, 120).strip()
            if name in DEFAULT_SEMANTIC_READ_TOOLS and name not in allowed:
                allowed.append(name)
        return allowed

    def _info(self, message: str):
        _log(self.logger, 'info', message)

    def _warn(self, message: str):
        _log(self.logger, 'warn', message)


def summarize_tool_result_for_trace(result: dict, *, max_chars: int = 800) -> dict:
    """Small JSONL-safe summary of a tool result for future SFT traces."""
    summary = {
        'name': safe_str(result.get('name'), 120),
        'status': safe_str(result.get('status'), 40),
    }
    if result.get('error'):
        summary['error'] = safe_str(result.get('error'), 240)
    if result.get('status') == 'ok':
        text = json.dumps(
            to_jsonable(result.get('result')),
            ensure_ascii=False,
            separators=(',', ':'),
        )
        summary['result_preview'] = safe_str(text, max_chars)
    return summary


def _extract_bt_state(context: dict) -> dict | None:
    bt = context.get('bt_state')
    if isinstance(bt, dict):
        return bt
    return _find_bt_like(context)


def _find_bt_like(value: Any) -> dict | None:
    if isinstance(value, dict):
        keys = set(value)
        if {'mode', 'action_status', 'active_action'}.issubset(keys):
            return value
        for item in value.values():
            found = _find_bt_like(item)
            if found is not None:
                return found
    if isinstance(value, list):
        for item in value[:40]:
            found = _find_bt_like(item)
            if found is not None:
                return found
    return None


def _summarize_formations(formations: Any) -> dict:
    if not isinstance(formations, list) or not formations:
        return {}
    statuses: dict[str, int] = {}
    active_ids = []
    for item in formations[:20]:
        if not isinstance(item, dict):
            continue
        status = safe_str(item.get('status') or item.get('state') or 'unknown', 80)
        statuses[status] = statuses.get(status, 0) + 1
        fid = safe_str(item.get('formation_id') or '', 120)
        if fid:
            active_ids.append(fid)
    return {
        'count': sum(statuses.values()),
        'statuses': statuses,
        'formation_ids': active_ids[:10],
    }


def _looks_busy(mode: str, action_status: str, active_action: str) -> bool:
    mode_l = mode.lower()
    status_l = action_status.lower()
    action_l = active_action.lower()
    if mode_l in {'mapf', 'formation'}:
        return True
    if action_l and action_l not in {'none', 'unknown'}:
        return True
    return status_l in {'warn', 'error', 'running', 'active', 'executing'}


def _coords(value: Any) -> tuple[float, float] | None:
    return coerce_point(value)


def _coord_list(value: Any) -> list[float] | None:
    coord = _coords(value)
    if coord is None:
        return None
    return [round(coord[0], 3), round(coord[1], 3)]


def _int_list(value: Any) -> list[int]:
    out = []
    for item in list(value or []):
        try:
            out.append(int(item))
        except (TypeError, ValueError):
            continue
    return out


def _snapshot_get(snapshot: dict, rid: int) -> dict | None:
    if rid in snapshot and isinstance(snapshot[rid], dict):
        return snapshot[rid]
    key = str(rid)
    if key in snapshot and isinstance(snapshot[key], dict):
        return snapshot[key]
    return None


def _mean_xy(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    if not points:
        return None
    return (
        sum(p[0] for p in points) / len(points),
        sum(p[1] for p in points) / len(points),
    )


def _spread_m(
    points: list[tuple[float, float]],
    center: tuple[float, float] | None,
) -> float | None:
    if center is None or not points:
        return None
    return max(_dist(point, center) for point in points)


def _dist(
    a: tuple[float, float] | list[float] | None,
    b: tuple[float, float] | list[float] | None,
) -> float:
    ca = _coords(a)
    cb = _coords(b)
    if ca is None or cb is None:
        return float('nan')
    return euclidean_distance(ca, cb)


def _float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _string_list(value: Any) -> list[str]:
    if isinstance(value, str):
        return [value]
    if not isinstance(value, (list, tuple, set)):
        return []
    return [safe_str(item, 120) for item in value if safe_str(item, 120)]


def _formation_offsets(
    formation: str,
    n: int,
    spacing: float,
) -> list[tuple[float, float]]:
    n = max(0, int(n))
    spacing = max(0.1, float(spacing))
    name = _norm(formation)
    if n == 0:
        return []
    if name == 'line':
        return [((i - (n - 1) / 2.0) * spacing, 0.0) for i in range(n)]
    if name == 'column':
        return [(0.0, (i - (n - 1) / 2.0) * spacing) for i in range(n)]
    if name == 'wedge':
        offsets = [(0.0, 0.0)]
        for i in range(1, n):
            depth = math.ceil(i / 2.0)
            side = -1.0 if i % 2 else 1.0
            offsets.append((-depth * spacing, side * depth * spacing * 0.7))
        return offsets
    if name == 'circle':
        if n == 1:
            return [(0.0, 0.0)]
        radius = spacing / (2.0 * math.sin(math.pi / n))
        return [
            (
                radius * math.cos(2.0 * math.pi * i / n),
                radius * math.sin(2.0 * math.pi * i / n),
            )
            for i in range(n)
        ]
    return []


def _supported_formations_for_zone(zone: dict | None) -> list[str]:
    if not isinstance(zone, dict):
        return []
    raw = (
        zone.get('supported_formations')
        or zone.get('formations')
        or zone.get('supported')
        or []
    )
    return [_norm(item) for item in _string_list(raw)]


def _min_pairwise_distance(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 2:
        return None
    best = None
    for i, point in enumerate(points):
        for other in points[i + 1:]:
            dist = euclidean_distance(point, other)
            if best is None or dist < best:
                best = dist
    return best


def _offset_spacing_estimate(offsets: list[tuple[float, float]]) -> float:
    if len(offsets) < 2:
        return DEFAULT_FORMATION_SPACING_M
    nearest = []
    for i, point in enumerate(offsets):
        best = None
        for j, other in enumerate(offsets):
            if i == j:
                continue
            dist = euclidean_distance(point, other)
            if best is None or dist < best:
                best = dist
        if best is not None:
            nearest.append(best)
    if not nearest:
        return DEFAULT_FORMATION_SPACING_M
    return sum(nearest) / len(nearest)


def _norm(value: Any) -> str:
    text = safe_str(value, 200).strip().lower()
    text = text.replace('_', ' ').replace('-', ' ')
    return re.sub(r'\s+', ' ', text)


def _line_has_any(text: str, needles: tuple[str, ...]) -> bool:
    lower = safe_str(text, 2000).lower()
    return any(needle in lower for needle in needles)


def _heuristic_mentions(line: str, canonical: str) -> bool:
    line_norm = _norm(line)
    canonical_norm = _norm(canonical)
    if canonical_norm and canonical_norm in line_norm:
        return True
    return canonical_norm.replace(' ', '') in line_norm.replace(' ', '')


def _ordered_supported(supported: set[str], preferred: list[str]) -> list[str]:
    supported = set(supported or [])
    ordered = [name for name in preferred if name in supported]
    extras = sorted(supported - set(ordered))
    return ordered + extras


def _semantic_args_are_safe(args: dict) -> tuple[bool, str]:
    try:
        text = json.dumps(args, ensure_ascii=False, separators=(',', ':'))
    except (TypeError, ValueError):
        return False, 'tool args must be JSON serializable'
    if len(text) > 2000:
        return False, 'tool args are too large'
    if _contains_unsafe_string(args):
        return False, 'tool args contain unsafe control characters'
    return True, ''


def _contains_unsafe_string(value: Any) -> bool:
    if isinstance(value, str):
        return '\x00' in value or len(value) > 500
    if isinstance(value, list):
        return any(_contains_unsafe_string(item) for item in value[:50])
    if isinstance(value, dict):
        return any(
            _contains_unsafe_string(k) or _contains_unsafe_string(v)
            for k, v in list(value.items())[:50]
        )
    return False


def _log(logger: Any | None, level: str, message: str):
    if logger is None:
        return
    method = getattr(logger, level, None)
    if method is None and level == 'warn':
        method = getattr(logger, 'warning', None)
    if method is not None:
        method(message)
