"""Deterministic post-execution verification for LLM plans.

The verifier is pure and read-only.  Runtime callers provide whatever state is
available: formation monitor status, BT state, recent event strings, and robot
poses.  Missing state produces an honest partial result instead of guessing.
"""

from __future__ import annotations

import math
import re
from typing import Any

from iros_llm_orchestrator.context.geometry_utils import (
    apply_pose,
    centroid,
    point_in_polygon,
)
from iros_llm_orchestrator.context.group_placement import resolve_room_boundary
from iros_llm_orchestrator.context.pose_cache import RobotPoseCache


FORMATION_STATES = {
    0: 'INACTIVE',
    1: 'FORMING',
    2: 'STABLE',
    3: 'DEGRADED',
    4: 'BROKEN',
}

ACTIVE_FORMATION_STATES = {'FORMING', 'STABLE', 'DEGRADED'}
ERROR_HINTS = (
    'registered but not activated',
    'not activated',
    'out of position',
    'too far from target',
    'formation setup failed',
    'follower stuck',
    'follower lost',
    'leader lost',
    'broken',
)


def verify_plan_execution_state(
    map_cfg: dict,
    args: dict,
    *,
    pose_snapshot: dict | None = None,
    formations_status: Any | None = None,
    bt_state: Any | None = None,
    recent_events: list[Any] | None = None,
    tolerance_m: float = 0.5,
) -> dict:
    """Verify whether the requested formation outcome currently holds."""
    args = args or {}
    last_plan = args.get('last_plan') or args.get('plan') or {}
    expected = args.get('expected') or {}
    original_request = _safe_str(args.get('original_user_request') or '')
    tolerance = _positive_float(
        args.get('tolerance_m') or args.get('formation_tolerance_m'),
        tolerance_m,
    )

    formation_expectations = _expected_formations(expected, last_plan)
    recent_errors = _recent_errors(
        bt_state,
        recent_events or [],
        args.get('last_failure'),
    )
    missing_state: list[str] = []
    checks: dict[str, Any] = {}

    if not formation_expectations:
        checks['formations_active'] = {
            'ok': True,
            'details': 'no formation leaves or expected formations to verify',
        }
        return {
            'ok': True,
            'confidence': 'full',
            'summary': 'no post-execution formation expectations to verify',
            'checks': checks,
            'recent_errors': recent_errors,
            'repair_recommendation': {
                'type': 'none',
                'reason': 'nothing repairable was detected',
                'repairable': False,
                'should_recompute_placement': False,
            },
        }

    normalized_status = normalize_formations_status(formations_status)
    if formations_status is None:
        missing_state.append('/formations/status')

    by_id = {
        item['formation_id']: item
        for item in normalized_status
        if item.get('formation_id')
    }

    active_check = _check_formations_active(formation_expectations, by_id)
    checks['formations_active'] = active_check

    follower_check = _check_followers_within_tolerance(
        formation_expectations,
        by_id,
        pose_snapshot or {},
        tolerance,
    )
    checks['followers_within_tolerance'] = follower_check

    room = _safe_str(expected.get('room') or args.get('room') or '').strip()
    if room:
        checks['robots_in_target_room'] = _check_robots_in_room(
            map_cfg,
            room,
            expected.get('groups') or [],
            pose_snapshot or {},
        )

    if follower_check.get('missing_state'):
        missing_state.extend(follower_check['missing_state'])
    if active_check.get('missing_state'):
        missing_state.extend(active_check['missing_state'])
    missing_state = sorted(dict.fromkeys(missing_state))

    hard_checks_ok = (
        bool(active_check.get('ok'))
        and bool(follower_check.get('ok'))
        and bool((checks.get('robots_in_target_room') or {'ok': True}).get('ok'))
    )
    ok = hard_checks_ok and not _recent_error_is_hard_failure(recent_errors)
    confidence = 'partial' if missing_state else 'full'
    if missing_state and not ok:
        confidence = 'partial'

    recommendation = _repair_recommendation(
        ok=ok,
        active_check=active_check,
        follower_check=follower_check,
        recent_errors=recent_errors,
        room=room,
        group_count=len(expected.get('groups') or []),
        missing_state=missing_state,
    )
    summary = _summary(
        ok=ok,
        expectations=formation_expectations,
        active_check=active_check,
        follower_check=follower_check,
        recent_errors=recent_errors,
        missing_state=missing_state,
        original_request=original_request,
    )
    if recent_errors:
        checks['recent_errors'] = recent_errors
    return {
        'ok': ok,
        'confidence': confidence,
        'summary': summary,
        'missing_state': missing_state,
        'checks': checks,
        'repair_recommendation': recommendation,
    }


def normalize_formations_status(value: Any) -> list[dict]:
    """Normalize /formations/status from ROS msg, MCP JSON, or test dicts."""
    if value is None:
        return []
    if isinstance(value, dict):
        if isinstance(value.get('formations'), list):
            return [_normalize_formation_status(item) for item in value['formations']]
        if value.get('formation_id'):
            return [_normalize_formation_status(value)]
        # MCP may wrap subscribe_once values in a keyed dict.
        out: list[dict] = []
        for item in value.values():
            out.extend(normalize_formations_status(item))
        return out
    if isinstance(value, list):
        out: list[dict] = []
        for item in value:
            out.extend(normalize_formations_status(item))
        return out
    formations = getattr(value, 'formations', None)
    if formations is not None:
        return [_normalize_formation_status(item) for item in list(formations)]
    if getattr(value, 'formation_id', None):
        return [_normalize_formation_status(value)]
    return []


def _normalize_formation_status(item: Any) -> dict:
    if isinstance(item, dict):
        state = item.get('state', item.get('formation_state', ''))
        follower_errors = item.get('follower_errors_m') or []
        return {
            'formation_id': _safe_str(item.get('formation_id') or item.get('id')),
            'leader_ns': _safe_str(item.get('leader_ns')),
            'follower_ns': [_safe_str(v) for v in list(item.get('follower_ns') or [])],
            'state': _state_name(state),
            'failure_code': item.get('failure_code', item.get('formation_failure_code')),
            'failure_reason': _safe_str(
                item.get('failure_reason') or item.get('formation_failure_reason')
            ),
            'follower_errors_m': [_float_or_none(v) for v in list(follower_errors)],
            'max_error_m': _float_or_none(item.get('max_error_m')),
            'mean_error_m': _float_or_none(item.get('mean_error_m')),
        }
    return {
        'formation_id': _safe_str(getattr(item, 'formation_id', '')),
        'leader_ns': _safe_str(getattr(item, 'leader_ns', '')),
        'follower_ns': [
            _safe_str(v) for v in list(getattr(item, 'follower_ns', []) or [])
        ],
        'state': _state_name(getattr(item, 'state', '')),
        'failure_code': getattr(item, 'failure_code', None),
        'failure_reason': _safe_str(getattr(item, 'failure_reason', '')),
        'follower_errors_m': [
            _float_or_none(v)
            for v in list(getattr(item, 'follower_errors_m', []) or [])
        ],
        'max_error_m': _float_or_none(getattr(item, 'max_error_m', None)),
        'mean_error_m': _float_or_none(getattr(item, 'mean_error_m', None)),
    }


def _expected_formations(expected: dict, last_plan: dict) -> list[dict]:
    plan_formations = _collect_formation_leaves(last_plan)
    by_id = {
        _safe_str(item.get('formation_id')): dict(item)
        for item in plan_formations
        if item.get('formation_id')
    }
    result: list[dict] = []
    for group in list(expected.get('groups') or []):
        if not isinstance(group, dict):
            continue
        name = _safe_str(group.get('name') or '').strip()
        formation = _safe_str(group.get('formation') or '').strip()
        fid = _safe_str(group.get('formation_id') or '').strip()
        if not fid and name and formation:
            fid = f'{name}_{formation}'
        robot_ids = _int_list(group.get('robot_ids') or [])
        plan_leaf = by_id.get(fid, {})
        result.append({
            'formation_id': fid,
            'group': name,
            'formation': formation,
            'robot_ids': robot_ids,
            'leader_ns': plan_leaf.get('leader_ns') or (
                f'robot_{robot_ids[0]}' if robot_ids else ''
            ),
            'follower_ns': list(plan_leaf.get('follower_ns') or [
                f'robot_{rid}' for rid in robot_ids[1:]
            ]),
            'offsets_x': list(plan_leaf.get('offsets_x') or []),
            'offsets_y': list(plan_leaf.get('offsets_y') or []),
        })
    for leaf in plan_formations:
        fid = _safe_str(leaf.get('formation_id'))
        if fid and fid not in {item['formation_id'] for item in result}:
            result.append({
                'formation_id': fid,
                'group': '',
                'formation': _formation_name_from_id(fid),
                'robot_ids': [],
                'leader_ns': leaf.get('leader_ns', ''),
                'follower_ns': list(leaf.get('follower_ns') or []),
                'offsets_x': list(leaf.get('offsets_x') or []),
                'offsets_y': list(leaf.get('offsets_y') or []),
            })
    return [item for item in result if item.get('formation_id')]


def _collect_formation_leaves(node: Any) -> list[dict]:
    if not isinstance(node, dict):
        return []
    if node.get('type') == 'formation':
        return [dict(node)]
    leaves = []
    for child in list(node.get('steps') or []):
        leaves.extend(_collect_formation_leaves(child))
    return leaves


def _check_formations_active(expectations: list[dict], by_id: dict) -> dict:
    missing = []
    inactive = []
    states = {}
    for expected in expectations:
        fid = expected['formation_id']
        status = by_id.get(fid)
        if status is None:
            missing.append(fid)
            continue
        state = _state_name(status.get('state'))
        states[fid] = state
        if state not in ACTIVE_FORMATION_STATES:
            inactive.append({'formation_id': fid, 'state': state})
    ok = not missing and not inactive
    result = {'ok': ok, 'states': states}
    if missing:
        result['missing'] = missing
        result['missing_state'] = ['/formations/status']
    if inactive:
        result['inactive'] = inactive
    return result


def _check_followers_within_tolerance(
    expectations: list[dict],
    by_id: dict,
    pose_snapshot: dict,
    tolerance: float,
) -> dict:
    failed = []
    missing = []
    checked = 0
    for expected in expectations:
        fid = expected['formation_id']
        status = by_id.get(fid)
        if status is not None and status.get('follower_errors_m'):
            follower_ns = status.get('follower_ns') or expected.get('follower_ns') or []
            for i, error in enumerate(status.get('follower_errors_m') or []):
                robot = follower_ns[i] if i < len(follower_ns) else f'follower_{i}'
                if error is None or error < 0.0:
                    missing.append({'formation_id': fid, 'robot': robot})
                    continue
                checked += 1
                if error > tolerance:
                    failed.append({
                        'formation_id': fid,
                        'robot': robot,
                        'distance_to_expected_m': round(float(error), 3),
                        'tolerance_m': round(tolerance, 3),
                    })
            continue
        pose_result = _follower_errors_from_poses(expected, pose_snapshot, tolerance)
        checked += pose_result['checked']
        failed.extend(pose_result['failed'])
        missing.extend(pose_result['missing'])
    ok = not failed and not missing
    result = {'ok': ok, 'checked_followers': checked}
    if failed:
        result['failed'] = failed
    if missing:
        result['missing'] = missing
        result['missing_state'] = ['/formations/status', 'robot_poses']
    return result


def _follower_errors_from_poses(
    expected: dict,
    pose_snapshot: dict,
    tolerance: float,
) -> dict:
    fid = expected['formation_id']
    leader_id = RobotPoseCache._robot_id_from_ns(expected.get('leader_ns', ''))
    leader = _snapshot_get(pose_snapshot, leader_id)
    if leader is None or leader.get('stale'):
        return {
            'checked': 0,
            'failed': [],
            'missing': [{'formation_id': fid, 'robot': expected.get('leader_ns', '')}],
        }
    follower_ns = list(expected.get('follower_ns') or [])
    offsets_x = list(expected.get('offsets_x') or [])
    offsets_y = list(expected.get('offsets_y') or [])
    n = min(len(follower_ns), len(offsets_x), len(offsets_y))
    checked = 0
    failed = []
    missing = []
    for i in range(n):
        robot = follower_ns[i]
        rid = RobotPoseCache._robot_id_from_ns(robot)
        pose = _snapshot_get(pose_snapshot, rid)
        if pose is None or pose.get('stale'):
            missing.append({'formation_id': fid, 'robot': robot})
            continue
        target = apply_pose(
            (float(offsets_x[i]), float(offsets_y[i])),
            (float(leader['x']), float(leader['y'])),
            float(leader['yaw']),
        )
        dist = math.hypot(float(pose['x']) - target[0], float(pose['y']) - target[1])
        checked += 1
        if dist > tolerance:
            failed.append({
                'formation_id': fid,
                'robot': robot,
                'distance_to_expected_m': round(dist, 3),
                'tolerance_m': round(tolerance, 3),
            })
    return {'checked': checked, 'failed': failed, 'missing': missing}


def _check_robots_in_room(
    map_cfg: dict,
    room: str,
    groups: list[dict],
    pose_snapshot: dict,
) -> dict:
    boundary = resolve_room_boundary(map_cfg or {}, room)
    if boundary is None:
        return {'ok': False, 'details': f'unknown room {room!r}'}
    polygon = boundary.get('polygon') or []
    failed = []
    missing = []
    for group in groups:
        robot_ids = _int_list((group or {}).get('robot_ids') or [])
        points = []
        for rid in robot_ids:
            pose = _snapshot_get(pose_snapshot, rid)
            if pose is None or pose.get('stale'):
                missing.append(f'robot_{rid}')
                continue
            points.append((float(pose['x']), float(pose['y'])))
        center = centroid(points) if points else None
        if center is not None and polygon and not point_in_polygon(center, polygon):
            failed.append({
                'group': _safe_str((group or {}).get('name')),
                'centroid': [round(center[0], 3), round(center[1], 3)],
            })
    ok = not failed and not missing
    result = {
        'ok': ok,
        'details': (
            f'group centroids are inside {room}'
            if ok else f'could not confirm all group centroids inside {room}'
        ),
    }
    if failed:
        result['failed'] = failed
    if missing:
        result['missing'] = missing
        result['missing_state'] = ['robot_poses']
    return result


def _recent_errors(bt_state: Any, recent_events: list[Any], last_failure: Any) -> list[str]:
    candidates: list[str] = []
    for key in ('last_error', 'action_summary', 'formation_failure_reason'):
        text = _field(bt_state, key)
        if text:
            candidates.append(text)
    if isinstance(last_failure, dict):
        for key in ('last_error', 'error', 'summary'):
            text = _safe_str(last_failure.get(key))
            if text:
                candidates.append(text)
    for item in recent_events[-10:]:
        if isinstance(item, dict):
            candidates.extend(
                _safe_str(item.get(key))
                for key in ('trigger', 'output', 'reason', 'error')
                if item.get(key)
            )
        else:
            candidates.append(_safe_str(item))
    out = []
    for text in candidates:
        lower = text.lower()
        if any(hint in lower for hint in ERROR_HINTS) or 'error' in lower:
            clean = text[:300]
            if clean and clean not in out:
                out.append(clean)
    return out[:8]


def _repair_recommendation(
    *,
    ok: bool,
    active_check: dict,
    follower_check: dict,
    recent_errors: list[str],
    room: str,
    group_count: int,
    missing_state: list[str],
) -> dict:
    if ok:
        return {
            'type': 'none',
            'reason': 'verification passed',
            'repairable': False,
            'should_recompute_placement': False,
        }
    if missing_state and not (
        active_check.get('missing') or follower_check.get('failed') or recent_errors
    ):
        return {
            'type': 'wait_for_state',
            'reason': 'required verification state is unavailable',
            'repairable': False,
            'should_recompute_placement': False,
        }
    should_recompute = bool(room and group_count >= 2)
    if follower_check.get('failed') or _recent_error_is_out_of_position(recent_errors):
        return {
            'type': 'restage_then_activate',
            'reason': 'followers are far from required offsets',
            'repairable': True,
            'should_recompute_placement': should_recompute,
        }
    if active_check.get('missing') or active_check.get('inactive'):
        return {
            'type': 'activate_or_restage',
            'reason': 'one or more expected formations are missing or inactive',
            'repairable': True,
            'should_recompute_placement': should_recompute,
        }
    return {
        'type': 'replan',
        'reason': 'requested outcome did not verify',
        'repairable': True,
        'should_recompute_placement': should_recompute,
    }


def _summary(
    *,
    ok: bool,
    expectations: list[dict],
    active_check: dict,
    follower_check: dict,
    recent_errors: list[str],
    missing_state: list[str],
    original_request: str,
) -> str:
    ids = [item['formation_id'] for item in expectations]
    if ok:
        return (
            f"{', '.join(ids)} active; followers within tolerance"
            if ids else 'verification passed'
        )
    bits = []
    if active_check.get('missing'):
        bits.append('missing formations: ' + ', '.join(active_check['missing']))
    if active_check.get('inactive'):
        bits.append(
            'inactive formations: '
            + ', '.join(item['formation_id'] for item in active_check['inactive'])
        )
    if follower_check.get('failed'):
        bits.append(f"{len(follower_check['failed'])} follower(s) out of tolerance")
    if missing_state:
        bits.append('missing state: ' + ', '.join(missing_state))
    if recent_errors:
        bits.append('recent error: ' + recent_errors[0])
    if not bits:
        bits.append('requested outcome did not verify')
    if original_request and len('; '.join(bits)) < 40:
        bits.append(f'for request: {original_request[:80]}')
    return '; '.join(bits)


def _recent_error_is_hard_failure(errors: list[str]) -> bool:
    return _recent_error_is_out_of_position(errors) or any(
        'broken' in err.lower() for err in errors
    )


def _recent_error_is_out_of_position(errors: list[str]) -> bool:
    return any(
        ('out of position' in err.lower()
         or 'registered but not activated' in err.lower()
         or 'too far from target' in err.lower())
        for err in errors
    )


def _snapshot_get(snapshot: dict, rid: int | None) -> dict | None:
    if rid is None:
        return None
    if rid in snapshot and isinstance(snapshot[rid], dict):
        return snapshot[rid]
    key = str(rid)
    if key in snapshot and isinstance(snapshot[key], dict):
        return snapshot[key]
    return None


def _state_name(value: Any) -> str:
    if isinstance(value, str):
        text = value.strip()
        if text.isdigit():
            return FORMATION_STATES.get(int(text), text)
        return text.upper()
    try:
        return FORMATION_STATES.get(int(value), str(value))
    except (TypeError, ValueError):
        return 'UNKNOWN'


def _field(value: Any, key: str) -> str:
    if isinstance(value, dict):
        return _safe_str(value.get(key))
    return _safe_str(getattr(value, key, ''))


def _formation_name_from_id(fid: str) -> str:
    parts = _safe_str(fid).split('_')
    return parts[-1] if parts else ''


def _int_list(value: Any) -> list[int]:
    out = []
    for item in list(value or []):
        try:
            out.append(int(item))
        except (TypeError, ValueError):
            match = re.fullmatch(r'robot[_-]?(\d+)', _safe_str(item))
            if match:
                out.append(int(match.group(1)))
    return out


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
    if value is None:
        return ''
    return str(value)
