"""Formation-aware guards for MAPF leaves.

The guard is deterministic and read-only.  It consumes the latest
``/formations/status`` snapshot, detects active formation membership, and
prevents direct MAPF control of active followers.
"""

from __future__ import annotations

import copy
import json
import re
from typing import Any, Callable


ACTIVE_STATES = {'FORMING', 'STABLE', 'DEGRADED'}
_ROBOT_ID_RE = re.compile(r'(?:robot[_-]?)?(\d+)', re.IGNORECASE)


def guard_plan_for_active_formations(
    plan: dict,
    formations_status: Any,
    *,
    log_fn: Callable[[str], None] | None = None,
) -> tuple[dict, dict | None]:
    """Return ``(guarded_plan, failure)`` for active-formation MAPF safety.

    ``failure`` is JSON-serialisable and shaped like PlanExecutor/BT failure
    metadata so the existing mission repair loop can reuse it directly.
    """
    log = log_fn or (lambda _msg: None)
    formations = active_formations_from_status(formations_status)
    log(f'Formation guard: active formations count={len(formations)}')
    if not formations:
        return copy.deepcopy(plan), None

    try:
        guarded, failure, _inactive = _guard_node(
            copy.deepcopy(plan),
            formations,
            inactive_fids=set(),
            log_fn=log,
        )
    except ActiveFormationGuardError as exc:
        return copy.deepcopy(plan), exc.failure
    return guarded, failure


def active_formations_from_status(formations_status: Any) -> list[dict]:
    """Normalize raw or context-normalized formation status into active records."""
    items = _formation_items(formations_status)
    out: list[dict] = []
    for item in items:
        fid = _safe_str(_field(item, 'formation_id'))
        leader_ns = _safe_str(_field(item, 'leader_ns'))
        followers = _followers(item)
        status = _state_name(_field(item, 'status', 'state'))
        active_flag = _field(item, 'active')
        active = (
            status in ACTIVE_STATES
            or (active_flag is True and status not in {'INACTIVE', 'BROKEN'})
        )
        leader_id = robot_id_from_ns(leader_ns)
        follower_ids = [rid for rid in (robot_id_from_ns(ns) for ns in followers)
                        if rid is not None]
        if not active or not fid or leader_id is None:
            continue
        out.append({
            'formation_id': fid,
            'leader_ns': leader_ns,
            'leader_id': leader_id,
            'follower_ns': followers,
            'follower_ids': follower_ids,
            'member_ids': [leader_id] + follower_ids,
            'status': status,
        })
    return out


def robot_id_from_ns(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        match = _ROBOT_ID_RE.fullmatch(value.strip())
        if match:
            return int(match.group(1))
    return None


def _guard_node(
    node: Any,
    formations: list[dict],
    *,
    inactive_fids: set[str],
    log_fn: Callable[[str], None],
) -> tuple[Any, dict | None, set[str]]:
    if not isinstance(node, dict):
        return node, None, inactive_fids

    node_type = node.get('type')
    if node_type == 'sequence':
        steps = []
        inactive = set(inactive_fids)
        for step in list(node.get('steps') or []):
            guarded, failure, inactive = _guard_node(
                step,
                formations,
                inactive_fids=inactive,
                log_fn=log_fn,
            )
            if failure:
                return node, failure, inactive
            steps.append(guarded)
            if isinstance(guarded, dict) and guarded.get('type') == 'disband':
                fid = _safe_str(guarded.get('formation_id'))
                if fid:
                    inactive.add(fid)
                    log_fn(
                        'Formation guard: inserted disband before individual '
                        f'movement formation_id={fid}'
                    )
        node['steps'] = steps
        return node, None, inactive

    if node_type == 'parallel':
        steps = []
        for step in list(node.get('steps') or []):
            guarded, failure, _unused = _guard_node(
                step,
                formations,
                inactive_fids=set(inactive_fids),
                log_fn=log_fn,
            )
            if failure:
                return node, failure, inactive_fids
            steps.append(guarded)
        node['steps'] = steps
        return node, None, inactive_fids

    if node_type == 'mapf':
        return _guard_mapf(node, formations, inactive_fids, log_fn), None, inactive_fids

    return node, None, inactive_fids


def _guard_mapf(
    leaf: dict,
    formations: list[dict],
    inactive_fids: set[str],
    log_fn: Callable[[str], None],
) -> dict:
    ids = [rid for rid in (robot_id_from_ns(r) for r in leaf.get('robot_ids') or [])
           if rid is not None]
    if not ids:
        return leaf

    goals = _expanded_goals(leaf, len(ids))
    keep: dict[int, list[float]] = {
        rid: list(goals[idx]) for idx, rid in enumerate(ids) if idx < len(goals)
    }
    rewritten = False

    for formation in formations:
        fid = formation['formation_id']
        if fid in inactive_fids:
            continue
        leader_id = int(formation['leader_id'])
        follower_ids = set(int(rid) for rid in formation.get('follower_ids') or [])
        member_ids = set(int(rid) for rid in formation.get('member_ids') or [])
        requested = set(ids).intersection(member_ids)
        requested_followers = sorted(set(ids).intersection(follower_ids))
        if not requested:
            continue
        log_fn(
            'Formation guard: MAPF leaf robots='
            f'{ids} intersects active formation formation_id={fid}'
        )
        if not requested_followers:
            continue
        if requested == member_ids:
            leader_goal = keep.get(leader_id)
            if leader_goal is None:
                leader_goal = _first_goal_for_ids(ids, goals)
            for rid in follower_ids:
                keep.pop(rid, None)
            keep[leader_id] = leader_goal
            rewritten = True
            log_fn(
                'Formation guard: rewriting full formation movement '
                f'formation_id={fid} leader=robot_{leader_id}'
            )
            continue

        follower_ns = [f'robot_{rid}' for rid in requested_followers]
        log_fn(
            'Formation guard: rejecting MAPF for active followers '
            f'formation_id={fid} followers={follower_ns}'
        )
        raise ActiveFormationGuardError(_failure(
            formation,
            follower_ns,
            leaf,
        ))

    if not rewritten:
        return leaf
    new_ids = [rid for rid in ids if rid in keep]
    new_goals = [keep[rid] for rid in new_ids]
    guarded = dict(leaf)
    guarded['robot_ids'] = new_ids
    guarded['goals'] = new_goals
    if len(new_ids) <= 1:
        guarded.pop('spread', None)
    guarded['reason'] = _append_reason(
        guarded.get('reason'),
        'formation guard moved active formations by leader only',
    )
    return guarded


class ActiveFormationGuardError(RuntimeError):
    def __init__(self, failure: dict):
        super().__init__(failure.get('last_error', 'active formation guard failed'))
        self.failure = failure


def _failure(formation: dict, followers: list[str], leaf: dict) -> dict:
    fid = formation.get('formation_id', '')
    leader = formation.get('leader_ns', '')
    hint = (
        f'move leader {leader} for whole formation {fid}, or disband '
        f'{fid} before moving followers independently'
    )
    return {
        'leaf_type': 'mapf',
        'failed_at_phase': 'formation_guard',
        'action_status': 'ERROR',
        'reason': 'mapf_targets_active_formation_followers',
        'last_error': (
            'mapf_targets_active_formation_followers: '
            f'formation_id={fid} followers={followers} leader={leader}'
        ),
        'formation_id': fid,
        'followers': followers,
        'leader': leader,
        'repair_hint': hint,
        'repairable': True,
        'failed_leaf': _jsonable(leaf),
    }


def _expanded_goals(leaf: dict, count: int) -> list[list[float]]:
    goals = []
    for goal in list(leaf.get('goals') or []):
        if isinstance(goal, (list, tuple)) and len(goal) >= 2:
            try:
                goals.append([float(goal[0]), float(goal[1])])
            except (TypeError, ValueError):
                continue
    if len(goals) == 1 and count > 1:
        return [list(goals[0]) for _ in range(count)]
    return goals


def _first_goal_for_ids(ids: list[int], goals: list[list[float]]) -> list[float]:
    if goals:
        return list(goals[0])
    return [0.0, 0.0]


def _append_reason(existing: Any, suffix: str) -> str:
    text = _safe_str(existing)
    if not text:
        return suffix
    if suffix in text:
        return text
    return f'{text}; {suffix}'


def _formation_items(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, dict):
        if isinstance(value.get('formations'), list):
            return list(value.get('formations') or [])
        return [value]
    if isinstance(value, list):
        return value
    return list(getattr(value, 'formations', []) or [])


def _followers(item: Any) -> list[str]:
    raw = _field(item, 'followers')
    if raw is None:
        raw = _field(item, 'follower_ns')
    return [_safe_str(ns) for ns in list(raw or [])]


def _state_name(value: Any) -> str:
    if isinstance(value, str):
        text = value.strip()
        if text.isdigit():
            return _state_name(int(text))
        return text.upper()
    try:
        return {
            0: 'INACTIVE',
            1: 'FORMING',
            2: 'STABLE',
            3: 'DEGRADED',
            4: 'BROKEN',
        }.get(int(value), f'UNKNOWN_{int(value)}')
    except (TypeError, ValueError):
        return 'UNKNOWN'


def _field(item: Any, *names: str) -> Any:
    for name in names:
        if isinstance(item, dict) and name in item:
            return item.get(name)
        if hasattr(item, name):
            return getattr(item, name)
    return None


def _safe_str(value: Any) -> str:
    if value is None:
        return ''
    return str(value).strip()


def _jsonable(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False)
        return value
    except TypeError:
        return str(value)
