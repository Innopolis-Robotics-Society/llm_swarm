"""One carrier per carry task.

A carry task latches whoever stands inside its radius first
(``task_manager_node._within``), and the MAPF server blocks every
non-participating robot into the planning grid before it checks
reachability (``block_skipped_robots`` then ``static_path_exists``). A plan
that routes a second robot into a carry pickup zone therefore breaks the
mission twice over: the stray robot takes the load, and by parking there it
turns the pickup into an obstacle for the robot that owns the delivery leg.

Observed twice in cell B, runs ``20260818_085836`` and ``20260818_090145``.
Both plans carried the correct sequence -- robot_1 to the pickup, then
robot_1 to the dropoff -- plus a stray parallel leaf sending robot_5 to the
same pickup point. robot_5 arrived first and took the load; robot_1's leg
then died on ``no static path ... footprint won't fit through``; the load
sat on a robot nobody had ordered to move until the operator stopped the
run. Identical to the metre in both runs.

Geometry cannot fix this. The goal-spreading helper separates duplicate
goals by 1.0 m while the carry radius is 1.5 m, so a spread robot still
latches the task. The duplicate is semantic, and it is removed
semantically: the robot that owns the delivery leg keeps its goal, the
others are dropped from the plan.
"""

from __future__ import annotations

import copy
import math
from typing import Any

_CONTAINER_TYPES = {'sequence', 'parallel'}


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point(raw: Any) -> tuple[float, float] | None:
    """Accept [x, y] and {'x': .., 'y': ..}; anything else is not a point."""
    if isinstance(raw, (list, tuple)) and len(raw) >= 2:
        try:
            return (float(raw[0]), float(raw[1]))
        except (TypeError, ValueError):
            return None
    if isinstance(raw, dict) and 'x' in raw and 'y' in raw:
        try:
            return (float(raw['x']), float(raw['y']))
        except (TypeError, ValueError):
            return None
    return None


def _mapf_leaves(node: Any, out: list[dict]) -> None:
    """Collect mapf leaves in plan order (depth-first, left to right)."""
    if not isinstance(node, dict):
        return
    if node.get('type') in _CONTAINER_TYPES:
        for step in node.get('steps') or []:
            _mapf_leaves(step, out)
        return
    if node.get('type') == 'mapf':
        out.append(node)


def _pairs(leaf: dict) -> list[tuple[int, tuple[float, float]]]:
    """(robot_id, goal) pairs of one mapf leaf, skipping unusable entries.

    A leaf whose goals are shorter than its robot_ids is malformed rather
    than empty; the extra robots are left alone here so that the malformed
    plan still reaches whatever validation exists downstream.
    """
    ids = node_ids(leaf)
    goals = list(leaf.get('goals') or [])
    pairs: list[tuple[int, tuple[float, float]]] = []
    for rid, raw in zip(ids, goals):
        pt = _point(raw)
        if pt is not None:
            pairs.append((rid, pt))
    return pairs


def node_ids(leaf: dict) -> list[int]:
    ids: list[int] = []
    for raw in leaf.get('robot_ids') or []:
        try:
            ids.append(int(raw))
        except (TypeError, ValueError):
            continue
    return ids


def _drop_robots(leaf: dict, drop: set[int]) -> None:
    """Remove the named robots and their goals from one leaf, in place."""
    ids = node_ids(leaf)
    goals = list(leaf.get('goals') or [])
    keep_ids: list[int] = []
    keep_goals: list[Any] = []
    for idx, rid in enumerate(ids):
        if rid in drop:
            continue
        keep_ids.append(rid)
        if idx < len(goals):
            keep_goals.append(goals[idx])
    leaf['robot_ids'] = keep_ids
    leaf['goals'] = keep_goals


def _prune_empty(node: Any) -> Any | None:
    """Drop mapf leaves left with no robots, then containers left with no steps."""
    if not isinstance(node, dict):
        return node
    if node.get('type') in _CONTAINER_TYPES:
        steps = []
        for step in node.get('steps') or []:
            kept = _prune_empty(step)
            if kept is not None:
                steps.append(kept)
        if not steps:
            return None
        node['steps'] = steps
        return node
    if node.get('type') == 'mapf' and not node_ids(node):
        return None
    return node


def enforce_single_carrier(
    plan: dict,
    tasks: dict,
) -> tuple[dict, list[dict]]:
    """Leave at most one robot routed into each carry task's pickup zone.

    ``tasks`` is the shape ``chat_server._get_task_context`` returns:
    ``{task_id: {type, position, radius, dropoff, ...}}``. Tasks that are
    not carries, or that carry no usable radius, are ignored -- this guard
    only knows the one rule it is named after.

    Returns the corrected plan and one record per task it touched. An empty
    record list means the plan was already sound, which is the normal case.
    """
    if not isinstance(plan, dict) or not tasks:
        return plan, []

    new_plan = copy.deepcopy(plan)
    leaves: list[dict] = []
    _mapf_leaves(new_plan, leaves)
    if not leaves:
        return plan, []

    records: list[dict] = []

    for task_id, task in tasks.items():
        if not isinstance(task, dict) or task.get('type') != 'carry':
            continue
        pickup = _point(task.get('position'))
        if pickup is None:
            continue
        try:
            radius = float(task.get('radius'))
        except (TypeError, ValueError):
            continue
        if radius <= 0.0:
            continue
        dropoff = _point(task.get('dropoff'))

        # Every robot whose goal lands inside the pickup zone, in plan order.
        # Order matters: with no delivery leg to disambiguate, the first
        # robot the plan names is the one the plan meant to send.
        inside: list[int] = []
        for leaf in leaves:
            for rid, goal in _pairs(leaf):
                if rid not in inside and _distance(goal, pickup) <= radius:
                    inside.append(rid)
        if len(inside) <= 1:
            continue

        # The carrier is whoever the plan also sends to the dropoff. Without
        # that evidence there is nothing to prefer, so keep the first.
        carriers: list[int] = []
        if dropoff is not None:
            for leaf in leaves:
                for rid, goal in _pairs(leaf):
                    if (rid in inside and rid not in carriers
                            and _distance(goal, dropoff) <= radius):
                        carriers.append(rid)
        keep = carriers[0] if carriers else inside[0]
        drop = {rid for rid in inside if rid != keep}

        for leaf in leaves:
            leaf_drop = drop.intersection(
                rid for rid, goal in _pairs(leaf)
                if _distance(goal, pickup) <= radius
            )
            if leaf_drop:
                _drop_robots(leaf, leaf_drop)

        records.append({
            'type':    'single_carrier',
            'task':    task_id,
            'kept':    keep,
            'dropped': sorted(drop),
            'reason':  ('kept the robot the plan also sends to the dropoff'
                        if carriers else
                        'no delivery leg names a carrier, kept the first'),
        })

    if not records:
        return plan, []

    pruned = _prune_empty(new_plan)
    if pruned is None:
        # Every leaf was a duplicate carrier. Refusing to return an empty
        # plan: an idle node is a plan the executor understands, an empty
        # dict is not.
        pruned = {'type': 'idle', 'reason': 'all steps were duplicate carriers'}
    return pruned, records
