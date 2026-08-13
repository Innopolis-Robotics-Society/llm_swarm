"""Plan executor — interprets and executes a nested sequence/parallel task tree.

Node types:
  leaf:      mapf | formation | idle
  container: sequence | parallel

Execution semantics:
  sequence — execute children one by one; abort on first failure
  parallel — for multiple mapf leaves: MERGE into one mapf (MAPF planner
             handles multi-robot coordination internally). For mixed types
             (mapf + formation): execute sequentially (formation requires
             its own BT mode switch, can't run truly in parallel).

This matches the actual BT architecture: LlmCommandReceiver accepts one
goal at a time, and MAPF handles intra-robot parallelism natively.

Public API:
  parse_plan(raw)                -> dict
  PlanExecutor(send_fn, log_fn)  -> executor
  await executor.run(plan)       -> bool
"""

from __future__ import annotations

import asyncio
import json
import math
import re
from typing import Awaitable, Callable

from iros_llm_orchestrator.common.plan_templating import resolve_plan_templates


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

_LEAF_TYPES      = {'mapf', 'formation', 'idle', 'disband'}
_CONTAINER_TYPES = {'sequence', 'parallel'}
_ALL_TYPES       = _LEAF_TYPES | _CONTAINER_TYPES

_ROBOT_ID_RE = re.compile(r'(?:robot[_-]?)?(\d+)', re.IGNORECASE)


def coerce_robot_id(value) -> int:
    """Coerce an LLM-supplied robot id to a plain int.

    Accepts ``10``, ``"10"``, ``"robot_10"``, ``"robot10"`` and integral
    floats. Small local models routinely emit the namespace string
    ``"robot_N"`` instead of the bare integer the plan schema expects.
    Raises ValueError on anything genuinely unparseable.
    """
    # bool is a subclass of int — reject it explicitly.
    if isinstance(value, bool):
        raise ValueError(f'invalid robot id: {value!r}')
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if value.is_integer():
            return int(value)
        raise ValueError(f'non-integer robot id: {value!r}')
    if isinstance(value, str):
        m = _ROBOT_ID_RE.fullmatch(value.strip())
        if m:
            return int(m.group(1))
    raise ValueError(f'cannot parse robot id: {value!r}')


def parse_plan(
    raw: str | dict,
    *,
    template_registry: dict[str, dict] | None = None,
) -> dict:
    """Parse and validate a plan tree. Raises ValueError on any error.

    ``template_registry``, when not None, resolves any ``{{ref.path}}``
    template string (see ``plan_templating.py``) against tool-call results
    from this chat turn *before* validation/coercion runs, so a resolved
    template is validated exactly like a literal the model typed by hand.
    Passing None (the default) skips resolution entirely and preserves the
    exact prior behaviour, which is what ``execute_server.py`` wants: an
    operator-approved plan already had every template resolved when it was
    first generated, and must replay byte-for-byte identically.
    """
    if isinstance(raw, str):
        raw = raw.strip()
        fenced = re.search(r'```(?:json)?\s*(\{.*\})\s*```', raw, re.DOTALL)
        if fenced:
            raw = fenced.group(1)
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f'plan is not valid JSON: {exc}') from exc
    else:
        obj = raw

    if not isinstance(obj, dict):
        raise ValueError('plan must be a JSON object')

    # Unwrap {"reply":"...", "plan":{...}}
    if 'plan' in obj:
        obj = obj['plan']

    if template_registry is not None:
        # PlanTemplateError is a ValueError subclass, so it lands in the same
        # exception surface parse_plan already raises for malformed JSON or a
        # bad schema — every existing caller's `except ValueError` retry/repair
        # path picks it up for free, no separate handling needed there.
        obj = resolve_plan_templates(obj, template_registry)

    return _validate_node(obj)


def _validate_node(node: dict, path: str = 'plan') -> dict:
    if not isinstance(node, dict):
        raise ValueError(f'{path}: expected object, got {type(node).__name__}')
    node_type = node.get('type', '')
    if node_type not in _ALL_TYPES:
        raise ValueError(
            f'{path}.type={node_type!r} invalid. Expected: {sorted(_ALL_TYPES)}')
    if node_type in _CONTAINER_TYPES:
        steps = node.get('steps', [])
        if not isinstance(steps, list) or not steps:
            raise ValueError(f'{path}.steps must be a non-empty list')
        for i, child in enumerate(steps):
            _validate_node(child, f'{path}.steps[{i}]')
    elif node_type == 'mapf':
        ids   = node.get('robot_ids', [])
        goals = node.get('goals', [])
        spread = bool(node.get('spread', False))
        if not ids:
            raise ValueError(f'{path}: mapf requires non-empty robot_ids')
        # A single centre for several robots can only mean "grid them around
        # this point" — infer spread even if the model omitted the flag.
        if len(goals) == 1 and len(ids) > 1:
            spread = True
        if spread:
            # spread=true: one center (the server grids the robots around it) or
            # already one per robot. _postprocess_plan expands/declumps it.
            if len(goals) not in (1, len(ids)):
                raise ValueError(
                    f'{path}: spread mapf needs 1 center goal or one per robot '
                    f'(got robot_ids={len(ids)}, goals={len(goals)})')
        elif len(ids) != len(goals):
            raise ValueError(
                f'{path}: robot_ids({len(ids)}) != goals({len(goals)})')
        node['spread'] = spread
        # Normalise robot ids in place — LLMs (especially small local models)
        # routinely emit "robot_10" instead of the bare int the schema wants.
        try:
            node['robot_ids'] = [coerce_robot_id(r) for r in ids]
        except ValueError as exc:
            raise ValueError(f'{path}: {exc}') from exc
        # Normalise goals to [float, float]; reject non-numeric coordinates.
        norm_goals = []
        for g in goals:
            if not (isinstance(g, (list, tuple)) and len(g) >= 2):
                raise ValueError(f'{path}: each goal must be [x, y]')
            try:
                norm_goals.append([float(g[0]), float(g[1])])
            except (TypeError, ValueError) as exc:
                raise ValueError(f'{path}: goal {g!r} is not numeric') from exc
        node['goals'] = norm_goals
    elif node_type == 'formation':
        if not node.get('formation_id'):
            raise ValueError(f'{path}: formation requires formation_id')
        if not node.get('leader_ns'):
            raise ValueError(f'{path}: formation requires leader_ns')
        if not node.get('follower_ns'):
            raise ValueError(f'{path}: formation requires follower_ns (at least one follower)')
        if node.get('offsets_x') is None or node.get('offsets_y') is None:
            raise ValueError(f'{path}: formation requires offsets_x and offsets_y')
        fn = node['follower_ns']
        ox = node['offsets_x']
        oy = node['offsets_y']
        if len(fn) != len(ox) or len(fn) != len(oy):
            raise ValueError(
                f'{path}: follower_ns({len(fn)}) != offsets_x({len(ox)}) '
                f'or offsets_y({len(oy)})')
    elif node_type == 'disband':
        if not node.get('formation_id'):
            raise ValueError(f'{path}: disband requires formation_id')
    return node


# ---------------------------------------------------------------------------
# Parallel flattening
# ---------------------------------------------------------------------------

class PlanConflictError(ValueError):
    """Two branches of one parallel send the same robot to different places.

    Raised instead of silently keeping one of the goals. See flatten_parallel.
    """


def flatten_parallel(node: dict) -> list[dict]:
    """Flatten a parallel node into an ordered list of commands to execute.

    Rules:
    - Multiple mapf leaves → merge into ONE mapf (MAPF planner handles
      multi-robot coordination; no need to send two separate commands).
    - idle in any branch → return [idle] immediately (stop takes priority).
    - mixed mapf+formation → keep separate, execute as mini-sequence
      (formation requires its own BT mode, can't run simultaneously with mapf).
    - A nested ``sequence`` keeps its order and runs AFTER the merged mapf.
    - A nested ``parallel`` is flattened into this one.

    THE NESTED-SEQUENCE RULE EXISTS BECAUSE IT WAS BROKEN
    This used to walk every branch with _collect_leaves, which recurses into
    containers indiscriminately, so the two legs of a carry expressed as

        parallel[ mapf(A), mapf(B), sequence[ mapf(C→pickup), mapf(C→dropoff) ] ]

    landed in the same merge bucket and last-write-wins kept only the dropoff.
    The robots drove straight to the delivery point, the task manager never saw
    them at the pickup, and the carry stayed PENDING -- indistinguishable from
    "never attempted". Worse, the mission reported success, because verification
    checks that MAPF goals were reached and those goals were, in fact, reached.
    Observed end to end in session 20260813_085814.

    Serialising after the merge costs mission time -- the carry starts once the
    point-task legs are on their way -- and that is the cheap side of the trade.
    """
    assert node['type'] == 'parallel'

    # Branches that may run concurrently, versus branches whose internal order
    # is load-bearing. A `sequence` is the second kind and must not be merged.
    concurrent: list[dict] = []
    ordered_tails: list[list[dict]] = []
    for step in node['steps']:
        if step.get('type') == 'sequence':
            tail = flatten_ordered(step)
            if tail:
                ordered_tails.append(tail)
        else:
            concurrent.extend(_collect_leaves(step))

    # idle takes priority, wherever it appears — including inside a sequence.
    if any(n['type'] == 'idle' for n in concurrent) or any(
            n['type'] == 'idle' for tail in ordered_tails for n in tail):
        return [{'type': 'idle', 'reason': 'idle in parallel branch'}]

    mapf_leaves      = [n for n in concurrent if n['type'] == 'mapf']
    non_mapf_leaves  = [n for n in concurrent if n['type'] != 'mapf']

    result: list[dict] = []

    if mapf_leaves:
        merged: dict[int, list] = {}
        merged_spread: dict[int, bool] = {}
        reasons: list[str]      = []
        conflicts: list[str]    = []
        for leaf in mapf_leaves:
            ids, goals, should_spread = _mapf_ids_goals_for_merge(leaf)
            for rid, g in zip(ids, goals):
                prev = merged.get(rid)
                # Same robot named twice with the SAME goal is the benign case
                # the old comment described: an operator phrasing one group
                # twice. Two DIFFERENT goals is not benign — one of them is
                # about to be dropped, and dropping it silently is how a broken
                # plan reports success.
                if prev is not None and not _same_goal(prev, g):
                    conflicts.append(
                        'robot_%d: (%.2f, %.2f) vs (%.2f, %.2f)'
                        % (rid, prev[0], prev[1], g[0], g[1]))
                merged[rid] = g
                merged_spread[rid] = should_spread
            if leaf.get('reason'):
                reasons.append(leaf['reason'])
        if conflicts:
            raise PlanConflictError(
                'parallel branches disagree on where these robots go, so one '
                'goal would be discarded: ' + '; '.join(conflicts)
                + '. Put the steps in a sequence if they are meant to happen '
                  'one after another.')
        robot_ids = list(merged.keys())
        goals = list(merged.values())
        if any(merged_spread.get(rid, False) for rid in robot_ids):
            goals = _spread_near_duplicate_goals(
                goals,
                spread_mask=[merged_spread.get(rid, False) for rid in robot_ids],
            )
        result.append({
            'type':      'mapf',
            'robot_ids': robot_ids,
            'goals':     goals,
            'reason':    ' + '.join(reasons),
        })

    # Non-mapf (formation etc.) run after the merged mapf
    result.extend(non_mapf_leaves)
    # Then each ordered branch, in its own order. Concatenating them is safe
    # because _run_parallel executes the list sequentially and each command
    # blocks until its robots arrive.
    for tail in ordered_tails:
        result.extend(tail)
    return result


def flatten_ordered(node: dict) -> list[dict]:
    """Ordered command list for any node: leaves as-is, sequences in order.

    Mutually recursive with flatten_parallel, so a parallel nested inside a
    sequence still gets its mapf leaves merged into one planner call.
    """
    t = node.get('type')
    if t in _LEAF_TYPES:
        return [node]
    if t == 'parallel':
        return flatten_parallel(node)
    if t == 'sequence':
        out: list[dict] = []
        for step in node.get('steps', []):
            out.extend(flatten_ordered(step))
        return out
    return []


def _same_goal(a, b, tol: float = 1e-6) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def _mapf_ids_goals_for_merge(
    leaf: dict,
) -> tuple[list[int], list[list[float]], bool]:
    """Return a merge-safe one-goal-per-robot view of a mapf leaf.

    The chat server normally expands ``spread:true`` during post-processing.
    Keep this fallback here because PlanExecutor is also used directly in
    tests and replay paths; a single center for N ids must never be truncated
    to just the first robot by ``zip(ids, goals)``.
    """
    ids = [coerce_robot_id(r) for r in leaf.get('robot_ids', [])]
    goals = [[float(g[0]), float(g[1])] for g in leaf.get('goals', [])]
    if len(goals) == 1 and len(ids) > 1:
        goals = [list(goals[0]) for _ in ids]
    return ids, goals, bool(leaf.get('spread', False))


def _spread_near_duplicate_goals(
    goals: list[list[float]],
    *,
    spread_mask: list[bool] | None = None,
    min_dist_m: float = 0.75,
    spacing_m: float = 1.0,
) -> list[list[float]]:
    if len(goals) <= 1:
        return goals
    result = [list(g) for g in goals]
    visited = [False] * len(goals)
    for i in range(len(goals)):
        if visited[i]:
            continue
        group = [i]
        visited[i] = True
        for j in range(i + 1, len(goals)):
            if visited[j]:
                continue
            dx = goals[j][0] - goals[i][0]
            dy = goals[j][1] - goals[i][1]
            if math.hypot(dx, dy) < min_dist_m:
                group.append(j)
                visited[j] = True
        if len(group) <= 1:
            continue
        if spread_mask is not None and not any(spread_mask[idx] for idx in group):
            continue
        cx = sum(goals[idx][0] for idx in group) / len(group)
        cy = sum(goals[idx][1] for idx in group) / len(group)
        if len(group) == 2:
            offsets = [(-spacing_m / 2.0, 0.0), (spacing_m / 2.0, 0.0)]
        else:
            radius = spacing_m / (2.0 * math.sin(math.pi / len(group)))
            offsets = [
                (
                    radius * math.cos(2.0 * math.pi * k / len(group)),
                    radius * math.sin(2.0 * math.pi * k / len(group)),
                )
                for k in range(len(group))
            ]
        for idx, (ox, oy) in zip(group, offsets):
            result[idx] = [round(cx + ox, 2), round(cy + oy, 2)]
    return result


def _collect_leaves(node: dict) -> list[dict]:
    """Recursively collect all leaf nodes from a subtree."""
    if node['type'] in _LEAF_TYPES:
        return [node]
    leaves: list[dict] = []
    for step in node.get('steps', []):
        leaves.extend(_collect_leaves(step))
    return leaves


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------

SendFn = Callable[[dict], Awaitable[bool]]
LogFn  = Callable[[str], None]
# Returns a ``mapf`` leaf to run before the given formation leaf, or None to
# skip staging. Used to inject server-side staging when the LLM emits a bare
# ``formation`` leaf with followers out of position.
PrestageHook = Callable[[dict], dict | None]
PlanGuardHook = Callable[[dict], tuple[dict, dict | None]]


class PlanExecutor:
    def __init__(
        self,
        send_fn: SendFn,
        log_fn: LogFn | None = None,
        *,
        formation_prestage_hook: PrestageHook | None = None,
        plan_guard_hook: PlanGuardHook | None = None,
    ):
        self._send  = send_fn
        self._log   = log_fn or (lambda _: None)
        self._depth = 0
        self._prestage_hook = formation_prestage_hook
        self._plan_guard_hook = plan_guard_hook
        # Set to the leaf node that produced a False return; remediation
        # callers use this to brief the LLM about which step broke.
        self.failed_leaf: dict | None = None
        self.guard_failure: dict | None = None

    async def run(self, plan: dict) -> bool:
        self.failed_leaf = None
        self.guard_failure = None
        if self._plan_guard_hook is not None:
            plan, failure = self._plan_guard_hook(plan)
            if failure:
                self.guard_failure = failure
                failed_leaf = failure.get('failed_leaf')
                if isinstance(failed_leaf, dict):
                    self.failed_leaf = failed_leaf
                else:
                    self.failed_leaf = {'type': failure.get('leaf_type', 'mapf')}
                self._log(
                    '✗ plan guard rejected execution: '
                    f"{failure.get('reason') or failure.get('last_error', '')}"
                )
                return False
        return await self._execute(plan)

    async def _execute(self, node: dict) -> bool:
        t = node['type']
        if t == 'sequence':
            return await self._run_sequence(node)
        if t == 'parallel':
            return await self._run_parallel(node)
        return await self._run_leaf(node)

    async def _run_sequence(self, node: dict) -> bool:
        steps = node['steps']
        self._log(f"{'  '*self._depth}⟶ sequence ({len(steps)} steps)")
        self._depth += 1
        for i, step in enumerate(steps):
            self._log(f"{'  '*self._depth}step {i+1}/{len(steps)}")
            ok = await self._execute(step)
            if not ok:
                self._log(f"{'  '*self._depth}✗ step {i+1} failed — aborting")
                self._depth -= 1
                return False
        self._depth -= 1
        return True

    async def _run_parallel(self, node: dict) -> bool:
        """Flatten parallel into ordered commands and run sequentially.

        True parallelism at robot level is handled by the MAPF planner —
        merged mapf sends all robots in one goal, planner routes them
        without collisions. We don't need OS-level concurrency here.
        """
        try:
            commands = flatten_parallel(node)
        except PlanConflictError as exc:
            # Fail loudly rather than dropping a goal. The remediation loop
            # sees this text and can re-plan; a silently discarded goal cannot
            # be re-planned because nothing knows it went missing.
            self._log(f"{'  '*self._depth}✗ parallel rejected: {exc}")
            return False
        label = ' + '.join(
            f"{c['type']}({len(c.get('robot_ids',[]))}r)"
            if c['type'] == 'mapf' else c['type']
            for c in commands)
        self._log(f"{'  '*self._depth}⟹  parallel → [{label}]")
        self._depth += 1
        for cmd in commands:
            ok = await self._run_leaf(cmd)
            if not ok:
                self._depth -= 1
                return False
        self._depth -= 1
        return True

    async def _run_leaf(self, node: dict) -> bool:
        t = node['type']
        ind = '  ' * self._depth
        if t == 'idle':
            self._log(f'{ind}⬛ idle')
        elif t == 'mapf':
            n = len(node.get('robot_ids', []))
            self._log(f"{ind}🚀 mapf {n} robot{'s' if n!=1 else ''}: {node.get('reason','')}")
        elif t == 'disband':
            self._log(f"{ind}🔴 disband formation {node.get('formation_id','')}: {node.get('reason','')}")
        elif t == 'formation':
            # MANDATORY prompt rule and emits a bare formation leaf with
            # followers out of position, run the implied mapf staging step
            # first. Skips silently when no hook is configured or when the
            # hook says staging is unnecessary.
            if self._prestage_hook is not None:
                staging = self._prestage_hook(node)
                if staging is not None:
                    n = len(staging.get('robot_ids', []))
                    self._log(
                        f"{ind}🔧 auto-stage {n} follower"
                        f"{'s' if n!=1 else ''} before "
                        f"{node.get('formation_id','')}: "
                        f"{staging.get('reason','')}")
                    ok = await self._send(staging)
                    if not ok:
                        if self.failed_leaf is None:
                            self.failed_leaf = staging
                        return False
            self._log(
                f"{ind}🔷 formation {node.get('formation_id','')} "
                f"leader={node.get('leader_ns','')}: {node.get('reason','')}")
        ok = await self._send(node)
        if not ok and self.failed_leaf is None:
            self.failed_leaf = node
        return ok
