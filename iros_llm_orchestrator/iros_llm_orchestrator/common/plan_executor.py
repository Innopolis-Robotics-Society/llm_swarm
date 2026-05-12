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
import re
from typing import Awaitable, Callable


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

_LEAF_TYPES      = {'mapf', 'formation', 'idle'}
_CONTAINER_TYPES = {'sequence', 'parallel'}
_ALL_TYPES       = _LEAF_TYPES | _CONTAINER_TYPES


def parse_plan(raw: str | dict) -> dict:
    """Parse and validate a plan tree. Raises ValueError on any error."""
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
        if not ids:
            raise ValueError(f'{path}: mapf requires non-empty robot_ids')
        if len(ids) != len(goals):
            raise ValueError(
                f'{path}: robot_ids({len(ids)}) != goals({len(goals)})')
        for g in goals:
            if not (isinstance(g, (list, tuple)) and len(g) >= 2):
                raise ValueError(f'{path}: each goal must be [x, y]')
    elif node_type == 'formation':
        if not node.get('formation_id'):
            raise ValueError(f'{path}: formation requires formation_id')
        if not node.get('leader_ns'):
            raise ValueError(f'{path}: formation requires leader_ns')
    return node


# ---------------------------------------------------------------------------
# Parallel flattening
# ---------------------------------------------------------------------------

def flatten_parallel(node: dict) -> list[dict]:
    """Flatten a parallel node into an ordered list of commands to execute.

    Rules:
    - Multiple mapf leaves → merge into ONE mapf (MAPF planner handles
      multi-robot coordination; no need to send two separate commands).
    - idle in any branch → return [idle] immediately (stop takes priority).
    - mixed mapf+formation → keep separate, execute as mini-sequence
      (formation requires its own BT mode, can't run simultaneously with mapf).
    - Nested containers inside parallel → recurse.
    """
    assert node['type'] == 'parallel'

    # Collect all leaves by walking the steps
    flat: list[dict] = []
    for step in node['steps']:
        flat.extend(_collect_leaves(step))

    # idle takes priority
    if any(n['type'] == 'idle' for n in flat):
        return [{'type': 'idle', 'reason': 'idle in parallel branch'}]

    # Separate mapf and non-mapf
    mapf_leaves      = [n for n in flat if n['type'] == 'mapf']
    non_mapf_leaves  = [n for n in flat if n['type'] != 'mapf']

    result: list[dict] = []

    # Merge all mapf leaves into one. If the same robot id shows up in two
    # branches (LLM occasionally produces this when an operator phrases the
    # same group twice), last-write-wins — the planner will reject duplicate
    # agents and we'd rather submit a coherent goal than fail the leaf.
    if mapf_leaves:
        merged: dict[int, list] = {}
        reasons: list[str]      = []
        for leaf in mapf_leaves:
            ids   = [int(r) for r in leaf.get('robot_ids', [])]
            goals = [[float(g[0]), float(g[1])] for g in leaf.get('goals', [])]
            for rid, g in zip(ids, goals):
                merged[rid] = g
            if leaf.get('reason'):
                reasons.append(leaf['reason'])
        result.append({
            'type':      'mapf',
            'robot_ids': list(merged.keys()),
            'goals':     list(merged.values()),
            'reason':    ' + '.join(reasons),
        })

    # Non-mapf (formation etc.) run after the merged mapf
    result.extend(non_mapf_leaves)
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


class PlanExecutor:
    def __init__(
        self,
        send_fn: SendFn,
        log_fn: LogFn | None = None,
        *,
        formation_prestage_hook: PrestageHook | None = None,
    ):
        self._send  = send_fn
        self._log   = log_fn or (lambda _: None)
        self._depth = 0
        self._prestage_hook = formation_prestage_hook
        # Set to the leaf node that produced a False return; remediation
        # callers use this to brief the LLM about which step broke.
        self.failed_leaf: dict | None = None

    async def run(self, plan: dict) -> bool:
        self.failed_leaf = None
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
        commands = flatten_parallel(node)
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
        elif t == 'formation':
            # Server-side staging safety net: even when the LLM ignores the
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