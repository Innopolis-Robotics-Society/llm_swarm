#!/usr/bin/env python3
"""
benchmark_ch3.py — offline channel-3 planning benchmark.

Calls the LLM directly (no ROS, no simulator).
Runs every test case from BENCHMARK.md and reports pass / fail.

Usage (from workspace root, source the workspace or set PYTHONPATH):
  python3 iros_llm_orchestrator/benchmark_ch3.py \\
      --map amongus \\
      --llm-mode ollama --llm-model qwen2.5:14b

  # Run a subset
  python3 iros_llm_orchestrator/benchmark_ch3.py \\
      --tests mapf_basic_01 formation_create_01 parallel_01

  # Infrastructure check (no LLM needed — verifies imports and prompt building)
  python3 iros_llm_orchestrator/benchmark_ch3.py --dry-run

Backends:
  ollama — local Ollama server  (--llm-endpoint / --llm-model)
  http   — OpenAI-compatible    (--llm-endpoint, LLM_API_KEY env var)
  local  — HuggingFace Transformers in-process

Note: the built-in 'mock' backend is a ch1/ch2 heuristic and does NOT produce
valid channel-3 JSON. Use --dry-run for infrastructure smoke tests.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable

# ── path bootstrap ─────────────────────────────────────────────────────────
# Makes the package importable from source without a colcon install.
_HERE = os.path.dirname(os.path.abspath(__file__))
_WORKSPACE = os.path.dirname(_HERE)
for _p in [_HERE, _WORKSPACE]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from iros_llm_orchestrator.common.llm_factory import get_llm_client   # noqa: E402
from iros_llm_orchestrator.common.plan_executor import parse_plan      # noqa: E402
from iros_llm_orchestrator.common.user_prompt import build_user_prompt # noqa: E402

# ── ANSI colours ───────────────────────────────────────────────────────────
_USE_COLOR = sys.stdout.isatty()

def _c(text: str, code: str) -> str:
    return f'\033[{code}m{text}\033[0m' if _USE_COLOR else text

GREEN  = lambda t: _c(t, '32')
RED    = lambda t: _c(t, '31')
YELLOW = lambda t: _c(t, '33')
CYAN   = lambda t: _c(t, '36')
BOLD   = lambda t: _c(t, '1')

# ═══════════════════════════════════════════════════════════════════════════
# Response parser (mirrors user_chat_node._parse_response without rclpy)
# ═══════════════════════════════════════════════════════════════════════════

def _parse_response(raw: str) -> tuple[str, dict]:
    text = raw.strip()
    start = text.find('{')
    if start == -1:
        raise ValueError('no JSON object in LLM output')
    depth = 0
    end = -1
    in_string = False
    escaped = False
    for i in range(start, len(text)):
        ch = text[i]
        if in_string:
            if escaped:
                escaped = False
            elif ch == '\\':
                escaped = True
            elif ch == '"':
                in_string = False
            continue
        if ch == '"':
            in_string = True
        elif ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                end = i + 1
                break
    if end == -1:
        raise ValueError('JSON object not closed')
    obj = json.loads(text[start:end])
    reply = obj.get('reply', text[:start].strip()) or text[:start].strip()
    plan = parse_plan(obj)
    return reply, plan

# ═══════════════════════════════════════════════════════════════════════════
# Validation helpers
# ═══════════════════════════════════════════════════════════════════════════

def _find_nodes(plan: dict, type_: str) -> list[dict]:
    out: list[dict] = []
    if plan.get('type') == type_:
        out.append(plan)
    for s in plan.get('steps', []):
        out.extend(_find_nodes(s, type_))
    return out

def _all_leaves(plan: dict) -> list[dict]:
    if plan.get('type') in ('mapf', 'formation', 'idle', 'disband'):
        return [plan]
    out: list[dict] = []
    for s in plan.get('steps', []):
        out.extend(_all_leaves(s))
    return out

def chk_type(plan: dict, expected: str) -> list[str]:
    t = plan.get('type', '')
    return [] if t == expected else [f'plan.type={t!r}, want {expected!r}']

def chk_robot_ids(node: dict, expected: list[int]) -> list[str]:
    got = sorted(node.get('robot_ids', []))
    want = sorted(expected)
    return [] if got == want else [f'robot_ids={got}, want {want}']

def chk_robot_ids_contain(node: dict, required: list[int]) -> list[str]:
    got = set(node.get('robot_ids', []))
    missing = [r for r in required if r not in got]
    return [] if not missing else [f'robot_ids missing {missing}']

def chk_goals_near(node: dict, cx: float, cy: float, radius: float = 4.0) -> list[str]:
    goals = node.get('goals', [])
    if not goals:
        return ['no goals']
    bad = [(round(g[0], 2), round(g[1], 2))
           for g in goals if math.hypot(g[0] - cx, g[1] - cy) > radius]
    return [] if not bad else [f'goals far from ({cx},{cy}) r={radius}: {bad}']

def chk_goals_distinct(node: dict, min_dist: float = 0.3) -> list[str]:
    goals = node.get('goals', [])
    for i, g1 in enumerate(goals):
        for g2 in goals[i + 1:]:
            if math.hypot(g1[0] - g2[0], g1[1] - g2[1]) < min_dist:
                return [f'duplicate goals: {g1} ~ {g2}']
    return []

def chk_formation(node: dict, leader: str | None = None,
                  min_followers: int = 1) -> list[str]:
    errs: list[str] = []
    if node.get('type') != 'formation':
        return [f'expected formation, got {node.get("type")!r}']
    if leader and node.get('leader_ns') != leader:
        errs.append(f'leader_ns={node.get("leader_ns")!r}, want {leader!r}')
    fn = node.get('follower_ns', [])
    if len(fn) < min_followers:
        errs.append(f'follower_ns has {len(fn)}, want >= {min_followers}')
    ox, oy = node.get('offsets_x', []), node.get('offsets_y', [])
    if not ox or not oy:
        errs.append('offsets_x or offsets_y missing/empty')
    elif len(ox) != len(fn) or len(oy) != len(fn):
        errs.append(f'offset length mismatch: ox={len(ox)} oy={len(oy)} fn={len(fn)}')
    return errs

def chk_idle_reason_prefix(plan: dict, prefix: str) -> list[str]:
    if plan.get('type') != 'idle':
        return [f'plan.type={plan.get("type")!r}, want idle']
    reason = plan.get('reason', '')
    if not reason.startswith(prefix):
        return [f'reason={reason!r} does not start with {prefix!r}']
    return []

def chk_leader_only(node: dict, leader_id: int,
                    followers: list[int]) -> list[str]:
    ids = node.get('robot_ids', [])
    errs: list[str] = []
    if leader_id not in ids:
        errs.append(f'leader {leader_id} not in robot_ids={ids}')
    bad = [r for r in followers if r in ids]
    if bad:
        errs.append(f'followers {bad} should not be in robot_ids (formation active)')
    return errs

def chk_parallel_has_mapf(plan: dict, ids: list[int],
                           cx: float, cy: float,
                           radius: float = 4.0) -> list[str]:
    """Inside a parallel, find a mapf step covering exactly `ids`."""
    if plan.get('type') not in ('parallel', 'sequence'):
        return [f'expected parallel/sequence, got {plan.get("type")!r}']
    steps = plan.get('steps', [])
    match = next(
        (s for s in steps
         if s.get('type') == 'mapf' and sorted(s.get('robot_ids', [])) == sorted(ids)),
        None,
    )
    if match is None:
        return [f'no mapf step for robot_ids={sorted(ids)}']
    return chk_goals_near(match, cx, cy, radius)

def chk_staging_has_no_leader(plan: dict, leader_id: int) -> list[str]:
    """Find the first mapf that contains followers and verify leader not in it."""
    mapf_nodes = _find_nodes(plan, 'mapf')
    # The staging mapf should not contain the leader
    staging_candidates = [m for m in mapf_nodes if leader_id not in m.get('robot_ids', [])]
    if not staging_candidates:
        return [f'no staging mapf found (every mapf includes leader {leader_id})']
    return []

def chk_has_disband(plan: dict, formation_id: str | None = None) -> list[str]:
    nodes = _find_nodes(plan, 'disband')
    if not nodes:
        return ['no disband step in plan']
    if formation_id:
        if not any(n.get('formation_id') == formation_id for n in nodes):
            ids = [n.get('formation_id') for n in nodes]
            return [f'disband found but formation_id={ids}, want {formation_id!r}']
    return []

# ═══════════════════════════════════════════════════════════════════════════
# Runtime contexts
# ═══════════════════════════════════════════════════════════════════════════

def _robot(x: float, y: float, yaw: float = 0.0) -> dict:
    return {'x': x, 'y': y, 'yaw': yaw, 'stale_ms': 80, 'stale': False}

# All robots at approximate spawn positions (2×2 cluster per group)
_SPAWN_ROBOTS: dict[str, dict] = {
    '0': _robot(-22.4, 9.4),  '1': _robot(-21.4, 9.4),   # cyan
    '2': _robot(-22.4, 10.4), '3': _robot(-21.4, 10.4),
    '4': _robot(-9.2, -6.4),  '5': _robot(-8.2, -6.4),   # magenta
    '6': _robot(-9.2, -5.4),  '7': _robot(-8.2, -5.4),
    '8': _robot(28.0, 1.4),   '9': _robot(29.0, 1.4),    # green
    '10': _robot(28.0, 2.4),  '11': _robot(29.0, 2.4),
    '12': _robot(0.1, -11.4), '13': _robot(1.1, -11.4),  # orange
    '14': _robot(0.1, -10.4), '15': _robot(1.1, -10.4),
    '16': _robot(17.1, 10.2), '17': _robot(18.1, 10.2),  # yellow
    '18': _robot(17.1, 11.2), '19': _robot(18.1, 11.2),
}

def ctx_spawn() -> dict:
    return {'source': 'benchmark', 'robots': dict(_SPAWN_ROBOTS), 'formations': []}

def ctx_no_robot12() -> dict:
    robots = {k: v for k, v in _SPAWN_ROBOTS.items() if k != '12'}
    return {'source': 'benchmark', 'robots': robots, 'formations': []}

def ctx_magenta_line_stable() -> dict:
    """Magenta line active; robot_4 led to cafeteria, followers in line."""
    robots = dict(_SPAWN_ROBOTS)
    robots.update({
        '4': _robot(2.7,  10.1),   # leader at cafeteria
        '5': _robot(1.2,  10.1),   # -1.5 x behind
        '6': _robot(-0.3, 10.1),   # -3.0 x behind
        '7': _robot(-1.8, 10.1),   # -4.5 x behind
    })
    return {
        'source': 'benchmark',
        'robots': robots,
        'formations': [{
            'formation_id': 'magenta_line',
            'leader_ns': 'robot_4',
            'followers': ['robot_5', 'robot_6', 'robot_7'],
            'status': 'STABLE',
        }],
    }

def ctx_cyan_wedge_stable() -> dict:
    """Cyan wedge active; robot_0 led somewhere, followers in wedge."""
    robots = dict(_SPAWN_ROBOTS)
    robots.update({
        '0': _robot(2.7,  10.1),
        '1': _robot(1.7,  10.7),
        '2': _robot(1.7,   9.5),
        '3': _robot(0.7,  10.1),
    })
    return {
        'source': 'benchmark',
        'robots': robots,
        'formations': [{
            'formation_id': 'cyan_wedge',
            'leader_ns': 'robot_0',
            'followers': ['robot_1', 'robot_2', 'robot_3'],
            'status': 'STABLE',
        }],
    }

def ctx_magenta_line_and_status() -> dict:
    """escalate_02: formations visible in context for state question."""
    return ctx_magenta_line_stable()

def ctx_robot4_at_cafeteria() -> dict:
    """escalate_03: robot_4 position visible for where-is query."""
    robots = dict(_SPAWN_ROBOTS)
    robots['4'] = _robot(2.7, 10.1)
    return {'source': 'benchmark', 'robots': robots, 'formations': []}

# ═══════════════════════════════════════════════════════════════════════════
# Test-case definitions
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class TC:
    id: str
    prompt: str
    validate: Callable[[dict, str], list[str]]
    context: dict | None = None  # None → no runtime context injected

    def run(self, plan: dict, reply: str) -> list[str]:
        try:
            return self.validate(plan, reply)
        except Exception as exc:
            return [f'validator error: {exc}']


def _tc(id_: str, prompt: str, fn, ctx=None) -> TC:
    return TC(id=id_, prompt=prompt, validate=fn, context=ctx)


TEST_CASES: list[TC] = [

    # ── mapf_basic ───────────────────────────────────────────────────────
    _tc('mapf_basic_01', 'send cyan to cafeteria', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, [0,1,2,3]) +
        chk_goals_near(p, 2.7, 10.1),
        ctx=ctx_spawn()),

    _tc('mapf_basic_02', 'orange robots go home', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, [12,13,14,15]) +
        chk_goals_near(p, 0.6, -10.9),
        ctx=ctx_spawn()),

    _tc('mapf_basic_03', 'move all robots to cafeteria', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, list(range(20))) +
        chk_goals_near(p, 2.7, 10.1, 5.0),
        ctx=ctx_spawn()),

    _tc('mapf_basic_04', 'drive yellow to navigation', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, [16,17,18,19]) +
        chk_goals_near(p, 29.5, 1.9),
        ctx=ctx_spawn()),

    _tc('mapf_basic_05', 'green to reactor', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, [8,9,10,11]) +
        chk_goals_near(p, -28.4, 0.9),
        ctx=ctx_spawn()),

    # ── mapf_spread ──────────────────────────────────────────────────────
    _tc('mapf_spread_01', 'place cyan at storage', lambda p, r:
        chk_type(p, 'mapf') or
        chk_robot_ids(p, [0,1,2,3]) +
        chk_goals_near(p, 0.6, -10.9),
        ctx=ctx_spawn()),

    _tc('mapf_spread_02',
        'drive magenta to cafeteria, locate these robots in cafeteria explicitly',
        lambda p, r: (
            chk_type(p, 'mapf') or
            chk_robot_ids(p, [4,5,6,7]) +
            chk_goals_near(p, 2.7, 10.1)
        ),
        ctx=ctx_spawn()),

    # ── mapf_explicit ────────────────────────────────────────────────────
    _tc('mapf_explicit_01',
        'place robot_12 at (1.0, -10.9), robot_13 at (0.0, -10.9), robot_14 at (1.0, -11.9)',
        lambda p, r: (
            chk_type(p, 'mapf') or (
                chk_robot_ids_contain(p, [12, 13, 14]) +
                (['spread should be False for explicit placement']
                 if p.get('spread') else []) +
                ([f'expected 3 goals, got {len(p.get("goals",[]))}']
                 if len(p.get('goals', [])) != 3 else []) +
                chk_goals_distinct(p)
            )
        ),
        ctx=ctx_spawn()),

    _tc('mapf_explicit_02', 'place cyan in a 2x2 grid at cafeteria', lambda p, r: (
        chk_type(p, 'mapf') or (
            chk_robot_ids(p, [0,1,2,3]) +
            ([f'expected 4 goals, got {len(p.get("goals",[]))}']
             if len(p.get('goals', [])) != 4 else []) +
            chk_goals_distinct(p) +
            chk_goals_near(p, 2.7, 10.1, 5.0)
        )
    ), ctx=ctx_spawn()),

    # ── parallel ─────────────────────────────────────────────────────────
    _tc('parallel_01', 'yellow to navigation, green to reactor', lambda p, r: (
        chk_type(p, 'parallel') or (
            chk_parallel_has_mapf(p, [16,17,18,19], 29.5, 1.9) +
            chk_parallel_has_mapf(p, [8,9,10,11], -28.4, 0.9)
        )
    ), ctx=ctx_spawn()),

    _tc('parallel_02', 'cyan and magenta both go to cafeteria', lambda p, r: (
        chk_type(p, 'parallel') or (
            chk_parallel_has_mapf(p, [0,1,2,3], 2.7, 10.1) +
            chk_parallel_has_mapf(p, [4,5,6,7], 2.7, 10.1)
        )
    ), ctx=ctx_spawn()),

    _tc('parallel_03', 'magenta to green home, green to magenta home', lambda p, r: (
        chk_type(p, 'parallel') or (
            chk_parallel_has_mapf(p, [4,5,6,7],  28.5, 1.9) +   # green home
            chk_parallel_has_mapf(p, [8,9,10,11], -8.7, -5.9)   # magenta home
        )
    ), ctx=ctx_spawn()),

    _tc('parallel_04', 'send all five groups to their home positions', lambda p, r: (
        chk_type(p, 'parallel') or (
            chk_parallel_has_mapf(p, [0,1,2,3],   -21.9,  9.9) +
            chk_parallel_has_mapf(p, [4,5,6,7],    -8.7, -5.9) +
            chk_parallel_has_mapf(p, [8,9,10,11],  28.5,  1.9) +
            chk_parallel_has_mapf(p, [12,13,14,15], 0.6, -10.9) +
            chk_parallel_has_mapf(p, [16,17,18,19], 17.6,  10.7)
        )
    ), ctx=ctx_spawn()),

    # ── sequence ─────────────────────────────────────────────────────────
    _tc('sequence_01', 'cyan go to cafeteria, then form a wedge', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_parallel_has_mapf(p, [0,1,2,3], 2.7, 10.1) +   # mapf somewhere in plan
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_0', min_followers=3)
             if _find_nodes(p, 'formation') else [])
        )
    ), ctx=ctx_spawn()),

    _tc('sequence_02', 'move orange to cafeteria, then send them to storage', lambda p, r: (
        chk_type(p, 'sequence') or (
            ([] if len(p.get('steps', [])) >= 2
             else [f'sequence has {len(p.get("steps",[]))} steps, want >= 2']) +
            (chk_robot_ids(p['steps'][0], [12,13,14,15]) +
             chk_goals_near(p['steps'][0], 2.7, 10.1) +
             chk_robot_ids(p['steps'][1], [12,13,14,15]) +
             chk_goals_near(p['steps'][1], 0.6, -10.9)
             if len(p.get('steps', [])) >= 2 else [])
        )
    ), ctx=ctx_spawn()),

    _tc('sequence_03',
        'cyan and magenta to cafeteria, then cyan forms a line while magenta goes home',
        lambda p, r: (
            chk_type(p, 'sequence') or (
                ([] if _find_nodes(p, 'formation') else ['no formation step in plan']) +
                (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_0')
                 if _find_nodes(p, 'formation') else []) +
                ([] if any(
                    s.get('type') == 'mapf' and
                    set(s.get('robot_ids', [])) == {4, 5, 6, 7}
                    for s in _all_leaves(p)
                ) else ['no mapf step covering magenta [4-7] for home'])
            )
        ), ctx=ctx_spawn()),

    _tc('sequence_04',
        'move green to cafeteria, after that to navigation, then stop',
        lambda p, r: (
            chk_type(p, 'sequence') or (
                ([f'sequence has {len(p.get("steps",[]))} steps, want >= 3']
                 if len(p.get('steps', [])) < 3 else []) +
                ([] if _find_nodes(p, 'idle') else ['no idle step at end'])
            )
        ), ctx=ctx_spawn()),

    # ── formation_create ─────────────────────────────────────────────────
    _tc('formation_create_01', 'magenta form a line', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=4) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_4', min_followers=3)
             if _find_nodes(p, 'formation') else [])
        )
    ), ctx=ctx_spawn()),

    _tc('formation_create_02', 'orange make a triangle formation', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=12) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_12', min_followers=1)
             if _find_nodes(p, 'formation') else [])
        )
    ), ctx=ctx_spawn()),

    _tc('formation_create_03', 'cyan form a wedge', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=0) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (lambda f: (
                chk_formation(f, leader='robot_0', min_followers=3) +
                ([] if f.get('offsets_x', [None])[0] is not None
                       and f['offsets_x'][0] < 0
                 else ['offsets_x[0] should be negative (behind leader)'])
            ))(_find_nodes(p, 'formation')[0])
            if _find_nodes(p, 'formation') else ['no formation step']
        )
    ), ctx=ctx_spawn()),

    _tc('formation_create_04', 'green form an abreast line', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=8) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (lambda f: (
                chk_formation(f, leader='robot_8', min_followers=3) +
                ([] if all(abs(ox) < 0.2 for ox in f.get('offsets_x', [1]))
                 else ['abreast: offsets_x should all be ~0 (side by side)']) +
                ([] if any(abs(oy) > 0.5 for oy in f.get('offsets_y', []))
                 else ['abreast: offsets_y should be non-zero'])
            ))(_find_nodes(p, 'formation')[0])
            if _find_nodes(p, 'formation') else ['no formation step']
        )
    ), ctx=ctx_spawn()),

    _tc('formation_create_05', 'form a line with orange, leader should be robot_15', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=15) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (lambda f: (
                chk_formation(f, leader='robot_15', min_followers=3) +
                ([f'robot_15 must not be in staging mapf robot_ids']
                 if any(15 in m.get('robot_ids', []) for m in _find_nodes(p, 'mapf'))
                 else [])
            ))(_find_nodes(p, 'formation')[0])
            if _find_nodes(p, 'formation') else ['no formation step']
        )
    ), ctx=ctx_spawn()),

    # ── formation_move ────────────────────────────────────────────────────
    _tc('formation_move_01', 'send the magenta line to storage', lambda p, r: (
        chk_type(p, 'mapf') or (
            chk_leader_only(p, leader_id=4, followers=[5,6,7]) +
            chk_goals_near(p, 0.6, -10.9)
        )
    ), ctx=ctx_magenta_line_stable()),

    _tc('formation_move_02', 'move cyan wedge to reactor', lambda p, r: (
        chk_type(p, 'mapf') or (
            chk_leader_only(p, leader_id=0, followers=[1,2,3]) +
            chk_goals_near(p, -28.4, 0.9)
        )
    ), ctx=ctx_cyan_wedge_stable()),

    _tc('formation_move_03', 'magenta line, head to cafeteria', lambda p, r: (
        chk_type(p, 'sequence') or (
            ([f'sequence has {len(p.get("steps",[]))} steps, want 3']
             if len(p.get('steps', [])) != 3 else []) +
            (chk_staging_has_no_leader(p, leader_id=4) +
             ([] if _find_nodes(p, 'formation') else ['no formation step']) +
             (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_4')
              if _find_nodes(p, 'formation') else []) +
             (lambda last: (
                 chk_type(last, 'mapf') +
                 chk_leader_only(last, leader_id=4, followers=[5,6,7]) +
                 chk_goals_near(last, 2.7, 10.1)
             ))(p['steps'][-1])
             if len(p.get('steps', [])) >= 3 else [])
        )
    ), ctx=ctx_spawn()),

    # ── disband ───────────────────────────────────────────────────────────
    _tc('disband_01', 'disband the magenta line', lambda p, r:
        chk_has_disband(p, 'magenta_line'),
        ctx=ctx_magenta_line_stable()),

    _tc('disband_02', 'break the magenta formation and send all four to storage', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_has_disband(p, 'magenta_line') +
            ([] if any(
                s.get('type') == 'mapf' and
                set(s.get('robot_ids', [])) == {4,5,6,7}
                for s in _all_leaves(p)
            ) else ['no mapf covering all magenta [4-7]']) +
            (lambda mapfs: (
                chk_goals_near(mapfs[0], 0.6, -10.9) if mapfs else []
            ))([s for s in _all_leaves(p)
                if s.get('type') == 'mapf' and set(s.get('robot_ids',[])) == {4,5,6,7}])
        )
    ), ctx=ctx_magenta_line_stable()),

    _tc('disband_03',
        'cancel the cyan wedge, then split: robot_0 to cafeteria, rest to storage',
        lambda p, r: (
            chk_type(p, 'sequence') or (
                chk_has_disband(p, 'cyan_wedge') +
                ([] if any(
                    s.get('type') == 'mapf' and 0 in s.get('robot_ids', [])
                    and len(s.get('robot_ids', [])) == 1
                    for s in _all_leaves(p)
                ) else ['no solo mapf for robot_0']) +
                ([] if any(
                    s.get('type') == 'mapf' and
                    set(s.get('robot_ids', [])) == {1,2,3}
                    for s in _all_leaves(p)
                ) else ['no mapf for followers {1,2,3} to storage'])
            )
        ), ctx=ctx_cyan_wedge_stable()),

    # ── formation_complex ─────────────────────────────────────────────────
    _tc('formation_complex_01', 'orange make a triangle and move to cafeteria', lambda p, r: (
        chk_type(p, 'sequence') or (
            chk_staging_has_no_leader(p, leader_id=12) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_12')
             if _find_nodes(p, 'formation') else []) +
            ([] if any(
                s.get('type') == 'mapf' and 12 in s.get('robot_ids', []) and
                len(s.get('robot_ids', [])) == 1
                for s in _all_leaves(p)
            ) else ['no mapf for leader-only move to cafeteria']) +
            (lambda final_mapfs: (
                chk_goals_near(final_mapfs[0], 2.7, 10.1) if final_mapfs else []
            ))([s for s in _all_leaves(p)
                if s.get('type') == 'mapf' and s.get('robot_ids') == [12]])
        )
    ), ctx=ctx_spawn()),

    _tc('formation_complex_02', 'yellow form a wedge at cafeteria', lambda p, r: (
        chk_type(p, 'sequence') or (
            ([] if any(
                s.get('type') == 'mapf' and
                set(s.get('robot_ids', [])) == {16,17,18,19}
                for s in _all_leaves(p)
            ) else ['no group mapf to cafeteria for yellow']) +
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_16', min_followers=3)
             if _find_nodes(p, 'formation') else [])
        )
    ), ctx=ctx_spawn()),

    _tc('formation_complex_03', 'cyan go to cafeteria, form a wedge, then go to reactor', lambda p, r: (
        chk_type(p, 'sequence') or (
            ([] if _find_nodes(p, 'formation') else ['no formation step']) +
            (chk_formation(_find_nodes(p, 'formation')[0], leader='robot_0')
             if _find_nodes(p, 'formation') else []) +
            ([] if any(
                s.get('type') == 'mapf' and s.get('robot_ids') == [0]
                for s in _all_leaves(p)
            ) else ['no leader-only mapf to reactor']) +
            (lambda final_mapfs: (
                chk_goals_near(final_mapfs[0], -28.4, 0.9) if final_mapfs else []
            ))([s for s in _all_leaves(p)
                if s.get('type') == 'mapf' and s.get('robot_ids') == [0]])
        )
    ), ctx=ctx_spawn()),

    # ── idle ──────────────────────────────────────────────────────────────
    _tc('idle_01', 'stop', lambda p, r:
        chk_type(p, 'idle') + (['reason empty'] if not p.get('reason') else []),
        ctx=ctx_spawn()),

    _tc('idle_02', 'halt everything', lambda p, r:
        chk_type(p, 'idle') + (['reason empty'] if not p.get('reason') else []),
        ctx=ctx_spawn()),

    _tc('idle_03', 'cancel all missions', lambda p, r:
        chk_type(p, 'idle') + (['reason empty'] if not p.get('reason') else []),
        ctx=ctx_spawn()),

    # ── escalate ─────────────────────────────────────────────────────────
    _tc('escalate_01',
        'orange make a triangle formation',
        lambda p, r: chk_idle_reason_prefix(p, 'needs_help:'),
        ctx=ctx_no_robot12()),

    _tc('escalate_02', 'what are the robots doing', lambda p, r: (
        chk_idle_reason_prefix(p, 'reply_only:') +
        (['reply mentions magenta_line but none found']
         if 'magenta' not in r.lower() else [])
    ), ctx=ctx_magenta_line_and_status()),

    _tc('escalate_03', 'where is robot_4', lambda p, r: (
        chk_idle_reason_prefix(p, 'reply_only:') +
        ([] if any(
            tok in r for tok in ['2.7', '10.1', 'cafeteria', 'кафетерий', 'кафетери']
        ) else ['reply should mention robot_4 position near cafeteria'])
    ), ctx=ctx_robot4_at_cafeteria()),

    _tc('escalate_04', 'form a triangle', lambda p, r: (
        (chk_idle_reason_prefix(p, 'needs_help:') or
         chk_idle_reason_prefix(p, 'clarify:'))
    ), ctx=ctx_spawn()),

    # ── edge ──────────────────────────────────────────────────────────────
    _tc('edge_01', 'send all robots home', lambda p, r: (
        chk_type(p, 'parallel') or (
            chk_parallel_has_mapf(p, [0,1,2,3],   -21.9,  9.9) +
            chk_parallel_has_mapf(p, [4,5,6,7],    -8.7, -5.9) +
            chk_parallel_has_mapf(p, [8,9,10,11],  28.5,  1.9) +
            chk_parallel_has_mapf(p, [12,13,14,15], 0.6, -10.9) +
            chk_parallel_has_mapf(p, [16,17,18,19], 17.6,  10.7)
        )
    ), ctx=ctx_spawn()),

    _tc('edge_02',
        'orange make a triangle formation',
        lambda p, r: chk_idle_reason_prefix(p, 'needs_help:'),
        ctx=ctx_no_robot12()),

    _tc('edge_03', 'put magenta right behind the cyan wedge leader', lambda p, r: (
        chk_type(p, 'mapf') or (
            chk_robot_ids(p, [4,5,6,7]) +
            chk_goals_near(p, 2.7, 10.1, 6.0)  # near robot_0
        )
    ), ctx=ctx_cyan_wedge_stable()),

    _tc('edge_04', 'move the formation to storage', lambda p, r: (
        chk_type(p, 'mapf') or (
            chk_leader_only(p, leader_id=4, followers=[5,6,7]) +
            chk_goals_near(p, 0.6, -10.9)
        )
    ), ctx=ctx_magenta_line_stable()),

    _tc('edge_05', 'green and yellow go to cafeteria, then form separate wedges',
        lambda p, r: (
            chk_type(p, 'sequence') or (
                ([f'sequence has {len(p.get("steps",[]))} steps, want >= 2']
                 if len(p.get('steps', [])) < 2 else []) +
                ([] if len(_find_nodes(p, 'formation')) >= 2
                 else [f'need 2 formation steps, found {len(_find_nodes(p,"formation"))}'])
            )
        ), ctx=ctx_spawn()),

    _tc('edge_06', 'repeat the last command', lambda p, r: (
        chk_idle_reason_prefix(p, 'needs_help:') or
        chk_idle_reason_prefix(p, 'clarify:')
    ), ctx=ctx_spawn()),
]

# ═══════════════════════════════════════════════════════════════════════════
# LLM call
# ═══════════════════════════════════════════════════════════════════════════

async def _call_llm(llm, messages: list[dict], timeout: float) -> str:
    full = ''
    async def _stream():
        nonlocal full
        async for chunk in llm.stream(messages):
            full += chunk or ''
    await asyncio.wait_for(_stream(), timeout=timeout)
    return full

# ═══════════════════════════════════════════════════════════════════════════
# Runner
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class Result:
    tc: TC
    passed: bool
    errors: list[str]
    reply: str
    raw: str
    elapsed: float
    parse_error: str = ''
    repeat_idx: int = 0

    def to_json(self) -> dict:
        return {
            'id': self.tc.id,
            'prompt': self.tc.prompt,
            'repeat_idx': self.repeat_idx,
            'passed': self.passed,
            'errors': self.errors,
            'reply': self.reply,
            'raw': self.raw,
            'elapsed_sec': round(self.elapsed, 3),
            'parse_error': self.parse_error,
        }


_DRY_RUN_RESPONSE = '{"reply":"dry-run","plan":{"type":"idle","reason":"dry-run:ok"}}'


async def run_tests(
    test_cases: list[TC],
    llm,
    map_name: str,
    timeout: float,
    verbose: bool,
    dry_run: bool = False,
    repeat: int = 1,
) -> list[Result]:
    results: list[Result] = []
    width = max(len(tc.id) for tc in test_cases)

    for tc in test_cases:
        for rep in range(repeat):
            t0 = time.monotonic()
            raw = ''
            reply = ''
            plan: dict | None = None
            parse_err = ''

            try:
                messages = build_user_prompt(
                    tc.prompt,
                    map_name=map_name,
                    runtime_context=tc.context,
                )
                if dry_run:
                    raw = _DRY_RUN_RESPONSE
                else:
                    raw = await _call_llm(llm, messages, timeout)
                reply, plan = _parse_response(raw)
            except asyncio.TimeoutError:
                parse_err = f'TIMEOUT after {timeout}s'
            except Exception as exc:
                parse_err = str(exc)

            elapsed = time.monotonic() - t0
            rep_tag = f'[{rep+1}/{repeat}]' if repeat > 1 else ''

            if dry_run:
                # Dry-run: only report prompt-build success, skip validation
                passed = not parse_err
                status_str = GREEN('OK  ') if passed else RED('ERR ')
                tag = CYAN(tc.id.ljust(width))
                print(f'  {status_str}  {tag} {rep_tag}  {tc.prompt[:55]}')
                if not passed:
                    print(f'       {RED("↳")} {parse_err}')
                results.append(Result(tc=tc, passed=passed, errors=[],
                                      reply=reply, raw=raw, elapsed=elapsed,
                                      parse_error=parse_err, repeat_idx=rep))
                continue

            if parse_err:
                errors = [f'parse/call error: {parse_err}']
                passed = False
            elif plan is not None:
                errors = tc.run(plan, reply)
                passed = not errors
            else:
                errors = ['no plan']
                passed = False

            r = Result(tc=tc, passed=passed, errors=errors,
                       reply=reply, raw=raw, elapsed=elapsed,
                       parse_error=parse_err, repeat_idx=rep)
            results.append(r)

            # Live progress line
            status = GREEN('PASS') if passed else RED('FAIL')
            tag = CYAN(tc.id.ljust(width))
            time_str = YELLOW(f'{elapsed:5.1f}s')
            print(f'  {status}  {tag} {rep_tag}  {time_str}  {tc.prompt[:55]}')
            if not passed:
                for e in errors:
                    print(f'       {RED("↳")} {e}')
            if verbose and raw:
                print(f'       raw: {raw[:200].replace(chr(10)," ")}')
                if reply:
                    print(f'       reply: {reply[:100]}')

    return results


def _print_summary(results: list[Result], dry_run: bool = False, repeat: int = 1) -> None:
    total  = len(results)
    passed = sum(1 for r in results if r.passed)
    failed = total - passed
    print()
    print(BOLD('─' * 60))
    if dry_run:
        if failed == 0:
            print(BOLD(GREEN(f'  DRY-RUN OK — {total} prompts built successfully')))
        else:
            print(BOLD(RED(f'  DRY-RUN: {failed}/{total} prompts failed to build')))
        print(BOLD('─' * 60))
        return

    if failed == 0:
        print(BOLD(GREEN(f'  ALL {total} TESTS PASSED')))
    else:
        print(BOLD(f'  {GREEN(str(passed))} / {total} passed   '
                   f'{RED(str(failed))} failed'))
        print()
        print(BOLD('  Failed tests:'))
        for r in results:
            if not r.passed:
                print(f'    {RED("✗")} {r.tc.id} [rep {r.repeat_idx+1}/{repeat}]')
                for e in r.errors:
                    print(f'        {e}')

    if repeat > 1:
        # pass@1 (first sample only) and pass@repeat (any-of-N per case)
        by_case: dict[str, list[Result]] = {}
        for r in results:
            by_case.setdefault(r.tc.id, []).append(r)
        n_cases = len(by_case)
        pass_at_1 = sum(1 for rs in by_case.values() if rs[0].passed)
        pass_at_n = sum(1 for rs in by_case.values() if any(r.passed for r in rs))
        print()
        print(BOLD(f'  pass@1  = {pass_at_1}/{n_cases} ({100*pass_at_1/n_cases:.1f}%)'))
        print(BOLD(f'  pass@{repeat} = {pass_at_n}/{n_cases} ({100*pass_at_n/n_cases:.1f}%)'))
    print(BOLD('─' * 60))


def _write_json(path: str, results: list[Result], args) -> None:
    payload = {
        'map': args.map,
        'llm_mode': args.llm_mode,
        'llm_model': args.llm_model,
        'llm_temperature': args.llm_temperature,
        'llm_num_ctx': args.llm_num_ctx,
        'repeat': args.repeat,
        'total': len(results),
        'passed': sum(1 for r in results if r.passed),
        'results': [r.to_json() for r in results],
    }
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    print(f'\n{BOLD("wrote")} {path}')


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Offline channel-3 LLM planning benchmark')
    parser.add_argument('--map',          default='amongus',
                        help='Map name (default: amongus)')
    parser.add_argument('--llm-mode',     default='mock',
                        choices=['mock', 'ollama', 'http', 'local'],
                        help='LLM backend (default: mock)')
    parser.add_argument('--llm-endpoint', default='',
                        help='Backend endpoint URL (ollama/http)')
    parser.add_argument('--llm-model',    default='',
                        help='Model name')
    parser.add_argument('--llm-max-tokens', type=int, default=2048)
    parser.add_argument('--llm-temperature', type=float, default=0.1)
    parser.add_argument('--llm-num-ctx',  type=int, default=32768)
    parser.add_argument('--timeout',      type=float, default=60.0,
                        help='Per-test LLM timeout in seconds (default: 60)')
    parser.add_argument('--tests',        nargs='+', metavar='ID',
                        help='Run only these test IDs')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Print raw LLM output for each test')
    parser.add_argument('--no-color',     action='store_true')
    parser.add_argument('--dry-run',      action='store_true',
                        help='Build prompts but skip LLM call (infrastructure check)')
    parser.add_argument('--repeat', type=int, default=1,
                        help='Run each test case N times independently (for pass@1/pass@N)')
    parser.add_argument('--json', metavar='PATH', default='',
                        help='Write full structured results (incl. raw LLM output) to PATH')
    args = parser.parse_args()

    global _USE_COLOR
    if args.no_color:
        _USE_COLOR = False

    # Select test cases
    all_ids = {tc.id for tc in TEST_CASES}
    if args.tests:
        unknown = set(args.tests) - all_ids
        if unknown:
            print(f'Unknown test IDs: {sorted(unknown)}')
            print(f'Available: {sorted(all_ids)}')
            sys.exit(1)
        selected = [tc for tc in TEST_CASES if tc.id in args.tests]
    else:
        selected = list(TEST_CASES)

    llm = None
    if not args.dry_run:
        endpoint = args.llm_endpoint or None
        llm = get_llm_client(
            mode=args.llm_mode,
            endpoint=endpoint,
            model=args.llm_model,
            max_tokens=args.llm_max_tokens,
            temperature=args.llm_temperature,
            num_ctx=args.llm_num_ctx,
            timeout=args.timeout,
        )

    mode_str = 'dry-run' if args.dry_run else f'{args.llm_mode}/{args.llm_model or "default"}'
    repeat_str = f'  repeat={args.repeat}' if args.repeat > 1 else ''
    print(BOLD(f'\nChannel-3 benchmark  map={args.map}  '
               f'mode={mode_str}  '
               f'tests={len(selected)}/{len(TEST_CASES)}{repeat_str}'))
    print(BOLD('─' * 60))

    results = asyncio.run(run_tests(
        selected, llm, args.map, args.timeout, args.verbose,
        dry_run=args.dry_run, repeat=args.repeat))

    _print_summary(results, dry_run=args.dry_run, repeat=args.repeat)
    if args.json:
        _write_json(args.json, results, args)
    failed = sum(1 for r in results if not r.passed)
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
