"""Prompt builder for user chat interface (channel 3).

Loads map context and system prompt from prompts/ directory.
All few-shot examples use the {"reply":"...", "plan":{...}} format.
"""

import json
import math
import os
from functools import lru_cache
from typing import Any

from iros_llm_orchestrator.common.execution_repair import compact_verification_json

try:
    import yaml as _yaml
    _YAML_OK = True
except ImportError:
    _YAML_OK = False

try:
    from ament_index_python.packages import get_package_share_directory
    _AMENT_OK = True
except ImportError:
    _AMENT_OK = False


# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------

def _prompts_dir() -> str:
    if _AMENT_OK:
        try:
            return os.path.join(
                get_package_share_directory('iros_llm_orchestrator'), 'prompts')
        except Exception:
            pass
    # Without ament: common/ → package-root/prompts/ (2 levels up, not 3)
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', 'prompts'))


def _map_descriptions_dir() -> str:
    if _AMENT_OK:
        try:
            return os.path.join(
                get_package_share_directory('iros_llm_swarm_simulation_lite'),
                'map_descriptions')
        except Exception:
            pass
    return os.path.normpath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..',
        'iros_llm_swarm_simulation_lite', 'map_descriptions'))


def _load_text(rel: str) -> str:
    with open(os.path.join(_prompts_dir(), rel), 'r', encoding='utf-8') as f:
        return f.read()


# ---------------------------------------------------------------------------
# Map context
# ---------------------------------------------------------------------------

@lru_cache(maxsize=4)
def load_map_config(map_name: str) -> dict:
    if not _YAML_OK:
        raise RuntimeError(
            'PyYAML required: pip install pyyaml --break-system-packages')
    dir_ = _map_descriptions_dir()
    for candidate in (f'{map_name}_description.yaml', f'{map_name}.yaml'):
        path = os.path.join(dir_, candidate)
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                return _yaml.safe_load(f)
    raise FileNotFoundError(
        f'Map config not found for {map_name!r} in {dir_!r}')


#: Grounding variants for ``build_map_context``. Ablation factor for E2
#: (see paper/E2_agent_task.md): how much symbolic scaffolding around the raw
#: coordinates the map block carries. Strictly nested — each variant drops a
#: superset of what the previous one drops.
#:
#:   full            everything (production default; do not change without
#:                   invalidating the E1/E1b baselines, which ran on it)
#:   locations_only  drop the heuristics block; keep factual geometry
#:                   (formation zones stay — they are named points with radii,
#:                   i.e. facts, not advice)
#:   coords_only     bare coordinates: also drop the map prose description,
#:                   location aliases, group aliases, and formation zones
GROUNDING_VARIANTS = ('full', 'locations_only', 'coords_only')


def build_map_context(map_name: str, grounding: str = 'full') -> str:
    if grounding not in GROUNDING_VARIANTS:
        raise ValueError(
            f'unknown grounding {grounding!r}; expected one of {GROUNDING_VARIANTS}')
    keep_prose   = grounding != 'coords_only'
    keep_aliases = grounding != 'coords_only'
    keep_zones   = grounding != 'coords_only'
    keep_heur    = grounding == 'full'

    cfg = load_map_config(map_name)
    b = cfg.get('bounds', {})
    lines = []
    if keep_prose:
        lines.append(
            f"Map: {cfg.get('name', map_name)} — {cfg.get('description','').strip()}")
    lines += [
        f"Bounds: X ∈ [{b.get('x_min')}, {b.get('x_max')}] m, "
        f"Y ∈ [{b.get('y_min')}, {b.get('y_max')}] m",
        '', 'Named locations:',
    ]
    # Build reverse alias map: canonical_name → [alias, ...]
    # Only include non-Russian aliases to keep context concise
    raw_aliases = cfg.get('location_aliases', {}) if keep_aliases else {}
    alias_map: dict[str, list[str]] = {}
    for alias, target in raw_aliases.items():
        # Skip Cyrillic aliases to keep context short
        if any('Ѐ' <= ch <= 'ӿ' for ch in alias):
            continue
        alias_map.setdefault(target, []).append(alias)

    for name, coords in cfg.get('named_locations', {}).items():
        aliases = alias_map.get(name, [])
        alias_str = f'  [also: {", ".join(aliases)}]' if aliases else ''
        lines.append(f'  {name:<22} ({coords[0]:.1f}, {coords[1]:.1f}){alias_str}')

    groups = cfg.get('robot_groups', {})
    if groups:
        lines += ['', 'Robot groups (color → ids → spawn center → individual spawn positions):']
        for gname, g in groups.items():
            ids     = g.get('ids', [])
            color   = g.get('color', gname)
            home    = g.get('home', [])
            raw_grp_aliases = g.get('aliases', [gname]) if keep_aliases else []
            aliases = [a for a in raw_grp_aliases
                       if not any('Ѐ' <= ch <= 'ӿ' for ch in str(a))]
            home_str = f'({home[0]:.1f}, {home[1]:.1f})' if home else '?'
            id_str   = ', '.join(f'robot_{i}' for i in ids)
            lines.append(f'  {color:<10} [{id_str}]  home={home_str}')
            if aliases:
                lines.append(f'             aliases: {", ".join(str(a) for a in aliases)}')
            spawn = g.get('spawn', {})
            if spawn:
                for rname, pos in spawn.items():
                    lines.append(f'             {rname}: ({pos[0]:.1f}, {pos[1]:.1f})')

    fzones = cfg.get('formation_zones', []) if keep_zones else []
    if fzones:
        lines += ['', 'Good spots for formations:']
        for z in fzones:
            note = f' ({z["note"]})' if 'note' in z else ''
            lines.append(
                f'  {z["name"]:<20} ({z["coords"][0]:.1f}, {z["coords"][1]:.1f}) '
                f'r={z["radius"]:.1f} m{note}')

    heuristics = cfg.get('heuristics', '').strip() if keep_heur else ''
    if heuristics:
        lines += ['', 'Heuristics:']
        for line in heuristics.splitlines():
            lines.append(f'  {line}')

    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Few-shot examples — all in {"reply":"...", "plan":{...}} format
# ---------------------------------------------------------------------------

def _ex(reply: str, plan: dict) -> str:
    """Serialize a few-shot example as the assistant would output it."""
    return json.dumps({'reply': reply, 'plan': plan}, ensure_ascii=False)


def _cluster(center: list, n: int, spacing: float = 1.0) -> list:
    """N distinct points on a centered grid around ``center``, ~spacing apart.

    Auto-spread is disabled, so the model must emit one distinct goal per robot
    (the MAPF planner cannot resolve identical goals). Few-shot example goals
    must therefore demonstrate distinct points, never repeated coordinates.
    """
    cx, cy = float(center[0]), float(center[1])
    if n <= 1:
        return [[round(cx, 2), round(cy, 2)]]
    cols = max(1, round(math.sqrt(n)))
    rows = (n + cols - 1) // cols
    pts = []
    for i in range(n):
        col, row = i % cols, i // cols
        pts.append([round(cx + (col - (cols - 1) / 2.0) * spacing, 2),
                    round(cy + (row - (rows - 1) / 2.0) * spacing, 2)])
    return pts


def _get_examples(map_name: str) -> list[dict]:
    cfg    = load_map_config(map_name)
    locs   = cfg.get('named_locations', {})
    groups = cfg.get('robot_groups', {})
    names  = list(locs.keys())

    def loc(name: str) -> list:
        return locs.get(name, [0.0, 0.0])

    center_key = next((k for k in ('center', 'central_hub') if k in locs),
                      names[0] if names else 'center')
    center = loc(center_key)

    # Group name is the dict key (the color); value has ids/home/etc.
    glist = list(groups.items())

    def _grp(i: int, fallback_name: str, fallback_ids: list) -> tuple:
        if i < len(glist):
            return glist[i][0], glist[i][1]
        return fallback_name, {'ids': fallback_ids, 'home': center}

    color0, g0 = _grp(0, 'cyan',    [0, 1, 2, 3])
    color1, g1 = _grp(1, 'magenta', [4, 5, 6, 7])
    color2, g2 = _grp(2, 'green',   [8, 9, 10, 11])
    color3, g3 = _grp(3, 'orange',  [12, 13, 14, 15])
    color4, g4 = _grp(4, 'yellow',  [16, 17, 18, 19])

    ids0 = list(g0['ids']);  home0 = g0.get('home', center)
    ids1 = list(g1['ids']);  home1 = g1.get('home', center)
    ids2 = list(g2['ids']);  home2 = g2.get('home', center)
    ids3 = list(g3['ids'])
    ids4 = list(g4['ids'])

    # Pick three spread-out named locations for variety across examples
    dest  = names[min(2, len(names) - 1)] if names else center_key
    dest2 = names[min(5, len(names) - 1)] if names else center_key
    dest3 = names[min(8, len(names) - 1)] if names else center_key

    return [
        # 1. cyan (0-3) to a named location
        {
            'user': f'{color0} robots to {dest.replace("_", " ")}',
            'out': _ex(
                f'Sending {color0} robots to {dest}.',
                {'type': 'mapf',
                 'robot_ids': ids0,
                 'goals': [loc(dest)], 'spread': True,
                 'reason': f'{color0} to {dest}'}
            ),
        },
        # 2. orange (12-15) — explicitly show this group's IDs
        {
            'user': f'{color3} robots to {dest2.replace("_", " ")}',
            'out': _ex(
                f'Sending {color3} robots to {dest2}.',
                {'type': 'mapf',
                 'robot_ids': ids3,
                 'goals': [loc(dest2)], 'spread': True,
                 'reason': f'{color3} to {dest2}'}
            ),
        },
        # 3. yellow (16-19) — explicitly show this group's IDs
        {
            'user': f'send {color4} robots to {dest3.replace("_", " ")}',
            'out': _ex(
                f'Sending {color4} robots to {dest3}.',
                {'type': 'mapf',
                 'robot_ids': ids4,
                 'goals': [loc(dest3)], 'spread': True,
                 'reason': f'{color4} to {dest3}'}
            ),
        },
        # 4. parallel: magenta (4-7) and green (8-11) swap homes
        {
            'user': f'{color1} to {color2} home, {color2} to {color1} home',
            'out': _ex(
                f'{color1.capitalize()} and {color2} swap homes simultaneously.',
                {'type': 'parallel', 'steps': [
                    {'type': 'mapf',
                     'robot_ids': ids1,
                     'goals': [home2], 'spread': True,
                     'reason': f'{color1} to {color2} home'},
                    {'type': 'mapf',
                     'robot_ids': ids2,
                     'goals': [home1], 'spread': True,
                     'reason': f'{color2} to {color1} home'},
                ]}
            ),
        },
        # 5. sequence: move then form (single formation example)
        {
            'user': f'{color0} go to {center_key}, then form a line',
            'out': _ex(
                f'{color0.capitalize()} moves to {center_key}, then forms a line.',
                {'type': 'sequence', 'steps': [
                    {'type': 'mapf', 'robot_ids': ids0,
                     'goals': [center], 'spread': True,
                     'reason': f'{color0} to {center_key}'},
                    {'type': 'formation', 'formation_id': 'line',
                     'leader_ns': f'robot_{ids0[0]}',
                     'follower_ns': [f'robot_{i}' for i in ids0[1:]],
                     'offsets_x': [-1.5 * (j + 1) for j in range(len(ids0) - 1)],
                     'offsets_y': [0.0] * (len(ids0) - 1),
                     'reason': 'line formation'},
                ]}
            ),
        },
        # 6. precise placement — distinct goal per robot, no spread
        {
            'user': f'place {color0} in a grid at {center_key.replace("_", " ")}',
            'out': _ex(
                f'Placing {color0} in a grid at {center_key}.',
                {'type': 'mapf',
                 'robot_ids': ids0,
                 'goals': _cluster(center, len(ids0)),
                 'reason': f'{color0} grid at {center_key}'}
            ),
        },
        # 7. stop
        {
            'user': 'stop',
            'out': _ex('Stopping all robots.',
                       {'type': 'idle', 'reason': 'operator stop'}),
        },
    ]


# ---------------------------------------------------------------------------
# Cached system prompts
# ---------------------------------------------------------------------------

@lru_cache(maxsize=8)
def _user_system(map_name: str, grounding: str = 'full') -> str:
    template = _load_text('user_chat_system.txt')
    return template.replace('{MAP_CONTEXT}', build_map_context(map_name, grounding))


@lru_cache(maxsize=1)
def _bt_event_system() -> str:
    return _load_text('bt_event_system.txt')


def _format_runtime_context(runtime_context: dict) -> str:
    context_json = json.dumps(
        runtime_context,
        ensure_ascii=False,
        separators=(',', ':'),
    )
    return (
        'Read-only current ROS/system context for this chat turn.\n'
        'Use it only as observed state. Do not claim direct ROS control, '
        'do not invent missing state, and say when state is unknown or stale.\n'
        'You must still return the existing JSON object with "reply" and '
        '"plan". For factual state/status questions that require no robot '
        'motion, answer in "reply" and return an idle no-op plan with a '
        'reason starting with "reply_only:".\n'
        f'Runtime context JSON:\n{context_json}'
    )


# ---------------------------------------------------------------------------
# Public builders
# ---------------------------------------------------------------------------

def build_obstacle_context_str(circles, rectangles, doors) -> str:
    lines = ['Current obstacles:']
    if not circles and not rectangles and not doors:
        lines.append('  none')
        return '\n'.join(lines)
    for c in circles:
        lines.append(f'  circle      {c.id:<20} at ({c.position.x:.1f}, {c.position.y:.1f}) r={c.radius:.2f}m')
    for r in rectangles:
        lines.append(f'  rectangle   {r.id:<20} at ({r.position.x:.1f}, {r.position.y:.1f}) {r.width:.1f}x{r.height:.1f}m')
    for d in doors:
        state = 'OPEN' if d.is_open else 'CLOSED'
        lines.append(f'  door        {d.id:<20} at ({d.position.x:.1f}, {d.position.y:.1f}) {d.width:.1f}x{d.height:.1f}m [{state}]')
    return '\n'.join(lines)


def build_user_prompt(
    user_message: str,
    history: list | None = None,
    map_name: str = 'warehouse',
    obstacle_context: str = '',
    runtime_context: dict | None = None,
    grounding: str = 'full',
) -> list:
    """Compose the channel-3 planning prompt.

    ``grounding`` selects how much symbolic scaffolding the map block carries;
    see ``GROUNDING_VARIANTS``. The default 'full' is the production path and
    is byte-identical to the pre-ablation behaviour — the E1/E1b baselines
    depend on that, so do not change it casually.
    """
    system_content = _user_system(map_name, grounding)

    if obstacle_context:
        system_content += '\n\n' + obstacle_context

    if runtime_context and runtime_context.get('source') != 'none':
        system_content += '\n\n' + _format_runtime_context(runtime_context)

    messages = [{'role': 'system', 'content': system_content}]

    for ex in _get_examples(map_name):
        messages.append({'role': 'user',      'content': ex['user']})
        messages.append({'role': 'assistant', 'content': ex['out']})
    if history:
        messages.extend(history)
    messages.append({'role': 'user', 'content': user_message})
    return messages


_REMEDIATION_RUBRIC = (
    'A plan you produced just failed during execution. The runtime context '
    'above has been refreshed via MCP after the failure.\n'
    'Either (a) produce a corrected plan that fixes the root cause, or '
    '(b) emit {"type":"idle","reason":"needs_help: <question>"} if you '
    'need operator input to proceed.\n'
    'Use "needs_help:" prefix for operator clarification, "clarify:" prefix '
    'if you want to ask a follow-up but believe a default exists. Keep the '
    'reply field short and in the operator\'s language.'
)


_EXECUTION_REPAIR_RUBRIC = (
    'A plan you produced executed, but deterministic post-execution verification '
    'says the requested outcome does not hold.\n'
    'Produce a repair plan only. If a formation failed because followers are out '
    'of position, do not repeat the same activation blindly: stage robots with '
    'MAPF near the required offsets, preferably using find_group_placement_in_room '
    'again for multi-group room formations, then activate the formation.\n'
    'Use read-only tools if needed. Keep the repair bounded and return the same '
    '{"reply":"...","plan":{...}} JSON object format.'
)


_MISSION_CONTINUATION_RUBRIC = (
    'You are still executing the same operator mission. Do not declare success '
    'unless deterministic verification says the mission is complete.\n'
    'Use read-only tools to inspect current state before producing the next '
    'plan when state or placement is uncertain. Prefer find_free_group_goals_in_room '
    'for occupied-room MAPF placement and find_group_placement_in_room for '
    'multi-group formations. If the last failure was an out-of-position '
    'formation, re-stage followers before activation. If occupied room goals '
    'were unsafe, recompute free room goals before MAPF.\n'
    'If a formation is active, do not plan MAPF for its followers directly. '
    'Move an active formation by moving only its leader; disband/deactivate '
    'the formation first if the operator wants independent follower movement. '
    'For all-robots commands, treat active formations as formation objects, '
    'not as independent robots.\n'
    'Return only the next bounded corrective JSON plan in the normal '
    '{"reasoning":"...","reply":"...","plan":{...}} format. Do not repeat '
    'the exact same failed plan.'
)


def _trim_text(value: Any, max_chars: int) -> str:
    text = '' if value is None else str(value)
    if len(text) <= max_chars:
        return text
    return text[:max(0, max_chars - 18)] + '...[truncated]'


def _compact_json_value(value: Any, max_chars: int = 2400) -> Any:
    text = json.dumps(value or {}, ensure_ascii=False, separators=(',', ':'))
    if len(text) <= max_chars:
        return value or {}
    return {
        'truncated': True,
        'original_chars': len(text),
        'json_prefix': _trim_text(text, max_chars),
    }


def _compact_runtime_context(runtime_context: dict | None) -> dict:
    if not isinstance(runtime_context, dict) or not runtime_context:
        return {}
    out: dict[str, Any] = {
        'source': runtime_context.get('source', 'unknown'),
    }
    if runtime_context.get('warnings'):
        out['warnings'] = [
            _trim_text(w, 180)
            for w in list(runtime_context.get('warnings') or [])[:4]
        ]
    bt = runtime_context.get('bt_state') or {}
    if isinstance(bt, dict) and bt:
        out['bt_state'] = {
            key: bt.get(key)
            for key in (
                'mode', 'action_status', 'active_action', 'last_error',
                'formation_id', 'leader_ns', 'formation_state',
                'formation_failure_reason',
            )
            if bt.get(key) not in (None, '', [])
        }
    formations = runtime_context.get('formations') or []
    if isinstance(formations, list) and formations:
        compact_formations = []
        for item in formations[:10]:
            if not isinstance(item, dict):
                continue
            compact_formations.append({
                key: item.get(key)
                for key in (
                    'formation_id', 'leader_ns', 'followers', 'follower_ns',
                    'status', 'state', 'failure_reason', 'max_error_m',
                    'mean_error_m',
                )
                if item.get(key) not in (None, '', [])
            })
        out['formations'] = compact_formations
    robots = runtime_context.get('robots') or {}
    if isinstance(robots, dict) and robots:
        compact_robots = {}
        for key in sorted(robots.keys(), key=str)[:24]:
            pose = robots.get(key) or {}
            if not isinstance(pose, dict):
                continue
            compact_robots[str(key)] = {
                pose_key: pose.get(pose_key)
                for pose_key in ('x', 'y', 'yaw', 'stale', 'stale_ms')
                if pose.get(pose_key) is not None
            }
        out['robots'] = compact_robots
    if runtime_context.get('robot_assignment'):
        out['robot_assignment'] = _compact_json_value(
            runtime_context.get('robot_assignment'),
            max_chars=1200,
        )
    events = runtime_context.get('recent_events') or []
    if isinstance(events, list) and events:
        out['recent_events'] = [_trim_text(e, 220) for e in events[-4:]]
    return out


def build_compact_mission_context(
    original_user_message: str,
    last_plan: dict,
    execution_result: dict,
    verification: dict,
    *,
    step: int,
    max_steps: int,
    remaining_time_sec: float,
    fresh_runtime_context: dict | None,
    previous_verification: dict | None = None,
    history: list | None = None,
) -> dict:
    """Return bounded mission context for continuation prompts."""
    return {
        'original_request': _trim_text(original_user_message, 800),
        'step': int(step),
        'max_steps': int(max_steps),
        'remaining_time_sec': round(float(remaining_time_sec), 1),
        'last_plan': _compact_json_value(last_plan, max_chars=2400),
        'execution_result': _compact_json_value(execution_result, max_chars=1000),
        'current_verification': _compact_json_value(verification, max_chars=1800),
        'previous_verification': _compact_json_value(
            previous_verification or {},
            max_chars=1200,
        ),
        'state_digest': _compact_runtime_context(fresh_runtime_context),
        'history': {'omitted_turns': len(history or [])},
        'available_tools': [
            'get_robot_position',
            'get_positions',
            'check_occupancy',
            'find_free_group_goals_in_room',
            'find_group_placement_in_room',
            'verify_plan_execution_state',
        ],
        'valid_plan_node_types': [
            'mapf', 'formation', 'disband', 'idle', 'sequence', 'parallel',
        ],
        'next_action_rules': [
            'do not declare success unless verification is ok',
            'do not repeat the exact same failed plan',
            'do not MAPF active formation followers directly',
            'move active formations by leader only',
            'disband active formations before independent follower movement',
            'use free/group placement tools before crowded room movement',
        ],
    }


def _compact_mission_system(map_name: str, obstacle_context: str) -> str:
    try:
        map_context = _trim_text(build_map_context(map_name), 3600)
    except Exception:
        map_context = f'Map: {map_name}'
    content = (
        'You are producing the next JSON plan for an already-running robot '
        'mission. Return exactly one JSON object with keys reasoning, reply, '
        'and plan. Valid plan node types: mapf, formation, disband, idle, '
        'sequence, parallel. Do not output prose outside JSON.\n\n'
        f'{map_context}\n\n'
        'Active formation safety rule: followers of active formations must not '
        'be moved directly by MAPF. Move the leader to move the formation, or '
        'disband first for independent movement.'
    )
    if obstacle_context:
        content += '\n\n' + _trim_text(obstacle_context, 1200)
    return content


def _format_attempts(attempts: list[dict]) -> str:
    lines = []
    for i, att in enumerate(attempts, start=1):
        leaf = att.get('leaf_type', '?')
        phase = att.get('failed_at_phase', '?')
        err = str(att.get('last_error', '') or '')[:240]
        lines.append(f'  - attempt {i}: leaf={leaf} phase={phase} error="{err}"')
    return '\n'.join(lines) if lines else '  (none recorded)'


def build_remediation_prompt(
    original_user_message: str,
    original_plan: dict,
    attempts: list[dict],
    failure_info: dict,
    fresh_runtime_context: dict | None,
    history: list | None = None,
    map_name: str = 'warehouse',
    obstacle_context: str = '',
) -> list:
    """Compose a remediation prompt for one retry attempt.

    Starts from the same system+examples+history+user-message stack as the
    first turn, then appends:
      * a remediation rubric (system),
      * the assistant's previously-produced plan,
      * a fresh failure summary with the per-attempt log.
    """
    messages = build_user_prompt(
        original_user_message,
        history=history,
        map_name=map_name,
        obstacle_context=obstacle_context,
        runtime_context=fresh_runtime_context,
    )
    # Merged into the single leading system message rather than appended as a
    # second system-role entry: some chat templates (e.g. Qwen3.5) statically
    # reject any non-first system message when generating a structured-output
    # grammar.
    messages[0]['content'] += '\n\n' + _REMEDIATION_RUBRIC
    messages.append({
        'role': 'assistant',
        'content': json.dumps(original_plan, ensure_ascii=False),
    })
    failed_leaf = (failure_info or {}).get('leaf_type', '?')
    last_error  = str((failure_info or {}).get('last_error', '') or '')[:240]
    phase       = (failure_info or {}).get('failed_at_phase', '?')
    summary = (
        f'The plan above failed.\n'
        f'Failed leaf: {failed_leaf} (phase={phase})\n'
        f'Last error: {last_error}\n'
        f'Per-attempt log:\n{_format_attempts(attempts)}\n'
        f'Original user request: {original_user_message}\n'
        'Produce either a corrected plan or a needs_help: idle reply.'
    )
    messages.append({'role': 'user', 'content': summary})
    return messages


def build_execution_repair_prompt(
    original_user_message: str,
    last_plan: dict,
    verification: dict,
    *,
    attempt: int,
    max_attempts: int,
    fresh_runtime_context: dict | None,
    history: list | None = None,
    map_name: str = 'warehouse',
    obstacle_context: str = '',
) -> list:
    """Compose a bounded repair prompt after post-execution verification fails."""
    messages = build_user_prompt(
        original_user_message,
        history=history,
        map_name=map_name,
        obstacle_context=obstacle_context,
        runtime_context=fresh_runtime_context,
    )
    # See build_remediation_prompt: merge rather than append a second system message.
    messages[0]['content'] += '\n\n' + _EXECUTION_REPAIR_RUBRIC
    messages.append({
        'role': 'assistant',
        'content': json.dumps(last_plan, ensure_ascii=False),
    })
    summary = (
        f'Post-execution verification failed after executing the plan above.\n'
        f'Repair attempt: {attempt}/{max_attempts}\n'
        f'Original user request: {original_user_message}\n'
        f'Verification result JSON:\n'
        f'{compact_verification_json(verification)}\n'
        'Produce a corrected repair plan. If verification says repairable=false, '
        'emit an idle needs_help plan with a clear reason instead.'
    )
    messages.append({'role': 'user', 'content': summary})
    return messages


def build_mission_continuation_prompt(
    original_user_message: str,
    last_plan: dict,
    execution_result: dict,
    verification: dict,
    *,
    step: int,
    max_steps: int,
    remaining_time_sec: float,
    fresh_runtime_context: dict | None,
    previous_verification: dict | None = None,
    history: list | None = None,
    map_name: str = 'warehouse',
    obstacle_context: str = '',
) -> list:
    """Compose a strict continuation prompt for mission supervision."""
    # Single leading system message — see build_remediation_prompt for why.
    messages = [{
        'role': 'system',
        'content': (_compact_mission_system(map_name, obstacle_context)
                    + '\n\n' + _MISSION_CONTINUATION_RUBRIC),
    }]
    compact_context = build_compact_mission_context(
        original_user_message,
        last_plan,
        execution_result,
        verification,
        step=step,
        max_steps=max_steps,
        remaining_time_sec=remaining_time_sec,
        fresh_runtime_context=fresh_runtime_context,
        previous_verification=previous_verification,
        history=history,
    )
    summary = (
        'Mission supervision requires another corrective/continuation plan.\n'
        'Use this compact mission context; full history and large tool outputs '
        'were intentionally omitted.\n'
        f'Remaining mission time: {remaining_time_sec:.1f}s\n'
        'Compact mission context JSON:\n'
        f'{json.dumps(compact_context, ensure_ascii=False, separators=(",", ":"))}\n'
        'If the last failure says mapf_targets_active_formation_followers, '
        'the previous plan tried to move followers of an active formation '
        'directly with MAPF. This is invalid: either move only the leader, '
        'or disband the formation first.\n'
        'Produce the next plan only. Do not repeat the exact same failed plan.'
    )
    messages.append({'role': 'user', 'content': summary})
    return messages


def build_bt_event_prompt(bt_state: Any, history: list | None = None) -> list:
    messages = [{'role': 'system', 'content': _bt_event_system()}]
    if history:
        messages.extend(history[-6:])
    messages.append({'role': 'user', 'content': (
        f'BT reports: [{bt_state.action_status}] '
        f'action={bt_state.active_action} '
        f"error='{bt_state.last_error}' "
        f"summary='{bt_state.action_summary}'"
    )})
    return messages
