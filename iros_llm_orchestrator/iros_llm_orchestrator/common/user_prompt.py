"""Prompt builder for user chat interface (channel 3).

Loads map context and system prompt from prompts/ directory.
All few-shot examples use the {"reply":"...", "plan":{...}} format.
"""

import json
import os
from functools import lru_cache
from typing import Any

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
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'prompts'))


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
    path = os.path.join(_map_descriptions_dir(), f'{map_name}.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        return _yaml.safe_load(f)


def build_map_context(map_name: str) -> str:
    cfg = load_map_config(map_name)
    b = cfg.get('bounds', {})
    lines = [
        f"Map: {cfg.get('name', map_name)} — {cfg.get('description','').strip()}",
        f"Bounds: X ∈ [{b.get('x_min')}, {b.get('x_max')}] m, "
        f"Y ∈ [{b.get('y_min')}, {b.get('y_max')}] m",
        '', 'Named locations:',
    ]
    for name, coords in cfg.get('named_locations', {}).items():
        lines.append(f'  {name:<22} ({coords[0]:.1f}, {coords[1]:.1f})')

    groups = cfg.get('robot_groups', {})
    if groups:
        lines += ['', 'Robot groups (color → ids → spawn center → individual spawn positions):']
        for gname, g in groups.items():
            ids     = g.get('ids', [])
            color   = g.get('color', gname)
            home    = g.get('home', [])
            aliases = g.get('aliases', [gname])
            home_str = f'({home[0]:.1f}, {home[1]:.1f})' if home else '?'
            id_str   = ', '.join(f'robot_{i}' for i in ids)
            lines.append(f'  {color:<10} [{id_str}]  home={home_str}')
            lines.append(f'             aliases: {", ".join(str(a) for a in aliases)}')
            spawn = g.get('spawn', {})
            if spawn:
                for rname, pos in spawn.items():
                    lines.append(f'             {rname}: ({pos[0]:.1f}, {pos[1]:.1f})')

    fzones = cfg.get('formation_zones', [])
    if fzones:
        lines += ['', 'Good spots for formations:']
        for z in fzones:
            note = f' ({z["note"]})' if 'note' in z else ''
            lines.append(
                f'  {z["name"]:<20} ({z["coords"][0]:.1f}, {z["coords"][1]:.1f}) '
                f'r={z["radius"]:.1f} m{note}')

    heuristics = cfg.get('heuristics', '').strip()
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

    gvals  = list(groups.values())
    # We need at least 2 groups for swap example
    g0 = gvals[0] if gvals          else {'ids': [0,1,2,3],  'color': 'cyan',    'home': center, 'spawn': {}}
    g1 = gvals[1] if len(gvals) > 1 else {'ids': [4,5,6,7],  'color': 'magenta', 'home': center, 'spawn': {}}
    g2 = gvals[2] if len(gvals) > 2 else {'ids': [8,9,10,11],'color': 'green',   'home': center, 'spawn': {}}

    ids0  = list(g0['ids']); color0 = g0.get('color','group0')
    ids1  = list(g1['ids']); color1 = g1.get('color','group1')
    ids2  = list(g2['ids']); color2 = g2.get('color','group2')
    home0 = g0.get('home', center)
    home1 = g1.get('home', center)
    spawn0 = g0.get('spawn', {})
    dest  = names[min(2, len(names)-1)] if names else center_key

    return [
        # 1. Simple: one group to a location
        {
            'user': f'{color0} robots to {dest.replace("_"," ")}',
            'out': _ex(
                f'Sending {color0} robots to {dest}.',
                {'type': 'mapf',
                 'robot_ids': ids0,
                 'goals': [loc(dest)] * len(ids0),
                 'reason': f'{color0} to {dest}'}
            ),
        },
        # 2. Group home — uses spawn positions
        {
            'user': f'send {color0} robots home',
            'out': _ex(
                f'Returning {color0} robots to their spawn positions.',
                {'type': 'mapf',
                 'robot_ids': ids0,
                 'goals': [home0] * len(ids0),
                 'reason': f'{color0} home'}
            ),
        },
        # 3. TWO groups swap homes — parallel with merged mapf
        {
            'user': f'{color0} robots to {color1} home, {color1} robots to {color0} home',
            'out': _ex(
                f'{color0.capitalize()} and {color1} swap homes simultaneously.',
                {'type': 'parallel', 'steps': [
                    {'type': 'mapf',
                     'robot_ids': ids0,
                     'goals': [home1] * len(ids0),
                     'reason': f'{color0} to {color1} home'},
                    {'type': 'mapf',
                     'robot_ids': ids1,
                     'goals': [home0] * len(ids1),
                     'reason': f'{color1} to {color0} home'},
                ]}
            ),
        },
        # 4. Three groups simultaneous
        {
            'user': f'{color0} to east, {color1} to center, {color2} to west',
            'out': _ex(
                f'{color0.capitalize()}, {color1}, and {color2} move simultaneously.',
                {'type': 'parallel', 'steps': [
                    {'type': 'mapf', 'robot_ids': ids0,
                     'goals': [loc(names[min(3,len(names)-1)])] * len(ids0),
                     'reason': f'{color0} east'},
                    {'type': 'mapf', 'robot_ids': ids1,
                     'goals': [center] * len(ids1),
                     'reason': f'{color1} center'},
                    {'type': 'mapf', 'robot_ids': ids2,
                     'goals': [loc(names[min(1,len(names)-1)])] * len(ids2),
                     'reason': f'{color2} west'},
                ]}
            ),
        },
        # 5. Sequential: go somewhere, then form
        {
            'user': f'{color0} go to {center_key}, then form a line',
            'out': _ex(
                f'{color0.capitalize()} moves to {center_key}, then forms a line.',
                {'type': 'sequence', 'steps': [
                    {'type': 'mapf', 'robot_ids': ids0,
                     'goals': [center] * len(ids0),
                     'reason': f'{color0} to {center_key}'},
                    {'type': 'formation', 'formation_id': 'line',
                     'leader_ns': f'robot_{ids0[0]}',
                     'follower_ns': [f'robot_{i}' for i in ids0[1:]],
                     'offsets_x': [-1.5 * (j+1) for j in range(len(ids0)-1)],
                     'offsets_y': [0.0] * (len(ids0)-1),
                     'reason': 'line formation'},
                ]}
            ),
        },
        # 6. Stop
        {
            'user': 'stop',
            'out': _ex('Stopping all robots.',
                       {'type': 'idle', 'reason': 'operator stop'}),
        },
    ]


# ---------------------------------------------------------------------------
# Cached system prompts
# ---------------------------------------------------------------------------

@lru_cache(maxsize=4)
def _user_system(map_name: str) -> str:
    template = _load_text('user_chat_system.txt')
    return template.replace('{MAP_CONTEXT}', build_map_context(map_name))


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
) -> list:
    system_content = _user_system(map_name)

    if obstacle_context:
        system_content += '\n\n' + obstacle_context

    messages = [{'role': 'system', 'content': system_content}]

    if runtime_context and runtime_context.get('source') != 'none':
        messages.append({
            'role': 'system',
            'content': _format_runtime_context(runtime_context),
        })
    for ex in _get_examples(map_name):
        messages.append({'role': 'user',      'content': ex['user']})
        messages.append({'role': 'assistant', 'content': ex['out']})
    if history:
        messages.extend(history)
    messages.append({'role': 'user', 'content': user_message})
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
