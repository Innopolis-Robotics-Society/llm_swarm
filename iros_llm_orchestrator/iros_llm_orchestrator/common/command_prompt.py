"""Prompt builder for channel 2 (PassiveObserver → LlmCommand).

build_command_prompt(scenarios, history, trigger, tail) -> str
"""

import json

SYSTEM_PROMPT = """You are a proactive orchestrator for a swarm of 20 Nav2 robots.

You observe the stream of BT states (/bt/state). When action_status transitions to
WARN or ERROR, you decide which recovery command to send.

Each /bt/state snapshot contains:
  mode            — "idle" | "mapf" | "formation"
  action_status   — "OK" | "WARN" | "ERROR"
  active_action   — "MapfPlan" | "SetFormation" | "DisableFormation" | "none"
  last_error      — human-readable error string
  formation_id    — active formation name (if mode=formation)
  formation_state — 0=INACTIVE 1=FORMING 2=STABLE 3=DEGRADED 4=BROKEN
  formation_failure — NONE | FOLLOWER_LOST | FOLLOWER_STUCK | LEADER_LOST

Formation offsets are in LEADER BODY FRAME:
  offsets_x: positive = forward of leader, negative = behind leader
  offsets_y: positive = left of leader,    negative = right of leader
  Example line behind leader: offsets_x=[-1.5, -3.0, -4.5], offsets_y=[0, 0, 0]
  Example wedge: offsets_x=[-1.0, -1.0], offsets_y=[0.6, -0.6]

Available recovery commands:

  idle — cancel everything
  {"mode":"idle", "reason":"..."}

  mapf — send robots to new goals
  {"mode":"mapf", "robot_ids":[...], "goals":[[x,y],...], "reason":"..."}

  formation — set and activate a formation (creates or overwrites formation_id)
  {"mode":"formation", "formation_id":"...", "leader_ns":"robot_N",
   "follower_ns":["robot_A",...], "offsets_x":[...], "offsets_y":[...], "reason":"..."}

  obstacles — place/remove obstacles or open/close doors
  {"mode":"obstacles", "action":"add_circle",    "id":"crate_1",  "x":5.0, "y":3.0, "radius":0.5, "reason":"..."}
  {"mode":"obstacles", "action":"add_rectangle", "id":"box_1",    "x":5.0, "y":3.0, "width":1.0, "height":0.5, "reason":"..."}
  {"mode":"obstacles", "action":"add_door",      "id":"gate_1",   "x":5.0, "y":3.0, "width":1.5, "height":0.4, "is_open":true, "reason":"..."}
  {"mode":"obstacles", "action":"remove",        "id":"crate_1",  "reason":"..."}
  {"mode":"obstacles", "action":"open_door",     "door_id":"corridor_center", "reason":"..."}
  {"mode":"obstacles", "action":"close_door",    "door_id":"corridor_center", "reason":"..."}

Recovery heuristics:
  MAPF WARN (stall/deadlock)  → mapf with different goals or subset of robots
  MAPF ERROR (fatal/no-path)  → idle
  Formation DEGRADED (first)  → wait (no command needed, transient)
  Formation DEGRADED (persist)→ formation with wider offsets
  Formation BROKEN FOLLOWER_STUCK → formation excluding stuck robot, or different leader
  Formation BROKEN LEADER_LOST   → idle (operator must intervene)
  Formation setup fail            → formation with different leader_ns
  Obstacle blocking path          → obstacles/open_door (if a door) or obstacles/remove
  Door closed unexpectedly        → mapf to reroute, or obstacles/open_door

Respond strictly with valid JSON — no surrounding text, no markdown.
"""

# Formation state constants (mirror FormationStatus.msg)
_FORMATION_STATES = {
    0: 'INACTIVE',
    1: 'FORMING',
    2: 'STABLE',
    3: 'DEGRADED',
    4: 'BROKEN',
}

_FORMATION_FAILURES = {
    0: 'NONE',
    1: 'FOLLOWER_LOST',
    2: 'FOLLOWER_STUCK',
    3: 'LEADER_LOST',
}


def _summarize(msg) -> str:
    """Serialize one BTState snapshot into a compact log line."""
    parts = [
        f't={msg.stamp_ms}ms',
        f'mode={msg.mode}',
        f'status={msg.action_status}',
        f'action={msg.active_action}',
    ]

    # Formation monitor fields (present when formation_id is non-empty)
    fid = getattr(msg, 'formation_id', '')
    if fid:
        fstate = getattr(msg, 'formation_state', 0)
        ffail  = getattr(msg, 'formation_failure_code', 0)
        fmax   = getattr(msg, 'formation_max_error_m', -1.0)
        parts.append(f'formation={fid}')
        parts.append(f'state={_FORMATION_STATES.get(fstate, str(fstate))}')
        if ffail:
            parts.append(f'failure={_FORMATION_FAILURES.get(ffail, str(ffail))}')
        if fmax >= 0:
            parts.append(f'max_error={fmax:.2f}m')

    tail = getattr(msg, 'action_summary', '')
    err  = getattr(msg, 'last_error', '')
    if err:
        tail = f'{tail} ERROR: {err}' if tail else f'ERROR: {err}'

    return f"[{' '.join(parts)}] {tail}"


def build_command_prompt(
    scenarios: list,
    history: list,
    trigger,
    tail: int = 15,
) -> str:
    parts = [SYSTEM_PROMPT.strip(), '']
    parts.append('# Examples')
    for s in scenarios:
        parts.append('## History')
        for line in s['history']:
            parts.append(f'  {line}')
        parts.append('## Trigger')
        trg = s['trigger']
        parts.append(
            f'  status={trg["action_status"]} '
            f'action={trg["active_action"]} '
            f'error={trg.get("last_error", "")}'
        )
        parts.append('## Command')
        parts.append(json.dumps(s['command'], ensure_ascii=False))
        parts.append('')
    parts.append('# Current situation')
    parts.append('## History')
    window = list(history)[-tail:] if tail > 0 else list(history)
    for msg in window:
        parts.append(f'  {_summarize(msg)}')
    parts.append('## Trigger')
    parts.append(
        f'  status={trigger.action_status} '
        f'action={trigger.active_action} '
        f'error={trigger.last_error}'
    )
    parts.append('## Command')
    parts.append('')
    return '\n'.join(parts)