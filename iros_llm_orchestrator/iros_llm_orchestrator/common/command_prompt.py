"""Prompt builder for channel 2 (PassiveObserver → LlmCommand).

build_command_prompt(scenarios, history, trigger, tail) -> str
"""

import json

SYSTEM_PROMPT = """You are a proactive orchestrator for a swarm of 20 Nav2 robots.

You observe the stream of BT states (/bt/state). When you see action_status
transition to WARN or ERROR, you decide which command to send so the swarm
recovers.

Available modes:
  "idle"       — cancel everything, stop all actions
  "mapf"       — send swarm to new goals via MAPF
  "formation"  — form a geometric formation

Respond strictly with valid JSON:
{
  "mode": "idle" | "mapf" | "formation",
  "robot_ids": [0, 1, 2, ...],              // mapf only
  "goals": [[x1, y1], [x2, y2], ...],       // mapf only (length == robot_ids)
  "formation_id": "wedge",                   // formation only
  "leader_ns": "robot_0",                    // formation only
  "follower_ns": ["robot_1", "robot_2"],    // formation only
  "offsets_x": [1.0, -1.0],                 // formation only
  "offsets_y": [0.0, 0.0],                  // formation only
  "reason": "brief justification"
}

No surrounding text, no markdown — only one JSON object.
"""


def _summarize(msg) -> str:
    parts = [
        f't={msg.stamp_ms}ms',
        f'mode={msg.mode}',
        f'status={msg.action_status}',
        f'action={msg.active_action}',
    ]
    tail = msg.action_summary
    if msg.last_error:
        tail = f'{tail} ERROR: {msg.last_error}' if tail else f'ERROR: {msg.last_error}'
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
