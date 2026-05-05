"""Few-shot prompt builder for channel 1 (LlmDecision action server).

build_decision_prompt(scenarios, level, event, log_buffer, tail) -> str
"""

import json

SYSTEM_PROMPT = """You are the supervisor orchestrator of a swarm of 20 Nav2 robots.
When a BT node (MapfPlan / SetFormation / DisableFormation) encounters a WARN or
receives a periodic INFO from the corresponding component, it sends you an event
and the last lines of the accumulated log.

Your task — choose exactly one of three decisions:

  "wait"   — situation is not critical; BT continues the current plan unchanged.
  "abort"  — situation is unrecoverable (collision, unreachable goal, fatal planner
             error). BT cancels the action and returns FAILURE.
  "replan" — current plan is stale and needs recalculation (timeout, deadlock,
             growing robot_stall with no progress, many replans_done in a row).
             BT cancels the action and signals for replanning.

Respond strictly with valid JSON:
{"decision": "wait"|"abort"|"replan", "reason": "brief justification"}

No surrounding text, no markdown blocks — only one JSON object.
"""


def build_decision_prompt(
    scenarios: list,
    level: str,
    event: str,
    log_buffer: list,
    tail: int = 20,
) -> str:
    parts = [SYSTEM_PROMPT.strip(), '']
    parts.append('# Examples')
    for s in scenarios:
        parts.append('## Input')
        parts.append(f"level: {s['level']}")
        parts.append(f"event: {s['event']}")
        parts.append('log_buffer:')
        for line in s['log_buffer']:
            parts.append(f'  {line}')
        parts.append('## Decision')
        parts.append(json.dumps(s['decision'], ensure_ascii=False))
        parts.append('')
    parts.append('# Current situation')
    parts.append(f'level: {level}')
    parts.append(f'event: {event}')
    parts.append('log_buffer:')
    for line in log_buffer[-tail:]:
        parts.append(f'  {line}')
    parts.append('## Decision')
    parts.append('')
    return '\n'.join(parts)
