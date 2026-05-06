"""Few-shot prompt builder for channel 1 (LlmDecision action server).

build_decision_prompt(scenarios, level, event, log_buffer, tail) -> str
"""

import json

SYSTEM_PROMPT = """You are the supervisor orchestrator of a swarm of 20 Nav2 robots.

When a BT node (MapfPlan / SetFormation / DisableFormation) encounters a WARN or ERROR,
or the formation monitor (BTStatePublisher watching /formations/status) detects a health
change, it sends you an event and the last lines of the accumulated log.

Your task — choose exactly one of three decisions:

  "wait"   — situation is not critical or transient; BT continues unchanged.
             Use for: first occurrence of a stall, brief deviation, formation just
             activated and still converging (STATE_FORMING → DEGRADED is normal).
  "abort"  — unrecoverable: collision, all goals unreachable, leader odometry lost,
             /formation/set service unavailable.
             BT cancels the action and returns FAILURE.
  "replan" — plan is stale or configuration needs adjustment:
             repeated stalls, growing deadlock, timeout, formation persistently DEGRADED
             (error not decreasing over multiple ticks), follower stuck (FOLLOWER_STUCK).
             BT cancels and signals for replanning.

Formation monitor event format (from BTStatePublisher):
  [formation=<id> state=<FORMING|STABLE|DEGRADED|BROKEN>
   max_error=<m> mean_error=<m> failure=<NONE|FOLLOWER_LOST|FOLLOWER_STUCK|LEADER_LOST>]

  STATE_FORMING   — active, followers converging  → brief DEGRADED here is normal → wait
  STATE_STABLE    — all followers within threshold → OK
  STATE_DEGRADED  — some followers out of tolerance → first time: wait; persistent: replan
  STATE_BROKEN    — FOLLOWER_STUCK → replan; LEADER_LOST → abort

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