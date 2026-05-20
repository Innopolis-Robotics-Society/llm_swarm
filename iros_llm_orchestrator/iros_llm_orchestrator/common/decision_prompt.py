"""Few-shot prompt builder for channel 1 (LlmDecision action server).

build_decision_prompt(scenarios, level, event, log_buffer, tail) -> str
"""

import json

DECISION_MCP_ALLOWED_TOOLS = (
    'subscribe_once',
    'get_action_status',
    'get_actions',
    'get_topics',
)

SYSTEM_PROMPT = """You are the supervisor orchestrator of a swarm of 20 Nav2 robots.

When a BT node (MapfPlan / SetFormation / DisableFormation) encounters a WARN or ERROR,
or the formation monitor (BTStatePublisher watching /formations/status) detects a health
change, it sends you an event and the last lines of the accumulated log.

Your task — choose exactly one of three decisions:

  "wait"   — situation is not critical or transient; BT continues unchanged.
             Use for: first occurrence of a stall, brief deviation, formation just
             activated and still converging (STATE_FORMING → DEGRADED is normal),
             AND for any planner-internal WARN that indicates self-recovery
             (see below).
  "abort"  — unrecoverable: collision in the world (not in the planner),
             all goals unreachable, leader odometry lost, /formation/set
             service unavailable. BT cancels the action and returns FAILURE.
  "replan" — plan is stale or configuration needs adjustment, AND the
             system has stopped making progress: arrived count frozen for
             3+ ticks, growing multi-robot deadlock, hard timeout, formation
             persistently DEGRADED (error not decreasing), follower stuck
             (FOLLOWER_STUCK). BT cancels the running action and returns
             FAILURE — do NOT pick this lightly.

DEFAULT TO "wait". Picking "replan" or "abort" cancels the running plan; if
you are uncertain, wait — the next tick will give more information.

Distinguish PLANNER-INTERNAL WARNs from ROBOT-LEVEL WARNs:

  Planner-internal WARN (echo "wait" — the system already recovered):
    • "replan did not reach collision-free; keeping old paths"
    • "warm seed rejected ... cold replan"
    • "left agents with empty paths (soft, keep trying)"
    • "stale segments"
    • Any WARN that explicitly says paths/old plan are being kept.
    These mean the LNS2/PBS planner tried to optimise, didn't find a
    better solution, and is continuing with the existing one. Robots
    keep moving. The fleet is fine.

  Robot-level WARN (look at progress before deciding):
    • "robot_N stalled"  • "robot_N deviated"  • "fatal collision"
    Read the log_buffer:
      – arrived count increasing across ticks → "wait"
      – arrived count frozen for 3+ ticks    → "replan"
      – fatal/unrecoverable                  → "abort"

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

AGENTIC_MCP_PROMPT = """
For WARN/ERROR only:
You may request read-only MCP observations before your final decision.

MCP is observation only. Request only the listed tools. Never request
write/control tools, never publish topics, call services, send action goals,
set/delete parameters, send /cmd_vel, directly control robots, or bypass the
BT/MAPF/Formation Manager path.

Allowed read-only decision tools and exact useful args:

1. subscribe_once for BT state:
{"name":"subscribe_once","args":{"topic":"/bt/state","msg_type":"iros_llm_swarm_interfaces/msg/BTState"}}

2. subscribe_once for formation status:
{"name":"subscribe_once","args":{"topic":"/formations/status","msg_type":"iros_llm_swarm_interfaces/msg/FormationsStatus"}}

3. get_action_status for the active MAPF goal action:
{"name":"get_action_status","args":{"action_name":"/swarm/set_goals"}}

4. get_actions:
{"name":"get_actions","args":{}}

5. get_topics:
{"name":"get_topics","args":{}}

Return exactly one JSON object in one of these shapes.

Tool request:
{"mode":"tool_request","tools":[{"name":"subscribe_once","args":{"topic":"/bt/state","msg_type":"iros_llm_swarm_interfaces/msg/BTState"}}],"reason":"Need current BT state"}

Final decision:
{"mode":"final","decision":"wait","reason":"Short temporary stall; no persistent failure evidence"}

If the event and log_buffer already contain enough evidence, return
mode=final immediately. The final decision must be exactly one of:
wait, replan, abort.
"""

AGENTIC_MCP_EXAMPLES = [
    {
        'title': 'Temporary warning: observe BT state, then wait',
        'input': {
            'level': 'WARN',
            'event': 'robot_3 stalled for 5s',
            'log_buffer': [
                '[t=3600ms status=executing arrived=8 active=12] WARN: robot_3 stalled',
            ],
        },
        'responses': [
            {
                'mode': 'tool_request',
                'tools': [{
                    'name': 'subscribe_once',
                    'args': {
                        'topic': '/bt/state',
                        'msg_type': 'iros_llm_swarm_interfaces/msg/BTState',
                    },
                }],
                'reason': 'Need current BT state before canceling a running plan',
            },
            {
                'mode': 'final',
                'decision': 'wait',
                'reason': 'First short stall and BT state still shows executing',
            },
        ],
    },
    {
        'title': 'Persistent MAPF stall: observe BT and action status, then replan',
        'input': {
            'level': 'WARN',
            'event': 'stall count is 4 and growing across cluster',
            'log_buffer': [
                '[t=3000ms status=executing arrived=6 active=14 stall=2]',
                '[t=6000ms status=executing arrived=6 active=14 stall=3]',
                '[t=9000ms status=executing arrived=6 active=14 stall=4]',
            ],
        },
        'responses': [
            {
                'mode': 'tool_request',
                'tools': [
                    {
                        'name': 'subscribe_once',
                        'args': {
                            'topic': '/bt/state',
                            'msg_type':
                                'iros_llm_swarm_interfaces/msg/BTState',
                        },
                    },
                    {
                        'name': 'get_action_status',
                        'args': {'action_name': '/swarm/set_goals'},
                    },
                ],
                'reason': 'Need current BT progress and MAPF action status',
            },
            {
                'mode': 'final',
                'decision': 'replan',
                'reason': 'Arrived count is frozen and action remains active',
            },
        ],
    },
    {
        'title': 'Fatal or unrecoverable failure: decide immediately',
        'input': {
            'level': 'ERROR',
            'event': 'formation broken: Leader robot_0 odom timeout (2.3s)',
            'log_buffer': [
                '[formation=wedge state=BROKEN failure=LEADER_LOST] ERROR',
            ],
        },
        'responses': [{
            'mode': 'final',
            'decision': 'abort',
            'reason': 'Leader odometry is lost; formation cannot continue',
        }],
    },
    {
        'title': 'Forbidden tools',
        'input': {
            'level': 'WARN',
            'event': 'operator asks whether to reset a service',
            'log_buffer': [],
        },
        'responses': [{
            'mode': 'final',
            'decision': 'wait',
            'reason': (
                'Do not request call_service, send_action_goal, publish_once, '
                'or any write/control tool from the decision channel'
            ),
        }],
    },
]


def build_decision_prompt(
    scenarios: list,
    level: str,
    event: str,
    log_buffer: list,
    tail: int = 20,
    agentic_enabled: bool = False,
) -> str:
    use_agentic = agentic_enabled and str(level).upper() in {'WARN', 'ERROR'}
    parts = [SYSTEM_PROMPT.strip(), '']
    if use_agentic:
        parts.append(AGENTIC_MCP_PROMPT.strip())
        parts.append('')
        parts.append('# Agentic MCP examples')
        for example in AGENTIC_MCP_EXAMPLES:
            parts.append(f"## {example['title']}")
            parts.append('Input:')
            parts.append(f"level: {example['input']['level']}")
            parts.append(f"event: {example['input']['event']}")
            parts.append('log_buffer:')
            for line in example['input']['log_buffer']:
                parts.append(f'  {line}')
            parts.append('Responses:')
            for response in example['responses']:
                parts.append(json.dumps(response, ensure_ascii=False))
            parts.append('')
    parts.append('# Examples')
    for s in scenarios:
        parts.append('## Input')
        parts.append(f"level: {s['level']}")
        parts.append(f"event: {s['event']}")
        parts.append('log_buffer:')
        for line in s['log_buffer']:
            parts.append(f'  {line}')
        parts.append('## Decision')
        decision = dict(s['decision'])
        if use_agentic:
            decision = {'mode': 'final', **decision}
        parts.append(json.dumps(decision, ensure_ascii=False))
        parts.append('')
    parts.append('# Current situation')
    parts.append(f'level: {level}')
    parts.append(f'event: {event}')
    parts.append('log_buffer:')
    for line in log_buffer[-tail:]:
        parts.append(f'  {line}')
    parts.append('## Decision or Tool Request' if use_agentic else '## Decision')
    parts.append('')
    return '\n'.join(parts)
