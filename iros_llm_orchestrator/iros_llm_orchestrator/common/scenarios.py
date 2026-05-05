"""Few-shot examples for channel 1 (LlmDecision) and channel 2 (LlmCommand).

  DECISION_SCENARIOS  — used by decision prompt builder (channel 1).
  COMMAND_SCENARIOS   — used by command prompt builder (channel 2).
"""

# ---------------------------------------------------------------------------
# Channel 1 — decision scenarios
# ---------------------------------------------------------------------------

DECISION_SCENARIOS = [
    # WARN -> wait
    {
        'level': 'WARN',
        'event': 'robot_3 stalled for 5s at (12.4, 8.1)',
        'log_buffer': [
            '[t=1200ms status=executing arrived=5 active=15 stall=0 replans=0]',
            '[t=2400ms status=executing arrived=8 active=12 stall=0 replans=0]',
            '[t=3600ms status=executing arrived=8 active=12 stall=1 replans=0] WARN: robot_3 stalled for 5s at (12.4, 8.1)',
        ],
        'decision': {'decision': 'wait', 'reason': 'single brief stall, fleet still progressing'},
    },
    {
        'level': 'WARN',
        'event': 'robot_11 momentarily deviated 0.4m from path',
        'log_buffer': [
            '[t=5000ms status=executing arrived=12 active=8 stall=0 replans=1]',
            '[t=6200ms status=executing arrived=12 active=8 stall=0 replans=1] WARN: robot_11 deviated 0.4m',
            '[t=7400ms status=executing arrived=13 active=7 stall=0 replans=1]',
        ],
        'decision': {'decision': 'wait', 'reason': 'small deviation already absorbed'},
    },
    # WARN -> replan
    {
        'level': 'WARN',
        'event': 'robot_3 stalled for 15s, 3 replans already tried',
        'log_buffer': [
            '[t=2000ms status=executing arrived=7 active=13 stall=1 replans=1] WARN: robot_3 stalled',
            '[t=5000ms status=executing arrived=7 active=13 stall=1 replans=2] WARN: robot_3 stalled again',
            '[t=9000ms status=replanning arrived=7 active=13 stall=1 replans=3] WARN: robot_3 stalled for 15s',
        ],
        'decision': {'decision': 'replan', 'reason': 'same robot stalling repeatedly, arrived count frozen'},
    },
    {
        'level': 'WARN',
        'event': 'stall count is 4 and growing across cluster',
        'log_buffer': [
            '[t=3000ms status=executing arrived=6 active=14 stall=2 replans=1]',
            '[t=6000ms status=executing arrived=6 active=14 stall=3 replans=2] WARN: deadlock forming',
            '[t=9000ms status=executing arrived=6 active=14 stall=4 replans=2] WARN: stall count is 4',
        ],
        'decision': {'decision': 'replan', 'reason': 'multi-robot stall suggests local deadlock'},
    },
    {
        'level': 'WARN',
        'event': 'executing timeout: plan not completed in 120s',
        'log_buffer': [
            '[t=100000ms status=executing arrived=10 active=10 stall=1 replans=2]',
            '[t=120000ms status=executing arrived=10 active=10 stall=2 replans=3] WARN: timeout',
        ],
        'decision': {'decision': 'replan', 'reason': 'timeout with half fleet still en route'},
    },
    # WARN -> abort
    {
        'level': 'WARN',
        'event': 'fatal collision detected near checkpoint C',
        'log_buffer': [
            '[t=1000ms status=executing arrived=4 active=16 stall=0 replans=0]',
            '[t=2200ms status=failed arrived=4 active=0 stall=8 replans=1] WARN: fatal collision',
        ],
        'decision': {'decision': 'abort', 'reason': 'fatal collision is not recoverable'},
    },
    {
        'level': 'WARN',
        'event': 'no valid agents: every robot unplanable',
        'log_buffer': [
            '[t=500ms status=validating arrived=0 active=0 stall=0 replans=0]',
            '[t=1200ms status=failed arrived=0 active=0 stall=0 replans=0] WARN: no valid agents',
        ],
        'decision': {'decision': 'abort', 'reason': 'zero valid agents — goals unreachable'},
    },
    {
        'level': 'WARN',
        'event': 'formation setup failed: leader_ns robot_1 unreachable',
        'log_buffer': [],
        'decision': {'decision': 'abort', 'reason': 'leader unreachable — formation cannot be established'},
    },
    # INFO -> wait
    {
        'level': 'INFO',
        'event': 'planner progressing normally',
        'log_buffer': [
            '[t=1000ms status=executing arrived=2 active=18 stall=0 replans=0] INFO: planner started',
            '[t=2500ms status=executing arrived=8 active=12 stall=0 replans=0] INFO: progressing normally',
            '[t=4000ms status=executing arrived=13 active=7 stall=0 replans=0] INFO: progressing normally',
        ],
        'decision': {'decision': 'wait', 'reason': 'steady progress, no stalls'},
    },
]

# ---------------------------------------------------------------------------
# Channel 2 — command scenarios
# ---------------------------------------------------------------------------

COMMAND_SCENARIOS = [
    {
        'history': [
            '[t=10000ms mode=mapf status=OK action=MapfPlan] [t=1200ms arrived=5 active=15 stall=0]',
            '[t=11000ms mode=mapf status=WARN action=MapfPlan] WARN: robot_3, robot_7 stalled',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'MapfPlan',
            'last_error': 'robot_3, robot_7 stalled repeatedly',
        },
        'command': {
            'mode': 'mapf',
            'robot_ids': [0, 1, 2, 4, 5, 6, 8, 9],
            'goals': [[15.0, 15.0]] * 8,
            'reason': 'replan without stalled robots 3 and 7',
        },
    },
    {
        'history': [
            '[t=4500ms mode=mapf status=OK action=MapfPlan] [t=1000ms arrived=2 active=18]',
            '[t=5000ms mode=mapf status=ERROR action=MapfPlan] fatal collision at (12, 8)',
        ],
        'trigger': {
            'action_status': 'ERROR',
            'active_action': 'MapfPlan',
            'last_error': 'fatal collision — operator intervention required',
        },
        'command': {'mode': 'idle', 'reason': 'halt on fatal error'},
    },
    {
        'history': [
            '[t=2500ms mode=formation status=OK action=SetFormation] configuring wedge',
            '[t=3000ms mode=formation status=WARN action=SetFormation] leader unreachable',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'SetFormation',
            'last_error': 'leader unreachable',
        },
        'command': {
            'mode': 'formation',
            'formation_id': 'line',
            'leader_ns': 'robot_1',
            'follower_ns': ['robot_2', 'robot_3'],
            'offsets_x': [-1.0, -2.0],
            'offsets_y': [0.0, 0.0],
            'reason': 'retry with different leader',
        },
    },
]
