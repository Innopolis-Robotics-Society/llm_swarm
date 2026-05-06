"""Few-shot examples for channel 1 (LlmDecision) and channel 2 (LlmCommand).

  DECISION_SCENARIOS  — used by decision prompt builder (channel 1).
  COMMAND_SCENARIOS   — used by command prompt builder (channel 2).
"""

# ---------------------------------------------------------------------------
# Channel 1 — decision scenarios
# ---------------------------------------------------------------------------

DECISION_SCENARIOS = [
    # ---- MAPF: WARN -> wait ----
    {
        'level': 'WARN',
        'event': 'robot_3 stalled for 5s at (12.4, 8.1)',
        'log_buffer': [
            '[t=1200ms status=executing arrived=5 active=15 stall=0 replans=0]',
            '[t=2400ms status=executing arrived=8 active=12 stall=0 replans=0]',
            '[t=3600ms status=executing arrived=8 active=12 stall=1 replans=0] WARN: robot_3 stalled for 5s',
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
    # ---- MAPF: WARN -> replan ----
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
    # ---- MAPF: WARN -> abort ----
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
    # ---- Formation setup: WARN -> abort ----
    {
        'level': 'WARN',
        'event': 'formation setup failed: leader_ns robot_1 unreachable',
        'log_buffer': [],
        'decision': {'decision': 'abort', 'reason': 'leader unreachable — formation cannot be established'},
    },
    # ---- Formation monitor: DEGRADED -> wait ----
    # Followers are still converging just after activation — give it time.
    {
        'level': 'WARN',
        'event': 'formation degraded, max_error=0.42m',
        'log_buffer': [
            '[formation=line state=FORMING max_error=0.28m mean_error=0.19m]',
            '[formation=line state=DEGRADED max_error=0.42m mean_error=0.31m] WARN: formation degraded',
        ],
        'decision': {'decision': 'wait', 'reason': 'formation just activated, followers still converging'},
    },
    # ---- Formation monitor: DEGRADED persistent -> replan ----
    # Error not decreasing over multiple ticks — something is blocking convergence.
    {
        'level': 'WARN',
        'event': 'formation degraded, max_error=0.48m',
        'log_buffer': [
            '[formation=wedge state=DEGRADED max_error=0.45m mean_error=0.38m] WARN: degraded',
            '[formation=wedge state=DEGRADED max_error=0.47m mean_error=0.39m] WARN: degraded',
            '[formation=wedge state=DEGRADED max_error=0.48m mean_error=0.40m] WARN: degraded, not improving',
        ],
        'decision': {'decision': 'replan', 'reason': 'error growing over multiple ticks — retry with adjusted parameters'},
    },
    # ---- Formation monitor: BROKEN follower stuck -> replan ----
    {
        'level': 'ERROR',
        'event': 'formation broken: Follower(s) stuck: [robot_2]',
        'log_buffer': [
            '[formation=line state=DEGRADED max_error=0.52m] robot_2 error not improving',
            '[formation=line state=BROKEN failure=FOLLOWER_STUCK] ERROR: robot_2 stuck',
        ],
        'decision': {'decision': 'replan', 'reason': 'follower stuck — retry excluding robot_2 or with a different leader'},
    },
    # ---- Formation monitor: BROKEN leader lost -> abort ----
    {
        'level': 'ERROR',
        'event': 'formation broken: Leader robot_0 odom timeout (2.3s)',
        'log_buffer': [
            '[formation=wedge state=STABLE max_error=0.09m]',
            '[formation=wedge state=BROKEN failure=LEADER_LOST] ERROR: robot_0 odom timeout 2.3s',
        ],
        'decision': {'decision': 'abort', 'reason': 'leader odometry lost — formation cannot be maintained without localization'},
    },
    # ---- INFO -> wait ----
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
    # ---- MAPF stall -> replan without stuck robots ----
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
    # ---- MAPF fatal -> idle ----
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
    # ---- Formation setup fail -> retry with different leader ----
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
    # ---- Formation DEGRADED (persistent) -> retry with wider offsets ----
    # Followers may be blocked by obstacles at tight offsets — widen spacing.
    {
        'history': [
            '[t=5000ms mode=formation status=OK action=none formation=line state=FORMING max_error=0.31m]',
            '[t=8000ms mode=formation status=WARN action=none formation=line state=DEGRADED max_error=0.47m]',
            '[t=11000ms mode=formation status=WARN action=none formation=line state=DEGRADED max_error=0.49m]',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'none',
            'last_error': 'formation degraded, max_error=0.49m',
        },
        'command': {
            'mode': 'formation',
            'formation_id': 'line',
            'leader_ns': 'robot_0',
            'follower_ns': ['robot_1', 'robot_2', 'robot_3'],
            'offsets_x': [-2.0, -4.0, -6.0],
            'offsets_y': [0.0, 0.0, 0.0],
            'reason': 'retry with wider inter-robot spacing to reduce congestion',
        },
    },
    # ---- Formation BROKEN follower stuck -> regroup then reform without stuck robot ----
    {
        'history': [
            '[t=6000ms mode=formation status=OK action=none formation=wedge state=STABLE max_error=0.11m]',
            '[t=9000ms mode=formation status=ERROR action=none formation=wedge state=BROKEN failure=FOLLOWER_STUCK]',
        ],
        'trigger': {
            'action_status': 'ERROR',
            'active_action': 'none',
            'last_error': 'formation broken: Follower(s) stuck: [robot_2]',
        },
        'command': {
            'mode': 'formation',
            'formation_id': 'wedge',
            'leader_ns': 'robot_0',
            'follower_ns': ['robot_1', 'robot_3'],
            'offsets_x': [-1.0, -1.0],
            'offsets_y': [0.6, -0.6],
            'reason': 'reform wedge excluding stuck robot_2',
        },
    },
    # ---- Formation BROKEN leader lost -> idle, operator must intervene ----
    {
        'history': [
            '[t=7000ms mode=formation status=OK action=none formation=line state=STABLE max_error=0.08m]',
            '[t=8000ms mode=formation status=ERROR action=none formation=line state=BROKEN failure=LEADER_LOST]',
        ],
        'trigger': {
            'action_status': 'ERROR',
            'active_action': 'none',
            'last_error': 'formation broken: Leader robot_0 odom timeout (2.1s)',
        },
        'command': {'mode': 'idle', 'reason': 'leader localization lost — halt until operator reassigns leader'},
    },
]