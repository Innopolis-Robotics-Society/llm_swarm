"""Few-shot scenarios for the command-channel prompt.

Each scenario has:
  history : list[str]            — one line per prior /bt/state snapshot
  trigger : dict                  — the fresh BTState that triggered the LLM
  command : dict                  — ground-truth JSON we expect the LLM to emit
"""


COMMAND_SCENARIOS = [
    # --- 1. WARN in MapfPlan: one robot stalling → replan without it ---
    {
        'history': [
            '[t=10000ms mode=mapf status=OK action=MapfPlan] '
            '[t=1200ms status=1 arrived=5 active=15 stall=0 replans=0]',
            '[t=10500ms mode=mapf status=OK action=MapfPlan] '
            '[t=1700ms status=1 arrived=6 active=14 stall=0 replans=0]',
            '[t=11000ms mode=mapf status=WARN action=MapfPlan] '
            '[t=2400ms status=1 arrived=6 active=14 stall=1 replans=0] '
            'WARN: robot_3 stalled',
            '[t=11500ms mode=mapf status=WARN action=MapfPlan] '
            '[t=2900ms status=1 arrived=6 active=14 stall=2 replans=0] '
            'WARN: robot_3, robot_7 stalled',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'MapfPlan',
            'last_error': 'robot_3, robot_7 stalled repeatedly',
        },
        'command': {
            'mode': 'mapf',
            'robot_ids': [0, 1, 2, 4, 5, 6, 8, 9],
            'goals': [
                [15.0, 15.0], [16.0, 15.0], [17.0, 15.0], [18.0, 15.0],
                [15.0, 16.0], [16.0, 16.0], [17.0, 16.0], [18.0, 16.0],
            ],
            'reason': 'replan without stalled robots 3 and 7',
        },
    },

    # --- 2. ERROR in MapfPlan: fatal → halt ---
    {
        'history': [
            '[t=4500ms mode=mapf status=OK action=MapfPlan] '
            '[t=1000ms status=1 arrived=2 active=18 stall=0 replans=0]',
            '[t=5000ms mode=mapf status=ERROR action=MapfPlan] '
            'fatal collision at (12, 8)',
        ],
        'trigger': {
            'action_status': 'ERROR',
            'active_action': 'MapfPlan',
            'last_error': 'fatal collision — operator intervention required',
        },
        'command': {
            'mode': 'idle',
            'reason': 'halt on fatal error, waiting for operator',
        },
    },

    # --- 3. WARN in SetFormation: leader unreachable → reform with new leader ---
    {
        'history': [
            '[t=2500ms mode=formation status=OK action=SetFormation] '
            'configuring wedge with leader robot_0',
            '[t=3000ms mode=formation status=WARN action=SetFormation] '
            'set formation failed: leader unreachable',
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
            'reason': 'retry with different leader since robot_0 unreachable',
        },
    },

    # --- 4. ERROR in SetFormation: retries exhausted → idle ---
    {
        'history': [
            '[t=8000ms mode=formation status=WARN action=SetFormation] retry 1/3',
            '[t=8500ms mode=formation status=WARN action=SetFormation] retry 2/3',
            '[t=9000ms mode=formation status=ERROR action=SetFormation] '
            'retry budget exhausted',
        ],
        'trigger': {
            'action_status': 'ERROR',
            'active_action': 'SetFormation',
            'last_error': 'retry budget exhausted',
        },
        'command': {
            'mode': 'idle',
            'reason': 'stop attempting formation, bring fleet to idle',
        },
    },

    # --- 5. WARN from MapfPlan with excessive replans → switch to formation ---
    {
        'history': [
            '[t=20000ms mode=mapf status=OK action=MapfPlan] '
            '[t=5000ms status=1 arrived=3 active=17 stall=0 replans=3]',
            '[t=21000ms mode=mapf status=WARN action=MapfPlan] '
            '[t=6000ms status=1 arrived=3 active=17 stall=0 replans=5] '
            'WARN: replans_done=5',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'MapfPlan',
            'last_error': 'replans_done=5, environment too dynamic',
        },
        'command': {
            'mode': 'formation',
            'formation_id': 'wedge',
            'leader_ns': 'robot_0',
            'follower_ns': ['robot_1', 'robot_2'],
            'offsets_x': [-1.0, -1.0],
            'offsets_y': [1.0, -1.0],
            'reason': 'MAPF unstable, fall back to formation navigation',
        },
    },

    # --- 6. WARN in DisableFormation → retry disable via formation mode swap ---
    {
        'history': [
            '[t=6000ms mode=formation status=OK action=DisableFormation] disbanding',
            '[t=6500ms mode=formation status=WARN action=DisableFormation] '
            'service unavailable',
        ],
        'trigger': {
            'action_status': 'WARN',
            'active_action': 'DisableFormation',
            'last_error': 'formation service unavailable',
        },
        'command': {
            'mode': 'idle',
            'reason': 'let formation service recover, then operator retries',
        },
    },
]
