"""
Library of few-shot examples for the decision prompt.

Covers the categories called out in the TZ:

  WARN -> wait    single, short, self-resolving issue
  WARN -> replan  repeated failures on same robot or plan clearly stale
  WARN -> abort   fatal / unrecoverable situation
  INFO -> wait    periodic heartbeat while everything is fine
  formation WARN  service-level WARN from SetFormation / DisableFormation

Keep one entry per distinct decision rationale — the prompt packs all of
them into every request, so adding duplicates wastes tokens.
"""

SCENARIOS = [
    # ---- WARN -> wait ----------------------------------------------------
    {
        'level': 'WARN',
        'event': 'robot_3 stalled for 5s at (12.4, 8.1)',
        'log_buffer': [
            '[t=1200ms status=executing arrived=5 active=15 stall=0 replans=0]',
            '[t=2400ms status=executing arrived=8 active=12 stall=0 replans=0]',
            '[t=3600ms status=executing arrived=8 active=12 stall=1 replans=0] WARN: robot_3 stalled for 5s at (12.4, 8.1)',
        ],
        'decision': {
            'decision': 'wait',
            'reason': 'single brief stall, nothing else has degraded, fleet still progressing',
        },
    },
    {
        'level': 'WARN',
        'event': 'robot_11 momentarily deviated 0.4m from path',
        'log_buffer': [
            '[t=5000ms status=executing arrived=12 active=8 stall=0 replans=1]',
            '[t=6200ms status=executing arrived=12 active=8 stall=0 replans=1] WARN: robot_11 momentarily deviated 0.4m from path',
            '[t=7400ms status=executing arrived=13 active=7 stall=0 replans=1]',
        ],
        'decision': {
            'decision': 'wait',
            'reason': 'small deviation already absorbed, arrived count keeps growing',
        },
    },

    # ---- WARN -> replan --------------------------------------------------
    {
        'level': 'WARN',
        'event': 'robot_3 stalled for 15s, 3 replans already tried',
        'log_buffer': [
            '[t=2000ms status=executing arrived=7 active=13 stall=1 replans=1] WARN: robot_3 stalled',
            '[t=5000ms status=executing arrived=7 active=13 stall=1 replans=2] WARN: robot_3 stalled again',
            '[t=9000ms status=replanning arrived=7 active=13 stall=1 replans=3] WARN: robot_3 stalled for 15s, 3 replans already tried',
        ],
        'decision': {
            'decision': 'replan',
            'reason': 'same robot stalling repeatedly, arrived count frozen — need fresh goals from upstream',
        },
    },
    {
        'level': 'WARN',
        'event': 'stall count is 4 and growing across cluster',
        'log_buffer': [
            '[t=3000ms status=executing arrived=6 active=14 stall=2 replans=1]',
            '[t=6000ms status=executing arrived=6 active=14 stall=3 replans=2] WARN: deadlock forming in narrow corridor',
            '[t=9000ms status=executing arrived=6 active=14 stall=4 replans=2] WARN: stall count is 4 and growing across cluster',
        ],
        'decision': {
            'decision': 'replan',
            'reason': 'multi-robot stall without progress suggests a local deadlock — fresh MAPF will untangle it',
        },
    },
    {
        'level': 'WARN',
        'event': 'executing timeout: plan not completed in 120s',
        'log_buffer': [
            '[t=100000ms status=executing arrived=10 active=10 stall=1 replans=2]',
            '[t=115000ms status=executing arrived=10 active=10 stall=2 replans=3]',
            '[t=120000ms status=executing arrived=10 active=10 stall=2 replans=3] WARN: executing timeout: plan not completed in 120s',
        ],
        'decision': {
            'decision': 'replan',
            'reason': 'timeout hit with half the fleet still en route — replan likely faster than waiting out more retries',
        },
    },

    # ---- WARN -> abort ---------------------------------------------------
    {
        'level': 'WARN',
        'event': 'fatal collision detected near checkpoint C',
        'log_buffer': [
            '[t=1000ms status=executing arrived=4 active=16 stall=0 replans=0]',
            '[t=2200ms status=failed arrived=4 active=0 stall=8 replans=1] WARN: fatal collision detected near checkpoint C',
        ],
        'decision': {
            'decision': 'abort',
            'reason': 'fatal collision is not recoverable by waiting or replanning',
        },
    },
    {
        'level': 'WARN',
        'event': 'no valid agents: every robot unplanable',
        'log_buffer': [
            '[t=500ms status=validating arrived=0 active=0 stall=0 replans=0]',
            '[t=1200ms status=failed arrived=0 active=0 stall=0 replans=0] WARN: no valid agents: every robot unplanable',
        ],
        'decision': {
            'decision': 'abort',
            'reason': 'zero valid agents — environment or goals are unreachable, no point retrying',
        },
    },
    {
        'level': 'WARN',
        'event': 'formation setup failed: leader_ns robot_1 unreachable',
        'log_buffer': [],
        'decision': {
            'decision': 'abort',
            'reason': 'leader is unreachable — formation cannot be established even after retry',
        },
    },

    # ---- INFO -> wait ----------------------------------------------------
    {
        'level': 'INFO',
        'event': 'planner progressing normally',
        'log_buffer': [
            '[t=1000ms status=executing arrived=2 active=18 stall=0 replans=0] INFO: planner started',
            '[t=2500ms status=executing arrived=8 active=12 stall=0 replans=0] INFO: planner progressing normally',
            '[t=4000ms status=executing arrived=13 active=7 stall=0 replans=0] INFO: planner progressing normally',
        ],
        'decision': {
            'decision': 'wait',
            'reason': 'steady progress, no stalls, no replans — healthy heartbeat',
        },
    },
    {
        'level': 'INFO',
        'event': 'formation holding, max offset error 0.08m',
        'log_buffer': [
            '[t=10000ms status=executing arrived=20 active=0 stall=0 replans=0] INFO: all robots arrived',
            '[t=20000ms status=executing arrived=20 active=0 stall=0 replans=0] INFO: formation holding, max offset error 0.08m',
        ],
        'decision': {
            'decision': 'wait',
            'reason': 'formation stable, all robots arrived, nothing to intervene on',
        },
    },
]
