#!/usr/bin/env python3
"""Score E3 session recordings offline.

    python3 paper/results/analyze_sessions.py                 # table, all runs
    python3 paper/results/analyze_sessions.py --cell no-LLM   # one cell
    python3 paper/results/analyze_sessions.py --detail 20260815_141553
    python3 paper/results/analyze_sessions.py --json out.json

WHAT IT SCORES AND WHY THAT DEFINITION
A run's outcome is the final state of /tasks/state restricted to the tasks the
mission actually asked for. Two details of that sentence do the work:

  * /tasks/state, not the MAPF action result. execution_verification.py reports
    "MAPF goals reached", which is whether robots arrived at the coordinates the
    planner was given -- a plan that sends the fleet to confidently wrong places
    scores 100% there. The task manager only flips a task to `done` when a robot
    is inside its radius, so it cannot be satisfied by a self-consistent but
    wrong plan.

  * restricted to the mission's targets. The scenario declares nine tasks; M2
    names five. Scoring M2 against nine would floor every cell at 5/9 and make
    the ablation differences look small; scoring M3 against five would hide the
    four it adds. MISSIONS below is the mapping, and it must stay in sync with
    the mission texts in paper/E3_spec.md section 5.

Everything else reported here is diagnosis, not score: it exists to tell a
failed run apart from a run where the substrate failed. A cell that reads 2/5
because the model planned badly and a cell that reads 2/5 because Nav2 wedged
two robots against a wall are the same number and different papers.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bagreader as B      # noqa: E402


# ─── mission definitions ───────────────────────────────────────────────────
# Target task sets, mirroring the mission texts in paper/E3_spec.md section 5.
# M4 is a formation mission and declares no tasks: it is scored on
# /formations/status, not here, and shows as n/a rather than 0/0.
MISSIONS: dict[str, list[str]] = {
    'M1': ['task_electrical', 'task_comms'],
    'M2': ['task_electrical', 'task_med', 'task_hall', 'task_comms',
           'carry_security_to_admin'],
    'M3': ['task_med', 'task_electrical', 'carry_engine_to_reactor',
           'task_reactor', 'carry_security_to_admin', 'task_hall',
           'task_comms', 'task_o2', 'carry_shields_to_reactor'],
    'M4': [],
}

# GoalStatus constants (action_msgs/msg/GoalStatus).
_GOAL_STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

_CHANNEL = {1: 'decision', 2: 'observer', 3: 'user'}

# A robot below this much travel never left its spawn zone. Chosen well above
# odometry jitter and well below the shortest real leg in the amongus map.
_MOVED_M = 0.5


# ─── per-run analysis ──────────────────────────────────────────────────────

_TOOL_CALLS_RE  = re.compile(r"calling tools \[([^\]]*)\]")
_TOOL_NAME_RE   = re.compile(r"'([a-z_]+)'")
_TOOL_FINAL_RE  = re.compile(r"final text after (\d+) iteration")


def _read_tool_loop(run_dir: str) -> dict:
    """Tool-calling activity, parsed out of launch.log.

    Pilot criterion 4 of section 6 asks whether the model actually calls its
    tools, and the answer lives only here: chat_server logs each round as
    ``tool_loop iter=N: calling tools [...]`` and nothing about tools reaches
    the mission JSONL. `llm_calls` cannot stand in for it -- that counter is
    bumped once per planning step (chat_server.py:1010), not once per network
    round trip, so it reads 1 whether the loop ran once or twenty times.

    Also worth separating: a second loop iteration is not evidence of a tool
    call. The loop re-enters when the model answers with prose or with an
    invalid plan, and both are counted here so that "the loop ran twice" is
    never mistaken for "the model used a tool".
    """
    path = os.path.join(run_dir, 'launch.log')
    out = {'present': False, 'n_calls': 0, 'names': {}, 'iterations': [],
           'prose_retries': 0, 'schema_retries': 0}
    if not os.path.isfile(path):
        return out
    out['present'] = True
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            if 'calling tools' in line:
                m = _TOOL_CALLS_RE.search(line)
                if m:
                    names = _TOOL_NAME_RE.findall(m.group(1))
                    out['n_calls'] += len(names)
                    for n in names:
                        out['names'][n] = out['names'].get(n, 0) + 1
            elif 'prose response detected' in line:
                out['prose_retries'] += 1
            elif 'plan schema error' in line:
                out['schema_retries'] += 1
            elif 'final text after' in line:
                m = _TOOL_FINAL_RE.search(line)
                if m:
                    out['iterations'].append(int(m.group(1)))
    return out


def _read_chat_log(run_dir: str) -> dict:
    """Channel-3 mission records from llm_chat/*.jsonl.

    This is the other half of the score and it is NOT in the bag: the guard,
    remediation, repair and supervision counters are what the ablation cells
    (no-remediation / no-repair / no-supervision) are actually measured on. A
    cell whose disabled mechanism reads zero in the baseline too has measured
    nothing, and that is only visible here.
    """
    d = os.path.join(run_dir, 'llm_chat')
    recs: list[dict] = []
    if os.path.isdir(d):
        for f in sorted(os.listdir(d)):
            if not f.endswith('.jsonl'):
                continue
            with open(os.path.join(d, f), encoding='utf-8') as fh:
                for line in fh:
                    line = line.strip()
                    if line:
                        try:
                            recs.append(json.loads(line))
                        except json.JSONDecodeError:
                            pass

    def num(rec: dict, key: str) -> int:
        try:
            return int(float(rec.get(key) or 0))
        except (TypeError, ValueError):
            return 0

    def truthy(v: object) -> bool:
        return str(v).lower() == 'true'

    # REDUCTION: last record per mission, never a sum.
    #
    # chat_server checkpoints at every phase boundary so a mission killed by
    # the operator still leaves its counters on disk. Those counters are
    # CUMULATIVE -- each checkpoint carries the running totals, not a delta --
    # so summing the rows of one mission multiplies every count by the number
    # of phases it survived. Grouping by mission_id and keeping the last row is
    # the only correct reduction, and it is also right for the older one-row
    # format, where the group has size one.
    #
    # Merged forward rather than last-row-wins: phases carry different fields
    # (only `executing` and `continued` carry plan_json, only `final` carries
    # success/info), so a mission killed after execution would lose its plan
    # under a strict last-row rule. Later rows still overwrite earlier ones,
    # which is what the cumulative counters need.
    #
    # Records without a mission_id are pre-checkpoint files; each is its own
    # mission, keyed by position so they cannot collide.
    missions: dict[str, dict] = {}
    for i, rec in enumerate(recs):
        key = str(rec.get('mission_id') or f'_norec{i}')
        missions[key] = {**missions.get(key, {}), **rec}
    finals = list(missions.values())

    # A mission whose last row is not `final` never reached a terminal path.
    # Its counters are usable; its outcome is not, and scoring it as a failure
    # would turn "the operator stopped watching" into "the model failed".
    killed = [m for m in finals
              if m.get('record_kind') and m.get('record_kind') != 'final']

    total = {k: sum(num(m, k) for m in finals) for k in (
        'llm_calls', 'remediation_attempts', 'verification_repair_attempts',
        'supervision_steps', 'guard_active_formation_fired',
        'guard_occupancy_rewrites', 'guard_formation_staging_fired',
        'guard_single_carrier_fired', 'guard_shared_dropoff_fired')}
    return {
        'present': bool(recs),
        'n_missions': len(finals),
        'n_rows': len(recs),
        'n_killed': len(killed),
        'last_phase': [m.get('record_kind', 'final') for m in finals],
        'totals': total,
        'llm_seconds': round(sum(float(m.get('llm_seconds') or 0)
                                 for m in finals), 1),
        'success': [truthy(m.get('success')) for m in finals],
        'verification_ok': [truthy(m.get('verification_ok'))
                            for m in finals],
        'info': [m.get('info', '') for m in finals],
        'plan_json': [m.get('plan_json', '') for m in finals],
        'plan_shape': [_plan_shape(m.get('plan_json', '')) for m in finals],
        'records': finals,
    }


_LEAF_TYPES = ('mapf', 'formation', 'idle', 'disband')


def _plan_shape(plan_json: str) -> dict:
    """Static properties of one plan tree, read from the committed JSON.

    The point of doing this here is that launch.log is deliberately not
    committed (see .gitignore), so the log line that explains a rejected plan
    does not survive collection. plan_json does. Recomputing the conflict rule
    from the tree means a run that scored zero because plan_executor refused it
    stays diagnosable from the light half of the recording alone.

    Mirrors plan_executor.flatten_parallel: two branches of ONE parallel that
    send the same robot to different goals is a PlanConflictError, and the
    whole parallel is refused -- so the visible outcome is a total failure, not
    a partial one.
    """
    try:
        root = json.loads(plan_json)
    except (json.JSONDecodeError, TypeError):
        return {'parsed': False}

    counts: dict[str, int] = {}
    depth = 0
    conflicts: list[str] = []
    robots: set[int] = set()
    nested_seq = False

    def leaves_under(n: dict) -> list[dict]:
        if n.get('type') in _LEAF_TYPES:
            return [n]
        return [x for s in n.get('steps', []) for x in leaves_under(s)]

    def check_parallel(n: dict) -> None:
        """Same rule the executor applies: concurrent branches only."""
        nonlocal nested_seq
        seen: dict[int, tuple[float, float]] = {}
        for step in n.get('steps', []):
            if step.get('type') == 'sequence':
                nested_seq = True  # serialised, not raced -- the 57c6fd5 path
                continue
            for leaf in leaves_under(step):
                for rid, g in zip(leaf.get('robot_ids') or [],
                                  leaf.get('goals') or []):
                    g = (round(float(g[0]), 2), round(float(g[1]), 2))
                    prev = seen.get(rid)
                    if prev is not None and prev != g:
                        conflicts.append(
                            'robot_%d: %s vs %s' % (rid, prev, g))
                    seen[rid] = g

    def walk(n: dict, d: int = 0) -> None:
        nonlocal depth
        depth = max(depth, d)
        t = n.get('type', '?')
        counts[t] = counts.get(t, 0) + 1
        robots.update(n.get('robot_ids') or [])
        if t == 'parallel':
            check_parallel(n)
        for s in n.get('steps', []):
            walk(s, d + 1)

    walk(root)
    return {
        'parsed': True,
        'root_type': root.get('type'),
        'node_counts': counts,
        'depth': depth,
        'n_leaves': sum(counts.get(t, 0) for t in _LEAF_TYPES),
        'robots_named': sorted(robots),
        'nested_sequence_in_parallel': nested_seq,
        'conflicts': conflicts,
    }


def _bag_path(run_dir: str) -> str | None:
    bag = os.path.join(run_dir, 'bag')
    if not os.path.isdir(bag):
        return None
    for f in sorted(os.listdir(bag)):
        if f.endswith('.db3'):
            return os.path.join(bag, f)
    return None


def _score_tasks(bag: B.Bag, targets: list[str]) -> dict:
    """Final status per task plus the sim time each one reached it.

    /tasks/state is a full snapshot every tick, so the transition time is the
    first tick that shows the new status -- accurate to the 5 Hz poll rate.
    """
    first_seen: dict[tuple[str, str], float] = {}
    final: dict[str, dict] = {}
    t0: float | None = None
    for m in bag.read('/tasks/state'):
        t = m.msg['header']['stamp']
        if t0 is None:
            t0 = t
        for s in m.msg['states']:
            tid, st = s['task']['id'], s['status']
            first_seen.setdefault((tid, st), t)
            final[tid] = {'status': st,
                          'type': s['task']['type'],
                          'assigned': list(s['assigned_robot_ids'])}

    declared = list(final)
    scope = targets if targets else []
    missing = [t for t in scope if t not in final]
    done = [t for t in scope if final.get(t, {}).get('status') == 'done']
    times = {t: round(first_seen[(t, 'done')] - (t0 or 0.0), 1)
             for t in done if (t, 'done') in first_seen}
    # Carry tasks pass through `carrying`; when a run ends mid-carry that is a
    # materially different failure from never having picked the cargo up.
    picked = [t for t in scope
              if final.get(t, {}).get('type') == 'carry'
              and (t, 'carrying') in first_seen]

    return {
        'declared': declared,
        'targets': scope,
        'missing_from_bag': missing,
        'final': final,
        'done': done,
        'n_done': len(done),
        'n_targets': len(scope),
        'done_times_s': times,
        'makespan_s': round(max(times.values()), 1) if times else None,
        'carry_picked_up': picked,
        'incidental_done': sorted(
            t for t, v in final.items()
            if v['status'] == 'done' and t not in scope),
    }


def _score_motion(bag: B.Bag) -> dict:
    """Path length per robot from odometry.

    Distance separates "the model never tasked this robot" from "the model
    tasked it and it never got there", which the task table alone cannot show.
    """
    dist: dict[int, float] = {}
    endpos: dict[int, tuple[float, float]] = {}
    for topic in bag.topics:
        if not (topic.startswith('/robot_') and topic.endswith('/odom')):
            continue
        rid = int(topic.split('/')[1].split('_')[1])
        d = 0.0
        prev: tuple[float, float] | None = None
        last: tuple[float, float] = (0.0, 0.0)
        for m in bag.read(topic):
            x, y, _ = m.msg['position']
            if prev is not None:
                d += math.hypot(x - prev[0], y - prev[1])
            prev = last = (x, y)
        dist[rid] = round(d, 2)
        endpos[rid] = (round(last[0], 2), round(last[1], 2))
    moved = sorted(r for r, d in dist.items() if d >= _MOVED_M)
    return {
        'distance_m': dict(sorted(dist.items())),
        'end_position': dict(sorted(endpos.items())),
        'robots_moved': moved,
        'n_robots_moved': len(moved),
        'total_distance_m': round(sum(dist.values()), 1),
    }


def _score_llm(bag: B.Bag) -> dict:
    if '/llm/events' not in bag.topics:
        return {'present': False, 'n_events': 0, 'by_channel': {}, 'events': []}
    evs = [m.msg for m in bag.read('/llm/events')]
    by: dict[str, int] = {}
    for e in evs:
        k = _CHANNEL.get(e['channel'], str(e['channel']))
        by[k] = by.get(k, 0) + 1
    return {
        'present': True,
        'n_events': len(evs),
        'by_channel': by,
        'events': [{'channel': _CHANNEL.get(e['channel'], e['channel']),
                    'stamp_ms': e['stamp_ms'],
                    'trigger': e['trigger'],
                    'output': e['output'],
                    'reason': e['reason']} for e in evs],
    }


def _score_bt(bag: B.Bag) -> dict:
    if '/bt/state' not in bag.topics:
        return {'present': False}
    status: dict[str, int] = {}
    errors: list[str] = []
    modes: list[str] = []
    thinking = 0
    for m in bag.read('/bt/state'):
        s = m.msg
        status[s['action_status']] = status.get(s['action_status'], 0) + 1
        if s['last_error'] and s['last_error'] not in errors:
            errors.append(s['last_error'])
        if not modes or modes[-1] != s['mode']:
            modes.append(s['mode'])
        thinking += bool(s['llm_thinking'])
    return {
        'present': True,
        'status_ticks': status,
        'n_warn': status.get('WARN', 0),
        'n_error': status.get('ERROR', 0),
        'mode_sequence': modes,
        'distinct_errors': errors,
        'thinking_ticks': thinking,
    }


def _score_mapf(bag: B.Bag) -> dict:
    """One entry per /swarm/set_goals goal: how it ended and what it cost."""
    goals: dict[str, dict] = {}
    fb_topic = '/swarm/set_goals/_action/feedback'
    if fb_topic in bag.topics:
        for m in bag.read(fb_topic):
            f = m.msg
            g = goals.setdefault(f['goal_id'], {
                'arrived': 0, 'stall': 0, 'replans': 0, 'deviated': 0,
                'elapsed_ms': 0, 'statuses': [], 'warnings': [],
                'terminal': None})
            g['arrived'] = max(g['arrived'], f['robots_arrived'])
            g['stall'] = max(g['stall'], f['robot_stall'])
            g['replans'] = max(g['replans'], f['replans_done'])
            g['deviated'] = max(g['deviated'], f['robots_deviated'])
            g['elapsed_ms'] = max(g['elapsed_ms'], f['elapsed_ms'])
            if not g['statuses'] or g['statuses'][-1] != f['status']:
                g['statuses'].append(f['status'])
            if f['warning'] and f['warning'] not in g['warnings']:
                g['warnings'].append(f['warning'])
    st_topic = '/swarm/set_goals/_action/status'
    if st_topic in bag.topics:
        last = bag.last(st_topic)
        if last:
            for e in last.msg['status_list']:
                g = goals.setdefault(e['goal_id'], {
                    'arrived': 0, 'stall': 0, 'replans': 0, 'deviated': 0,
                    'elapsed_ms': 0, 'statuses': [], 'warnings': [],
                    'terminal': None})
                g['terminal'] = _GOAL_STATUS.get(e['status'], e['status'])
    # A goal that ends SUCCEEDED having planned nobody. The action server
    # always calls succeed() and carries the outcome in result.success
    # (mapf_lns2_node.cpp: every failure path does succeed() with
    # success=false), and the result payload is not a recorded topic. So the
    # terminal status alone reads as a clean goal for a refusal -- which is
    # exactly how run 20260818_085836 looked until the launch.log was read.
    # Zero arrivals with no elapsed time is the signature to surface.
    refused = [
        gid for gid, g in goals.items()
        if g['terminal'] == 'SUCCEEDED' and not g['arrived']
        and not g['elapsed_ms'] and not g['stall']
    ]
    return {
        'n_goals': len(goals),
        'goals': goals,
        'refused': refused,
        'total_replans': sum(g['replans'] for g in goals.values()),
        'total_stall': sum(g['stall'] for g in goals.values()),
        'terminal': [g['terminal'] for g in goals.values()],
        'warnings': sorted({w for g in goals.values() for w in g['warnings']}),
    }


# Narrow stretches of the M4 route, measured off amongus.pgm at its native
# 0.05 m along the corridor line x = 2.70 (leader y, metres). Everything else
# on the traverse is wide. eps_ss is defined on the wide part only (section
# 3.1): a steady-state error averaged across the pinch would mix holding the
# column with squeezing it through.
#
# The measurement also corrects the spec. Section "M4" describes the corridor
# as narrowing to 1.30 m; the map says 2.60 m, at y = 1.60, free from x = 1.40
# to x = 4.00. So the lateral budget for the formation is 2.16 m rather than
# 1.08 m after two robot radii. A column is still the conservative choice, but
# it is not the only one that fits.
_M4_NARROW_BANDS = ((1.60, -0.25), (-3.40, -4.10))
_M4_LEADER_X = 2.70


def _in_narrow(y: float) -> bool:
    return any(lo >= y >= hi for lo, hi in _M4_NARROW_BANDS)


def _score_formation(bag: B.Bag) -> dict:
    """Metrics 11-14 of section 3.1, from /formations/status.

    -1.0 in an error field means the monitor had no data for that follower,
    not a zero error. Averaged in as zero it would turn a lost follower into
    perfect tracking -- the worst run into the best -- so those samples are
    dropped and the dropped fraction is reported next to the numbers.

    t_stable and f_degraded depend on the monitor's own thresholds
    (stable_thresh_m 0.15, degraded_thresh_m 0.35) and are meaningless without
    them; eps_ss and eps_peak do not. Both are reported for that reason.
    """
    topic = '/formations/status'
    out: dict[str, Any] = {'present': False}
    if topic not in bag.topics:
        return out

    samples: list[tuple[float, int, list[float]]] = []   # t, state, valid errors
    dropped = kept = 0
    leaders: set[str] = set()
    for m in bag.read(topic):
        for f in m.msg['formations']:
            errs = [e for e in f['errors_m'] if e >= 0.0]
            dropped += sum(1 for e in f['errors_m'] if e < 0.0)
            kept += len(errs)
            samples.append((m.t, f['state'], errs))
            if f['leader_ns']:
                leaders.add(f['leader_ns'].strip('/'))
    if not samples:
        return out

    active = [s for s in samples if s[1] != 0]           # not INACTIVE
    out['present'] = True
    out['n_samples'] = len(samples)
    out['n_active'] = len(active)
    out['dropped_frac'] = (round(dropped / (dropped + kept), 4)
                           if (dropped + kept) else 0.0)
    if not active:
        return out

    # 13: activation (first non-INACTIVE) to first STABLE.
    t_act = active[0][0]
    t_stable = next((t for t, st, _ in active if st == 2), None)
    out['t_stable_s'] = round(t_stable - t_act, 2) if t_stable else None

    # 14: share of active samples in DEGRADED. Sample-counted rather than
    # time-weighted -- the monitor publishes at a fixed 10 Hz, so the two are
    # the same unless it stalls, and a stalled monitor should not be silently
    # smoothed over.
    out['f_degraded'] = round(
        sum(1 for _, st, _ in active if st == 3) / len(active), 4)
    out['f_broken'] = round(
        sum(1 for _, st, _ in active if st == 4) / len(active), 4)

    all_err = [e for _, _, es in active for e in es]
    out['eps_peak_m'] = round(max(all_err), 3) if all_err else None
    out['eps_all_median_m'] = (round(statistics.median(all_err), 3)
                               if all_err else None)

    # 11: eps_ss on the wide part only, which needs to know where the leader
    # was at each sample. The status message names the leader but not its
    # position, so it is read back out of the leader's odom.
    out['leaders'] = sorted(leaders)
    track: list[tuple[float, float]] = []
    for ns in sorted(leaders):
        odom = f'/{ns}/odom'
        if odom in bag.topics:
            track = [(m.t, m.msg['position'][1]) for m in bag.read(odom)]
            break
    if track:
        wide, narrow = [], []
        k = 0
        for ts, _, es in active:
            while k + 1 < len(track) and track[k + 1][0] <= ts:
                k += 1
            (narrow if _in_narrow(track[k][1]) else wide).extend(es)
        out['eps_ss_wide_m'] = (round(statistics.median(wide), 3)
                                if wide else None)
        out['eps_narrow_m'] = (round(statistics.median(narrow), 3)
                               if narrow else None)
        out['n_wide'], out['n_narrow'] = len(wide), len(narrow)
    else:
        out['eps_ss_wide_m'] = None
        out['problems_note'] = 'leader odom missing, eps_ss falls back to eps_all'
    return out


def _is_live(run_dir: str) -> bool:
    """True while a run is still recording.

    Opening a bag that rosbag2 is writing can kill the recorder (see the note
    in bagreader.Bag.__init__), and a half-written bag scores wrong anyway.
    session.json gets `finished_at` only when record_session.sh shuts the
    stack down, so its absence is the signal. A directory with no session.json
    at all is treated as live: it is more likely a run being set up than a
    finished one.
    """
    sj = os.path.join(run_dir, 'session.json')
    if not os.path.isfile(sj):
        return True
    try:
        with open(sj, encoding='utf-8') as fh:
            return not json.load(fh).get('finished_at')
    except Exception:
        return True


def analyse_run(run_dir: str) -> dict:
    """Score one session directory. Never raises on a broken run -- a run that
    cannot be scored has to appear in the table as unscorable, not vanish."""
    out: dict[str, Any] = {'stamp': os.path.basename(run_dir.rstrip('/')),
                           'dir': run_dir, 'problems': []}

    sj = os.path.join(run_dir, 'session.json')
    if os.path.isfile(sj):
        with open(sj, encoding='utf-8') as fh:
            s = json.load(fh)
        out['session'] = s
        out['cell'] = s.get('cell')
        out['mission'] = s.get('mission')
        out['repeat'] = s.get('repeat_index')
        out['planner'] = s.get('planner')
        out['model'] = (s.get('llm_model') or {}).get('value')
        out['provider'] = (s.get('llm_routing') or {}).get(
            'openrouter_provider')
        out['operator_commands'] = s.get('operator_commands_logged')
        out['git_commit'] = (s.get('git_commit') or '')[:8]
        out['git_dirty'] = s.get('git_dirty')
        out['factors'] = {k: v.get('value') for k, v in
                          (s.get('ablation_factors') or {}).items()}
        if not s.get('bag_finalised'):
            out['problems'].append('bag not finalised')
        if not s.get('finished_at'):
            out['problems'].append('no finished_at (run killed?)')
    else:
        out['problems'].append('no session.json')
        out['cell'] = out['mission'] = None

    # Operator notes live beside the runs; surface them so a deviation from the
    # protocol travels with the number it affects instead of being lost in a
    # README nobody opens next to a table.
    notes = os.path.join(run_dir, 'NOTE.md')
    if os.path.isfile(notes):
        with open(notes, encoding='utf-8') as fh:
            out['operator_note'] = fh.read().strip()

    out['chat'] = _read_chat_log(run_dir)
    out['tools'] = _read_tool_loop(run_dir)
    if out['cell'] and out['cell'] != 'no-LLM' and not out['chat']['present']:
        out['problems'].append(
            'no llm_chat/*.jsonl -- guard and remediation counters lost, '
            'this run cannot score an ablation cell')
    if out['chat'].get('n_killed'):
        out['problems'].append(
            'mission killed mid-flight, last phase %s -- counters are valid '
            'up to that point but the outcome is unknown, do not score it as '
            'a failure' % ', '.join(
                p for p in out['chat']['last_phase'] if p != 'final'))
    for sh in out['chat']['plan_shape']:
        if sh.get('parsed') and sh['conflicts']:
            out['problems'].append(
                'plan refused by the executor: one parallel sends '
                + '; '.join(sh['conflicts'])
                + ' -- the whole branch is dropped, so this run scores zero '
                  'for a plan that was otherwise sound')

    path = _bag_path(run_dir)
    if not path:
        out['problems'].append('no bag (.db3 stripped or never recorded)')
        return out
    out['bag'] = path
    out['bag_bytes'] = os.path.getsize(path)

    with B.Bag(path) as bag:
        lo, hi = bag.span()
        out['wall_duration_s'] = round(hi - lo, 1)
        targets = MISSIONS.get(out.get('mission') or '', [])
        if out.get('mission') and out['mission'] not in MISSIONS:
            out['problems'].append(f"unknown mission {out['mission']}")
        if '/tasks/state' in bag.topics:
            out['tasks'] = _score_tasks(bag, targets)
            if out['tasks']['missing_from_bag']:
                out['problems'].append(
                    'mission targets absent from /tasks/state: '
                    + ', '.join(out['tasks']['missing_from_bag']))
        else:
            out['problems'].append('/tasks/state not recorded -- unscorable')
        out['motion'] = _score_motion(bag)
        out['llm'] = _score_llm(bag)
        out['bt'] = _score_bt(bag)
        out['mapf'] = _score_mapf(bag)
        out['formation'] = _score_formation(bag)

    # Cross-checks the operator cannot see while the run is happening.
    cell = out.get('cell')
    if cell == 'no-LLM':
        if out['llm']['n_events']:
            out['problems'].append(
                f"no-LLM cell recorded {out['llm']['n_events']} LLM events")
    elif cell and out['llm'].get('present') and not out['llm']['n_events']:
        out['problems'].append(
            'LLM cell with zero /llm/events -- channel 3 not captured')
    if cell and cell != 'no-LLM' and not out.get('operator_commands'):
        out['problems'].append(
            'operator_commands_logged == 0 -- mission text may not have been '
            'sent through the panel')
    tools = out.get('tools') or {}
    factors = (out.get('session') or {}).get('ablation_factors') or {}
    tool_arm = (factors.get('tool_calling_enabled') or {}).get('value')
    if cell and cell != 'no-LLM' and tool_arm and tools.get('present'):
        if not tools.get('n_calls'):
            out['problems'].append(
                'tool_calling is on but the model called no tool at all '
                '(section 6, criterion 4) -- the tool-calling arm is doing '
                'the same thing the constrained arm would')
    elif cell and cell != 'no-LLM' and tool_arm and not tools.get('present'):
        out['problems'].append(
            'no launch.log -- whether the model used its tools cannot be '
            'told for this run')
    refused = (out.get('mapf') or {}).get('refused') or []
    if refused:
        out['problems'].append(
            f'{len(refused)} MAPF goal(s) ended SUCCEEDED having planned '
            'nobody -- the planner refused them (result.success is false but '
            'the result payload is not recorded); read launch.log for the '
            'reason, and do not read the terminal status as a clean goal')
    return out


# ─── reporting ─────────────────────────────────────────────────────────────

def _score_str(r: dict) -> str:
    t = r.get('tasks')
    if not t:
        return '  --'
    if not t['n_targets']:
        return ' n/a'
    return f"{t['n_done']}/{t['n_targets']}"


def print_table(runs: list[dict]) -> None:
    hdr = ('run', 'cell', 'msn', 'rep', 'score', 'mkspn', 'dur',
           'moved', 'dist', 'calls', 'tools', 'remed', 'repair', 'guard',
           'warn', 'err', 'replan', 'stall', '!')
    rows = []
    for r in runs:
        t, m = r.get('tasks'), r.get('motion') or {}
        bt, mp = r.get('bt') or {}, r.get('mapf') or {}
        ct = (r.get('chat') or {}).get('totals') or {}
        guard = sum(ct.get(k, 0) for k in (
            'guard_active_formation_fired', 'guard_occupancy_rewrites',
            'guard_formation_staging_fired', 'guard_single_carrier_fired',
            'guard_shared_dropoff_fired'))
        rows.append((
            r['stamp'], str(r.get('cell')), str(r.get('mission')),
            str(r.get('repeat')), _score_str(r),
            str(t['makespan_s']) if t and t.get('makespan_s') else '-',
            str(r.get('wall_duration_s', '-')),
            str(m.get('n_robots_moved', '-')),
            str(m.get('total_distance_m', '-')),
            str(ct.get('llm_calls', '-')),
            str((r.get('tools') or {}).get('n_calls', '-')
                if (r.get('tools') or {}).get('present') else '-'),
            str(ct.get('remediation_attempts', '-')),
            str(ct.get('verification_repair_attempts', '-')),
            str(guard) if ct else '-',
            str(bt.get('n_warn', '-')), str(bt.get('n_error', '-')),
            str(mp.get('total_replans', '-')), str(mp.get('total_stall', '-')),
            str(len(r['problems'])) if r['problems'] else ''))
    w = [max(len(str(x[i])) for x in [hdr] + rows) for i in range(len(hdr))]
    line = '  '.join('-' * n for n in w)
    print('  '.join(str(h).ljust(w[i]) for i, h in enumerate(hdr)))
    print(line)
    for row in rows:
        print('  '.join(str(c).ljust(w[i]) for i, c in enumerate(row)))
    print(line)


def print_cells(runs: list[dict]) -> None:
    by: dict[tuple[str, str], list[dict]] = {}
    for r in runs:
        by.setdefault((str(r.get('cell')), str(r.get('mission'))), []).append(r)
    print()
    print('per cell')
    print('%-12s %-4s %3s  %-11s  %-9s  %s'
          % ('cell', 'msn', 'n', 'mean score', 'full runs', 'mean makespan'))
    print('-' * 68)
    for (cell, msn), rs in sorted(by.items()):
        scored = [r for r in rs if r.get('tasks') and r['tasks']['n_targets']]
        if not scored:
            print('%-12s %-4s %3d  %-11s  %-9s  %s'
                  % (cell, msn, len(rs), 'n/a', '-', '-'))
            continue
        n_t = scored[0]['tasks']['n_targets']
        fr = [r['tasks']['n_done'] / n_t for r in scored]
        full = sum(1 for x in fr if x == 1.0)
        mks = [r['tasks']['makespan_s'] for r in scored
               if r['tasks'].get('makespan_s')]
        sd = f' ±{statistics.stdev(fr) * 100:.0f}' if len(fr) > 1 else ''
        print('%-12s %-4s %3d  %5.1f%%%-5s  %d/%-7d  %s'
              % (cell, msn, len(rs), statistics.mean(fr) * 100, sd,
                 full, len(scored),
                 f'{statistics.mean(mks):.0f} s' if mks else '-'))
    print()


def print_mechanism_check(runs: list[dict]) -> None:
    """The E3 section 6 kill criterion, evaluated on the data instead of by eye.

    An ablation cell can only measure a mechanism that fires in the baseline.
    If remediation never runs when it is enabled, the `no-remediation` cell is
    a re-run of the baseline with a different label, and its five runs buy
    nothing. Reporting this next to the scores keeps a null result from being
    read as "the mechanism does not matter".
    """
    keys = ('remediation_attempts', 'verification_repair_attempts',
            'guard_active_formation_fired', 'guard_occupancy_rewrites',
            'guard_formation_staging_fired', 'guard_single_carrier_fired',
            'guard_shared_dropoff_fired')
    live = [r for r in runs if (r.get('chat') or {}).get('present')]
    if not live:
        return
    print('mechanism activity (E3 section 6 kill criterion)')
    print('-' * 68)
    print('%-34s %6s  %s' % ('mechanism', 'total', 'runs where it fired'))
    dead = []
    for k in keys:
        vals = [(r['stamp'], (r['chat']['totals'].get(k, 0))) for r in live]
        fired = [s for s, v in vals if v]
        tot = sum(v for _, v in vals)
        print('%-34s %6d  %d/%d' % (k, tot, len(fired), len(live)))
        if not tot:
            dead.append(k)
    if dead:
        print(f'\n  {len(dead)} of {len(keys)} mechanisms never fired across '
              f'{len(live)} LLM runs:')
        for k in dead:
            print(f'    - {k}')
        print('  An ablation cell for any of these measures nothing yet.')
    print()


def print_problems(runs: list[dict]) -> None:
    bad = [r for r in runs if r['problems']]
    if not bad:
        return
    print('problems')
    print('-' * 68)
    for r in bad:
        print(f"{r['stamp']}  [{r.get('cell')}]")
        for p in r['problems']:
            print(f'    ! {p}')
    print()


def print_detail(r: dict) -> None:
    print('=' * 74)
    print(f"{r['stamp']}   cell={r.get('cell')}  mission={r.get('mission')}"
          f"  rep={r.get('repeat')}")
    print(f"model={r.get('model')}  provider={r.get('provider')}"
          f"  planner={r.get('planner')}")
    print(f"commit={r.get('git_commit')}"
          f"{' (dirty)' if r.get('git_dirty') else ''}"
          f"  duration={r.get('wall_duration_s')} s"
          f"  factors={r.get('factors')}")
    print('=' * 74)
    if r.get('operator_note'):
        print('\noperator note')
        for ln in r['operator_note'].splitlines():
            print('  ' + ln)
    for p in r['problems']:
        print(f'  ! {p}')

    t = r.get('tasks')
    if t:
        print(f"\ntasks   score {_score_str(r)}"
              f"   makespan {t.get('makespan_s')} s")
        for tid in t['declared']:
            f = t['final'][tid]
            mark = '*' if tid in t['targets'] else ' '
            when = t['done_times_s'].get(tid)
            print('  %s %-28s %-8s %-9s %s%s'
                  % (mark, tid, f['type'], f['status'],
                     ','.join(f['assigned']) or '-',
                     f'   +{when}s' if when else ''))
        print('  (* = required by this mission)')
        if t['incidental_done']:
            print('  incidentally done: ' + ', '.join(t['incidental_done']))

    m = r.get('motion')
    if m:
        print(f"\nmotion  {m['n_robots_moved']}/20 robots moved,"
              f" {m['total_distance_m']} m total")
        movers = [f'r{k}:{v}' for k, v in m['distance_m'].items()
                  if v >= _MOVED_M]
        print('  ' + ('  '.join(movers) if movers else 'nobody moved'))
        idle = [f'r{k}' for k, v in m['distance_m'].items() if v < _MOVED_M]
        if idle:
            print('  idle: ' + ' '.join(idle))

    mp = r.get('mapf')
    if mp and mp['n_goals']:
        print(f"\nmapf    {mp['n_goals']} goal(s)")
        for gid, g in mp['goals'].items():
            print('  %s  %-9s arrived=%d stall=%d replans=%d dev=%d  %.0fs'
                  % (gid[:8], g['terminal'], g['arrived'], g['stall'],
                     g['replans'], g['deviated'], g['elapsed_ms'] / 1000))
            for w in g['warnings']:
                print(f'      warn: {w}')

    bt = r.get('bt')
    if bt and bt.get('present'):
        print(f"\nbt      ticks {bt['status_ticks']}"
              f"  thinking={bt['thinking_ticks']}")
        print('  modes: ' + ' -> '.join(bt['mode_sequence']))
        for e in bt['distinct_errors']:
            print(f'  error: {e}')

    fm = r.get('formation') or {}
    if fm.get('present') and fm.get('n_active'):
        print(f"\nstroy   {fm['n_active']}/{fm['n_samples']} active samples,"
              f" leader {', '.join(fm.get('leaders') or ['?'])}")
        print('  11 eps_ss (wide)   = %s m   over %s samples'
              % (fm.get('eps_ss_wide_m'), fm.get('n_wide', '-')))
        print('     eps in the pinch= %s m   over %s samples'
              % (fm.get('eps_narrow_m'), fm.get('n_narrow', '-')))
        print('     eps all         = %s m' % fm.get('eps_all_median_m'))
        print('  12 eps_peak        = %s m' % fm.get('eps_peak_m'))
        print('  13 t_stable        = %s s' % fm.get('t_stable_s'))
        print('  14 f_degraded      = %s   (f_broken %s)'
              % (fm.get('f_degraded'), fm.get('f_broken')))
        print('     dropped samples = %s  (-1.0 = no data, excluded)'
              % fm.get('dropped_frac'))

    tl = r.get('tools') or {}
    if tl.get('present'):
        names = ('  '.join(f'{k}x{v}' for k, v in sorted(tl['names'].items()))
                 or 'none called')
        print(f"\ntools   {tl['n_calls']} tool call(s): {names}")
        print('  loop iterations per step: %s   prose retries: %d   '
              'schema retries: %d'
              % (tl['iterations'] or '-', tl['prose_retries'],
                 tl['schema_retries']))

    ch = r.get('chat') or {}
    if ch.get('present'):
        print(f"\nchat    {ch['n_missions']} mission record(s),"
              f" {ch['llm_seconds']} s in the LLM")
        print('  ' + '  '.join(f'{k}={v}' for k, v in ch['totals'].items()))
        for i, inf in enumerate(ch['info']):
            print(f"  [{i}] success={ch['success'][i]}"
                  f" verified={ch['verification_ok'][i]}: {inf}")
            sh = ch['plan_shape'][i]
            if not sh.get('parsed'):
                print('      plan: unparsable JSON')
                continue
            print('      plan: root=%s depth=%d leaves=%d robots=%d %s'
                  % (sh['root_type'], sh['depth'], sh['n_leaves'],
                     len(sh['robots_named']),
                     'seq-in-parallel' if sh['nested_sequence_in_parallel']
                     else ''))
            for c in sh['conflicts']:
                print(f'      CONFLICT {c}')

    llm = r.get('llm') or {}
    print(f"\nllm     {llm.get('n_events', 0)} events {llm.get('by_channel')}")
    for e in llm.get('events', []):
        print(f"  [{e['channel']}] trigger: {e['trigger'][:160]}")
        print(f"      output: {e['output'][:400]}")
        if e['reason']:
            print(f"      reason: {e['reason'][:300]}")
    print()


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sessions', default=os.path.join(here, 'sessions'),
                    help='sessions root (default: paper/results/sessions)')
    ap.add_argument('--cell', action='append',
                    help='restrict to a cell; repeatable')
    ap.add_argument('--detail', action='append',
                    help='full breakdown for a run stamp; repeatable')
    ap.add_argument('--json', help='write the full analysis to this path')
    args = ap.parse_args()

    if not os.path.isdir(args.sessions):
        print(f'no sessions directory: {args.sessions}', file=sys.stderr)
        return 1
    dirs = sorted(os.path.join(args.sessions, d)
                  for d in os.listdir(args.sessions)
                  if os.path.isdir(os.path.join(args.sessions, d)))
    if not dirs:
        print(f'no runs under {args.sessions}', file=sys.stderr)
        return 1

    live = [d for d in dirs if _is_live(d)]
    for d in live:
        print(f'skipping {os.path.basename(d)}: still recording '
              '(reading its bag would kill the recorder)', file=sys.stderr)
    runs = [analyse_run(d) for d in dirs if d not in set(live)]
    if args.cell:
        want = {c.lower() for c in args.cell}
        runs = [r for r in runs if str(r.get('cell')).lower() in want]
    if not runs:
        print('no runs matched', file=sys.stderr)
        return 1

    if args.detail:
        want = set(args.detail)
        for r in runs:
            if r['stamp'] in want:
                print_detail(r)
    else:
        print_table(runs)
        print_cells(runs)
        print_mechanism_check(runs)
        print_problems(runs)

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(runs, fh, indent=2, ensure_ascii=False)
        print(f'wrote {args.json}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
