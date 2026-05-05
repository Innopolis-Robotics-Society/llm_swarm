"""Parsers for LLM output — both the decision channel and the command channel.

decision channel (channel 1)
  parse_llm_decision(raw) -> "wait" | "abort" | "replan"

command channel (channels 2 & 3)
  parse_llm_command(raw) -> validated dict
"""

import json
import re

# ---------------------------------------------------------------------------
# Decision parser (channel 1)
# ---------------------------------------------------------------------------

_VALID_DECISIONS = {'wait', 'abort', 'replan'}
_FENCED_DEC = re.compile(r'```(?:json)?\s*(\{.*?\})\s*```', re.DOTALL | re.IGNORECASE)
_LOOSE_DEC  = re.compile(r'\{[^{}]*"decision"\s*:\s*"[^"]*"[^{}]*\}', re.DOTALL)


def parse_llm_decision(raw: str) -> str:
    """Parse LLM output into one of {wait, abort, replan}.

    Handles fenced JSON, loose JSON, and unknown decision values.
    Falls back to 'wait' (safe default — keeps the BT running).
    """
    if not raw:
        return 'wait'
    text = raw.strip()
    fenced = _FENCED_DEC.search(text)
    candidates = [fenced.group(1)] if fenced else []
    candidates.extend(m.group(0) for m in _LOOSE_DEC.finditer(text))
    for candidate in candidates:
        try:
            obj = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        decision = obj.get('decision', '')
        if not isinstance(decision, str):
            continue
        decision = decision.strip().lower()
        if decision in _VALID_DECISIONS:
            return decision
        if decision == '':
            return 'wait'
    return 'wait'


# ---------------------------------------------------------------------------
# Command parser (channels 2 & 3)
# ---------------------------------------------------------------------------

_VALID_MODES = {'idle', 'mapf', 'formation'}
_FENCED_CMD  = re.compile(r'```(?:json)?\s*(\{.*?\})\s*```', re.DOTALL | re.IGNORECASE)


def parse_llm_command(raw: str) -> dict:
    """Parse LLM command output into a validated dict.

    Expected JSON shapes:
      {"mode": "idle", "reason": "..."}
      {"mode": "mapf",      "robot_ids": [...], "goals": [[x,y],...], ...}
      {"mode": "formation", "formation_id": "...", "leader_ns": "...", ...}

    Raises ValueError on any parse or validation failure.
    """
    if not raw:
        raise ValueError('empty LLM output')
    text = raw.strip()
    fenced = _FENCED_CMD.search(text)
    if fenced:
        candidate = fenced.group(1)
    else:
        start = text.find('{')
        end   = text.rfind('}')
        if start == -1 or end == -1 or end <= start:
            raise ValueError('no JSON object in LLM output')
        candidate = text[start:end + 1]
    try:
        obj = json.loads(candidate)
    except json.JSONDecodeError as exc:
        raise ValueError(f'invalid JSON: {exc}') from exc
    if not isinstance(obj, dict):
        raise ValueError('top-level JSON is not an object')
    mode = str(obj.get('mode', '')).strip().lower()
    if mode not in _VALID_MODES:
        raise ValueError(f'invalid mode: {mode!r}')
    result: dict = {'mode': mode, 'reason': str(obj.get('reason', ''))}
    if mode == 'mapf':
        robot_ids = obj.get('robot_ids', [])
        goals     = obj.get('goals', [])
        if not robot_ids or not goals:
            raise ValueError('mode=mapf requires non-empty robot_ids and goals')
        if len(robot_ids) != len(goals):
            raise ValueError('robot_ids and goals length mismatch')
        for g in goals:
            if not (isinstance(g, (list, tuple)) and len(g) >= 2):
                raise ValueError('each goal must be [x, y]')
        result['robot_ids'] = [int(r) for r in robot_ids]
        result['goals']     = [[float(g[0]), float(g[1])] for g in goals]
    if mode == 'formation':
        fid    = obj.get('formation_id', '')
        leader = obj.get('leader_ns', '')
        if not fid or not leader:
            raise ValueError('mode=formation requires formation_id and leader_ns')
        result['formation_id'] = str(fid)
        result['leader_ns']    = str(leader)
        result['follower_ns']  = [str(f) for f in obj.get('follower_ns', [])]
        result['offsets_x']    = [float(o) for o in obj.get('offsets_x', [])]
        result['offsets_y']    = [float(o) for o in obj.get('offsets_y', [])]
        if len(result['follower_ns']) != len(result['offsets_x']) or \
           len(result['follower_ns']) != len(result['offsets_y']):
            raise ValueError('follower_ns / offsets length mismatch')
    return result
