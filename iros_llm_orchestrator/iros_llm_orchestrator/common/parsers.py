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

_VALID_MODES = {'idle', 'mapf', 'formation', 'obstacles'}
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

    if mode == 'obstacles':
        _OBSTACLE_ACTIONS = {'add_circle', 'add_rectangle', 'add_door', 'remove', 'open_door', 'close_door'}
        action = str(obj.get('action', '')).strip().lower()
        if action not in _OBSTACLE_ACTIONS:
            raise ValueError(f'obstacles: invalid action {action!r}; must be one of {_OBSTACLE_ACTIONS}')
        result['action'] = action
        if action in ('add_circle', 'add_rectangle', 'add_door'):
            oid = str(obj.get('id', '')).strip()
            if not oid:
                raise ValueError(f'obstacles/{action}: id required')
            result['id'] = oid
            result['x']  = float(obj.get('x', 0.0))
            result['y']  = float(obj.get('y', 0.0))
            if action == 'add_circle':
                result['radius'] = float(obj.get('radius', 0.5))
            else:
                result['width']  = float(obj.get('width',  1.0))
                result['height'] = float(obj.get('height', 0.4))
            if action == 'add_door':
                result['is_open'] = bool(obj.get('is_open', True))
        elif action == 'remove':
            oid = str(obj.get('id', '')).strip()
            if not oid:
                raise ValueError('obstacles/remove: id required')
            result['id'] = oid
        else:  # open_door / close_door
            door_id = str(obj.get('door_id', '')).strip()
            if not door_id:
                raise ValueError(f'obstacles/{action}: door_id required')
            result['door_id'] = door_id

    return result
