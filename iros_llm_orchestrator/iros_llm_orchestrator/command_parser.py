"""Parser for LLM command-channel output (channel 2).

Extracts a single JSON object from the raw LLM text, validates the shape
per chosen mode, and returns a normalized dict. Raises ValueError on any
problem — the caller (PassiveObserver) logs and skips.
"""

import json
import re

VALID_MODES = {'idle', 'mapf', 'formation'}

_FENCED = re.compile(r'```(?:json)?\s*(\{.*?\})\s*```', re.DOTALL | re.IGNORECASE)


def parse_llm_command(raw: str) -> dict:
    if not raw:
        raise ValueError('empty LLM output')

    text = raw.strip()

    fenced = _FENCED.search(text)
    if fenced:
        candidate = fenced.group(1)
    else:
        start = text.find('{')
        end = text.rfind('}')
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
    if mode not in VALID_MODES:
        raise ValueError(f'invalid mode: {mode!r}')

    result = {
        'mode': mode,
        'reason': str(obj.get('reason', '')),
    }

    if mode == 'mapf':
        robot_ids = obj.get('robot_ids', [])
        goals = obj.get('goals', [])
        if not robot_ids or not goals:
            raise ValueError('mode=mapf requires non-empty robot_ids and goals')
        if len(robot_ids) != len(goals):
            raise ValueError('robot_ids and goals length mismatch')
        for g in goals:
            if not (isinstance(g, (list, tuple)) and len(g) >= 2):
                raise ValueError('each goal must be [x, y]')
        result['robot_ids'] = [int(r) for r in robot_ids]
        result['goals'] = [[float(g[0]), float(g[1])] for g in goals]

    if mode == 'formation':
        fid = obj.get('formation_id', '')
        leader = obj.get('leader_ns', '')
        if not fid or not leader:
            raise ValueError('mode=formation requires formation_id and leader_ns')
        result['formation_id'] = str(fid)
        result['leader_ns'] = str(leader)
        result['follower_ns'] = [str(f) for f in obj.get('follower_ns', [])]
        result['offsets_x'] = [float(o) for o in obj.get('offsets_x', [])]
        result['offsets_y'] = [float(o) for o in obj.get('offsets_y', [])]
        if len(result['follower_ns']) != len(result['offsets_x']) or \
           len(result['follower_ns']) != len(result['offsets_y']):
            raise ValueError('follower_ns / offsets length mismatch')

    return result
