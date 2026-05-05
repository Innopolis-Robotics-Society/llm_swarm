"""Mock LLM client — deterministic keyword heuristic, zero dependencies.

Used for unit tests, CI, and hardware bring-up without a GPU.
"""

import asyncio
import json

from iros_llm_orchestrator.common.llm_factory import LLMClientBase

_ABORT_KW  = ('collision','fatal','unreachable','no valid agents','pbs failed','cancelled')
_REPLAN_KW = ('timeout','stall','stalled','stuck','deviated','out of lives','replans_done')
_SIT_MARK  = '# Current situation'


def _tail(prompt: str) -> str:
    idx = prompt.rfind(_SIT_MARK)
    return prompt if idx == -1 else prompt[idx:]


class MockLLMClient(LLMClientBase):
    def __init__(self, max_tokens: int = 256, temperature: float = 0.2):
        self.max_tokens  = max_tokens
        self.temperature = temperature

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        await asyncio.sleep(0.01)
        text = _tail(prompt) if isinstance(prompt, str) else str(prompt)
        low  = text.lower()

        if prompt_kind == 'decision':
            if any(w in low for w in _ABORT_KW):
                return json.dumps({'decision': 'abort',
                                   'reason': 'mock: fatal/unreachable keyword'})
            if any(w in low for w in _REPLAN_KW):
                return json.dumps({'decision': 'replan',
                                   'reason': 'mock: stall/timeout keyword'})
            return json.dumps({'decision': 'wait',
                               'reason': 'mock: no critical keyword'})

        # command kind
        if 'error' in low or any(w in low for w in _ABORT_KW):
            return json.dumps({'mode': 'idle', 'reason': 'mock: halt on error'})
        if 'mapfplan' in low.replace('_','') or 'stall' in low:
            return json.dumps({
                'mode': 'mapf',
                'robot_ids': [0, 1, 2],
                'goals': [[10.0, 10.0], [11.0, 10.0], [12.0, 10.0]],
                'reason': 'mock: replan via MAPF',
            })
        return json.dumps({'mode': 'idle', 'reason': 'mock: fallback'})
