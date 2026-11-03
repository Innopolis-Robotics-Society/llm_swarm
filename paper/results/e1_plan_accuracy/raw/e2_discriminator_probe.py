#!/usr/bin/env python3
"""Part B follow-on — is the discriminator's SORT POSITION the failure cause?

Part B established that qwen3.5-9b alphabetises object keys whenever a response
schema is present, on any schema, overriding the in-context examples. The
mechanism claim that follows is:

    a oneOf discriminated by a `type` field breaks on this model iff `type`
    does not sort first, because alphabetisation forces the branch to be
    chosen by whichever key sorts first instead of by the discriminator.

In PLAN_RESPONSE_SCHEMA, `type` sorts LAST in the mapf branch
(goals, reason, robot_ids, spread, type) and in the formation branch
(follower_ns, ..., type) — and those are exactly the two branches qwen3.5-9b
never emits.

This probe tests the claim directly, and tests the fix it implies, on two
minimal two-branch unions that differ only in the discriminator's NAME:

    C1  discriminator "type"    -> sorts last  (goals, reason, type)
    C2  discriminator "action"  -> sorts first (action, goals, reason)

Both are given a request that can only be satisfied by the branch carrying
`goals`. If C1 picks the wrong branch and C2 picks the right one, the sort
position of the discriminator is the cause and renaming it is a real fix.

Run on qwen3.5-9b (the only model that alphabetises) with qwen3.5:4b as a
control that should be unaffected by the rename.

Usage (inside the container):
    python3 paper/results/raw/e2_discriminator_probe.py
"""

from __future__ import annotations

import asyncio
import collections
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(
    os.path.join(_HERE, '..', '..', '..', 'iros_llm_orchestrator')))

from iros_llm_orchestrator.common.llm_factory import get_llm_client  # noqa: E402

MODELS = ['qwen3.5-9b:latest', 'qwen3.5:4b']
REPEAT = 5


def union(disc: str) -> dict:
    """Two-branch discriminated union; `disc` names the discriminator."""
    return {
        'type': 'object',
        'properties': {
            'reply': {'type': 'string'},
            'plan': {
                'oneOf': [
                    {
                        'type': 'object',
                        'properties': {
                            disc: {'const': 'move'},
                            'goals': {'type': 'array',
                                      'items': {'type': 'number'}},
                            'reason': {'type': 'string'},
                        },
                        'required': [disc, 'goals', 'reason'],
                    },
                    {
                        'type': 'object',
                        'properties': {
                            disc: {'const': 'wait'},
                            'reason': {'type': 'string'},
                        },
                        'required': [disc, 'reason'],
                    },
                ]
            },
        },
        'required': ['reply', 'plan'],
    }


PROBES = {
    # 'type' sorts after 'goals' and 'reason' -> discriminator emitted LAST
    'C1_type': {'disc': 'type', 'sorts_first': False},
    # 'action' sorts before 'goals' and 'reason' -> discriminator emitted FIRST
    'C2_action': {'disc': 'action', 'sorts_first': True},
}

SYSTEM = ('You control a robot fleet. Reply with a single JSON object.\n'
          'Use a "move" plan when the operator asks the robots to go '
          'somewhere, and a "wait" plan only when no movement is requested.\n')
USER = 'Send the robots to coordinates 3.0, 4.0.'   # unambiguously "move"


async def run() -> dict:
    out: dict = {'repeat': REPEAT, 'user_prompt': USER, 'probes': {}}
    for pname, p in PROBES.items():
        schema = union(p['disc'])
        out['probes'][pname] = {
            'discriminator': p['disc'],
            'discriminator_sorts_first': p['sorts_first'],
            'correct_branch': 'move',
            'models': {},
        }
        for model in MODELS:
            llm = get_llm_client(
                mode='ollama', endpoint=None, model=model,
                temperature=0.1, max_tokens=2048, num_ctx=32768, timeout=180.0)
            msgs = [{'role': 'system', 'content': SYSTEM},
                    {'role': 'user', 'content': USER}]
            chosen: collections.Counter = collections.Counter()
            orders: collections.Counter = collections.Counter()
            raws: list[str] = []
            for _ in range(REPEAT):
                buf = ''
                async for c in llm.stream(msgs, response_format=schema):
                    buf += c or ''
                raws.append(buf)
                try:
                    plan = json.loads(buf).get('plan') or {}
                    chosen[plan.get(p['disc'], '<none>')] += 1
                    orders[','.join(plan.keys())] += 1
                except Exception:
                    chosen['<unparseable>'] += 1

            n_ok = chosen['move']
            out['probes'][pname]['models'][model] = {
                'branch_chosen': dict(chosen.most_common()),
                'plan_key_order': dict(orders.most_common()),
                'correct': n_ok,
                'of': REPEAT,
                'samples': raws[:1],
            }
            print(f'{pname:11s} {model:20s} correct={n_ok}/{REPEAT} '
                  f'chose={dict(chosen.most_common())} '
                  f'order={dict(orders.most_common())}')
    return out


def main() -> None:
    out = asyncio.run(run())
    dest = os.path.abspath(os.path.join(_HERE, '..', 'e2_discriminator_probe.json'))
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f'\nwrote {dest}')


if __name__ == '__main__':
    main()
