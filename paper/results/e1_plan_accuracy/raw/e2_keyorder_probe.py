#!/usr/bin/env python3
"""Part B — does qwen3.5-9b alphabetise keys on ANY schema, or only on ours?

Part A established that the alphabetical order qwen3.5-9b produces under
constrained decoding is not coming from the prompt, the client, or the payload:

  * no template injects the schema into the prompt
  * nothing on the outbound path sorts keys (sort_keys defaults to False and
    the only explicit uses are the prompt fingerprint and mission_supervision,
    neither on this path)
  * the two outbound bodies differ only in `model`, and the schema reaches the
    wire in declaration order (type, robot_ids, goals, spread, reason)
  * the grammar is any-order over the *required* properties: qwen3.5:4b emits
    exactly the required arrays in declared order and qwen3.5-9b emits the same
    property sets alphabetically, and both drop the optional `spread`

Two explanations survive:

  (a) learned preference — qwen3.5-9b has been trained to emit alphabetically
      ordered keys whenever a response schema is in play, strongly enough to
      override the seven in-context examples that all put `type` first;
  (b) something structural about PLAN_RESPONSE_SCHEMA specifically (its size,
      its recursion, its six-branch oneOf).

This probe separates them with a toy schema carrying none of that structure:
no oneOf, no recursion, no plan vocabulary, two or three keys. Declaration
order and alphabetical order are deliberately made to disagree, and in probe B2
alphabetical order puts `type` in the MIDDLE, so a model that alphabetises
cannot be mistaken for one that merely reverses.

  B1: declared [type, alpha_first]              alphabetical [alpha_first, type]
  B2: declared [type, alpha_first, zulu_last]   alphabetical [alpha_first, type, zulu_last]

If both models emit declaration order here, the 9b behaviour is triggered by
our schema and (b) survives. If 9b alphabetises even here, it is (a): a
schema-mode convention independent of what the schema contains.

Usage (inside the container):
    python3 paper/results/raw/e2_keyorder_probe.py
"""

from __future__ import annotations

import asyncio
import collections
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ORCH = os.path.abspath(os.path.join(_HERE, '..', '..', '..',
                                     'iros_llm_orchestrator'))
sys.path.insert(0, _ORCH)

from iros_llm_orchestrator.common.llm_factory import get_llm_client  # noqa: E402

MODELS = ['qwen3.5:4b', 'qwen3.5-9b:latest']
REPEAT = 5

PROBES = {
    'B1': {
        'schema': {
            'type': 'object',
            'properties': {
                'type': {'const': 'greeting'},
                'alpha_first': {'type': 'string'},
            },
            'required': ['type', 'alpha_first'],
        },
        'declared': ['type', 'alpha_first'],
        'alphabetical': ['alpha_first', 'type'],
        'prompt': 'Greet the operator. Put your greeting text in alpha_first.',
    },
    'B2': {
        'schema': {
            'type': 'object',
            'properties': {
                'type': {'const': 'greeting'},
                'alpha_first': {'type': 'string'},
                'zulu_last': {'type': 'string'},
            },
            'required': ['type', 'alpha_first', 'zulu_last'],
        },
        'declared': ['type', 'alpha_first', 'zulu_last'],
        'alphabetical': ['alpha_first', 'type', 'zulu_last'],
        'prompt': ('Greet the operator. Put the greeting in alpha_first and a '
                   'short sign-off in zulu_last.'),
    },
}

# Mirrors the shape of the real prompt: a system message that shows the schema
# with `type` FIRST, so declaration order is also what the in-context example
# demonstrates. Any alphabetical output is therefore against the prompt.
SYSTEM = ('You reply with a single JSON object and nothing else.\n'
          'Example of the expected shape:\n'
          '{"type": "greeting", "alpha_first": "hello"}\n')


async def run() -> dict:
    out: dict = {'probes': {}, 'repeat': REPEAT}
    for pname, p in PROBES.items():
        out['probes'][pname] = {
            'declared_order': p['declared'],
            'alphabetical_order': p['alphabetical'],
            'schema': p['schema'],
            'models': {},
        }
        for model in MODELS:
            llm = get_llm_client(
                mode='ollama',
                endpoint='http://localhost:11434/api/chat',
                # 2048/32768 as in E1b: both qwen3.5 variants are thinking
                # models and a small budget is consumed entirely by the
                # reasoning block, leaving `content` empty.
                model=model, temperature=0.1, max_tokens=2048,
                num_ctx=32768, timeout=180.0,
            )
            messages = [{'role': 'system', 'content': SYSTEM},
                        {'role': 'user', 'content': p['prompt']}]
            orders: collections.Counter = collections.Counter()
            raws: list[str] = []
            for _ in range(REPEAT):
                buf = ''
                async for chunk in llm.stream(messages,
                                              response_format=p['schema']):
                    buf += chunk
                raws.append(buf)
                try:
                    obj = json.loads(buf)
                    orders[','.join(obj.keys())] += 1
                except Exception:
                    orders['<unparseable>'] += 1

            decl = ','.join(p['declared'])
            alpha = ','.join(p['alphabetical'])
            verdict = ('declaration' if orders[decl] > orders[alpha]
                       else 'alphabetical' if orders[alpha] > orders[decl]
                       else 'mixed/other')
            out['probes'][pname]['models'][model] = {
                'key_orders': dict(orders.most_common()),
                'n_declaration_order': orders[decl],
                'n_alphabetical_order': orders[alpha],
                'verdict': verdict,
                'samples': raws[:2],
            }
            print(f'{pname}  {model:20s} {verdict:12s} {dict(orders.most_common())}')
    return out


def main() -> None:
    out = asyncio.run(run())
    dest = os.path.join(_HERE, '..', 'e2_keyorder_probe.json')
    dest = os.path.abspath(dest)
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f'\nwrote {dest}')


if __name__ == '__main__':
    main()
