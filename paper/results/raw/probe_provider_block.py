#!/usr/bin/env python3
"""Is the 403 a Venice quirk or a systemic block across OpenRouter providers?

Venice returns

    HTTP 403 {"success": false, "error": "Access denied by security policy."}

in 0.4-1.1 s -- before generation -- for a specific, reproducible set of E2
cases on qwen3.5-9b: every sequence_*, every formation_create_*, part of
parallel_*. The same long prompts pass with the schema switched off, and a
short prompt passes with the schema on, so the trigger is the combination of a
~9.7k-token prompt and the recursive six-branch response_format.

Before paying to re-run a cell elsewhere, this asks the cheap question: does
any other provider serve these exact requests? OpenRouter routes qwen3.5-9b to
five, and `provider.order` with allow_fallbacks=false pins one so the answer is
attributable.

Prompts are built by the harness itself (build_user_prompt with the case's own
context and the full grounding), so what goes on the wire is byte-identical to
what the sweep sends -- a probe with a hand-written prompt would prove nothing.

Usage (inside the container, LLM_API_KEY set):
    python3 paper/results/raw/probe_provider_block.py
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.request

BENCH = '/home/fabian/ros2_ws/src/iros_llm_orchestrator'
sys.path.insert(0, BENCH)
os.chdir(BENCH)

import benchmark_ch3 as B                                     # noqa: E402
from iros_llm_orchestrator.common.plan_schema import (        # noqa: E402
    PLAN_RESPONSE_SCHEMA as SCHEMA)

def _env_list(name: str, default: list[str]) -> list[str]:
    raw = os.environ.get(name, '').strip()
    return [x.strip() for x in raw.split(',') if x.strip()] if raw else default


# Overridable so the same probe can vet a provider before paying for a 45-call
# repair on a different model. Defaults reproduce the original qwen3.5-9b run.
MODEL = os.environ.get('PROBE_MODEL', '').strip() or 'qwen/qwen3.5-9b'
PROVIDERS = _env_list('PROBE_PROVIDERS',
                      ['Venice', 'DeepInfra', 'SiliconFlow', 'Parasail',
                       'Together'])
# Three cases that failed 5/5 on Venice, one per shape that broke.
CASES = _env_list('PROBE_CASES',
                  ['sequence_01', 'formation_create_01', 'parallel_02'])
SCHEMA_ON = os.environ.get('PROBE_SCHEMA', '1').strip().lower() not in (
    '0', 'false', 'no')
KEY = os.environ.get('LLM_API_KEY') or ''
URL = 'https://openrouter.ai/api/v1/chat/completions'


def messages_for(case_id: str):
    tc = next(t for t in B.TEST_CASES if t.id == case_id)
    return B.build_user_prompt(
        tc.prompt, map_name='amongus_description',
        runtime_context=tc.context, grounding='full')


def ask(msgs, provider: str | None, schema: bool):
    body = {'model': MODEL, 'messages': msgs,
            'max_tokens': 8192, 'temperature': 0.1}
    if schema:
        body['response_format'] = {
            'type': 'json_schema',
            'json_schema': {'name': 'plan_response', 'schema': SCHEMA}}
    if provider:
        body['provider'] = {'order': [provider], 'allow_fallbacks': False}
    req = urllib.request.Request(
        URL, data=json.dumps(body).encode(),
        headers={'Authorization': 'Bearer ' + KEY,
                 'Content-Type': 'application/json'})
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            d = json.load(r)
    except Exception as exc:
        raw = exc.read().decode() if hasattr(exc, 'read') else str(exc)
        raw = raw.replace(KEY, '<KEY>') if KEY else raw
        code = ''
        try:
            code = str(json.loads(raw).get('error', {}).get('code', ''))
        except Exception:
            pass
        short = 'HTTP ' + code if code else raw[:60]
        if 'security policy' in raw:
            short += ' security-policy'
        return time.time() - t0, short, None
    usage = d.get('usage') or {}
    return (time.time() - t0, 'OK ' + str(d.get('provider')),
            usage.get('completion_tokens'))


def main() -> None:
    if not KEY:
        print('FATAL: LLM_API_KEY не задан'); sys.exit(1)
    print('модель %s, схема %s, промпты собраны харнессом\n'
          % (MODEL, 'ВКЛЮЧЕНА' if SCHEMA_ON else 'выключена'))
    print('%-22s %-13s %7s  %s' % ('кейс', 'провайдер', 'сек', 'итог'))
    for cid in CASES:
        msgs = messages_for(cid)
        chars = sum(len(m['content']) for m in msgs)
        print('-- %s (промпт %d символов, ~%d токенов)' % (cid, chars, chars // 4))
        for prov in PROVIDERS:
            el, verdict, out = ask(msgs, prov, SCHEMA_ON)
            extra = ('' if out is None else '  out=%s' % out)
            print('%-22s %-13s %6.1fs  %s%s' % ('', prov, el, verdict, extra),
                  flush=True)


if __name__ == '__main__':
    main()
