#!/usr/bin/env python3
"""E2 Part 0 — post-hoc node-type analysis over the existing raw runs.

No new model calls. For every E2 cell (2 models x {baseline, schema_on,
grounding_locations_only, grounding_coords_only}) this extracts the top-level
plan node type the model actually emitted and reports the distribution against
the type each case's validator expected.

The point is to test *why* qwen3.5-9b collapses under the schema. Sec. 6.2
asserts it "emits an idle plan nearly regardless of the request"; that claim is
currently an inference from category scores rather than a measurement. Three
candidate mechanisms make different predictions about this histogram:

  branch-order bias  -> collapse lands on `mapf`   (first oneOf alternative)
  cheapest-valid-exit-> collapse lands on `idle`   (fewest required fields: 2),
                        with `disband` (3 fields) as the runner-up
  model-specific     -> qwen3.5:4b under the identical schema shows no such
                        skew, i.e. the grammar permits the correct answers

Emits e2_node_distribution.json and prints a readable report.

Usage:
    python3 paper/results/analyze_e2_nodes.py
"""

from __future__ import annotations

import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')
BENCH_DIR = os.path.abspath(
    os.path.join(HERE, '..', '..', 'iros_llm_orchestrator'))
sys.path.insert(0, BENCH_DIR)

import benchmark_ch3 as B  # noqa: E402  (needs the path insert above)

# Required-field count per oneOf branch, in the order the branches are declared
# in plan_schema._NODE_SCHEMA. Both facts matter: the declared order tests the
# branch-order hypothesis, the field count tests the cheapest-exit hypothesis.
BRANCH_ORDER = ['mapf', 'formation', 'disband', 'idle', 'sequence', 'parallel']
REQUIRED_FIELDS = {
    'mapf': 4,        # type, robot_ids, goals, reason
    'formation': 7,   # type, formation_id, leader_ns, follower_ns, ox, oy, reason
    'disband': 3,     # type, formation_id, reason
    'idle': 2,        # type, reason
    'sequence': 2,    # type, steps  (but steps recurses -> not a cheap exit)
    'parallel': 2,    # type, steps
}

CONDITIONS = [
    ('baseline',                 'e1b_{m}.json'),
    ('schema_on',                'e2_schema_on_{m}.json'),
    ('grounding_locations_only', 'e2_grounding_locations_only_{m}.json'),
    ('grounding_coords_only',    'e2_grounding_coords_only_{m}.json'),
]
MODELS = [
    ('qwen3.5:4b', 'qwen3.5-4b'),
    ('qwen3.5-9b:latest', 'qwen3.5-9b'),
    # Third model, different family: only the schema factor was run for it, so
    # its two grounding cells are absent by design.
    ('qwen2.5:14b', 'qwen2.5-14b'),
]


def expected_types() -> dict[str, str]:
    """Recover each case's expected top-level node type from its validator.

    chk_type(p, 'X') is the first term of nearly every validator and is joined
    with `or`, so a stub that returns a non-empty list records X and
    short-circuits the rest. Cases keyed on chk_idle_reason_prefix expect idle;
    disband_01 accepts any plan containing a disband node.
    """
    rec: dict[str, str] = {}
    cur = [None]
    real_type, real_idle = B.chk_type, B.chk_idle_reason_prefix

    def spy_type(plan, expected):
        rec.setdefault(cur[0], expected)
        return ['<short-circuit>']

    def spy_idle(plan, prefix):
        rec.setdefault(cur[0], 'idle')
        return ['<short-circuit>']

    B.chk_type, B.chk_idle_reason_prefix = spy_type, spy_idle
    try:
        for tc in B.TEST_CASES:
            cur[0] = tc.id
            tc.run({'type': '<probe>'}, '')
    finally:
        B.chk_type, B.chk_idle_reason_prefix = real_type, real_idle

    # disband_01 has no chk_type gate: chk_has_disband accepts a bare disband
    # node or one nested in a sequence.
    rec.setdefault('disband_01', 'disband')
    return rec


def emitted_type(raw: str) -> tuple[str, dict | None]:
    """Top-level plan node type actually emitted, via the harness's own parser."""
    try:
        _reply, plan = B._parse_response(raw)
    except Exception:
        return ('<unparseable>', None)
    if not isinstance(plan, dict) or not plan:
        return ('<unparseable>', None)
    t = plan.get('type')
    if not isinstance(t, str):
        return ('<no-type-key>', plan)
    return (t, plan)


def analyse(path: str, exp: dict[str, str]) -> dict:
    d = json.load(open(path, encoding='utf-8'))
    hist: collections.Counter = collections.Counter()
    confusion: dict[str, collections.Counter] = collections.defaultdict(
        collections.Counter)
    first_plan_key: collections.Counter = collections.Counter()
    first_top_key: collections.Counter = collections.Counter()
    prefixes: list[str] = []

    for r in d['results']:
        t, plan = emitted_type(r.get('raw') or '')
        hist[t] += 1
        confusion[exp.get(r['id'], '?')][t] += 1

        # Key order is only meaningful on output the grammar actually shaped.
        raw = r.get('raw') or ''
        try:
            obj = json.loads(raw)
            if isinstance(obj, dict):
                first_top_key[next(iter(obj), '<empty>')] += 1
                p = obj.get('plan')
                if isinstance(p, dict):
                    first_plan_key[next(iter(p), '<empty>')] += 1
                    prefixes.append(json.dumps(p, ensure_ascii=False)[:30])
        except Exception:
            first_top_key['<not-strict-json>'] += 1

    total = len(d['results'])
    return {
        'source_file': os.path.basename(path),
        'llm_model': d.get('llm_model'),
        'grounding': d.get('grounding', 'full'),
        'schema_constrained': d.get('schema_constrained', False),
        'total_calls': total,
        'emitted_type_counts': dict(hist.most_common()),
        'emitted_type_pct': {k: round(100 * v / total, 1)
                             for k, v in hist.most_common()},
        'first_top_level_key': dict(first_top_key.most_common()),
        'first_plan_key': dict(first_plan_key.most_common()),
        'plan_prefix_top10': [
            f'{c}x {s}' for s, c in collections.Counter(prefixes).most_common(10)],
        'confusion_expected_to_emitted': {
            k: dict(v.most_common()) for k, v in sorted(confusion.items())},
    }


def bar(pct: float, width: int = 28) -> str:
    return '#' * int(round(pct / 100 * width))


def main() -> None:
    exp = expected_types()
    print(f'Expected top-level types over {len(B.TEST_CASES)} cases: '
          f'{dict(collections.Counter(exp.values()).most_common())}\n')

    out: dict = {
        'analysis': 'E2 Part 0 — emitted top-level plan node type distribution',
        'note': ('Post-hoc over existing E2/E1b raw files; no new model calls. '
                 'Emitted type is parsed from the recorded `raw` field with the '
                 'harness\'s own _parse_response, so it matches what the '
                 'validators saw.'),
        'oneOf_branch_order': BRANCH_ORDER,
        'required_field_count_per_branch': REQUIRED_FIELDS,
        'expected_type_per_case': exp,
        'cells': {},
    }

    for model, slug in MODELS:
        for label, pattern in CONDITIONS:
            path = os.path.join(RAW, pattern.format(m=slug))
            if not os.path.exists(path):
                print(f'  missing: {os.path.basename(path)}')
                continue
            res = analyse(path, exp)
            out['cells'][f'{model}|{label}'] = res

            print(f'── {model}  /  {label}  '
                  f'(schema={res["schema_constrained"]}, '
                  f'grounding={res["grounding"]}) ──')
            for t, c in res['emitted_type_counts'].items():
                pct = res['emitted_type_pct'][t]
                print(f'    {t:16s} {c:4d}  {pct:5.1f}%  {bar(pct)}')
            print(f'    first key of plan object: {res["first_plan_key"]}')
            print()

    dest = os.path.join(HERE, 'e2_node_distribution.json')
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f'wrote {dest}')


if __name__ == '__main__':
    main()
