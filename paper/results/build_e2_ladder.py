#!/usr/bin/env python3
"""Score the E2 schema ladder on expressible subsets only.

The ladder asks which feature of PLAN_RESPONSE_SCHEMA breaks qwen3.5-9b:

    S1  six-branch recursive oneOf   (the shipped schema)
    S2  four-branch oneOf, no $ref   -> S1 vs S2 isolates recursion
    S3  single branch, no oneOf      -> S2 vs S3 isolates the oneOf construct
    S1n S1 minus the idle branch     -> removes the cheapest valid exit

THE SCORING RULE THIS FILE EXISTS TO ENFORCE:
S2 cannot express a plan whose correct answer is a sequence or parallel node,
and each S3 run can express only its own node type. Scoring those variants over
all 44 cases would measure "the grammar forbids the right answer" rather than
"the grammar breaks the model". So every comparison here is computed on a
declared subset, and S1 and the schema-off baseline are RECOMPUTED on that same
subset rather than quoted from the full-44 numbers in Sec. 6.2. Mixing the two
is the specific error that would invalidate the ladder, so the subset is
carried in the output next to every figure.

Usage:
    python3 paper/results/build_e2_ladder.py
"""

from __future__ import annotations

import collections
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, '..', '..', 'iros_llm_orchestrator')))

import benchmark_ch3 as B          # noqa: E402
from analyze_e2_nodes import expected_types, emitted_type  # noqa: E402

MODEL = 'qwen3.5-9b:latest'


def _load(name: str) -> dict | None:
    path = os.path.join(RAW, name)
    if not os.path.exists(path):
        print(f'  missing: {name}')
        return None
    return json.load(open(path, encoding='utf-8'))


def score(d: dict, subset: set[str]) -> dict:
    """pass@1 / pass@5 plus emitted-node histogram over `subset` only."""
    by: dict[str, list] = collections.OrderedDict()
    for r in d['results']:
        if r['id'] in subset:
            by.setdefault(r['id'], []).append(r)

    missing = subset - set(by)
    n = len(by)
    if n == 0:
        return {'n_cases': 0, 'error': 'no results for subset'}

    p1 = sum(1 for rs in by.values() if rs[0]['passed'])
    p5 = sum(1 for rs in by.values() if any(x['passed'] for x in rs))
    hist: collections.Counter = collections.Counter()
    for rs in by.values():
        for r in rs:
            hist[emitted_type(r.get('raw') or '')[0]] += 1

    out = {
        'n_cases': n,
        'n_calls': sum(len(rs) for rs in by.values()),
        'pass_at_1': {'count': p1, 'of': n, 'pct': round(100 * p1 / n, 1)},
        'pass_at_5': {'count': p5, 'of': n, 'pct': round(100 * p5 / n, 1)},
        'emitted_top_level_types': dict(hist.most_common()),
    }
    if missing:
        out['MISSING_CASES'] = sorted(missing)
    return out


def main() -> None:
    exp = expected_types()
    by_type: dict[str, list[str]] = collections.defaultdict(list)
    for cid, t in exp.items():
        by_type[t].append(cid)

    mapf = set(by_type['mapf'])
    idle = set(by_type['idle'])
    disband = set(by_type['disband'])
    s2_subset = mapf | idle | disband

    baseline = _load('e1b_qwen3.5-9b.json')
    s1 = _load('e2_schema_on_qwen3.5-9b.json')
    s2 = _load('e2_ladder_s2.json')
    s3_parts = {t: _load(f'e2_ladder_s3_{t}.json')
                for t in ('mapf', 'idle', 'disband')}
    s1n = _load('e2_ladder_s1_noidle.json')

    if not (baseline and s1):
        print('FATAL: baseline or S1 raw file missing')
        sys.exit(1)

    # S3 is three separate single-branch runs; merge them into one virtual run
    # covering exactly the union of their subsets.
    s3_merged = None
    if all(s3_parts.values()):
        s3_merged = {'results': [r for d in s3_parts.values()
                                 for r in d['results']]}

    out: dict = {
        'analysis': 'E2 schema ladder on qwen3.5-9b',
        'model': MODEL,
        'scoring_rule': (
            'Every variant is scored ONLY on cases whose reference answer it '
            'can express, and S1 / schema-off are recomputed on that same '
            'subset. These numbers are NOT comparable to the full-44 figures '
            'in Sec. 6.2 and must never be placed in the same column.'),
        'variants': {
            'S1':        'six-branch recursive oneOf (shipped PLAN_RESPONSE_SCHEMA)',
            'S2':        'four-branch oneOf, no $defs/$ref, no recursion',
            'S3':        'single branch inlined, no oneOf, type pinned by const',
            'S1_noidle': 'S1 minus the idle branch, still recursive',
        },
        'subsets': {
            'mapf_expressible':   sorted(mapf),
            'idle_expressible':   sorted(idle),
            'disband_expressible': sorted(disband),
            's2_s3_expressible':  sorted(s2_subset),
        },
        'subset_sizes': {
            'mapf_expressible': len(mapf),
            'idle_expressible': len(idle),
            'disband_expressible': len(disband),
            's2_s3_expressible': len(s2_subset),
            'excluded_sequence': len(by_type['sequence']),
            'excluded_parallel': len(by_type['parallel']),
            'full_suite': len(exp),
        },
        'comparisons': {},
    }

    # ── comparison A: the 23-case S2/S3-expressible subset ──────────────
    a: dict = {
        'subset': 's2_s3_expressible',
        'n_cases': len(s2_subset),
        'note': ('sequence- and parallel-answer cases are excluded because S2 '
                 'and S3 cannot express them at all.'),
        'cells': {
            'schema_off_baseline': score(baseline, s2_subset),
            'S1': score(s1, s2_subset),
        },
    }
    if s2:
        a['cells']['S2'] = score(s2, s2_subset)
    if s3_merged:
        a['cells']['S3'] = score(s3_merged, s2_subset)
    out['comparisons']['A_s2_s3_expressible'] = a

    # ── comparison B: the 13-case mapf subset (the branch that vanished) ──
    b: dict = {
        'subset': 'mapf_expressible',
        'n_cases': len(mapf),
        'note': ('The mapf branch is the one qwen3.5-9b never emits at top '
                 'level under S1. This subset asks under which schema it '
                 'becomes reachable again.'),
        'cells': {
            'schema_off_baseline': score(baseline, mapf),
            'S1': score(s1, mapf),
        },
    }
    if s2:
        b['cells']['S2'] = score(s2, mapf)
    if s3_parts['mapf']:
        b['cells']['S3_mapf'] = score(s3_parts['mapf'], mapf)
    if s1n:
        b['cells']['S1_noidle'] = score(s1n, mapf)
    out['comparisons']['B_mapf_expressible'] = b

    dest = os.path.join(HERE, 'e2_ladder_summary.json')
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    # ── report ─────────────────────────────────────────────────────────
    for key, comp in out['comparisons'].items():
        print('=' * 74)
        print(f'{key}   subset={comp["subset"]}  n={comp["n_cases"]} cases '
              f'(of {len(exp)})')
        print(f'  {comp["note"]}')
        print(f'  {"cell":22s}{"pass@1":>12s}{"pass@5":>12s}   emitted top-level types')
        for name, c in comp['cells'].items():
            if c.get('n_cases', 0) == 0:
                print(f'  {name:22s}  (no data)')
                continue
            h = ' '.join(f'{k}={v}' for k, v in
                         list(c['emitted_top_level_types'].items())[:5])
            print(f'  {name:22s}{c["pass_at_1"]["pct"]:>11.1f}%'
                  f'{c["pass_at_5"]["pct"]:>11.1f}%   {h}')
        print()

    print(f'wrote {dest}')


if __name__ == '__main__':
    main()
