#!/usr/bin/env python3
"""Build e2_summary.json from the per-condition raw files.

Also re-derives E1b's summary from its raw files and asserts the result matches
the committed e1b_summary.json. That check is the point: the original E1b
summary builder was never committed, so this script reconstructs its taxonomy
and proves the reconstruction is faithful before applying it to E2. If the
assertion fails, the failure-mode counts here are NOT comparable to E1's table
and must not be presented as if they were.

Usage:
    python3 paper/results/build_e2_summary.py            # verify + write
    python3 paper/results/build_e2_summary.py --check    # verify only
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')

# ── failure-mode taxonomy ──────────────────────────────────────────────────
# One mode per failed call, first match wins. Reconstructed from E1b and
# validated against e1b_summary.json (see _verify_e1b).

_JSON_SYNTAX = (
    'no JSON object', 'JSON object not closed', 'not valid JSON',
    'Expecting', 'Unterminated', 'Extra data', 'Invalid control character',
)
_FORMATION_FIELDS = (
    'follower_ns', 'offsets_x', 'offsets_y', 'offset length mismatch',
    'leader_ns', 'abreast', 'expected formation',
    # Staging-before-formation is a formation-protocol error, not a generic
    # structural one — E1b counted it here and the reconstruction only
    # reproduces its numbers with this classification.
    'no staging mapf found',
)


def classify(result: dict) -> str:
    """Assign one failure mode per failed call.

    Scans *every* error a validator returned, not just the first, and applies
    the priority order below. That matters: a case can fail with both a
    generic structural complaint and a specific formation-field complaint, and
    E1b attributed those to the specific one. Classifying on errors[0] alone
    reproduces three of five E1b models but not the other two.
    """
    errs = result.get('errors') or []
    blob = ' | '.join(str(e) for e in errs) or str(result.get('parse_error') or '')

    if any(s in blob for s in _JSON_SYNTAX):
        return 'unparseable_json'
    if any(s in blob for s in _FORMATION_FIELDS):
        return 'wrong_formation_fields'
    # Cross-field arity from plan_executor: robot_ids(N) != goals(M)
    if 'robot_ids(' in blob and '!= goals' in blob:
        return 'wrong_goals'
    if 'does not start with' in blob or 'reason empty' in blob:
        return 'wrong_escalation_reason'
    if 'robot_ids=' in blob or 'robot_ids missing' in blob:
        return 'wrong_robot_ids'
    # 'each goal must be [x, y]' is singular — the plural check below misses it.
    if 'goals' in blob or 'each goal must be' in blob:
        return 'wrong_goals'
    if ('plan.type=' in blob or ' step' in blob or 'sequence has' in blob
            or 'staging' in blob or 'expected parallel' in blob):
        return 'wrong_structure'
    return 'other_validator'


def category(case_id: str) -> str:
    return re.sub(r'_\d+$', '', case_id)


def summarize(path: str) -> dict:
    d = json.load(open(path, encoding='utf-8'))
    by: dict[str, list] = collections.OrderedDict()
    for r in d['results']:
        by.setdefault(r['id'], []).append(r)

    n = len(by)
    p1 = sum(1 for rs in by.values() if rs[0]['passed'])
    p5 = sum(1 for rs in by.values() if any(x['passed'] for x in rs))
    el = [r['elapsed_sec'] for r in d['results']]
    el_s = sorted(el)

    cats: dict[str, dict] = collections.OrderedDict()
    for cid, rs in by.items():
        c = cats.setdefault(category(cid), {'n': 0, '_p1': 0, '_p5': 0})
        c['n'] += 1
        c['_p1'] += 1 if rs[0]['passed'] else 0
        c['_p5'] += 1 if any(x['passed'] for x in rs) else 0
    for c in cats.values():
        c['pass_at_1_pct'] = round(100 * c.pop('_p1') / c['n'], 1)
        c['pass_at_5_pct'] = round(100 * c.pop('_p5') / c['n'], 1)

    modes = collections.Counter(
        classify(r) for r in d['results'] if not r['passed'])
    timeouts = sum(1 for r in d['results']
                   if 'TIMEOUT' in str(r.get('parse_error', '')))

    return {
        'source_file': os.path.basename(path),
        'llm_mode': d.get('llm_mode'),
        'llm_model': d.get('llm_model'),
        'llm_max_tokens': d.get('llm_max_tokens'),
        'repeat': d.get('repeat'),
        'num_cases': n,
        'total_calls': len(d['results']),
        'total_passed_calls': sum(1 for r in d['results'] if r['passed']),
        'pass_at_1': {'count': p1, 'of': n, 'pct': round(100 * p1 / n, 1)},
        'pass_at_5': {'count': p5, 'of': n, 'pct': round(100 * p5 / n, 1)},
        'elapsed_sec': {
            'mean': round(statistics.mean(el), 2),
            'median': round(statistics.median(el), 2),
            'p90': round(el_s[int(0.9 * (len(el_s) - 1))], 2),
        },
        'timeouts': timeouts,
        'by_category': cats,
        'failure_modes_all_reps': dict(modes.most_common()),
        'cases_with_any_failure': sum(
            1 for rs in by.values() if not all(x['passed'] for x in rs)),
        # E2 additions — absent from E1b's schema, appended not substituted.
        'grounding': d.get('grounding', 'full'),
        'schema_constrained': d.get('schema_constrained', False),
        'prompt_fingerprint': d.get('prompt_fingerprint'),
    }


def _verify_e1b() -> bool:
    """Re-derive E1b from raw and compare to the committed summary."""
    ref_path = os.path.join(HERE, 'e1b_summary.json')
    if not os.path.exists(ref_path):
        print('SKIP verification: e1b_summary.json absent')
        return True
    ref = json.load(open(ref_path, encoding='utf-8'))['models']
    ok = True
    for model, exp in ref.items():
        src = os.path.join(RAW, exp['source_file'])
        if not os.path.exists(src):
            print(f'  SKIP {model}: {exp["source_file"]} absent')
            continue
        got = summarize(src)
        for field in ('pass_at_1', 'pass_at_5', 'by_category',
                      'failure_modes_all_reps', 'timeouts',
                      'total_passed_calls', 'cases_with_any_failure'):
            if got[field] != exp[field]:
                ok = False
                print(f'  MISMATCH {model}.{field}')
                print(f'    expected {exp[field]}')
                print(f'    got      {got[field]}')
    print('E1b reconstruction:', 'MATCHES committed summary' if ok else 'DIVERGES')
    return ok


CONDITIONS = [
    ('baseline',                 'e1b_{m}.json'),
    ('schema_on',                'e2_schema_on_{m}.json'),
    ('grounding_locations_only', 'e2_grounding_locations_only_{m}.json'),
    ('grounding_coords_only',    'e2_grounding_coords_only_{m}.json'),
]
_SCHEMA_ONLY = ('baseline', 'schema_on')

# (ollama tag, filename slug, conditions expected for this model).
# qwen2.5:14b is a follow-up third data point added to test whether the schema
# effect is family-specific; only the schema factor was run for it, so the two
# grounding cells are not expected and their absence is not a missing file.
MODELS = [
    ('qwen3.5:4b',        'qwen3.5-4b',  None),
    ('qwen3.5-9b:latest', 'qwen3.5-9b',  None),
    ('qwen2.5:14b',       'qwen2.5-14b', _SCHEMA_ONLY),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='verify the E1b reconstruction and exit')
    args = ap.parse_args()

    faithful = _verify_e1b()
    if args.check:
        sys.exit(0 if faithful else 1)
    if not faithful:
        print('\nRefusing to write e2_summary.json: the failure-mode taxonomy '
              'does not reproduce E1b, so the counts would not be comparable '
              'to E1\'s table.')
        sys.exit(1)

    out: dict = {
        'experiment': ('E2 — grounding and constraint ablation '
                       '(schema-constrained decoding on/off; '
                       'symbolic-vs-raw grounding x3)'),
        'baseline_note': (
            'The "baseline" condition is not a new run: it is the E1b data for '
            'the same model, reused as the schema-off / grounding-full arm. '
            'Reuse was gated on the prompt fingerprint '
            '42d660fe681eaa3857c239bbe8d8e97043ae15c07ef8fe088a202e49048c37ed '
            'matching between the pre-ablation code and the shipped code.'),
        'conditions': {},
    }
    for model, slug, expected in MODELS:
        per: dict = {}
        for label, pattern in CONDITIONS:
            if expected is not None and label not in expected:
                continue
            path = os.path.join(RAW, pattern.format(m=slug))
            if not os.path.exists(path):
                print(f'  MISSING (expected): {os.path.basename(path)}')
                continue
            per[label] = summarize(path)
        out['conditions'][model] = per

    dest = os.path.join(HERE, 'e2_summary.json')
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f'wrote {dest}')


if __name__ == '__main__':
    main()
