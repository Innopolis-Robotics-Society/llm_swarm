#!/usr/bin/env python3
"""Build the hosted-model summaries from the e1ref_*/e2ref_* raw files.

The local (ollama) models are summarised by build_e2_summary.py into
`e1_plan_accuracy/local_tools/`. The hosted (OpenRouter) models were run with
the same harness and the same 44-case set, but their numbers only ever existed
as raw JSON. This script applies the identical `summarize()` and failure-mode
taxonomy to them, so the two halves of the benchmark are scored the same way
and can be put in one table.

`e1ref_*` is the schema-off condition (the hosted counterpart of E1b);
`e2ref_*` is the schema-on condition (the counterpart of E2's `schema_on`).

Usage:
    python3 paper/results/scripts/build_hosted_summary.py
    python3 paper/results/scripts/build_hosted_summary.py --check
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.dirname(HERE)
RAW = os.path.join(RESULTS, 'e1_plan_accuracy', 'raw')
HOSTED = os.path.join(RESULTS, 'e1_plan_accuracy', 'hosted_tools')
sys.path.insert(0, HERE)

import build_e2_summary as BE  # noqa: E402

# (condition, filename glob, output name). The mt2048 file is a skipped stub
# (llm_model says "NOT RUN") and is excluded by the guard in _load().
CONDITIONS = [
    ('schema_off', 'e1ref_*.json', 'e1ref_summary.json'),
    ('schema_on', 'e2ref_*.json', 'e2ref_summary.json'),
]


def _load(path: str) -> dict | None:
    d = json.load(open(path, encoding='utf-8'))
    model = str(d.get('llm_model') or '')
    if 'NOT RUN' in model or not d.get('results'):
        return None
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='report, write nothing')
    args = ap.parse_args()

    os.makedirs(HOSTED, exist_ok=True)
    for condition, pattern, out_name in CONDITIONS:
        models: dict[str, dict] = {}
        for path in sorted(glob.glob(os.path.join(RAW, pattern))):
            d = _load(path)
            if d is None:
                print(f'skip (stub or empty): {os.path.basename(path)}',
                      file=sys.stderr)
                continue
            s = BE.summarize(path)
            s['source_file'] = os.path.basename(path)
            s['llm_model'] = d.get('llm_model')
            s['llm_max_tokens'] = d.get('llm_max_tokens')
            s['schema_constrained'] = d.get('schema_constrained')
            s['prompt_fingerprint'] = d.get('prompt_fingerprint')
            models[str(d.get('llm_model'))] = s
        payload = {
            'condition': condition,
            'note': ('Hosted OpenRouter models, same 44-case set and same '
                     'scoring as the local cells. pass_at_1_pct is call-level '
                     '(passed calls / total calls); pass_at_5_pct is '
                     'case-level (cases with at least one pass).'),
            'models': models,
        }
        dest = os.path.join(HOSTED, out_name)
        if args.check:
            print(f'would write {os.path.relpath(dest, RESULTS)} '
                  f'({len(models)} models)')
            continue
        with open(dest, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, indent=2, ensure_ascii=False)
            fh.write('\n')
        print(f'wrote {os.path.relpath(dest, RESULTS)} ({len(models)} models)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
