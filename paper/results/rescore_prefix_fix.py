#!/usr/bin/env python3
"""Re-score stored runs after the either-prefix validator fix.

THE BUG
escalate_04 and edge_06 accepted "either needs_help: or clarify:" written as

    chk_idle_reason_prefix(p, 'needs_help:') or chk_idle_reason_prefix(p, 'clarify:')

These checks return [] on success and [err] on failure, so `a or b` evaluates to
b whenever a passes. The expression therefore reports an error whenever EITHER
prefix is missing -- and since a reason cannot begin with both, it always did.
Both cases were unpassable for every model in E1, E2 and the hosted reference.

Both are escalation cases, and the paper leans on "escalation judgment is weak
and essentially flat across all five models" as its strongest argument for
keeping escalation outside the model. The flatness was partly this: on
escalate_04 all six models tested emit the identical reason
"clarify: no group specified for triangle formation", which is the literal
worked example in prompts/user_chat_system.txt. They escalate correctly and
fail a string check the prompt never asked them to satisfy.

WHAT THIS DOES
Re-runs the two fixed validators against the raw model output already stored in
every result file, and rewrites `passed`, `errors` and the file's `passed`
count. No model is called: the answers are the ones originally given.

Each file gets a `rescored` block recording what changed, so a corrected file is
never mistaken for one that was right all along.

Usage (inside the container):
    python3 paper/results/rescore_prefix_fix.py            # dry run, prints diffs
    python3 paper/results/rescore_prefix_fix.py --write    # apply
"""

from __future__ import annotations

import collections
import datetime
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, '..', '..', 'iros_llm_orchestrator')))

import benchmark_ch3 as B                                  # noqa: E402

AFFECTED = ('escalate_04', 'edge_06')
BY_ID = {tc.id: tc for tc in B.TEST_CASES}


def rescore_file(path: str, write: bool) -> tuple[int, int, dict]:
    doc = json.load(open(path, encoding='utf-8'))
    changes: dict[str, dict[str, int]] = {}
    flips = 0

    for r in doc['results']:
        if r['id'] not in AFFECTED:
            continue
        if (r.get('parse_error') or '').strip():
            continue                       # transport/parse failure, not ours
        tc = BY_ID.get(r['id'])
        if tc is None:
            continue
        try:
            _reply, plan = B._parse_response(r.get('raw') or '')
        except Exception:
            continue
        if plan is None:
            continue
        errs = tc.run(plan, r.get('reply') or '')
        new_passed = not errs
        if new_passed != bool(r['passed']):
            flips += 1
            c = changes.setdefault(r['id'], {'to_pass': 0, 'to_fail': 0})
            c['to_pass' if new_passed else 'to_fail'] += 1
        r['passed'] = new_passed
        r['errors'] = errs

    # `passed` in these files counts CALLS, not cases: benchmark_ch3 writes
    # passed/total over every (case, repeat) pair. Recomputing it per-case here
    # would silently replace the metric with a different one -- 182 would
    # become 39 and look like a catastrophic regression instead of a fix.
    old_passed = doc.get('passed')
    doc['passed'] = sum(1 for r in doc['results'] if r['passed'])
    assert doc.get('total') in (None, len(doc['results'])), (
        'total=%r but %d results: passed/total are not call-level here'
        % (doc.get('total'), len(doc['results'])))

    if write and flips:
        doc.setdefault('rescored', []).append({
            'at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'what': ('either-prefix validator fixed (or -> and) for '
                     + ', '.join(AFFECTED)),
            'calls_flipped': flips,
            'per_case': changes,
            'passed_before': old_passed,
            'passed_after': doc['passed'],
            'note': ('scored from the stored raw output; no model was called '
                     'again'),
        })
        json.dump(doc, open(path, 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=2)

    return old_passed, doc['passed'], changes


def main() -> None:
    write = '--write' in sys.argv
    raw = os.path.join(HERE, 'raw')
    files = []
    for pat in ('e1_*.json', 'e1b_*.json', 'e2_*.json',
                'e1ref_*.json', 'e2ref_*.json'):
        files += glob.glob(os.path.join(raw, pat))
    files = sorted(f for f in files
                   if 'summary' not in f and 'manifest' not in f)
    if not files:
        print('нет файлов результатов'); return

    print('РЕЖИМ:', 'ЗАПИСЬ' if write else 'сухой прогон (добавьте --write)')
    print('%-46s %14s  %s' % ('файл', 'passed', 'что изменилось'))
    total = 0
    for f in files:
        old, new, ch = rescore_file(f, write)
        total += sum(v['to_pass'] + v['to_fail'] for v in ch.values())
        mark = '' if old == new else '  <-- сдвиг'
        desc = ', '.join('%s +%d' % (k, v['to_pass'])
                         for k, v in ch.items() if v['to_pass']) or '—'
        print('%-46s %5s -> %-5s %s%s' % (os.path.basename(f)[:-5],
                                          old, new, desc, mark))
    print('\nвсего перевернулось вызовов: %d' % total)
    if not write:
        print('ничего не записано')


if __name__ == '__main__':
    main()
