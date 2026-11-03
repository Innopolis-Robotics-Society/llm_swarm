#!/usr/bin/env python3
"""Re-run reference cases that failed in transport, not in the model.

WHY THIS EXISTS
benchmark_ch3.py has no retry anywhere: an HTTP error is caught as a generic
exception and the case is scored `passed = False` with the exception text in
`parse_error`. A provider that returns 403 or 429 is therefore indistinguishable
in the pass rate from a model that planned badly.

This was not hypothetical. The first hosted cell (qwen3.5-9b, routed by
OpenRouter to Venice) returned

    HTTP LLM 403: {"success": false, "error": "Access denied by security policy."}

on all five repeats of one case, in 0.4 s each -- the request never reached the
model. Left alone, that model would have been scored on 43 usable cases against
44 for every other model, with the 44th counted as a planning failure.

WHAT IT DOES
Finds every case with a transport-shaped error, re-runs the WHOLE case (all
repeats, not just the failed ones), and splices the fresh results in. Re-running
whole cases rather than individual repeats avoids a case whose five samples were
drawn under two different conditions.

WHAT IT DELIBERATELY DOES NOT REPAIR
Timeouts and model-side errors. A TIMEOUT after 240 s is a real property of the
model on that prompt -- a slow model is a worse model, and hiding that behind a
retry would flatter it. Only errors that prove the prompt never reached the
model are repaired.

HONESTY
Every repair is recorded in a `repairs` block in the output json: which cases,
how many attempts, what the original error was, and when. A repaired file is
never silently equivalent to a clean one, and the manifest must say so.

Usage (inside the container, LLM_API_KEY set):
    python3 paper/results/raw/repair_transport_errors.py            # all e*ref_*.json
    python3 paper/results/raw/repair_transport_errors.py FILE...    # specific ones
    MAX_ATTEMPTS=5 python3 ... repair_transport_errors.py           # default 3
"""

from __future__ import annotations

import datetime
import glob
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.abspath(os.path.join(HERE, '..', '..', '..',
                                     'iros_llm_orchestrator'))
ENDPOINT = 'https://openrouter.ai/api/v1/chat/completions'
MAX_ATTEMPTS = int(os.environ.get('MAX_ATTEMPTS', '3'))
BACKOFF_SEC = 20

# Substrings that prove the call never reached the model. Kept deliberately
# narrow: anything not on this list is treated as a real result.
TRANSPORT_MARKERS = (
    'HTTP LLM 403', 'HTTP LLM 429', 'HTTP LLM 500', 'HTTP LLM 502',
    'HTTP LLM 503', 'HTTP LLM 504', 'HTTP LLM 520', 'HTTP LLM 524',
    'Access denied by security policy',
    'rate-limited upstream', 'Provider returned error',
    'Connection reset', 'Remote end closed connection',
)


def is_transport(err: str) -> bool:
    return bool(err) and any(m in err for m in TRANSPORT_MARKERS)


def affected_cases(doc: dict) -> dict[str, str]:
    """{case_id: first transport error seen} for cases needing a re-run."""
    out: dict[str, str] = {}
    for r in doc['results']:
        err = (r.get('parse_error') or '').strip()
        if is_transport(err) and r['id'] not in out:
            out[r['id']] = err[:160]
    return out


def rerun(doc: dict, case_ids: list[str], dest: str) -> bool:
    cmd = [
        'python3', 'benchmark_ch3.py',
        '--map', doc['map'],
        '--llm-mode', doc['llm_mode'],
        '--llm-endpoint', ENDPOINT,
        '--llm-model', doc['llm_model'],
        '--llm-max-tokens', str(doc['llm_max_tokens']),
        '--llm-num-ctx', str(doc['llm_num_ctx']),
        '--llm-temperature', str(doc['llm_temperature']),
        '--timeout', '240',
        '--repeat', str(doc['repeat']),
        '--no-color',
        '--grounding', doc.get('grounding') or 'full',
        '--tests', *case_ids,
        '--json', dest,
    ]
    if doc.get('schema_constrained'):
        cmd.append('--schema-constrained')
        if doc.get('schema_variant'):
            cmd += ['--schema-variant', doc['schema_variant']]
    proc = subprocess.run(cmd, cwd=BENCH, capture_output=True, text=True)
    if proc.returncode not in (0, 1):     # 1 = some cases failed, still valid
        print('    harness exited %d: %s' % (proc.returncode,
                                             proc.stderr.strip()[:200]))
    return os.path.exists(dest) and os.path.getsize(dest) > 0


def repair(path: str) -> None:
    doc = json.load(open(path, encoding='utf-8'))
    bad = affected_cases(doc)
    name = os.path.basename(path)
    if not bad:
        print('%-46s чисто' % name)
        return

    print('%-46s %d кейс(ов) с транспортной ошибкой' % (name, len(bad)))
    for cid, err in bad.items():
        print('    %-20s %s' % (cid, err))

    fixed: dict[str, int] = {}
    pending = list(bad)
    for attempt in range(1, MAX_ATTEMPTS + 1):
        if not pending:
            break
        print('  попытка %d/%d по %d кейсам' % (attempt, MAX_ATTEMPTS,
                                                len(pending)))
        with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as tf:
            dest = tf.name
        try:
            if not rerun(doc, pending, dest):
                print('    прогон не дал файла')
            else:
                fresh = json.load(open(dest, encoding='utf-8'))
                by_case: dict[str, list] = {}
                for r in fresh['results']:
                    by_case.setdefault(r['id'], []).append(r)
                still = []
                for cid in pending:
                    rs = by_case.get(cid) or []
                    if not rs:
                        still.append(cid)
                        continue
                    if any(is_transport((r.get('parse_error') or '')) for r in rs):
                        still.append(cid)
                        continue
                    # splice: drop every old repeat of this case, add the new
                    doc['results'] = [r for r in doc['results'] if r['id'] != cid]
                    doc['results'].extend(rs)
                    fixed[cid] = attempt
                    print('    %-20s починен' % cid)
                pending = still
        finally:
            os.path.exists(dest) and os.unlink(dest)
        if pending and attempt < MAX_ATTEMPTS:
            print('    ждём %ds перед следующей попыткой' % BACKOFF_SEC)
            time.sleep(BACKOFF_SEC)

    # passed/total in these files are CALL-level: benchmark_ch3 counts every
    # (case, repeat) pair, so a 44-case run at repeat=5 writes total=220. Case
    # -level rates are pass@1 / pass@5 and are derived later by
    # build_e2_summary. Writing cases here instead would leave this one file
    # counting something different from its siblings while still being labelled
    # `passed`, which is exactly the trap rescore_prefix_fix.py guards against.
    doc['total'] = len(doc['results'])
    doc['passed'] = sum(1 for r in doc['results'] if r['passed'])

    doc.setdefault('repairs', []).append({
        'at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'reason': ('transport errors: the call never reached the model, so the '
                   'case was re-run in full rather than scored as a planning '
                   'failure'),
        'original_errors': bad,
        'repaired': fixed,
        'unrepaired': pending,
        'max_attempts': MAX_ATTEMPTS,
    })
    json.dump(doc, open(path, 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)

    if pending:
        print('  ОСТАЛИСЬ НЕПОЧИНЕННЫМИ: %s' % ', '.join(pending))
        print('  Провайдер отдаёт ошибку устойчиво. Эти кейсы нельзя считать '
              'провалами планирования: либо исключить их из ВСЕХ моделей, '
              'либо увести прогон на другого провайдера и записать замену.')
    else:
        print('  все починены, pass=%d/%d' % (doc['passed'], doc['total']))


def main() -> None:
    if not os.environ.get('LLM_API_KEY'):
        print('FATAL: LLM_API_KEY не задан'); sys.exit(1)
    args = sys.argv[1:]
    files = args or sorted(glob.glob(os.path.join(HERE, 'e1ref_*.json')) +
                           glob.glob(os.path.join(HERE, 'e2ref_*.json')))
    if not files:
        print('нечего чинить: файлы результатов не найдены'); sys.exit(0)
    for f in files:
        repair(f)


if __name__ == '__main__':
    main()
