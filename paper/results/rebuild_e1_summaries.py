#!/usr/bin/env python3
"""Regenerate e1_summary.json / e1b_summary.json from the rescored raw runs.

Needed because the either-prefix validator fix (see rescore_prefix_fix.py)
changed pass counts in every raw file, while the committed E1 summaries still
carried the pre-fix numbers. build_e2_summary.py refuses to run while the two
disagree -- correctly, since its whole guard is "my reconstruction of E1b's
taxonomy must reproduce the committed E1b summary".

The per-model block is rebuilt with build_e2_summary.summarize(), which is the
same function whose faithfulness that guard checks, so E1 and E2 stay scored by
one implementation rather than two.

Every regenerated file keeps its original prose fields and gains a `rescored`
note, so nothing silently pretends the numbers were always these.

Usage (inside the container):
    python3 paper/results/rebuild_e1_summaries.py           # dry run
    python3 paper/results/rebuild_e1_summaries.py --write
"""

from __future__ import annotations

import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(HERE, '..', '..', 'iros_llm_orchestrator')))

import build_e2_summary as BE                                # noqa: E402

NOTE = ('regenerated after the either-prefix validator fix: escalate_04 and '
        'edge_06 were unpassable (chk(a) or chk(b) with list-returning checks '
        'reports an error whenever either prefix is absent). Scored from the '
        'stored raw output; no model was called again.')


def rebuild(summary_path: str, write: bool) -> None:
    doc = json.load(open(summary_path, encoding='utf-8'))
    models = doc.get('models') or doc.get('conditions')
    if not models:
        print('%s: нет блока models/conditions, пропускаю' % summary_path)
        return

    changed = []
    for name, block in models.items():
        src = block.get('source_file')
        if not src:
            continue
        # e1_summary.json stores source_file as an absolute path from whatever
        # machine built it; joining that with RAW just yields the foreign path.
        path = os.path.join(RAW, os.path.basename(src))
        if not os.path.exists(path):
            print('   %-24s нет файла %s' % (name, src))
            continue
        fresh = BE.summarize(path)
        before = (block.get('pass_at_1', {}).get('count'),
                  block.get('pass_at_5', {}).get('count'),
                  block.get('total_passed_calls'))
        for k, v in fresh.items():
            if k in ('source_file',):
                continue
            block[k] = v
        after = (block['pass_at_1']['count'], block['pass_at_5']['count'],
                 block['total_passed_calls'])
        if before != after:
            changed.append('%s: pass@1 %s→%s, pass@5 %s→%s, вызовов %s→%s'
                           % (name, before[0], after[0], before[1], after[1],
                              before[2], after[2]))

    for line in changed:
        print('   ' + line)
    if not changed:
        print('   без изменений')

    if write and changed:
        doc.setdefault('rescored', []).append({
            'at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'what': NOTE,
            'changes': changed,
        })
        with open(summary_path, 'w', encoding='utf-8') as f:
            json.dump(doc, f, ensure_ascii=False, indent=2)
        print('   записано')


def main() -> None:
    write = '--write' in sys.argv
    print('РЕЖИМ:', 'ЗАПИСЬ' if write else 'сухой прогон (добавьте --write)')
    for name in ('e1_summary.json', 'e1b_summary.json'):
        p = os.path.join(HERE, name)
        if not os.path.exists(p):
            print('%s: нет' % name)
            continue
        print(name)
        rebuild(p, write)


if __name__ == '__main__':
    main()
