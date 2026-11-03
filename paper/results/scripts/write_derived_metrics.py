#!/usr/bin/env python3
"""Write a per-run derived/metrics.json next to every rosbag.

The article's numbers come from the bags, and the bags are gitignored. This
script materialises the analysis of each bag as a small JSON file that *is*
committed, so a reader of the repository can see the per-run evidence without
the multi-hundred-megabyte recordings.

Usage:
    python3 paper/results/scripts/write_derived_metrics.py            # all roots
    python3 paper/results/scripts/write_derived_metrics.py --root paper/results/e2_mission_series
    python3 paper/results/scripts/write_derived_metrics.py --check    # do not write

Each output file is the full analyse_run() record for that run, with `dir` and
`bag` rewritten relative to the repository root.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(RESULTS))
sys.path.insert(0, HERE)

import analyze_sessions as A  # noqa: E402

DEFAULT_ROOTS = [
    os.path.join(RESULTS, 'e2_mission_series'),
    os.path.join(RESULTS, 'qualitative_cases'),
    os.path.join(RESULTS, 'm3_guard_comparison', 'pre_guard'),
]


def find_runs(root: str) -> list[str]:
    """Return every directory under `root` that holds a session.json."""
    found: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d != 'derived')
        if 'session.json' in filenames:
            found.append(dirpath)
            dirnames[:] = []
    return sorted(found)


def rel(path: str) -> str:
    return os.path.relpath(path, REPO)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--root', action='append',
                    help='root to scan; repeatable (default: the three result roots)')
    ap.add_argument('--check', action='store_true',
                    help='report what would be written, write nothing')
    args = ap.parse_args()

    roots = args.root or DEFAULT_ROOTS
    written = 0
    for root in roots:
        if not os.path.isdir(root):
            print(f'skip (absent): {rel(root)}', file=sys.stderr)
            continue
        for run_dir in find_runs(root):
            if A._is_live(run_dir):
                print(f'skip (live): {rel(run_dir)}', file=sys.stderr)
                continue
            rec = A.analyse_run(run_dir)
            rec['dir'] = rel(run_dir)
            if rec.get('bag'):
                rec['bag'] = rel(rec['bag'])
            dest = os.path.join(run_dir, 'derived', 'metrics.json')
            if args.check:
                print(f'would write {rel(dest)}')
                continue
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            with open(dest, 'w', encoding='utf-8') as fh:
                json.dump(rec, fh, indent=2, ensure_ascii=False)
                fh.write('\n')
            written += 1
            print(f'wrote {rel(dest)}')
    if not args.check:
        print(f'\n{written} file(s) written')
    return 0


if __name__ == '__main__':
    sys.exit(main())
