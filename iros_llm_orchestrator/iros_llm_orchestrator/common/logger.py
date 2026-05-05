"""
Append-only JSONL writer for the SFT dataset.

One file per UTC day under dataset_path. Each record stores the full
call: original goal, built prompt, raw LLM output and parsed decision.
When the team has enough human-graded rows this is already in the exact
shape TRL's SFTTrainer consumes.
"""

import json
import os
from datetime import datetime, timezone


class DecisionLogger:
    def __init__(self, path):
        self.path = os.path.expanduser(path)
        os.makedirs(self.path, exist_ok=True)

    def _current_file(self):
        date_str = datetime.now(timezone.utc).strftime('%Y%m%d')
        return os.path.join(self.path, f'decisions_{date_str}.jsonl')

    def log(self, record):
        enriched = dict(record)
        enriched['timestamp'] = datetime.now(timezone.utc).isoformat()
        target = self._current_file()
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(enriched, ensure_ascii=False) + '\n')
