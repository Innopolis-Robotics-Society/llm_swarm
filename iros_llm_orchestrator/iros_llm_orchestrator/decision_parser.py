"""
Parse LLM raw output into one of {"wait", "abort", "replan"}.

The LLM is asked to emit a single JSON object, but models drift:
  * they wrap the JSON in ```json fences
  * they prepend chain-of-thought prose
  * they add trailing commentary after the JSON
  * they break JSON syntax with trailing commas or unterminated strings

On any parse failure OR an unknown `decision` value, fall back to
"wait" — the safe default (keeps the BT running, does not flip a healthy
system into FAILURE).
"""

import json
import re


VALID_DECISIONS = {'wait', 'abort', 'replan'}

_FENCED = re.compile(r"```(?:json)?\s*(\{.*?\})\s*```", re.DOTALL | re.IGNORECASE)
_LOOSE = re.compile(r'\{[^{}]*"decision"\s*:\s*"[^"]*"[^{}]*\}', re.DOTALL)


def parse_llm_decision(raw):
    if not raw:
        return 'wait'

    text = raw.strip()

    fenced = _FENCED.search(text)
    candidates = [fenced.group(1)] if fenced else []
    candidates.extend(m.group(0) for m in _LOOSE.finditer(text))

    for candidate in candidates:
        try:
            obj = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        decision = obj.get('decision', '')
        if not isinstance(decision, str):
            continue
        decision = decision.strip().lower()
        if decision in VALID_DECISIONS:
            return decision
        if decision == '':
            return 'wait'  # empty string per spec is equivalent to wait

    return 'wait'
