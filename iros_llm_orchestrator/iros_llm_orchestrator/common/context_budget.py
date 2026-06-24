"""Small prompt-size and completion-budget helpers for LLM calls."""

from __future__ import annotations

import json
from typing import Any


def estimate_tokens(value: Any) -> int:
    """Rough token estimate without a tokenizer.

    Four UTF-8-ish characters per token is conservative enough for guarding
    local OpenAI-compatible servers against request budget errors.
    """
    if isinstance(value, str):
        text = value
    else:
        text = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
    return max(1, len(text) // 4)


def completion_budget_for_prompt(
    prompt: Any,
    *,
    context_window_tokens: int,
    default_completion_tokens: int,
    min_completion_tokens: int,
    margin_tokens: int,
    extra: Any | None = None,
) -> dict:
    """Return a JSON-serialisable keep/reduce/abort budget decision."""
    prompt_est = estimate_tokens(prompt)
    extra_est = estimate_tokens(extra) if extra is not None else 0
    input_est = prompt_est + extra_est
    default_completion = max(1, int(default_completion_tokens))
    min_completion = max(1, int(min_completion_tokens))
    context_window = int(context_window_tokens)
    margin = max(0, int(margin_tokens))
    if context_window <= 0:
        return {
            'ok': True,
            'action': 'keep',
            'input_est_tokens': input_est,
            'available_completion_tokens': default_completion,
            'max_completion_tokens': default_completion,
            'reason': 'context guard disabled',
        }
    available = context_window - input_est - margin
    if available < min_completion:
        return {
            'ok': False,
            'action': 'abort',
            'input_est_tokens': input_est,
            'available_completion_tokens': max(0, available),
            'max_completion_tokens': 0,
            'reason': 'context too large after compaction',
        }
    max_completion = min(default_completion, available)
    action = 'reduce' if max_completion < default_completion else 'keep'
    return {
        'ok': True,
        'action': action,
        'input_est_tokens': input_est,
        'available_completion_tokens': available,
        'max_completion_tokens': max_completion,
        'reason': (
            'completion budget reduced to fit context'
            if action == 'reduce' else 'completion budget fits context'
        ),
    }
