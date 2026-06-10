"""Small pure helpers for post-execution verification repair loops."""

from __future__ import annotations

import json
from typing import Any


def verification_allows_repair(verification: dict | None) -> bool:
    if not isinstance(verification, dict):
        return False
    rec = verification.get('repair_recommendation') or {}
    return bool(rec.get('repairable'))


def should_attempt_repair(
    verification: dict | None,
    *,
    attempt: int,
    max_attempts: int,
    enabled: bool,
) -> bool:
    if not enabled:
        return False
    if int(attempt) >= max(0, int(max_attempts)):
        return False
    return verification_allows_repair(verification)


def verification_summary(verification: dict | None) -> str:
    if not isinstance(verification, dict):
        return 'verification unavailable'
    return str(verification.get('summary') or 'verification produced no summary')


def append_verification_to_reply(reply: str, verification: dict | None) -> str:
    summary = verification_summary(verification)
    if not summary:
        return reply or ''
    base = reply or ''
    if 'verification:' in base.lower():
        return base
    return (base + '\n\n' if base else '') + f'Verification: {summary}'


def verification_failure_info(verification: dict | None) -> dict:
    rec = (verification or {}).get('repair_recommendation') or {}
    return {
        'leaf_type': 'verification',
        'failed_at_phase': 'post_execution',
        'action_status': 'VERIFICATION_FAILED',
        'last_error': verification_summary(verification)[:240],
        'repair_type': rec.get('type', ''),
        'repairable': bool(rec.get('repairable')),
    }


def compact_verification_json(verification: dict | None, *, max_chars: int = 5000) -> str:
    text = json.dumps(verification or {}, ensure_ascii=False, separators=(',', ':'))
    if len(text) <= max_chars:
        return text
    return text[: max(0, max_chars - 20)] + '...[truncated]'
