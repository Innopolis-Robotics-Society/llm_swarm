"""Internal guard for reply-only chat turns.

This does not change the public plan schema. It only prevents factual
state questions from accidentally executing an LLM-produced idle plan.
"""

from __future__ import annotations


_QUESTION_HINTS = (
    '?',
    'what ',
    'what\'s',
    'where ',
    'why ',
    'how ',
    'which ',
    'is ',
    'are ',
    'status',
    'doing',
    'current',
    'right now',
    'failure',
    'failed',
    'ошибка',
    'почему',
    'что ',
    'где ',
    'как ',
    'сейчас',
    'статус',
)

_COMMAND_HINTS = (
    'stop',
    'halt',
    'cancel',
    'send',
    'move',
    'go to',
    ' form ',
    'return',
    'execute',
    'останов',
    'стоп',
    'иди',
    'ехать',
    'отправ',
    'сформ',
    'верни',
)


def should_skip_reply_only_execution(
    user_text: str,
    plan: dict,
    context: dict | None = None,
) -> bool:
    if not isinstance(plan, dict) or plan.get('type') != 'idle':
        return False
    if _is_direct_stop_command(user_text):
        return False
    reason = str(plan.get('reason', '')).strip().lower()
    if reason.startswith('reply_only') or reason.startswith('no-op'):
        return True
    if context and context.get('source') not in (None, 'none'):
        return _looks_like_factual_question(user_text)
    return False


def _is_direct_stop_command(text: str) -> bool:
    low = text.strip().lower()
    return low in {'stop', 'halt', 'cancel', 'стоп', 'остановить', 'отмена'}


def _looks_like_factual_question(text: str) -> bool:
    low = f' {text.strip().lower()} '
    if any(hint in low for hint in _COMMAND_HINTS):
        return False
    return any(hint in low for hint in _QUESTION_HINTS)
