"""Prompt builder for the command-channel (channel 2).

Different from `prompt_builder.py` (channel 1 / decision) — here the LLM
must emit a full JSON command with mode + parameters, not a single-word
verdict. The prompt follows the same structure: system prompt, few-shot
scenarios, then the current situation.
"""

import json


SYSTEM_PROMPT = """Ты — проактивный оркестратор роя из 20 роботов под управлением Nav2 BT.

Ты наблюдаешь за потоком состояний BT (/bt/state). Когда видишь, что action_status
перешёл в WARN или ERROR, — ты решаешь, какую команду отправить, чтобы рой вышел
из проблемы.

Доступные режимы:
  "idle"       — ничего не делать, все действия отменены
  "mapf"       — отправить рой по новым целям через MAPF
  "formation"  — построить формацию

Отвечай строго валидным JSON:
{
  "mode": "idle" | "mapf" | "formation",
  "robot_ids": [0, 1, 2, ...],              // только для mapf
  "goals": [[x1, y1], [x2, y2], ...],       // только для mapf (длина == robot_ids)
  "formation_id": "wedge",                   // только для formation
  "leader_ns": "robot_0",                    // только для formation
  "follower_ns": ["robot_1", "robot_2"],    // только для formation
  "offsets_x": [1.0, -1.0],                  // только для formation
  "offsets_y": [0.0, 0.0],                   // только для formation
  "reason": "короткое обоснование"
}

Без текста вокруг, без markdown-блоков — только один JSON-объект.
"""


def _summarize(msg) -> str:
    parts = [
        f't={msg.stamp_ms}ms',
        f'mode={msg.mode}',
        f'status={msg.action_status}',
        f'action={msg.active_action}',
    ]
    tail = msg.action_summary
    if msg.last_error:
        tail = f'{tail} ERROR: {msg.last_error}' if tail else f'ERROR: {msg.last_error}'
    return f"[{' '.join(parts)}] {tail}"


def build_command_prompt(scenarios, history, trigger, tail: int = 15) -> str:
    parts = [SYSTEM_PROMPT.strip(), '']

    parts.append('# Примеры')
    for scenario in scenarios:
        parts.append('## История')
        for line in scenario['history']:
            parts.append(f'  {line}')
        parts.append('## Триггер')
        trg = scenario['trigger']
        parts.append(
            f'  status={trg["action_status"]} '
            f'action={trg["active_action"]} '
            f'error={trg.get("last_error", "")}'
        )
        parts.append('## Команда')
        parts.append(json.dumps(scenario['command'], ensure_ascii=False))
        parts.append('')

    parts.append('# Текущая ситуация')
    parts.append('## История')
    window = list(history)[-tail:] if tail and tail > 0 else list(history)
    for msg in window:
        parts.append(f'  {_summarize(msg)}')
    parts.append('## Триггер')
    parts.append(
        f'  status={trigger.action_status} '
        f'action={trigger.active_action} '
        f'error={trigger.last_error}'
    )
    parts.append('## Команда')
    parts.append('')

    return '\n'.join(parts)
