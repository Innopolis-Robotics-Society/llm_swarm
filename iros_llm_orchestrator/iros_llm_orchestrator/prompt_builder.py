"""
Few-shot prompt builder for the LLM decision service.

Layout:
  <system prompt>

  # Примеры
  ## Вход
  level: ...
  event: ...
  log_buffer:
    ...
  ## Решение
  {"decision": "...", "reason": "..."}

  ...more examples...

  # Текущая ситуация
  level: ...
  event: ...
  log_buffer:
    ...
  ## Решение
  <-- LLM continues here -->

Tail limit keeps the prompt from exploding on long runs — MAPF can emit
feedback several times per second, and Nemotron starts hallucinating once
the context gets too noisy.
"""

import json


SYSTEM_PROMPT = """Ты — верховный оркестратор роя из 20 роботов под управлением Nav2 BT.
Когда BT-нод (MapfPlan / SetFormation / DisableFormation) встречает WARN или
получает периодический INFO от соответствующего компонента, он отправляет
тебе событие и последние строки накопленного лога.

Твоя задача — выбрать ровно одно из трёх решений:

  "wait"   — ситуация не критична, BT продолжает текущий план без изменений.
  "abort"  — ситуация безнадёжна (коллизия, недостижимая цель, фатальная
             ошибка планировщика). BT отменяет экшен и возвращает FAILURE.
  "replan" — текущий план устарел и нужен перерасчёт (таймаут, затор,
             растущий robot_stall без прогресса, много replans_done подряд).
             BT отменяет экшен и сигнализирует о перепланировании.

Отвечай строго валидным JSON формата:
{"decision": "wait"|"abort"|"replan", "reason": "краткое обоснование"}

Никакого текста вокруг, никаких markdown-блоков — только один JSON-объект.
"""


def build_few_shot_prompt(scenarios, level, event, log_buffer, tail=20):
    parts = [SYSTEM_PROMPT.strip(), '']

    parts.append('# Примеры')
    for scenario in scenarios:
        parts.append('## Вход')
        parts.append(f"level: {scenario['level']}")
        parts.append(f"event: {scenario['event']}")
        parts.append('log_buffer:')
        for line in scenario['log_buffer']:
            parts.append(f'  {line}')
        parts.append('## Решение')
        parts.append(json.dumps(scenario['decision'], ensure_ascii=False))
        parts.append('')

    parts.append('# Текущая ситуация')
    parts.append(f'level: {level}')
    parts.append(f'event: {event}')
    parts.append('log_buffer:')
    for line in log_buffer[-tail:]:
        parts.append(f'  {line}')
    parts.append('## Решение')
    parts.append('')  # LLM продолжит тут

    return '\n'.join(parts)
