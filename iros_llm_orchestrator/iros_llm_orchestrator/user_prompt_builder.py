"""Prompt builder for the user chat interface (channel 3).

Converts free-form natural language (Russian / English) into a structured
LlmCommand JSON that the BT runner can execute directly.

Unlike channel 1 (decision server, reactive) and channel 2 (passive observer,
proactive), channel 3 is user-driven: no BT state is involved, the user just
describes what they want in plain language.

Key design decisions:
- System prompt embeds the full warehouse layout and named locations so the
  user never has to remember exact coordinates.
- Conversation history (last N turns) is included as proper chat turns so
  references like "те же роботы" or "теперь в центр" resolve correctly.
- Few-shot examples are injected as alternating user/assistant turns right
  after the system prompt, before the history — this is the standard
  few-shot pattern for chat models.
- Output format is identical to LlmCommand.Goal so command_parser can
  validate it unchanged.
"""

import json


# ---------------------------------------------------------------------------
# Warehouse layout — must match Stage world (warehouse.world, 30×30 m)
# ---------------------------------------------------------------------------

WAREHOUSE_CONTEXT = """\
Склад: 30 × 30 м. Начало координат — нижний левый угол. +x → вправо, +y → вверх.

Именованные точки (используй их когда пользователь называет место):
  "центр"                   (15.0, 15.0)
  "левый нижний угол"       ( 2.0,  2.0)
  "правый нижний угол"      (28.0,  2.0)
  "левый верхний угол"      ( 2.0, 28.0)
  "правый верхний угол"     (28.0, 28.0)
  "зона загрузки"           ( 3.0,  5.0)   ← оранжевые роботы, нижний-левый кластер
  "зона выгрузки"           (27.0, 25.0)   ← синие роботы, верхний-правый кластер
  "левый центр"             ( 8.0, 15.0)
  "правый центр"            (22.0, 15.0)
  "верхний центр"           (15.0, 22.0)
  "нижний центр"            (15.0,  8.0)

Роботы:
  robot_0 .. robot_9   — оранжевые, база: зона загрузки  (нижний левый)
  robot_10 .. robot_19 — синие,     база: зона выгрузки  (верхний правый)

Когда несколько роботов едут в одну точку — распредели цели кластером с шагом 1.5 м.
Когда пользователь говорит "остальные на месте" или "не двигать" — не включай их в команду.
"""

SYSTEM_PROMPT = f"""\
Ты — командный интерпретатор роя из 20 роботов. Ты общаешься с оператором.

Пользователь пишет произвольную команду на русском или английском.
Твоя задача — ответить кратко по-человечески И сформировать JSON-команду.

{WAREHOUSE_CONTEXT}
Доступные режимы:
  "mapf"      — отправить роботов к точкам через anti-collision планировщик
  "formation" — выстроить формацию
  "idle"      — остановить всё

Отвечай строго одним JSON-объектом такого вида (без текста вокруг, без markdown):
{{
  "reply": "Краткий человекочитаемый ответ оператору — что делаешь и почему.",
  "command": {{
    "mode": "mapf",
    "robot_ids": [0, 1],
    "goals": [[x0, y0], [x1, y1]],
    "reason": "..."
  }}
}}

Поле "reply" — всегда на том же языке что и запрос пользователя.
Поле "command" — структура зависит от режима:

  mapf:      mode, robot_ids, goals (один goal на один robot_id), reason
  formation: mode, formation_id, leader_ns, follower_ns, offsets_x, offsets_y, reason
  idle:      mode, reason

Важно: если несколько роботов едут в одну зону — ты всё равно указываешь ОДНУ точку,
система сама раскидает их кластером. Не пытайся вручную считать offsets для кластера.
"""


# ---------------------------------------------------------------------------
# Few-shot examples (injected as alternating user/assistant turns)
# ---------------------------------------------------------------------------

def _cluster(cx: float, cy: float, n: int, step: float = 1.5):
    """Spread n goals around (cx, cy) in a compact cluster."""
    cols = max(1, round(n ** 0.5))
    goals = []
    for i in range(n):
        col = i % cols
        row = i // cols
        goals.append([
            round(cx + (col - (cols - 1) / 2.0) * step, 2),
            round(cy + row * step, 2),
        ])
    return goals


_EXAMPLES = [
    # 1. Two groups to opposite corners
    {
        "user": "роботы 1 2 3 в правый верхний угол, роботы 4 5 6 в левый нижний, остальные на месте",
        "out": {
            "reply": "Отправляю роботов 1-3 в правый верхний угол и роботов 4-6 в левый нижний. Остальные остаются на месте.",
            "command": {
                "mode": "mapf",
                "robot_ids": [1, 2, 3, 4, 5, 6],
                "goals": [[28.0, 28.0], [28.0, 28.0], [28.0, 28.0],
                          [2.0, 2.0],   [2.0, 2.0],   [2.0, 2.0]],
                "reason": "robots 1-3 to top-right, robots 4-6 to bottom-left",
            },
        },
    },
    # 2. Single robot with explicit coordinates
    {
        "user": "robot_7 to x:24 y:0",
        "out": {
            "reply": "Отправляю robot_7 в точку (24, 0).",
            "command": {
                "mode": "mapf",
                "robot_ids": [7],
                "goals": [[24.0, 0.0]],
                "reason": "robot_7 to (24, 0)",
            },
        },
    },
    # 3. All robots to center — same point, system will cluster
    {
        "user": "все роботы в центр",
        "out": {
            "reply": "Отправляю все 20 роботов в центр склада (15, 15). Система автоматически расположит их кластером.",
            "command": {
                "mode": "mapf",
                "robot_ids": list(range(20)),
                "goals": [[15.0, 15.0]] * 20,
                "reason": "all 20 robots to center",
            },
        },
    },
    # 4. Emergency stop
    {
        "user": "стоп, всем остановиться",
        "out": {
            "reply": "Останавливаю всех роботов.",
            "command": {
                "mode": "idle",
                "reason": "emergency stop",
            },
        },
    },
    # 5. Formation
    {
        "user": "robot_0 ведёт, robot_1 и robot_2 сзади в линию",
        "out": {
            "reply": "Выстраиваю линию: robot_0 ведёт, robot_1 и robot_2 следуют сзади.",
            "command": {
                "mode": "formation",
                "formation_id": "line",
                "leader_ns": "robot_0",
                "follower_ns": ["robot_1", "robot_2"],
                "offsets_x": [-1.5, -3.0],
                "offsets_y": [0.0, 0.0],
                "reason": "line formation: robot_0 leads",
            },
        },
    },
    # 6. Orange group to unloading zone
    {
        "user": "оранжевые роботы в зону выгрузки",
        "out": {
            "reply": "Отправляю оранжевых роботов (0-9) в зону выгрузки (27, 25).",
            "command": {
                "mode": "mapf",
                "robot_ids": list(range(10)),
                "goals": [[27.0, 25.0]] * 10,
                "reason": "orange robots (0-9) to unloading zone",
            },
        },
    },
]


def build_user_prompt(user_message: str, history: list | None = None) -> list:
    """Build a messages list for the Ollama /api/chat endpoint.

    Returns a list of {"role": ..., "content": ...} dicts.

    Args:
        user_message: The latest user utterance (natural language).
        history: Previous turns as [{"role": "user"/"assistant", "content": ...}].
                 Allows the model to resolve references like "те же роботы".

    Returns:
        Full messages list ready for Ollama's messages field.
    """
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Few-shot examples as proper alternating turns
    for ex in _EXAMPLES:
        messages.append({"role": "user",      "content": ex["user"]})
        messages.append({"role": "assistant", "content": json.dumps(ex["out"], ensure_ascii=False)})

    # Conversation history so the model can resolve references
    if history:
        messages.extend(history)

    # Current user message
    messages.append({"role": "user", "content": user_message})

    return messages


# ---------------------------------------------------------------------------
# BT event prompt — for inline warning analysis in user chat
# ---------------------------------------------------------------------------

_BT_EVENT_SYSTEM = """\
Ты — советник оператора роя роботов. BT-система только что сообщила о проблеме.
Кратко объясни что произошло (1-2 предложения) и предложи одно конкретное действие:
можно подождать, остановить всё ("стоп"), или задать новую команду.
Отвечай на том же языке что и предыдущий разговор. Без JSON — только текст.
"""


def build_bt_event_prompt(bt_state, history: list | None = None) -> list:
    """Build messages for analysing a BT WARN/ERROR event.

    Args:
        bt_state: BTState message from /bt/state.
        history:  Conversation history for language/context continuity.
    """
    messages = [{"role": "system", "content": _BT_EVENT_SYSTEM}]

    if history:
        messages.extend(history[-6:])  # last 3 turns for context

    event = (
        f"BT сообщает: [{bt_state.action_status}] "
        f"action={bt_state.active_action} "
        f"error='{bt_state.last_error}' "
        f"summary='{bt_state.action_summary}'"
    )
    messages.append({"role": "user", "content": event})
    return messages