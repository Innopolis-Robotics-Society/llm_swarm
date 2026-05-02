# LLM-оркестратор для Nav2 BT Swarm — документация разработчика

## Оглавление

1. [Архитектура](#2-архитектура)
3. [Ключевые понятия](#3-ключевые-понятия)
4. [Два канала LLM](#4-два-канала-llm)
5. [ROS2 интерфейсы](#5-ros2-интерфейсы)
6. [Пакеты и файлы](#6-пакеты-и-файлы)
7. [BT-ноды (C++)](#7-bt-ноды-c)
8. [Python-ноды](#8-python-ноды)
9. [Промпты и few-shot сценарии](#9-промпты-и-few-shot-сценарии)
10. [Mock-режим LLM](#10-mock-режим-llm)
11. [Сбор датасета для SFT](#11-сбор-датасета-для-sft)
12. [Запуск и демонстрация](#12-запуск-и-демонстрация)



## 1. Архитектура

```
┌──────────────────────────────────────────────────────────┐
│                Nav2 BT (C++ процесс)                     │
│                                                          │
│  Blackboard (shared memory, доступ только изнутри BT)    │
│  ┌────────────────────────────────────────────────────┐  │
│  │ @mode         = "idle" | "mapf" | "formation"     │  │
│  │ @robot_ids    = [0, 1, 2, ...]                    │  │
│  │ @goals        = [Point, Point, ...]               │  │
│  │ @action_status = "OK" | "WARN" | "ERROR"          │  │
│  │ @active_action = "MapfPlan" | "SetFormation" | ...│  │
│  │ @last_error   = "robot_3 stalled" | ""            │  │
│  │ @action_summary = "..." (одна строка feedback)    │  │
│  │ @formation_id, @leader_ns, @follower_ns, ...      │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌────────────┐  ┌───────────────┐  ┌───────────────┐  │
│  │ BTState    │  │ LlmCommand   │  │ ModeDispatch  │  │
│  │ Publisher  │→ │ Receiver     │  │ (CheckMode +  │  │
│  │(→/bt/state)│  │(←/llm/command)│  │ MapfPlan/     │  │
│  └────────────┘  └───────────────┘  │ SetFormation) │  │
│                                      └───────────────┘  │
│                                                          │
│  MapfPlan ──→ /llm/decision (канал 1)                   │
│  SetFormation ──→ /llm/decision (канал 1)               │
│  DisableFormation ──→ /llm/decision (канал 1)           │
└──────────────────────────────────────────────────────────┘
         │ publish                    ▲ action goal
         ▼                            │
┌──────────────────────────────────────────────────────────┐
│           Python ноды (отдельный процесс)                │
│                                                          │
│  decision_server.py  ← action server /llm/decision       │
│  (канал 1: реактивный, по запросу от BT-нода)            │
│                                                          │
│  passive_observer.py ← subscriber /bt/state              │
│  (канал 2: проактивный, сам вмешивается)                 │
│  → action client /llm/command                            │
└──────────────────────────────────────────────────────────┘
         │ TODO
         ▼ HTTP POST
┌──────────────────────┐
│ vLLM / mock          │
│ Nemotron-3-Nano-30B  │
└──────────────────────┘
```

---

## 3. Ключевые понятия

### Nav2 BT Blackboard

Blackboard — это key-value хранилище **внутри памяти** процесса BT. Это не ROS-топик, не сервис. Доступ к нему только через C++ API `config().blackboard->get/set(...)` изнутри BT-нодов. Python-нода или любой внешний процесс не может ни читать, ни писать в blackboard напрямую.

Для связи blackboard с внешним миром используются два BT-нода-моста:
- `BTStatePublisher` — каждый тик BT читает ключи из blackboard и публикует их в ROS-топик `/bt/state`.
- `LlmCommandReceiver` — action server, принимает команды извне и записывает их в blackboard.

### ReactiveSequence

XML дерева обёрнуто в `ReactiveSequence`. Отличие от обычной `Sequence`: ReactiveSequence тикает **всех** детей на каждом тике, начиная с первого. Это нужно чтобы `BTStatePublisher` и `LlmCommandReceiver` выполнялись каждый тик, параллельно с `ModeDispatch`.

### Thread safety

BT крутится в своём треде. ROS2 executor крутится в своём. Когда `LlmCommandReceiver` получает action goal через ROS-callback (executor-тред), он не может сразу писать в blackboard — это было бы data race. Вместо этого goal сохраняется в `pending_goal_` под мьютексом, а реальная запись в blackboard происходит в `tick()` (BT-тред) на следующем тике.

---

## 4. Два канала LLM

### Канал 1 — реактивный (`/llm/decision`)

**Кто инициирует:** BT-нод (`MapfPlan`, `SetFormation`, `DisableFormation`).

**Когда:** конкретный BT-нод получает feedback с warning'ом или его сервисный вызов провалился.

**Как работает:**
1. `MapfPlan` получает `feedback->warning = "robot_3 stalled"` от MAPF планировщика.
2. `MapfPlan` формирует action goal: `{level: "WARN", event: "robot_3 stalled", log_buffer: [последние 50 строк]}`.
3. Отправляет на `/llm/decision` (action client → action server `decision_server.py`).
4. `decision_server.py` строит few-shot промпт, зовёт LLM, парсит JSON-ответ.
5. Возвращает `{decision: "wait" | "abort" | "replan"}`.
6. `MapfPlan` применяет вердикт: `wait` — продолжить, `abort` — FAILURE, `replan` — FAILURE + `@mapf_decision="replan"`.

**Для `SetFormation` и `DisableFormation`:** при `!res->success` после сервисного вызова отправляют запрос в `/llm/decision`. При `wait` — retry (до `max_retries`), при `abort`/`replan` — FAILURE.

**Всегда активен** — не зависит от параметров. Работает пока BT крутится и `decision_server.py` запущен.

### Канал 2 — проактивный (`/bt/state` → `/llm/command`)

**Кто инициирует:** LLM сама, через `passive_observer.py`.

**Когда:** observer видит `action_status ∈ {WARN, ERROR}` в топике `/bt/state`.

**Как работает:**
1. `BTStatePublisher` публикует snapshot blackboard в `/bt/state` каждый тик.
2. `passive_observer.py` подписан на `/bt/state`, держит буфер последних 20 состояний.
3. Когда `msg.action_status` равен WARN или ERROR (и прошёл cooldown, и не идёт другой вызов):
   - Строит command-промпт с историей и few-shot примерами.
   - Зовёт LLM — получает полную JSON-команду (`{mode, robot_ids, goals, reason, ...}`).
   - Отправляет action goal на `/llm/command`.
4. `LlmCommandReceiver` (BT-нод, action server) принимает goal, пишет в blackboard: `@mode`, `@robot_ids`, `@goals` и т.д.
5. На следующем тике `ModeDispatch` видит новый `@mode` и переключает ветку BT.

**Выключен по умолчанию.** Включается launch-аргументом `enable_passive_observer:=true`. Параметр `enabled` можно менять на лету через `ros2 param set`.

### Сравнение каналов

| Свойство | Канал 1 | Канал 2 |
|----------|---------|---------|
| Интерфейс | `/llm/decision` (action) | `/bt/state` (topic) → `/llm/command` (action) |
| Инициатор | BT-нод | LLM сама |
| Ответ | Один из трёх: wait/abort/replan | Полная команда: mode + параметры |
| Контекст | Конкретный feedback + ring buffer | Последние 20 snapshot'ов blackboard |
| По умолчанию | Всегда активен | Выключен |
| Датасет | `~/.ros/llm_decisions/` | `~/.ros/llm_commands/` |

---

## 5. ROS2 интерфейсы

Все интерфейсы определены в пакете `iros_llm_swarm_interfaces`.

### `msg/BTState.msg`

Snapshot blackboard, публикуется `BTStatePublisher` каждый тик.

| Поле | Тип | Описание |
|------|-----|----------|
| `mode` | string | `idle` / `mapf` / `formation` |
| `action_status` | string | `OK` / `WARN` / `ERROR` |
| `active_action` | string | `MapfPlan` / `SetFormation` / `DisableFormation` / `none` |
| `action_summary` | string | Одна строка feedback'а |
| `last_error` | string | Текст ошибки (пусто при OK) |
| `robot_ids` | uint32[] | Текущие robot_ids (контекст mapf) |
| `goals` | Point[] | Текущие цели (контекст mapf) |
| `formation_id` | string | Контекст formation |
| `leader_ns` | string | Контекст formation |
| `stamp_ms` | int64 | Timestamp в миллисекундах |

### `action/LlmDecision.action` (канал 1)

| Часть | Поля | Описание |
|-------|------|----------|
| Goal | `level` (string), `event` (string), `log_buffer` (string[]) | Запрос от BT-нода: уровень (WARN/INFO), событие, лог |
| Result | `decision` (string) | `wait` / `abort` / `replan` / `""` |
| Feedback | `stage` (string) | `received` / `thinking` / `done` |

### `action/LlmCommand.action` (канал 2)

| Часть | Поля | Описание |
|-------|------|----------|
| Goal | `mode`, `robot_ids`, `goals`, `formation_id`, `leader_ns`, `follower_ns`, `offsets_x`, `offsets_y`, `reason` | Полная команда от LLM |
| Result | `success` (bool), `info` (string) | Результат применения |
| Feedback | `stage` (string) | `received` / `applied` |

---

## 6. Пакеты и файлы

### `iros_llm_swarm_interfaces`

ROS2 интерфейсы (msg/srv/action). Генерируются при сборке в C++ и Python. LLM-специфичные файлы:
- `msg/BTState.msg`
- `action/LlmDecision.action`
- `action/LlmCommand.action`

### `iros_llm_swarm_bt`

C++ BT-ноды и runner. LLM-специфичные компоненты:

```
include/iros_llm_swarm_bt/swarm_bt_nodes.hpp
  ├── class MapfPlan         — action client /swarm/set_goals + action client /llm/decision
  ├── class SetFormation     — service client /formation/set + action client /llm/decision
  ├── class DisableFormation — service client /formation/deactivate + action client /llm/decision
  ├── class BTStatePublisher — publisher /bt/state (sync, каждый тик)
  └── class LlmCommandReceiver — action server /llm/command (sync, pending_goal + mutex)

src/swarm_bt_nodes.cpp      — реализация всех нодов
behavior_trees/swarm_navigate_to_pose.xml — XML дерева с ReactiveSequence
scripts/fleet_cmd.py        — CLI утилита для отправки команд BT через /fleet/cmd
```

### `iros_llm_orchestrator`

Python ROS2 пакет с двумя нодами:

```
iros_llm_orchestrator/
├── decision_server.py          — action server /llm/decision (канал 1)
├── passive_observer.py         — subscriber /bt/state + action client /llm/command (канал 2)
├── llm_client.py               — обёртка: mock / http / local backends
├── prompt_builder.py           — few-shot промпт для канала 1 (decision)
├── command_prompt_builder.py   — few-shot промпт для канала 2 (command)
├── decision_parser.py          — парсер JSON → "wait"/"abort"/"replan"
├── command_parser.py           — парсер JSON → полная команда {mode, robot_ids, ...}
├── scenarios.py                — 10 few-shot примеров для канала 1
├── command_scenarios.py        — 6 few-shot примеров для канала 2
├── decision_logger.py          — append-only JSONL логгер (один файл на UTC-день)

config/orchestrator.yaml        — параметры обоих нодов
launch/orchestrator.launch.py   — launch двух нод с аргументом enable_passive_observer
test/                           — unit и integration тесты
docs/
  ├── implementation_guide.md   — детали реализации
  ├── replan_workflow.md        — дизайн-документ по replan (открытый вопрос)
  └── llm_pipeline_theory.md    — теоретические основы
```

### `iros_llm_swarm_bringup`

Launch файлы. LLM-специфичный: `swarm_full_demo.launch.py` — запускает всю систему одной командой (симуляция + MAPF + Nav2 + LLM orchestrator + BT runner).

---

## 7. BT-ноды (C++)

### BTStatePublisher

Тип: `SyncActionNode`. Всегда возвращает `SUCCESS`.

Каждый тик: читает из blackboard ключи `@mode`, `@action_status`, `@active_action`, `@action_summary`, `@last_error`, `@robot_ids`, `@goals`, `@formation_id`, `@leader_ns`, добавляет `stamp_ms` и публикует в `/bt/state` с QoS `best_effort`.

Внутри `tick()` нет никаких `RCLCPP_INFO` — BT тикает 10-100 Hz, логирование заспамит консоль.

Векторные поля blackboard (`@robot_ids`, `@goals`) обёрнуты в `try/catch` — на старте BT ключей может не быть, и `blackboard->get<T>()` кинет исключение.

### LlmCommandReceiver

Тип: `SyncActionNode`. Всегда возвращает `SUCCESS`.

Action server на `/llm/command`. Получает goal от `passive_observer.py` и записывает содержимое в blackboard.

Паттерн потокобезопасности:
1. `handle_accepted()` (executor-тред): `pending_goal_ = goal; pending_handle_ = handle;` под мьютексом.
2. `tick()` (BT-тред): забирает `pending_goal_` под мьютексом, вызывает `apply_to_blackboard()`.

Валидация в `handle_goal()`: mode должен быть один из `idle`/`mapf`/`formation`, иначе REJECT. Если уже есть необработанный goal — тоже REJECT (один goal за раз).

### MapfPlan

Тип: `StatefulActionNode`. Action client к `/swarm/set_goals` и `/llm/decision`.

Ключевые LLM-механизмы:
- **Ring buffer** (`info_buffer_`, 50 строк) — каждый feedback формируется в строку и добавляется в буфер.
- **Blackboard-записи**: при каждом feedback — `@action_status`, `@action_summary`, `@last_error`.
- **LLM-запросы**: при `fb->warning` не пустом — немедленный `send_to_llm("WARN", ...)`. При `fb->info` не пустом — периодический (раз в `llm_log_interval_sec`).
- **Применение вердикта**: в `onRunning()` проверяется `llm_result_future_`. `wait` → ничего, `abort` → cancel action + FAILURE, `replan` → cancel + FAILURE + `@mapf_decision="replan"`.
- **Защита от спама**: `llm_pending_` — пока летит один запрос, новые не отправляются.

### SetFormation / DisableFormation

Обёртки над сервисами `/formation/set` и `/formation/deactivate`. LLM-интеграция по образцу MapfPlan но проще:
- При `!res->success` → `send_to_llm("WARN", last_error_)`.
- `wait` → retry (счётчик `retry_count_` до `max_retries_`).
- `abort` → FAILURE.
- `replan` → FAILURE + `@formation_decision="replan"` (для SetFormation) или collapse к `abort` (для DisableFormation, т.к. «перепланировать отмену» бессмысленно).

---

## 8. Python-ноды

### decision_server.py (канал 1)

Action server `/llm/decision`. Принимает goal, строит промпт, зовёт LLM, возвращает вердикт.

Защиты:
- `asyncio.Semaphore(max_concurrent=1)` — не более одного одновременного вызова LLM.
- `asyncio.wait_for(..., timeout=10s)` — жёсткий таймаут.
- При любом сбое → `default_on_error = "wait"` (безопасный дефолт, не ломает работающую систему).
- `ReentrantCallbackGroup` — чтобы feedback publishing не блокировался внутри execute callback.

Каждый вызов логируется в `~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl`.

### passive_observer.py (канал 2)

Подписан на `/bt/state` (QoS: `best_effort` — должен совпадать с publisher'ом в `BTStatePublisher`).

Триггер: `msg.action_status ∈ {WARN, ERROR}` + cooldown 10 сек + `is_thinking == False` + `enabled == True`.

При срабатывании триггера:
1. Строит command-промпт с историей (последние 20 состояний) и few-shot примерами.
2. Зовёт LLM через `asyncio.run_coroutine_threadsafe()` в отдельном asyncio event loop (фоновый тред). Это нужно потому что ROS2 executor не поддерживает `create_task` для корутин.
3. Парсит ответ в JSON-команду (`parse_llm_command()`).
4. Отправляет action goal на `/llm/command`.
5. Логирует всё (включая ошибки) в `~/.ros/llm_commands/decisions_YYYYMMDD.jsonl`.

По умолчанию **выключен** (`enabled: false`). Включается через launch-аргумент `enable_passive_observer:=true` или `ros2 param set llm_passive_observer enabled true` на лету.

### llm_client.py

Три backend'а:
- `mock` — детерминированная эвристика по ключевым словам в промпте. Для `prompt_kind='decision'`: `timeout/stall/stuck → replan`, `collision/unreachable → abort`, иначе `wait`. Для `prompt_kind='command'`: парсит `status=` и `action=` из промпта, возвращает соответствующую JSON-команду.
- `http` — POST на OpenAI-совместимый endpoint (vLLM / TGI). Параметры: `endpoint`, `model`, `max_tokens`, `temperature`.
- `local` — заглушка под будущий инференс через `transformers` (не реализован).

---

## 9. Промпты и few-shot сценарии

### Канал 1 — decision prompt

System prompt объясняет LLM три возможных вердикта (`wait`, `abort`, `replan`). Few-shot примеры (10 штук, файл `scenarios.py`) покрывают:
- WARN → wait (одиночный stall)
- WARN → replan (множественные stalls)
- WARN → abort (коллизия)
- INFO → wait (всё нормально)
- Formation-сценарии

Формат ответа: `{"decision": "wait"|"abort"|"replan", "reason": "..."}`.

### Канал 2 — command prompt

System prompt объясняет LLM три режима (`idle`, `mapf`, `formation`) и формат JSON-команды со всеми полями. Few-shot примеры (6 штук, файл `command_scenarios.py`) покрывают:
- MapfPlan WARN → replan без проблемных роботов
- MapfPlan ERROR → idle (halt)
- SetFormation WARN → formation с другим leader'ом
- SetFormation ERROR → idle
- MapfPlan WARN (too many replans) → formation (сменить стратегию)
- DisableFormation WARN → idle

Формат ответа: `{"mode": "...", "robot_ids": [...], "goals": [[x,y],...], "formation_id": "...", ...}`.

---

## 10. Mock-режим LLM

По умолчанию оба канала работают в mock-режиме (`llm_mode: "mock"`). Это детерминированная эвристика без нейросети — для разработки и тестирования.

Mock не делает HTTP-запросов, не требует GPU, возвращает ответ мгновенно. Эвристика описана в `llm_client.py`:

Для канала 1 (`_generate_mock`): ищет в промпте ключевые слова `timeout`, `stall`, `stuck`, `collision`, `unreachable`, `fatal` и возвращает соответствующий вердикт.

Для канала 2 (`_generate_mock_command`): парсит из раздела `# Текущая ситуация` значения `status=` и `action=`, возвращает JSON-команду:
- status=ERROR → `{"mode": "idle"}`
- status=WARN + action=MapfPlan → `{"mode": "mapf", ...}` (replan)
- status=WARN + action=SetFormation → `{"mode": "formation", ...}` (retry с другим leader)
- иначе → `{"mode": "idle"}`

---

## 11. Сбор датасета для SFT

Каждый вызов LLM автоматически записывается в JSONL. Файлы ротируются по UTC-дням.

### Канал 1: `~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl`

```json
{
  "timestamp": "2026-04-30T10:23:45+00:00",
  "goal": {"level": "WARN", "event": "robot_3 stalled", "log_buffer": [...]},
  "prompt": "...полный few-shot промпт...",
  "raw_output": "{\"decision\": \"wait\", \"reason\": \"...\"}",
  "decision": "wait"
}
```

### Канал 2: `~/.ros/llm_commands/decisions_YYYYMMDD.jsonl`

```json
{
  "timestamp": "2026-04-30T10:23:46+00:00",
  "trigger": {"action_status": "WARN", "active_action": "MapfPlan", "last_error": "..."},
  "history_snapshot": [{"mode": "mapf", "action_status": "WARN", ...}, ...],
  "prompt": "...полный command-промпт...",
  "raw_output": "{\"mode\": \"mapf\", ...}",
  "command": {"mode": "mapf", "robot_ids": [...], ...},
  "error": ""
}
```

### Путь к SFT

1. Набрать минимум 500-1000 записей в каждом датасете.
2. Вручную или автоматически (по исходу миссии) разметить `approved: true/false`.
3. Конвертировать в формат TRL SFTTrainer: `{messages: [{role: "user", content: prompt}, {role: "assistant", content: raw_output}]}`.
4. Обучить LoRA-адаптер поверх Nemotron через ClearML агент.
5. Подключить адаптер к vLLM (`--enable-lora`), поменять `llm_model` в yaml на имя адаптера.

---

## 12. Запуск и демонстрация

### Сборка

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Полный стек одной командой

```bash
# Только канал 1 (по умолчанию)
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py

# Оба канала (канал 2 включён)
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py enable_passive_observer:=true
```

Ждать ~22 секунды до сообщения `==== Full demo ready ====`. В этот момент: Stage симулятор + RViz + Nav2 + MAPF + LLM orchestrator + BT runner — всё работает.

### Запуск сценариев

В отдельном терминале (после `source install/setup.bash`):

```bash
# 4 робота к свободным целям (baseline, warning'ов не будет)
ros2 run iros_llm_swarm_bt fleet_cmd --scenario simple

# 20 роботов кросс-свап через весь warehouse (стресс-тест MAPF)
ros2 run iros_llm_swarm_bt fleet_cmd --scenario stress

# 4 робота в одну точку у стены (провокация ошибок)
ros2 run iros_llm_swarm_bt fleet_cmd --scenario unreachable

# Простая формация
ros2 run iros_llm_swarm_bt fleet_cmd --scenario formation_simple

# Вернуть в idle
ros2 run iros_llm_swarm_bt fleet_cmd --scenario idle
```

Ручной режим:
```bash
ros2 run iros_llm_swarm_bt fleet_cmd \
  --mode mapf --robots 0,1,2,3 \
  --goals "15.0,15.0;16.5,15.0;15.0,16.5;16.5,16.5"
```

### Мониторинг

```bash
# Поток /bt/state (best_effort QoS!)
ros2 topic echo /bt/state --qos-profile sensor_data --once

# Частота публикации
ros2 topic hz /bt/state --qos-profile sensor_data

# Warning'и из MAPF feedback
ros2 topic echo /swarm/set_goals/_action/feedback --field feedback.warning

# Датасеты
ls -la ~/.ros/llm_decisions/ ~/.ros/llm_commands/
cat ~/.ros/llm_commands/*.jsonl | tail -1 | python3 -m json.tool
```

### Ручная проверка канала 2

Если MAPF не выдаёт warning'ов (сценарий слишком простой), можно вкинуть искусственный:

```bash
ros2 topic pub /bt/state iros_llm_swarm_interfaces/msg/BTState \
  "{mode: 'mapf', action_status: 'WARN', active_action: 'MapfPlan', \
    action_summary: 'manual test', last_error: 'robot_3 stalled', stamp_ms: 0}" \
  --qos-profile sensor_data -r 2
# Подождать 2 секунды, Ctrl+C
# В логах orchestrator'а должно появиться "Triggered by status=WARN ..."
```

---

