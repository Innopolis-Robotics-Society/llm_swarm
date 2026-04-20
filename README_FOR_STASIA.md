Хорошая работа — правильная структура пакета, плагинная регистрация, тестовые XML. Но есть ряд проблем, которые нужно решить перед интеграцией с LLM. Разберу по пунктам.

## Что сделано хорошо

- Чистая pluginlib-регистрация через `BT_REGISTER_NODES` — ноды загружаются динамически
- CSV-парсинг для портов — простое решение для передачи списков через blackboard/XML
- Тестовый runner с blackboard — можно быстро проверить каждую ноду
- Отдельные XML для тестирования каждой ноды + pipeline XML
- Output-порты (`mapf_ok`, `mapf_info`, `formation_enabled`) — результаты пишутся в blackboard

## Проблема 1: все ноды `SyncActionNode` — блокируют BT

Это **главная** проблема. `SyncActionNode::tick()` блокирует поток BT до завершения. `MapfPlan` вызывает `spin_until_future_complete` с таймаутом 120 секунд (строка 148). Всё это время BT полностью заморожен — никакие другие ноды не тикаются, recovery не работает, cancellation невозможна.

С нашим long-lived action это стало ещё критичнее: action теперь живёт минуты (пока все роботы доедут), значит `MapfPlan::tick()` будет блокировать BT на **минуты**.

Решение — `BT::StatefulActionNode` или `nav2_behavior_tree::BtActionNode<SetGoals>`:

```cpp
class MapfPlan : public BT::StatefulActionNode
{
  BT::NodeStatus onStart() override {
    // Отправить goal async, вернуть RUNNING
  }
  BT::NodeStatus onRunning() override {
    // Проверить future, вернуть RUNNING/SUCCESS/FAILURE
  }
  void onHalted() override {
    // Cancel goal
  }
};
```

## Проблема 2: `rclcpp::spin_until_future_complete(node, ...)` в BT-ноде

Строки 130, 148, 239, 283 — вызов `spin_until_future_complete` на `node` который уже спинится в executor. В лучшем случае это работает случайно, в худшем — deadlock. Nav2 решает это через свой `BtActionNode`/`BtServiceNode`, у которых внутри отдельный callback group и правильная обработка futures.

## Проблема 3: action client создаётся при каждом tick

Строка 93: `auto client = rclcpp_action::create_client<SetGoals>(node, "/swarm/set_goals")` — клиент создаётся заново при каждом вызове `tick()`. Это значит новое подключение к action server, новый discovery, ожидание 5 секунд. Клиент должен создаваться один раз в конструкторе.

То же самое для service clients в `SetFormation` (строка 224) и `DisableFormation` (строка 273).

## Проблема 4: goal building зашит в BT-ноду

`MapfPlan` сам генерирует grid pattern из `goal_x`/`goal_y` (строки 104–121). Это должен делать вызывающий код (LLM или другая BT-нода). `MapfPlan` должен принимать готовые `robot_ids[]` + `goals[]`, а не вычислять позиции.

Для LLM-интеграции это важно — LLM будет генерировать конкретные координаты для каждого робота, а не "все к одной точке сеткой".

## Проблема 5: `behaviortree_cpp_v3` вместо v4

Nav2 Humble использует BehaviorTree.CPP v3, так что это работает. Но стоит знать, что в v4 синтаксис портов и регистрации изменился. Пока не проблема, но при миграции на Jazzy/Rolling нужно будет обновить.

## Проблема 6: нет condition-нод

Для подхода 2 (LLM через blackboard) нужны condition-ноды для проверки режима:

```xml
<!-- Сейчас нет способа проверить mode в BT -->
<Fallback>
  <Sequence>
    <CheckMode expected="mapf"/>  <!-- нужна эта нода -->
    <MapfPlan .../>
  </Sequence>
  <Sequence>
    <CheckMode expected="formation"/>
    <SetFormation .../>
  </Sequence>
</Fallback>
```

## Проблема 7: `test_swarm_pipeline.xml` — Sequence, но должен быть Fallback

Текущий pipeline:
```xml
<Sequence>
  <MapfPlan .../>
  <SetFormation .../>
</Sequence>
```

Это значит "сначала MAPF, потом formation" — последовательно. Но MAPF и formation — взаимоисключающие режимы. Должен быть `Fallback` с условием, или переключение по `@mode`.

## Итого: план доработки

| Что | Приоритет | Сложность |
|---|---|---|
| `MapfPlan` → `StatefulActionNode` (неблокирующий) | Критично | Средне |
| Убрать `spin_until_future_complete`, использовать async futures | Критично | Средне |
| Action/service клиенты в конструктор | Просто | Просто |
| Убрать grid-pattern из MapfPlan, принимать готовые goals | Средне | Просто |
| Добавить `CheckMode` condition-ноду | Нужно для LLM | Просто |
| Добавить `EmergencyStop` action-ноду | Нужно для LLM | Просто |
| Написать главный XML с mode-switching | Нужно для LLM | Средне |

Фундамент хороший — структура пакета, плагины, порты. Основная работа — переделать `MapfPlan` из sync в async, и добавить condition-ноды для LLM.







----------------------------------------------


Нужно добавить формовщика!


----------------------------------------------







## Поэтапный план до рабочего Подхода 2

### Фаза 1 — Исправить BT-ноды (то что обсудили)

Переделать `MapfPlan` из `SyncActionNode` в `StatefulActionNode`, вынести client creation в конструктор, убрать `spin_until_future_complete`. То же для `SetFormation` и `DisableFormation`. После этого BT-дерево может тикаться без блокировок.

### Фаза 2 — Добавить condition-ноды и emergency stop

Три новых ноды:

```cpp
// Читает @mode из blackboard, сравнивает с expected
class CheckMode : public BT::ConditionNode {
  BT::NodeStatus tick() override {
    auto mode = getInput<std::string>("mode");
    auto expected = getInput<std::string>("expected");
    return (mode.value() == expected.value())
        ? BT::NodeStatus::SUCCESS
        : BT::NodeStatus::FAILURE;
  }
};

// Проверяет, изменились ли goals/mode с прошлого тика
class HasNewCommand : public BT::ConditionNode { ... };

// Шлёт пустые пути всем роботам
class EmergencyStop : public BT::StatefulActionNode { ... };
```

### Фаза 3 — Написать главный XML с mode-switching

```xml
<root BTCPP_format="4" main_tree_to_execute="FleetMain">
<BehaviorTree ID="FleetMain">
  <ReactiveSequence>

    <!-- Аварийная остановка по флагу из blackboard -->
    <Fallback>
      <Inverter><IsEqual key="{@emergency}" value="true"/></Inverter>
      <EmergencyStop robot_ids_csv="{@robot_ids_csv}"/>
    </Fallback>

    <!-- Основная логика: выбор режима -->
    <ReactiveFallback>

      <!-- Idle — ничего не делать -->
      <CheckMode mode="{@mode}" expected="idle"/>

      <!-- MAPF -->
      <Sequence>
        <CheckMode mode="{@mode}" expected="mapf"/>
        <MapfPlan
          robot_ids_csv="{@robot_ids_csv}"
          goals_csv="{@goals_csv}"
          mapf_ok="{@mapf_ok}"
          mapf_info="{@mapf_info}"/>
      </Sequence>

      <!-- Formation -->
      <Sequence>
        <CheckMode mode="{@mode}" expected="formation"/>
        <SetFormation
          formation_id="{@formation_id}"
          leader_ns="{@leader_ns}"
          follower_ns_csv="{@follower_ns_csv}"
          offsets_x_csv="{@offsets_x_csv}"
          offsets_y_csv="{@offsets_y_csv}"
          activate="true"/>
      </Sequence>

    </ReactiveFallback>
  </ReactiveSequence>
</BehaviorTree>
</root>
```

`@`-префикс — глобальный blackboard, доступный из любого поддерева.

### Фаза 4 — Fleet BT Runner (lifecycle node)

Заменить `test_bt_runner` на полноценную lifecycle-ноду, которая тикает дерево в цикле:

```cpp
class FleetBTRunner : public nav2_util::LifecycleNode {
  // configure(): загрузить XML, создать дерево
  // activate(): запустить timer для тиков
  // deactivate(): остановить timer

  void tick_tree() {
    // Вызывается по таймеру, например 10 Hz
    tree_.tickOnce();
  }

  // Blackboard доступен снаружи через shared_ptr
  BT::Blackboard::Ptr blackboard_;
};
```

### Фаза 5 — ROS-сервис для записи в blackboard

Это мост между LLM и BT. Один простой generic-сервис:

```
# SetBlackboardParam.srv
string key
string value
string type   # "string", "double", "int", "bool"
---
bool success
```

Нода `FleetBTRunner` хостит этот сервис:

```cpp
void on_set_param(const SetBlackboardParam::Request::SharedPtr req, ...) {
  if (req->type == "string")
    blackboard_->set<std::string>(req->key, req->value);
  else if (req->type == "double")
    blackboard_->set<double>(req->key, std::stod(req->value));
  // ...
  res->success = true;
}
```

Плюс сервис для чтения текущего состояния:

```
# GetFleetStatus.srv
---
string mode
string mapf_status       # "idle" / "executing" / "replanning"
uint32 robots_arrived
uint32 robots_active
uint32 replans_done
bool formation_enabled
string active_formation
```

### Фаза 6 — LLM Agent (Python node)

```python
class FleetLLMAgent(Node):
    def __init__(self):
        super().__init__('fleet_llm_agent')

        self.set_param_client = self.create_client(
            SetBlackboardParam, '/fleet/set_param')
        self.status_client = self.create_client(
            GetFleetStatus, '/fleet/status')

        # LLM tools — это то, что LLM может вызывать
        self.tools = [
            {
                "name": "send_robots_to_goals",
                "description": "Send robots to specific goals using MAPF",
                "parameters": {
                    "robot_ids": "comma-separated ids",
                    "goals": "list of {x, y} coordinates"
                }
            },
            {
                "name": "set_formation",
                "description": "Arrange robots in formation",
                "parameters": {
                    "type": "line/wedge/diamond",
                    "leader": "robot namespace",
                    "followers": "comma-separated namespaces"
                }
            },
            {
                "name": "stop_all",
                "description": "Emergency stop all robots"
            },
            {
                "name": "get_status",
                "description": "Get current fleet status"
            }
        ]

        # Цикл принятия решений
        self.timer = self.create_timer(5.0, self.decision_loop)

    def decision_loop(self):
        # 1. Собрать статус
        status = self.call_get_status()

        # 2. Спросить LLM
        response = call_llm(
            system_prompt=FLEET_SYSTEM_PROMPT,
            messages=self.history,
            tools=self.tools,
            context=f"Current fleet status: {status}"
        )

        # 3. Выполнить tool calls → пишут в blackboard
        for tool_call in response.tool_calls:
            self.execute_tool(tool_call)

    def execute_tool(self, tool_call):
        if tool_call.name == "send_robots_to_goals":
            # Записать в blackboard через сервис
            self.set_bb("mode", "mapf", "string")
            self.set_bb("robot_ids_csv", tool_call.args["robot_ids"], "string")
            self.set_bb("goals_csv", tool_call.args["goals"], "string")

        elif tool_call.name == "set_formation":
            self.set_bb("mode", "formation", "string")
            self.set_bb("formation_id", tool_call.args["type"], "string")
            self.set_bb("leader_ns", tool_call.args["leader"], "string")
            # ...

        elif tool_call.name == "stop_all":
            self.set_bb("emergency", "true", "string")

    def set_bb(self, key, value, type_str):
        req = SetBlackboardParam.Request()
        req.key = key
        req.value = value
        req.type = type_str
        self.set_param_client.call(req)
```

### Полный стек при работе

```
LLM Agent (5 Hz decision loop)
  │ "роботы 0-9 на склад B"
  │
  │  service: /fleet/set_param {key:"mode", value:"mapf"}
  │  service: /fleet/set_param {key:"robot_ids_csv", value:"0,1,...,9"}
  │  service: /fleet/set_param {key:"goals_csv", value:"..."}
  ▼
Fleet BT Runner (10 Hz tick loop)
  │ blackboard: @mode="mapf", @robot_ids_csv="0,1,...,9"
  │
  │ BT XML: CheckMode → mode==mapf → MapfPlan
  │ MapfPlan отправляет action goal в /swarm/set_goals
  ▼
MAPF Planner (long-lived action)
  │ plan → dispatch paths → monitor → replan → ...
  │ feedback → BT пишет в blackboard → LLM читает через /fleet/status
  ▼
path_follower × 20 → controller_server × 20 → роботы едут
```

### Порядок реализации

| Фаза | Что делать | Зависит от |
|---|---|---|
| 1 | Async BT-ноды | Ничего |
| 2 | Condition-ноды | Фаза 1 |
| 3 | Главный XML | Фазы 1-2 |
| 4 | Fleet BT Runner | Фаза 3 |
| 5 | Blackboard сервис | Фаза 4 |
| 6 | LLM Agent | Фаза 5 |

Фазы 1-4 — чистый C++/ROS. Фаза 5 — один простой сервис. Фаза 6 — Python + LLM API. Каждая фаза тестируется независимо: после фазы 4 можно управлять флотом через `ros2 service call`, без LLM.

