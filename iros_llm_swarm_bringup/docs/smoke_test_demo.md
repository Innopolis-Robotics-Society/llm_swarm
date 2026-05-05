# Smoke Test: Full Demo

Пошаговая верификация `swarm_full_demo.launch.py` + `fleet_cmd`.

1. Сборка и source:
   ```bash
   colcon build && source install/setup.bash
   ```

2. Запустить демо:
   ```bash
   ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py
   ```
   Дождаться сообщения `==== Full demo ready ====` (≈22 сек).

3. В другом терминале (с `source install/setup.bash`) проверить actions:
   ```bash
   ros2 action list | grep -E "set_goals|llm"
   ```
   Должно быть 3 actions.

4. Проверить топик BT-runner-а:
   ```bash
   ros2 topic info /fleet/cmd
   ```
   `Subscription count: 1`.

5. Послать простой сценарий:
   ```bash
   ros2 run iros_llm_swarm_bt fleet_cmd --scenario simple
   ```

6. В логе launch появятся:
   - `fleet/cmd: robot_ids=...`
   - `fleet/cmd: goals=...`
   - `fleet/cmd: mode=mapf`
   - `MapfPlan IDLE -> RUNNING`

7. В RViz роботы поехали.

8. Через ~30 секунд:
   ```bash
   ros2 run iros_llm_swarm_bt fleet_cmd --scenario stress
   ```

9. В логе должны появиться WARN'ы от MAPF, реакции `decision_server` и `passive_observer`.

10. Проверить датасет команд:
    ```bash
    ls ~/.ros/llm_commands/
    ```
    Должны быть JSONL-файлы.

11. Последняя команда от LLM:
    ```bash
    cat ~/.ros/llm_commands/*.jsonl | tail -1 | python3 -m json.tool
    ```

## Проверка канала 2 (passive observer)

### Запуск с каналом 2

```bash
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py enable_passive_observer:=true
```

### Ручная проверка канала 2 (без реального WARN от MAPF)

В отдельном терминале отправить искусственный WARN в /bt/state:

```bash
ros2 topic pub /bt/state iros_llm_swarm_interfaces/msg/BTState \
  "{mode: 'mapf', action_status: 'WARN', active_action: 'MapfPlan', \
    action_summary: 'manual test', last_error: 'robot_3 stalled', stamp_ms: 0}" \
  --qos-profile sensor_data -r 2
```

Подождать ~2 секунды (cooldown). В логах orchestrator'а должно появиться:
```
[llm_passive_observer] WARN: Triggered by status=WARN action=MapfPlan error='robot_3 stalled'
[llm_passive_observer] LLM command: {'mode': 'mapf', ...}
[llm_passive_observer] Applied: success=True
```

Ctrl+C через 5 секунд. Проверить свежую запись в датасете:
```bash
cat ~/.ros/llm_commands/*.jsonl | tail -1 | python3 -m json.tool
```
Timestamp должен быть сегодняшним.
