from pathlib import Path
from typing import Any

import yaml

from .task_model import CARRY, POINT, Task, TaskInstance


def load_tasks(scenario_yaml: str | Path, scenario_name: str) -> list[TaskInstance]:
    path = Path(scenario_yaml)
    raw: Any = yaml.safe_load(path.read_text())
    scenarios = raw.get("scenarios", {})
    scenario = scenarios.get(scenario_name, {})
    entries = scenario.get("tasks", [])
    return [_parse(e) for e in entries]


def _parse(entry: dict[str, Any]) -> TaskInstance:
    task_type = entry["type"]
    if task_type not in (POINT, CARRY):
        raise ValueError(f"Unknown task type: {task_type!r}")

    pos = entry["position"]
    drop = entry.get("dropoff", [0.0, 0.0])

    task = Task(
        id=str(entry["id"]),
        type=task_type,
        label=str(entry.get("label", entry["id"])),
        radius=float(entry["radius"]),
        position=(float(pos[0]), float(pos[1])),
        dropoff=(float(drop[0]), float(drop[1])),
    )
    return TaskInstance(task=task)
