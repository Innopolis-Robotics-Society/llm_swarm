from dataclasses import dataclass, field

POINT = "point"
CARRY = "carry"

PENDING = "pending"
CARRYING = "carrying"
DONE = "done"


@dataclass
class Task:
    id: str
    type: str
    label: str
    radius: float
    position: tuple[float, float]
    dropoff: tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))


@dataclass
class TaskInstance:
    task: Task
    status: str = PENDING
    assigned_robot_ids: list[str] = field(default_factory=list)

    def reset(self) -> None:
        self.status = PENDING
        self.assigned_robot_ids.clear()
