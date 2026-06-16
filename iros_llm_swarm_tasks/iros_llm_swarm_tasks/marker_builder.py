from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

from .task_model import CARRY, DONE, PENDING, TaskInstance

# Slot offsets within each task's marker namespace
_SLOT_ZONE = 0
_SLOT_LABEL = 1
_SLOT_DROPOFF = 2
_SLOT_ARROW = 3
_SLOT_CARGO = 4

_COLOR_PENDING = ColorRGBA(r=0.0, g=0.8, b=0.8, a=0.5)   # cyan
_COLOR_DONE = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.3)       # grey
_COLOR_DROPOFF = ColorRGBA(r=0.0, g=1.0, b=0.4, a=0.5)    # green
_COLOR_CARGO = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)      # orange

_LIFETIME = Duration(sec=0, nanosec=0)  # never expire


def build_marker_array(instances: list[TaskInstance], frame_id: str = "map") -> MarkerArray:
    arr = MarkerArray()
    for inst in instances:
        arr.markers.extend(_markers_for(inst, frame_id))
    return arr


def delete_markers_for(task_id: str, frame_id: str = "map") -> list[Marker]:
    """Explicit DELETE markers for all slots of a removed task."""
    markers = []
    for slot in (_SLOT_ZONE, _SLOT_LABEL, _SLOT_DROPOFF, _SLOT_ARROW, _SLOT_CARGO):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = f"task_{task_id}"
        m.id = slot
        m.action = Marker.DELETE
        markers.append(m)
    return markers


def _base(inst: TaskInstance, slot: int, frame_id: str) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = f"task_{inst.task.id}"
    m.id = slot
    m.action = Marker.ADD
    m.lifetime = _LIFETIME
    m.frame_locked = False
    return m


def _markers_for(inst: TaskInstance, frame_id: str) -> list[Marker]:
    task = inst.task
    status = inst.status
    markers: list[Marker] = []

    color = _COLOR_DONE if status == DONE else _COLOR_PENDING
    x, y = task.position

    # Zone cylinder
    zone = _base(inst, _SLOT_ZONE, frame_id)
    zone.type = Marker.CYLINDER
    zone.pose.position = Point(x=x, y=y, z=0.05)
    zone.scale = Vector3(x=task.radius * 2.0, y=task.radius * 2.0, z=0.1)
    zone.color = color
    markers.append(zone)

    # Label
    label = _base(inst, _SLOT_LABEL, frame_id)
    label.type = Marker.TEXT_VIEW_FACING
    label.pose.position = Point(x=x, y=y, z=0.4)
    label.scale = Vector3(x=0.0, y=0.0, z=0.2)
    label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    label.text = task.label
    markers.append(label)

    if task.type == CARRY:
        dx, dy = task.dropoff

        # Dropoff cylinder
        drop = _base(inst, _SLOT_DROPOFF, frame_id)
        drop.type = Marker.CYLINDER
        drop.pose.position = Point(x=dx, y=dy, z=0.05)
        drop.scale = Vector3(x=task.radius * 2.0, y=task.radius * 2.0, z=0.1)
        drop.color = _COLOR_DROPOFF
        markers.append(drop)

        # Arrow pickup → dropoff
        arrow = _base(inst, _SLOT_ARROW, frame_id)
        arrow.type = Marker.ARROW
        arrow.points = [
            Point(x=x, y=y, z=0.1),
            Point(x=dx, y=dy, z=0.1),
        ]
        arrow.scale = Vector3(x=0.05, y=0.1, z=0.1)
        arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        markers.append(arrow)

    return markers


