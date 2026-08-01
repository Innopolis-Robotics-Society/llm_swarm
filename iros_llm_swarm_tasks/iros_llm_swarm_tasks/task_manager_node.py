import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import tf2_ros
from visualization_msgs.msg import MarkerArray

from iros_llm_swarm_interfaces.msg import (
    Task as TaskMsg,
    TaskState as TaskStateMsg,
    TaskStates as TaskStatesMsg,
)
from iros_llm_swarm_interfaces.srv import (
    AddTask,
    ListTasks,
    RemoveTask,
    ResetAll,
    ResetTask,
)

from .marker_builder import build_marker_array, delete_markers_for
from .task_loader import load_tasks
from .task_model import CARRY, CARRYING, DONE, PENDING, Task, TaskInstance

_TRANSIENT_LOCAL = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

_MAP_FRAME = "map"
_POLL_HZ = 5.0


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("task_manager")

        self.declare_parameter("scenario_yaml", "")
        self.declare_parameter("scenario", "")
        self.declare_parameter("num_robots", 20)

        scenario_yaml: str = self.get_parameter("scenario_yaml").get_parameter_value().string_value
        scenario: str = self.get_parameter("scenario").get_parameter_value().string_value
        self._num_robots: int = self.get_parameter("num_robots").get_parameter_value().integer_value

        self._instances: dict[str, TaskInstance] = {}

        if scenario_yaml and scenario:
            for inst in load_tasks(scenario_yaml, scenario):
                self._instances[inst.task.id] = inst
            self.get_logger().info(
                f"Loaded {len(self._instances)} tasks for scenario '{scenario}'"
            )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, "/tasks/markers", _TRANSIENT_LOCAL)
        # Mission ground truth. TRANSIENT_LOCAL so a late subscriber (RViz, a
        # recorder started after bring-up) immediately gets the current
        # snapshot instead of waiting for the next tick.
        self._state_pub = self.create_publisher(TaskStatesMsg, "/tasks/state", _TRANSIENT_LOCAL)

        self.create_service(AddTask, "/tasks/add", self._on_add)
        self.create_service(RemoveTask, "/tasks/remove", self._on_remove)
        self.create_service(ListTasks, "/tasks/list", self._on_list)
        self.create_service(ResetTask, "/tasks/reset", self._on_reset)
        self.create_service(ResetAll, "/tasks/reset_all", self._on_reset_all)

        self._poll_timer = self.create_timer(1.0 / _POLL_HZ, self._poll)
        self._publish_markers()

    # ------------------------------------------------------------------ #
    # Services
    # ------------------------------------------------------------------ #

    def _on_add(self, req: AddTask.Request, res: AddTask.Response) -> AddTask.Response:
        msg = req.task
        if msg.id in self._instances:
            res.success = False
            res.message = f"Task {msg.id!r} already exists"
            return res

        task = Task(
            id=msg.id,
            type=msg.type,
            label=msg.label,
            radius=msg.radius,
            position=(msg.position[0], msg.position[1]),
            dropoff=(msg.dropoff[0], msg.dropoff[1]),
        )
        self._instances[msg.id] = TaskInstance(task=task)
        self._publish_markers()
        res.success = True
        res.message = ""
        return res

    def _on_remove(self, req: RemoveTask.Request, res: RemoveTask.Response) -> RemoveTask.Response:
        if req.id not in self._instances:
            res.success = False
            res.message = f"Task {req.id!r} not found"
            return res
        del self._instances[req.id]
        arr = MarkerArray()
        arr.markers.extend(delete_markers_for(req.id, _MAP_FRAME))
        arr.markers.extend(build_marker_array(list(self._instances.values()), _MAP_FRAME).markers)
        self._marker_pub.publish(arr)
        res.success = True
        res.message = ""
        return res

    def _on_list(self, _req: ListTasks.Request, res: ListTasks.Response) -> ListTasks.Response:
        res.states = [_to_state_msg(inst) for inst in self._instances.values()]
        return res

    def _on_reset(self, req: ResetTask.Request, res: ResetTask.Response) -> ResetTask.Response:
        if req.id not in self._instances:
            res.success = False
            res.message = f"Task {req.id!r} not found"
            return res
        self._instances[req.id].reset()
        self._publish_markers()
        res.success = True
        res.message = ""
        return res

    def _on_reset_all(self, _req: ResetAll.Request, res: ResetAll.Response) -> ResetAll.Response:
        for inst in self._instances.values():
            inst.reset()
        self._publish_markers()
        return res

    # ------------------------------------------------------------------ #
    # TF poll
    # ------------------------------------------------------------------ #

    def _publish_states(self) -> None:
        """Publish the full task snapshot on /tasks/state.

        Republished every tick rather than on change: a recording (or a panel)
        started mid-run must still see the complete state, and mission scoring
        reads this topic offline from a bag. Cheap at nine tasks and 5 Hz.
        """
        msg = TaskStatesMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = _MAP_FRAME
        msg.states = [_to_state_msg(inst) for inst in self._instances.values()]
        self._state_pub.publish(msg)

    def _poll(self) -> None:
        # Publish before the early return: an empty task set is a legitimate
        # state, and a recording that sees no messages at all is
        # indistinguishable from a dead node.
        self._publish_states()
        if not self._instances:
            return

        robot_positions: list[Optional[tuple[float, float]]] = []
        now = rclpy.time.Time()

        for i in range(self._num_robots):
            try:
                tf = self._tf_buffer.lookup_transform(
                    _MAP_FRAME,
                    f"robot_{i}/base_link",
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
                t = tf.transform.translation
                robot_positions.append((t.x, t.y))
            except Exception:  # noqa: BLE001
                robot_positions.append(None)

        changed = False
        for inst in self._instances.values():
            if inst.status == DONE:
                continue
            if self._update_instance(inst, robot_positions):
                changed = True

        if changed:
            self._publish_markers()

    def _update_instance(
        self,
        inst: TaskInstance,
        robot_positions: list[Optional[tuple[float, float]]],
    ) -> bool:
        task = inst.task
        changed = False

        for i, pos in enumerate(robot_positions):
            if pos is None:
                continue
            robot_id = f"robot_{i}"

            if task.type == "point":
                if inst.status == PENDING and _within(pos, task.position, task.radius):
                    inst.status = DONE
                    if robot_id not in inst.assigned_robot_ids:
                        inst.assigned_robot_ids.append(robot_id)
                    changed = True
                    break

            elif task.type == CARRY:
                if inst.status == PENDING and _within(pos, task.position, task.radius):
                    inst.status = CARRYING
                    inst.assigned_robot_ids = [robot_id]
                    changed = True

                elif inst.status == CARRYING:
                    carrier = inst.assigned_robot_ids[0] if inst.assigned_robot_ids else None
                    if robot_id == carrier and _within(pos, task.dropoff, task.radius):
                        inst.status = DONE
                        changed = True
                        break

        return changed

    # ------------------------------------------------------------------ #
    # Markers
    # ------------------------------------------------------------------ #

    def _publish_markers(self) -> None:
        arr = build_marker_array(list(self._instances.values()), _MAP_FRAME)
        self._marker_pub.publish(arr)


def _within(a: tuple[float, float], b: tuple[float, float], radius: float) -> bool:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy) <= radius


def _to_state_msg(inst: TaskInstance) -> TaskStateMsg:
    msg = TaskStateMsg()
    msg.task.id = inst.task.id
    msg.task.type = inst.task.type
    msg.task.label = inst.task.label
    msg.task.radius = inst.task.radius
    msg.task.position = list(inst.task.position)
    msg.task.dropoff = list(inst.task.dropoff)
    msg.status = inst.status
    msg.assigned_robot_ids = list(inst.assigned_robot_ids)
    return msg


def main() -> None:
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
