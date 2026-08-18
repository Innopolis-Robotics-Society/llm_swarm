#!/usr/bin/env python3
"""E3 no-LLM cell: drive a mission with a known-correct plan.

WHY THIS EXISTS
The no-LLM cell measures the ceiling of the substrate. Everything downstream of
the plan -- MAPF, Nav2, the followers, the task manager -- is identical to the
LLM cells; only the planner is replaced by a script that cannot get the plan
wrong. Whatever gap remains between this cell and cell B is the cost of letting
a language model choose the goals.

That only works if the script is scored by the same instrument. Goals are read
from /tasks/list at run time rather than written into this file, so the driver
and the task manager cannot disagree about where a task is: the number the
manager checks against is the number the robots are sent to.

WHY NOT demo_20_robot.py
The existing demo is hard-coded for a 30x30 m warehouse (goals like (15,15),
(26,22)) while E3 runs on `amongus`, whose origin is in the middle of the map.
More importantly it never touches the task system, so it completes no tasks and
there is nothing for metrics 1, 2 and 7 to score. It is a liveness demo, not a
baseline.

THE CARRY TASK IS THE WHOLE DIFFICULTY
A carry task is not two point tasks. task_manager_node latches ONE carrier --
the first robot to come within radius of the pickup -- into
`assigned_robot_ids`, and only that robot arriving at the dropoff completes it.
A plan that sends a robot straight to the dropoff leaves the task `pending`,
which on paper is indistinguishable from a robot that never moved. So this
driver dispatches the pickup leg, waits for the manager to publish CARRYING,
and only then dispatches the dropoff leg. That ordering is the thing the
scripted arm is supposed to get right by construction.

USAGE (inside the container, after the stack is up and Nav2 is active)
    ros2 run iros_llm_swarm_bt e3_scripted_driver            # mission M2
    ros2 run iros_llm_swarm_bt e3_scripted_driver --mission m1
    ros2 run iros_llm_swarm_bt e3_scripted_driver --mission m3 --timeout 900
    ros2 run iros_llm_swarm_bt e3_scripted_driver --dry-run

--dry-run prints the legs it would dispatch and drives nothing, but it still
needs the task manager up: the whole point is that goals come from
/tasks/list rather than from constants in this file, so there is nothing to
print without it. It is the cheapest way to confirm the scenario really
declares the tasks a mission names before committing a stack to a run.

Ctrl+C cancels the in-flight goal and stops the fleet, matching
test_send_goals.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from iros_llm_swarm_interfaces.action import SetGoals
from iros_llm_swarm_interfaces.msg import TaskState, TaskStates
from iros_llm_swarm_interfaces.srv import ListTasks

# task_manager_node publishes the whole snapshot, not one task per message, and
# it does so RELIABLE/TRANSIENT_LOCAL depth 1. Subscribing with default QoS
# silently receives nothing: the durability kinds are incompatible, so rclpy
# never matches the endpoints and the callback simply never fires. That failure
# looks exactly like "the carry never registered", which is the one thing this
# driver has to get right.
_TASKS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)

# Robot groups, mirroring the colour names the operator uses in the LLM arms.
# Kept here rather than read from the scenario because the LLM arms resolve them
# from the prompt's grounding block, and the point of this cell is that the
# mapping is not in question. Two robots per team, the same ten the mission text implies. Kept as named
# constants so a pair reads as "who", not as two magic numbers.
CYAN    = [0, 1]
GREEN   = [8, 9]
MAGENTA = [4, 5]
ORANGE  = [12, 13]
YELLOW  = [16, 17]

# Missions, matching the operator text in E3_spec.md §5 leg for leg.
# A leg is (robot_ids, task_id); carry tasks expand into two waves internally.
#
# EVERY PAIR IS DRAWN FROM TWO DIFFERENT TEAMS, exactly as the text says. That
# is not cosmetic: this cell is the baseline every LLM cell is compared against,
# so it has to attempt the same mission. If it drove a different composition the
# gap would be read as "the model allocated badly" and nothing in the recording
# would contradict that -- session.json records `mission: M2` either way.
#
# The allocation below is also what the model itself produced from this text in
# a live run ([4,12], [0,13], [16,8], [17,9], [1,5]), so the scripted arm is not
# being handed an easier reading of the same words.
MISSIONS: dict[str, list[tuple[list[int], str]]] = {
    'm1': [
        ([MAGENTA[0], ORANGE[0]], 'task_electrical'),
        ([ORANGE[1], YELLOW[0]],  'task_comms'),
    ],
    'm2': [
        ([MAGENTA[0], ORANGE[0]], 'task_electrical'),
        ([CYAN[0], ORANGE[1]],    'task_med'),
        ([YELLOW[0], GREEN[0]],   'task_hall'),
        ([YELLOW[1], GREEN[1]],   'task_comms'),
        ([CYAN[1], MAGENTA[1]],   'carry_security_to_admin'),
    ],
    # M3 is "complete all declared tasks". Full completion is not expected even
    # from a correct plan -- two carries end in the reactor, a dead-end spur
    # that blocks once occupied -- so this exists to give the substrate ceiling
    # on the hard mission, not to succeed. task_o2 appears here and nowhere
    # else: M3 names every declared task by definition, while M2 and M1 avoid
    # it because its zone sits 1.85 m from the o2 room against a 1.5 m radius
    # (E3_spec.md §5, rule 2).
    'm3': [
        ([MAGENTA[0], ORANGE[0]], 'task_electrical'),
        ([CYAN[0], ORANGE[1]],    'task_med'),
        ([YELLOW[0], GREEN[0]],   'task_hall'),
        ([YELLOW[1], GREEN[1]],   'task_comms'),
        ([2, 18],                 'task_o2'),
        ([3, 19],                 'task_reactor'),
        ([CYAN[1], MAGENTA[1]],   'carry_security_to_admin'),
        ([6, 10],                 'carry_engine_to_reactor'),
        ([7, 11],                 'carry_shields_to_reactor'),
    ],
}


# Metres each robot is displaced from a task point it shares with others.
#
# WHY ROBOTS CANNOT SHARE A POINT
# Sending two robots to the identical coordinate asks MAPF for a plan where
# two footprints occupy one cell. The solver will not produce it, so they
# fight over the last metre and at least one never registers inside the
# radius. The first five recorded no-LLM runs
# (paper/results/sessions/20260815_14*) needed this patched in by hand before
# they would score.
#
# WHY THE GROUPING IS BY POINT AND NOT BY ASSIGNMENT
# Spreading within each mission leg is not enough, because two legs can name
# the SAME coordinate. In M3 both carry_engine_to_reactor and
# carry_shields_to_reactor drop at (-29.65, -2.85): four robots, two legs, one
# destination. Offsetting per leg gives each leg a tidy pair and still stacks
# the two pairs on top of each other. The displacement therefore has to be
# computed over every robot arriving at a point, across the whole dispatch.
#
# 1.0 m is what the recorded runs used and is kept so they stay reproducible.
# It sits inside every declared task radius (1.5 m in `amongus`), so every
# robot still scores the task, and it is wider than the 0.22 m footprint plus
# inflation, so the planner can seat them side by side.
_SPREAD_M = 1.0


def _spread_offsets(n: int) -> list[tuple[float, float]]:
    """Displacements for `n` robots converging on one task point.

    One robot takes the point itself. Two go left and right, which is exactly
    what the recorded no-LLM runs did, so those stay reproducible. Three and
    four fill in below and above, giving the diamond the LLM arms produce
    unprompted for whole-team assignments.

    Beyond four the pattern falls back to an even circle. Note that at n > 6
    the arc between neighbours drops under the footprint diameter, so a
    scenario that puts that many robots on one point needs a larger radius,
    not just this function -- no mission in E3 does.
    """
    if n <= 1:
        return [(0.0, 0.0)]
    ring = [(-_SPREAD_M, 0.0), (_SPREAD_M, 0.0),
            (0.0, -_SPREAD_M), (0.0, _SPREAD_M)]
    if n <= len(ring):
        return ring[:n]
    return [(_SPREAD_M * math.cos(2.0 * math.pi * i / n),
             _SPREAD_M * math.sin(2.0 * math.pi * i / n)) for i in range(n)]


class ScriptedDriver(Node):
    def __init__(self, mission: str, timeout: float, dry_run: bool):
        super().__init__('e3_scripted_driver')
        self._mission = mission
        self._timeout = timeout
        self._dry_run = dry_run
        self._cb = ReentrantCallbackGroup()

        self._tasks: dict[str, TaskState] = {}
        self.create_subscription(
            TaskStates, '/tasks/state', self._on_states, _TASKS_QOS,
            callback_group=self._cb)
        self._list = self.create_client(
            ListTasks, '/tasks/list', callback_group=self._cb)
        self._goals = ActionClient(
            self, SetGoals, '/swarm/set_goals', callback_group=self._cb)
        self._active_handle = None

    # ---------------------------------------------------------------- tasks
    def _on_states(self, msg: TaskStates) -> None:
        for st in msg.states:
            self._tasks[st.task.id] = st

    def fetch_tasks(self) -> dict[str, TaskState]:
        """Snapshot /tasks/list. This is the authoritative goal source."""
        if not self._list.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(
                '/tasks/list is not up. The task manager is part of the stack '
                'under test: without it nothing scores, so this is a failed '
                'run rather than something to work around.')
        fut = self._list.call_async(ListTasks.Request())
        deadline = time.time() + 15.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            raise RuntimeError('/tasks/list did not answer within 15 s')
        found = {s.task.id: s for s in fut.result().states}
        for s in found.values():
            self._tasks.setdefault(s.task.id, s)
        return found

    def status_of(self, task_id: str) -> str:
        st = self._tasks.get(task_id)
        return st.status if st else 'unknown'

    def carrier_of(self, task_id: str) -> str | None:
        st = self._tasks.get(task_id)
        if st and st.assigned_robot_ids:
            return st.assigned_robot_ids[0]
        return None

    def await_status(self, task_id: str, want: str, timeout: float) -> bool:
        """Block until a task reaches `want`, or give up.

        /swarm/set_goals resolves the moment the robots are at their goals, but
        the task manager only learns that on its next TF poll at 5 Hz, and the
        pickup radius is checked against a transform that may lag further.
        Reading the status immediately after the action returns therefore races
        the manager and loses often enough to matter: the carry would look
        PENDING, the dropoff leg would be skipped, and the run would score a
        planning failure that never happened.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.status_of(task_id) == want:
                return True
            time.sleep(0.2)
        return False

    # ---------------------------------------------------------------- goals
    def drive(self, assignments: list[tuple[list[int], tuple[float, float]]],
              label: str) -> bool:
        """Send one /swarm/set_goals goal and block until it resolves.

        One action call carries every robot in this wave, because the server
        plans the whole fleet jointly -- issuing five separate calls would ask
        five independent planners to share corridors.
        """
        # Group by destination first: two legs of one wave can name the same
        # coordinate (both M3 carries drop at the reactor), and only a
        # by-point view sees that.
        by_point: dict[tuple[float, float], list[int]] = {}
        for robots, (x, y) in assignments:
            key = (round(float(x), 3), round(float(y), 3))
            by_point.setdefault(key, []).extend(robots)

        ids: list[int] = []
        pts: list[Point] = []
        for (x, y), robots in by_point.items():
            for r, (dx, dy) in zip(robots, _spread_offsets(len(robots))):
                ids.append(r)
                pts.append(Point(x=x + dx, y=y + dy, z=0.0))

        pretty = ', '.join(
            'robot_%d->(%.2f, %.2f)' % (r, p.x, p.y) for r, p in zip(ids, pts))
        self.get_logger().info('[%s] %s' % (label, pretty))
        if self._dry_run:
            return True

        if not self._goals.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('/swarm/set_goals action server absent')
            return False

        goal = SetGoals.Goal(robot_ids=ids, goals=pts)
        send = self._goals.send_goal_async(goal, feedback_callback=self._fb)
        deadline = time.time() + self._timeout
        while not send.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send.done():
            self.get_logger().error('[%s] goal not accepted in time' % label)
            return False
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error('[%s] goal rejected by server' % label)
            return False
        self._active_handle = handle

        res = handle.get_result_async()
        while not res.done() and time.time() < deadline:
            time.sleep(0.1)
        self._active_handle = None
        if not res.done():
            self.get_logger().error(
                '[%s] timed out after %.0f s — cancelling' % (label, self._timeout))
            handle.cancel_goal_async()
            return False

        r = res.result().result
        self.get_logger().info(
            '[%s] success=%s replans=%d exec=%.1fs — %s'
            % (label, r.success, r.total_replans, r.total_execution_sec,
               r.message))
        return bool(r.success)

    def _fb(self, msg) -> None:
        f = msg.feedback
        if f.warning:
            self.get_logger().warn(f.warning)

    def cancel(self) -> None:
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()

    # -------------------------------------------------------------- mission
    def run(self) -> int:
        legs = MISSIONS[self._mission]
        found = self.fetch_tasks()
        missing = [tid for _, tid in legs if tid not in found]
        if missing:
            self.get_logger().error(
                'tasks absent from /tasks/list: %s — wrong scenario loaded?'
                % ', '.join(missing))
            return 2

        # Wave 1: every point task, plus the pickup leg of every carry. All of
        # it goes in one call so the planner deconflicts the whole fleet at
        # once, the same way it does for an LLM plan.
        wave1: list[tuple[list[int], tuple[float, float]]] = []
        carries: list[tuple[list[int], str]] = []
        for robots, tid in legs:
            t = found[tid].task
            wave1.append((robots, (t.position[0], t.position[1])))
            if t.type == 'carry':
                carries.append((robots, tid))

        ok = self.drive(wave1, 'wave 1: point tasks + carry pickups')
        if self._dry_run:
            for robots, tid in carries:
                t = found[tid].task
                self.drive([(robots, (t.dropoff[0], t.dropoff[1]))],
                           'wave 2 (dry): %s dropoff' % tid)
            return 0
        if not ok:
            self.get_logger().warn(
                'wave 1 did not fully succeed; continuing to the dropoff legs '
                'anyway, because a partial wave still leaves real state for '
                'the task manager to score')

        # Wave 2: dropoffs, and only for carries the manager actually latched.
        # Dispatching a dropoff for a task still PENDING would drive a robot to
        # a place that cannot complete anything and would corrupt the odometry
        # evidence used to tell "went in the wrong order" from "never went".
        for robots, tid in carries:
            if not self.await_status(tid, 'carrying', timeout=15.0):
                self.get_logger().error(
                    '%s is %s after 15 s, not carrying — the pickup never '
                    'registered, so the dropoff leg is skipped rather than '
                    'faked' % (tid, self.status_of(tid)))
                continue
            carrier = self.carrier_of(tid)
            t = found[tid].task
            # Send the whole pair. Only the latched carrier can complete the
            # task, but keeping its partner with it matches what the LLM arms
            # are asked to do and keeps corridor load comparable.
            self.get_logger().info('%s carried by %s' % (tid, carrier))
            self.drive([(robots, (t.dropoff[0], t.dropoff[1]))],
                       'wave 2: %s dropoff' % tid)

        self.report(found)
        return 0

    def report(self, found: dict[str, TaskState]) -> None:
        done = [t for t in found if self.status_of(t) == 'done']
        carrying = [t for t in found if self.status_of(t) == 'carrying']
        pending = [t for t in found if self.status_of(t) == 'pending']
        self.get_logger().info('─' * 58)
        self.get_logger().info(
            'mission %s: %d/%d tasks done, %d mid-carry, %d pending'
            % (self._mission, len(done), len(found), len(carrying),
               len(pending)))
        for tid in sorted(carrying):
            self.get_logger().info(
                '  half-credit (picked up, not delivered): %s' % tid)
        for tid in sorted(pending):
            self.get_logger().info('  not completed: %s' % tid)
        self.get_logger().info(
            'These counts are this driver\'s view. The bag is the record of '
            'truth; /tasks/state carries the same states at 5 Hz.')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mission', default='m2', choices=sorted(MISSIONS),
                    help='which E3 mission to drive (default m2)')
    ap.add_argument('--timeout', type=float, default=900.0,
                    help='seconds per wave, matching '
                         'llm_mission_max_duration_sec (default 900)')
    ap.add_argument('--dry-run', action='store_true',
                    help='resolve tasks and print the legs without driving')
    args = ap.parse_args()

    rclpy.init()
    node = ScriptedDriver(args.mission, args.timeout, args.dry_run)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted — cancelling the active goal')
        node.cancel()
        time.sleep(1.0)
        rc = 130
    except Exception as exc:                     # noqa: BLE001 — top level
        node.get_logger().error('driver failed: %s' % exc)
        rc = 1
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
