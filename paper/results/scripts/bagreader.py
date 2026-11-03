"""Offline reader for the E3 session bags — no ROS, no container.

WHY THIS IS HAND-ROLLED
The recordings are scored on a laptop, months after the run, by whoever is
writing the paper. Requiring a sourced ROS 2 Humble workspace with
iros_llm_swarm_interfaces built just to count how many tasks finished makes
the data unreadable the moment the container image drifts -- and the container
is exactly the thing we already know does not survive (see the sessions/
placement rationale in .gitignore). So the handful of message types the
campaign actually scores are decoded here directly from CDR.

WHAT IS DECODED
Only what E3 scores, and nothing else:
  iros_llm_swarm_interfaces/msg/TaskStates                  <- ground truth
  iros_llm_swarm_interfaces/msg/BTState
  iros_llm_swarm_interfaces/msg/LlmEvent
  iros_llm_swarm_interfaces/msg/FormationsStatus          <- metrics 11-14
  iros_llm_swarm_interfaces/action/SetGoals_FeedbackMessage
  nav_msgs/msg/Odometry                                     <- position only
Anything else in the bag is skipped. If a future cell needs another type, add
a decoder rather than reaching for rclpy: the point is that this file has no
dependencies beyond the standard library.

CDR NOTES
rosbag2 stores each message as an XCDR1 encapsulation: a 4-byte header
(0x00 0x01 = little-endian CDR, then two bytes of options) followed by the
body. Alignment is counted from the start of the body, not the start of the
buffer, which is why _Cursor tracks `base`. Strings carry a uint32 length that
INCLUDES the NUL terminator. Sequences carry a uint32 count; fixed-size arrays
carry none.
"""

from __future__ import annotations

import sqlite3
import struct
from typing import Any, Callable, Iterator, NamedTuple

__all__ = [
    'Bag',
    'BagMessage',
    'DECODERS',
    'topic_counts',
]


# ─── CDR cursor ────────────────────────────────────────────────────────────

class _Cursor:
    """Little-endian CDR read head with the alignment rules applied."""

    __slots__ = ('buf', 'pos', 'base')

    def __init__(self, buf: bytes, base: int = 4) -> None:
        self.buf = buf
        self.base = base          # offset alignment is measured from
        self.pos = base

    def align(self, n: int) -> None:
        off = (self.pos - self.base) % n
        if off:
            self.pos += n - off

    def _prim(self, fmt: str, size: int) -> Any:
        self.align(size)
        v = struct.unpack_from(fmt, self.buf, self.pos)[0]
        self.pos += size
        return v

    def u8(self) -> int:
        v = self.buf[self.pos]
        self.pos += 1
        return v

    def i8(self) -> int:
        return self._prim('<b', 1)

    def boolean(self) -> bool:
        return self.u8() != 0

    def u16(self) -> int:
        return self._prim('<H', 2)

    def i16(self) -> int:
        return self._prim('<h', 2)

    def u32(self) -> int:
        return self._prim('<I', 4)

    def i32(self) -> int:
        return self._prim('<i', 4)

    def u64(self) -> int:
        return self._prim('<Q', 8)

    def i64(self) -> int:
        return self._prim('<q', 8)

    def f32(self) -> float:
        return self._prim('<f', 4)

    def f64(self) -> float:
        return self._prim('<d', 8)

    def string(self) -> str:
        n = self.u32()
        if n == 0:
            return ''
        raw = self.buf[self.pos:self.pos + n - 1]     # drop the NUL
        self.pos += n
        return raw.decode('utf-8', 'replace')

    def seq(self, item: Callable[[], Any]) -> list:
        return [item() for _ in range(self.u32())]

    def array(self, item: Callable[[], Any], n: int) -> list:
        return [item() for _ in range(n)]

    def skip_f64_array(self, n: int) -> None:
        self.align(8)
        self.pos += 8 * n


# ─── shared sub-messages ───────────────────────────────────────────────────

def _time(c: _Cursor) -> float:
    """builtin_interfaces/Time as float seconds."""
    sec = c.i32()
    nsec = c.u32()
    return sec + nsec * 1e-9


def _header(c: _Cursor) -> dict:
    return {'stamp': _time(c), 'frame_id': c.string()}


def _point(c: _Cursor) -> tuple[float, float, float]:
    return (c.f64(), c.f64(), c.f64())


# ─── message decoders ──────────────────────────────────────────────────────

def decode_task_states(buf: bytes) -> dict:
    """iros_llm_swarm_interfaces/msg/TaskStates"""
    c = _Cursor(buf)
    hdr = _header(c)

    def one_state() -> dict:
        task = {
            'id': c.string(),
            'type': c.string(),
            'label': c.string(),
            'radius': c.f64(),
            'position': (c.f64(), c.f64()),      # float64[2], fixed
            'dropoff': (c.f64(), c.f64()),       # float64[2], fixed
        }
        return {
            'task': task,
            'status': c.string(),
            'assigned_robot_ids': c.seq(c.string),
        }

    return {'header': hdr, 'states': c.seq(one_state)}


def decode_bt_state(buf: bytes) -> dict:
    """iros_llm_swarm_interfaces/msg/BTState"""
    c = _Cursor(buf)
    return {
        'mode': c.string(),
        'action_status': c.string(),
        'active_action': c.string(),
        'action_summary': c.string(),
        'last_error': c.string(),
        'robot_ids': c.seq(c.u32),
        'goals': c.seq(lambda: _point(c)),
        'formation_id': c.string(),
        'leader_ns': c.string(),
        'llm_thinking': c.boolean(),
        'formation_state': c.u8(),
        'formation_failure_code': c.u8(),
        'formation_failure_reason': c.string(),
        'formation_max_error_m': c.f32(),
        'formation_mean_error_m': c.f32(),
        'stamp_ms': c.i64(),
    }


def decode_llm_event(buf: bytes) -> dict:
    """iros_llm_swarm_interfaces/msg/LlmEvent"""
    c = _Cursor(buf)
    return {
        'stamp_ms': c.i64(),
        'channel': c.u8(),
        'trigger': c.string(),
        'output': c.string(),
        'reason': c.string(),
    }


def decode_set_goals_feedback(buf: bytes) -> dict:
    """iros_llm_swarm_interfaces/action/SetGoals_FeedbackMessage

    The action wrapper prefixes the feedback struct with a 16-byte goal UUID.
    """
    c = _Cursor(buf)
    uuid = bytes(c.array(c.u8, 16))
    return {
        'goal_id': uuid.hex(),
        'status': c.string(),
        'elapsed_ms': c.u32(),
        'robots_arrived': c.u32(),
        'robots_active': c.u32(),
        'robots_deviated': c.u32(),
        'replans_done': c.u32(),
        'robot_stall': c.u32(),
        'info': c.string(),
        'warning': c.string(),
    }


def decode_goal_status_array(buf: bytes) -> dict:
    """action_msgs/msg/GoalStatusArray -- terminal status of the MAPF action."""
    c = _Cursor(buf)

    def one() -> dict:
        uuid = bytes(c.array(c.u8, 16))          # GoalInfo.goal_id
        stamp = _time(c)                         # GoalInfo.stamp
        return {'goal_id': uuid.hex(), 'stamp': stamp, 'status': c.i8()}

    return {'status_list': c.seq(one)}


def decode_odometry(buf: bytes) -> dict:
    """nav_msgs/msg/Odometry -- header, frame and position only.

    Orientation, covariance and twist are skipped: E3 scores where a robot
    ended up and how far it drove, nothing that needs the rest.
    """
    c = _Cursor(buf)
    hdr = _header(c)
    child = c.string()
    x, y, z = _point(c)
    return {'header': hdr, 'child_frame_id': child, 'position': (x, y, z)}


_FORMATION_STATE = {
    0: 'INACTIVE', 1: 'FORMING', 2: 'STABLE', 3: 'DEGRADED', 4: 'BROKEN',
}
_FORMATION_FAILURE = {
    0: 'NONE', 1: 'FOLLOWER_LOST', 2: 'FOLLOWER_STUCK', 3: 'LEADER_LOST',
}


def decode_formations_status(buf: bytes) -> dict:
    """iros_llm_swarm_interfaces/msg/FormationsStatus

    Metrics 11-14 of section 3.1 come from here and nowhere else: the follower
    errors are published at 10 Hz for the whole traverse, so eps_ss, eps_peak,
    t_stable and f_degraded are all read off this stream.

    -1.0 in an error field means "not available for this follower" and is not a
    measurement -- callers must drop it rather than average it in, which is why
    the raw value is passed through unchanged here.
    """
    c = _Cursor(buf)
    hdr = _header(c)

    def one() -> dict:
        return {
            'header':         _header(c),
            'formation_id':   c.string(),
            'leader_ns':      c.string(),
            'follower_ns':    c.seq(c.string),
            'state':          c.u8(),
            'failure_code':   c.u8(),
            'failure_reason': c.string(),
            'errors_m':       c.seq(c.f32),
            'max_error_m':    c.f32(),
            'mean_error_m':   c.f32(),
        }

    return {'header': hdr, 'formations': c.seq(one)}


DECODERS: dict[str, Callable[[bytes], Any]] = {
    'iros_llm_swarm_interfaces/msg/TaskStates': decode_task_states,
    'iros_llm_swarm_interfaces/msg/BTState': decode_bt_state,
    'iros_llm_swarm_interfaces/msg/LlmEvent': decode_llm_event,
    'iros_llm_swarm_interfaces/action/SetGoals_FeedbackMessage':
        decode_set_goals_feedback,
    'action_msgs/msg/GoalStatusArray': decode_goal_status_array,
    'nav_msgs/msg/Odometry': decode_odometry,
    'iros_llm_swarm_interfaces/msg/FormationsStatus':
        decode_formations_status,
}


# ─── bag access ────────────────────────────────────────────────────────────

class BagMessage(NamedTuple):
    topic: str
    t: float           # wall-clock seconds (rosbag2 receive timestamp)
    msg: Any


class Bag:
    """Read-only access to a single-file rosbag2 sqlite3 recording."""

    def __init__(self, db3_path: str) -> None:
        self.path = db3_path
        # immutable=1, not mode=ro. A read-only connection still takes a
        # shared lock, and that is enough to kill a rosbag2 recorder writing
        # the same file: it dies with SqliteException "database is locked" and
        # leaves the bag without its metadata.yaml. That happened to run
        # 20260818_110246, which lost the last six minutes of a 9/9 mission
        # because the analyser was reading the set while it recorded.
        # immutable=1 promises the file will not change and skips locking
        # entirely -- so callers must not point it at a live recording, which
        # is what analyze_sessions._is_live() is for.
        self._con = sqlite3.connect(f'file:{db3_path}?immutable=1', uri=True)
        self.topics = {
            name: (tid, typ)
            for tid, name, typ in self._con.execute(
                'SELECT id, name, type FROM topics')
        }

    def close(self) -> None:
        self._con.close()

    def __enter__(self) -> 'Bag':
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def type_of(self, topic: str) -> str | None:
        e = self.topics.get(topic)
        return e[1] if e else None

    def read(self, topic: str) -> Iterator[BagMessage]:
        """Yield decoded messages for one topic, in recorded order.

        Raises KeyError for an unknown topic (a silent empty iterator here
        would score a missing topic as a clean run) and NotImplementedError
        for a type with no decoder.
        """
        if topic not in self.topics:
            raise KeyError(f'{topic} not in {self.path}')
        tid, typ = self.topics[topic]
        dec = DECODERS.get(typ)
        if dec is None:
            raise NotImplementedError(f'no decoder for {typ} ({topic})')
        cur = self._con.execute(
            'SELECT timestamp, data FROM messages WHERE topic_id=? '
            'ORDER BY timestamp', (tid,))
        for ts, blob in cur:
            yield BagMessage(topic, ts * 1e-9, dec(bytes(blob)))

    def last(self, topic: str) -> BagMessage | None:
        out = None
        for m in self.read(topic):
            out = m
        return out

    def span(self) -> tuple[float, float]:
        lo, hi = self._con.execute(
            'SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
        return ((lo or 0) * 1e-9, (hi or 0) * 1e-9)


def topic_counts(db3_path: str) -> dict[str, tuple[str, int]]:
    """{topic: (type, message_count)} without decoding anything."""
    con = sqlite3.connect(f'file:{db3_path}?mode=ro', uri=True)
    try:
        return {
            name: (typ, n)
            for name, typ, n in con.execute(
                'SELECT t.name, t.type, COUNT(m.id) FROM topics t '
                'LEFT JOIN messages m ON m.topic_id = t.id '
                'GROUP BY t.id ORDER BY t.name')
        }
    finally:
        con.close()
