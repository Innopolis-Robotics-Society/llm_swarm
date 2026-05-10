"""Read-only ROS topic snapshot provider for /llm/chat."""

from __future__ import annotations

from collections import deque
from typing import Any

from iros_llm_orchestrator.context.provider import (
    ChatContextConfig,
    ChatContextProvider,
    bound_context,
    known_robot_ids,
    map_summary,
    safe_str,
    utc_now,
)


FORMATION_STATES = {
    0: 'INACTIVE',
    1: 'FORMING',
    2: 'STABLE',
    3: 'DEGRADED',
    4: 'BROKEN',
}

FORMATION_FAILURES = {
    0: 'NONE',
    1: 'FOLLOWER_LOST',
    2: 'FOLLOWER_STUCK',
    3: 'LEADER_LOST',
}


class RosReadonlyContextProvider(ChatContextProvider):
    """Cache selected ROS topics and return a small normalized context."""

    def __init__(self, node: Any, config: ChatContextConfig):
        super().__init__(config)
        self._node = node
        self._latest_bt = None
        self._latest_formations = None
        self._recent_events = deque(maxlen=12)
        self._subscriptions = []
        self._warnings: list[str] = []
        self._create_subscriptions()

    def _create_subscriptions(self):
        try:
            from iros_llm_swarm_interfaces.msg import (
                BTState,
                FormationsStatus,
                LlmEvent,
            )
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        except Exception as exc:
            self._warnings.append(f'ROS context imports unavailable: {exc}')
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        if self.config.include_bt_state:
            self._subscriptions.append(
                self._node.create_subscription(
                    BTState, '/bt/state', self._on_bt_state, qos)
            )
        if self.config.include_recent_events:
            self._subscriptions.append(
                self._node.create_subscription(
                    LlmEvent, '/llm/events', self._on_event, reliable_qos)
            )
        if self.config.include_formations:
            self._subscriptions.append(
                self._node.create_subscription(
                    FormationsStatus,
                    '/formations/status',
                    self._on_formations,
                    qos,
                )
            )

    def _on_bt_state(self, msg):
        self._latest_bt = msg

    def _on_event(self, msg):
        self._recent_events.append(normalize_llm_event(msg))

    def _on_formations(self, msg):
        self._latest_formations = msg

    async def get_context(self) -> dict:
        context = {
            'timestamp': utc_now(),
            'source': 'ros_readonly',
            'warnings': list(self._warnings),
        }

        if self.config.include_bt_state:
            if self._latest_bt is None:
                context['bt_state'] = {'status': 'unknown'}
                context['warnings'].append('No /bt/state received yet')
            else:
                context['bt_state'] = normalize_bt_state(self._latest_bt)
                context['robots'] = derive_robot_summary(
                    self._latest_bt,
                    self.config.map_config,
                )

        if self.config.include_formations:
            if self._latest_formations is not None:
                context['formations'] = normalize_formations_status(
                    self._latest_formations)

        if self.config.include_map_summary:
            context['map'] = map_summary(
                self.config.map_name,
                self.config.map_config,
            )

        if self.config.include_recent_events:
            context['recent_events'] = list(self._recent_events)

        if not context['warnings']:
            context.pop('warnings', None)
        return bound_context(context, self.config.max_chars)


def normalize_bt_state(msg: Any) -> dict:
    goals = []
    for point in list(getattr(msg, 'goals', []) or [])[:40]:
        goals.append([
            round(float(getattr(point, 'x', 0.0)), 3),
            round(float(getattr(point, 'y', 0.0)), 3),
        ])
    return {
        'mode': safe_str(getattr(msg, 'mode', ''), 80),
        'action_status': safe_str(getattr(msg, 'action_status', ''), 80),
        'active_action': safe_str(getattr(msg, 'active_action', ''), 120),
        'action_summary': safe_str(getattr(msg, 'action_summary', ''), 240),
        'last_error': safe_str(getattr(msg, 'last_error', ''), 240),
        'robot_ids': [int(r) for r in list(getattr(msg, 'robot_ids', []) or [])],
        'goals': goals,
        'formation_id': safe_str(getattr(msg, 'formation_id', ''), 120),
        'leader_ns': safe_str(getattr(msg, 'leader_ns', ''), 120),
        'llm_thinking': bool(getattr(msg, 'llm_thinking', False)),
        'formation_state': formation_state_name(
            getattr(msg, 'formation_state', 0)),
        'formation_failure_code': formation_failure_name(
            getattr(msg, 'formation_failure_code', 0)),
        'formation_failure_reason': safe_str(
            getattr(msg, 'formation_failure_reason', ''), 240),
        'formation_max_error_m': float(
            getattr(msg, 'formation_max_error_m', -1.0)),
        'formation_mean_error_m': float(
            getattr(msg, 'formation_mean_error_m', -1.0)),
        'stamp_ms': int(getattr(msg, 'stamp_ms', 0)),
    }


def derive_robot_summary(msg: Any, map_config: dict) -> dict:
    known = set(known_robot_ids(map_config))
    active = {
        int(rid)
        for rid in list(getattr(msg, 'robot_ids', []) or [])
        if str(rid).strip()
    }
    mode = safe_str(getattr(msg, 'mode', ''), 80)
    if mode == 'idle' and known:
        return {
            'active': [],
            'idle': sorted(known),
            'unknown': [],
        }
    if known:
        return {
            'active': sorted(active),
            'idle': sorted(known - active),
            'unknown': [],
        }
    return {
        'active': sorted(active),
        'idle': [],
        'unknown': [],
    }


def normalize_llm_event(msg: Any) -> str:
    channel = int(getattr(msg, 'channel', 0))
    trigger = safe_str(getattr(msg, 'trigger', ''), 180)
    output = safe_str(getattr(msg, 'output', ''), 180)
    reason = safe_str(getattr(msg, 'reason', ''), 180)
    return f'channel={channel} trigger={trigger} output={output} reason={reason}'


def normalize_formations_status(msg: Any) -> list[dict]:
    formations = []
    for formation in list(getattr(msg, 'formations', []) or [])[:20]:
        formations.append({
            'formation_id': safe_str(getattr(formation, 'formation_id', ''), 120),
            'leader_ns': safe_str(getattr(formation, 'leader_ns', ''), 120),
            'followers': [
                safe_str(ns, 120)
                for ns in list(getattr(formation, 'follower_ns', []) or [])[:40]
            ],
            'status': formation_state_name(getattr(formation, 'state', 0)),
            'failure_code': formation_failure_name(
                getattr(formation, 'failure_code', 0)),
            'failure_reason': safe_str(
                getattr(formation, 'failure_reason', ''), 240),
            'max_error_m': float(getattr(formation, 'max_error_m', -1.0)),
            'mean_error_m': float(getattr(formation, 'mean_error_m', -1.0)),
        })
    return formations


def formation_state_name(value: Any) -> str:
    try:
        return FORMATION_STATES.get(int(value), f'UNKNOWN_{int(value)}')
    except (TypeError, ValueError):
        return 'UNKNOWN'


def formation_failure_name(value: Any) -> str:
    try:
        return FORMATION_FAILURES.get(int(value), f'UNKNOWN_{int(value)}')
    except (TypeError, ValueError):
        return 'UNKNOWN'
