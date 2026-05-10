"""Factory and shared helpers for chat context providers."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any


DEFAULT_MCP_READ_TOOLS = (
    'get_topics',
    'get_topic_type',
    'get_topic_details',
    'subscribe_once',
    'get_nodes',
    'get_node_details',
    'get_services',
    'get_service_type',
    'get_service_details',
    'get_actions',
    'get_action_details',
    'get_action_status',
)

BLOCKED_MCP_TOOLS = {
    'publish_once',
    'publish_for_durations',
    'call_service',
    'set_parameter',
    'delete_parameter',
    'send_action_goal',
    'cancel_action_goal',
    'connect_to_robot',
    'disconnect_from_robot',
    'set_websocket_ip',
    'set_websocket_uri',
    'ping_robot',
    'ping_robots',
}


@dataclass
class ChatContextConfig:
    provider: str = 'none'
    timeout_sec: float = 2.0
    max_chars: int = 6000
    include_bt_state: bool = True
    include_formations: bool = True
    include_map_summary: bool = True
    include_recent_events: bool = True
    map_name: str = 'cave'
    map_config: dict[str, Any] = field(default_factory=dict)
    mcp_enabled: bool = False
    mcp_transport: str = 'stdio'
    mcp_command: str = 'uvx'
    mcp_args: list[str] = field(
        default_factory=lambda: ['ros-mcp', '--transport=stdio'])
    mcp_tool_allowlist: list[str] = field(
        default_factory=lambda: list(DEFAULT_MCP_READ_TOOLS))


class ChatContextProvider:
    """Small async interface used by /llm/chat before prompt construction."""

    def __init__(self, config: ChatContextConfig):
        self.config = config

    async def get_context(self) -> dict:
        raise NotImplementedError


class NoneContextProvider(ChatContextProvider):
    async def get_context(self) -> dict:
        return bound_context(
            {
                'timestamp': utc_now(),
                'source': 'none',
            },
            self.config.max_chars,
        )


def make_context_provider(
    node: Any,
    config: ChatContextConfig,
) -> ChatContextProvider:
    provider = (config.provider or 'none').strip().lower()
    if provider == 'ros_readonly':
        from iros_llm_orchestrator.context.ros_readonly_provider import (
            RosReadonlyContextProvider,
        )
        return RosReadonlyContextProvider(node, config)
    if provider == 'mcp_readonly':
        from iros_llm_orchestrator.context.mcp_readonly_provider import (
            McpReadonlyContextProvider,
        )
        logger = node.get_logger() if hasattr(node, 'get_logger') else None
        return McpReadonlyContextProvider(config, logger=logger)
    return NoneContextProvider(config)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def safe_str(value: Any, limit: int = 500) -> str:
    text = '' if value is None else str(value)
    text = ''.join(ch if ch.isprintable() or ch in '\n\t' else '?' for ch in text)
    if len(text) > limit:
        return text[: max(0, limit - 3)] + '...'
    return text


def to_jsonable(value: Any, depth: int = 0) -> Any:
    if depth > 8:
        return safe_str(value, 120)
    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        return safe_str(value)
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v, depth + 1) for v in value[:100]]
    if isinstance(value, dict):
        out = {}
        for key, item in list(value.items())[:100]:
            out[safe_str(key, 80)] = to_jsonable(item, depth + 1)
        return out
    if hasattr(value, 'text'):
        return safe_str(getattr(value, 'text'))
    if hasattr(value, '__dict__'):
        return {
            key: to_jsonable(item, depth + 1)
            for key, item in vars(value).items()
            if not key.startswith('_')
        }
    return safe_str(value)


def compact_json(data: dict) -> str:
    return json.dumps(to_jsonable(data), ensure_ascii=False, separators=(',', ':'))


def bound_context(context: dict, max_chars: int) -> dict:
    max_chars = max(200, int(max_chars or 200))
    context = to_jsonable(context)
    if len(compact_json(context)) <= max_chars:
        return context

    bounded = dict(context)
    warnings = list(bounded.get('warnings') or [])
    warnings.append(f'context truncated to {max_chars} characters')
    bounded['warnings'] = warnings

    for key in ('recent_events', 'mcp', 'formations', 'robots'):
        if len(compact_json(bounded)) <= max_chars:
            break
        if key in bounded:
            if isinstance(bounded[key], list):
                bounded[key] = bounded[key][:3]
            elif isinstance(bounded[key], dict):
                bounded[key] = {
                    sub_key: bounded[key][sub_key]
                    for sub_key in list(bounded[key])[:5]
                }

    while len(compact_json(bounded)) > max_chars and 'recent_events' in bounded:
        events = bounded.get('recent_events') or []
        if not events:
            bounded.pop('recent_events', None)
            break
        bounded['recent_events'] = events[:-1]

    if len(compact_json(bounded)) > max_chars:
        return {
            'timestamp': bounded.get('timestamp', utc_now()),
            'source': bounded.get('source', 'unknown'),
            'warnings': warnings,
        }
    return bounded


def map_summary(map_name: str, map_config: dict[str, Any]) -> dict:
    named = map_config.get('named_locations') or {}
    zones = [safe_str(name, 80) for name in named.keys()]
    zones.extend(
        safe_str(zone.get('name'), 80)
        for zone in (map_config.get('formation_zones') or [])
        if zone.get('name')
    )
    deduped = []
    for zone in zones:
        if zone and zone not in deduped:
            deduped.append(zone)
    return {
        'name': map_config.get('name') or map_name,
        'known_zones': deduped[:80],
    }


def known_robot_ids(map_config: dict[str, Any]) -> list[int]:
    ids: set[int] = set()
    for group in (map_config.get('robot_groups') or {}).values():
        for rid in group.get('ids') or []:
            try:
                ids.add(int(rid))
            except (TypeError, ValueError):
                continue
    return sorted(ids)


def safe_mcp_allowlist(raw_tools: list[str] | tuple[str, ...]) -> tuple[list[str], list[str]]:
    allowed = []
    warnings = []
    for tool in raw_tools or []:
        name = safe_str(tool, 120).strip()
        if not name:
            continue
        if name in BLOCKED_MCP_TOOLS:
            warnings.append(f'MCP tool blocked: {name}')
            continue
        if name not in DEFAULT_MCP_READ_TOOLS:
            warnings.append(f'MCP tool not in read-only allowlist: {name}')
            continue
        if name not in allowed:
            allowed.append(name)
    return allowed, warnings
