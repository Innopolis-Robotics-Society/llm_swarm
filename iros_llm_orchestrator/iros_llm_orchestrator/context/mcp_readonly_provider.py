"""Optional read-only MCP context provider.

The LLM never receives tool access. This provider calls a fixed set of
allowlisted read tools and returns a bounded snapshot.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

from iros_llm_orchestrator.context.provider import (
    ChatContextConfig,
    ChatContextProvider,
    bound_context,
    map_summary,
    safe_mcp_allowlist,
    to_jsonable,
    utc_now,
)


class McpReadonlyContextProvider(ChatContextProvider):
    def __init__(self, config: ChatContextConfig):
        super().__init__(config)
        self._allowlist, self._allowlist_warnings = safe_mcp_allowlist(
            config.mcp_tool_allowlist)

    async def get_context(self) -> dict:
        context = {
            'timestamp': utc_now(),
            'source': 'mcp_readonly',
            'warnings': list(self._allowlist_warnings),
        }
        if self.config.include_map_summary:
            context['map'] = map_summary(
                self.config.map_name,
                self.config.map_config,
            )
        if not self.config.mcp_enabled:
            context['warnings'].append('MCP context disabled')
            return bound_context(context, self.config.max_chars)
        if not self._allowlist:
            context['warnings'].append('No read-only MCP tools allowed')
            return bound_context(context, self.config.max_chars)

        try:
            snapshot = await asyncio.wait_for(
                self._collect_snapshot(),
                timeout=max(0.1, self.config.timeout_sec),
            )
            context.update(snapshot)
        except Exception as exc:
            context['warnings'].append(
                f'MCP server unavailable; continuing without live MCP context: {exc}'
            )
        if not context['warnings']:
            context.pop('warnings', None)
        return bound_context(context, self.config.max_chars)

    async def _collect_snapshot(self) -> dict:
        if self.config.mcp_transport != 'stdio':
            raise RuntimeError(
                f'unsupported MCP transport for Phase 2: {self.config.mcp_transport}'
            )

        try:
            from mcp import ClientSession, StdioServerParameters
            from mcp.client.stdio import stdio_client
        except ImportError as exc:
            raise RuntimeError('mcp Python SDK is not installed') from exc

        server_params = StdioServerParameters(
            command=self.config.mcp_command,
            args=list(self.config.mcp_args),
        )
        out: dict[str, Any] = {'mcp': {}}
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                for tool_name, args in self._planned_calls():
                    if tool_name not in self._allowlist:
                        continue
                    out['mcp'][tool_name] = await self._call_tool(
                        session,
                        tool_name,
                        args,
                    )
        return out

    def _planned_calls(self) -> list[tuple[str, dict]]:
        calls: list[tuple[str, dict]] = [
            ('get_topics', {}),
            ('get_nodes', {}),
            ('get_services', {}),
            ('get_actions', {}),
        ]
        if self.config.include_bt_state:
            calls.append((
                'subscribe_once',
                {
                    'topic': '/bt/state',
                    'msg_type': 'iros_llm_swarm_interfaces/msg/BTState',
                },
            ))
        if self.config.include_formations:
            calls.append((
                'subscribe_once',
                {
                    'topic': '/formations/status',
                    'msg_type': 'iros_llm_swarm_interfaces/msg/FormationsStatus',
                },
            ))
        return calls

    async def _call_tool(self, session: Any, name: str, args: dict) -> Any:
        result = await session.call_tool(name, args)
        return _tool_result_to_jsonable(result)


def _tool_result_to_jsonable(result: Any) -> Any:
    data = to_jsonable(result)
    if isinstance(data, dict) and 'content' in data:
        content = data.get('content') or []
        parsed = []
        for item in content:
            if isinstance(item, dict) and 'text' in item:
                text = item.get('text') or ''
                try:
                    parsed.append(json.loads(text))
                except (TypeError, json.JSONDecodeError):
                    parsed.append(text)
            else:
                parsed.append(item)
        return parsed
    return data
