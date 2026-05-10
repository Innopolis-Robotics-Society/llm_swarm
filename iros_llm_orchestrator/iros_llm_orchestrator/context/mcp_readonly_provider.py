"""Optional read-only MCP context provider.

The LLM never receives tool access. This provider calls a fixed set of
allowlisted read tools and returns a bounded snapshot.
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import socket
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
    def __init__(self, config: ChatContextConfig, logger: Any | None = None):
        super().__init__(config)
        self._logger = logger
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
            warnings = self._preflight_warnings()
            if warnings:
                context['warnings'].extend(warnings)
                self._log_warning_list(context['warnings'])
                return bound_context(context, self.config.max_chars)
            snapshot = await asyncio.wait_for(
                self._collect_snapshot(),
                timeout=max(0.1, self.config.timeout_sec),
            )
            context.update(snapshot)
        except Exception as exc:
            context['warnings'].append(
                f'MCP server unavailable; continuing without live MCP context: {exc}'
            )
        self._log_warning_list(context.get('warnings') or [])
        if not context['warnings']:
            context.pop('warnings', None)
        return bound_context(context, self.config.max_chars)

    def _preflight_warnings(self) -> list[str]:
        warnings: list[str] = []
        cmd = self._command_display()
        resolved = self._resolve_command()
        self._info(f'MCP command: {cmd}')
        self._info(
            f'MCP command path resolution: '
            f'{resolved if resolved else "not found"}')

        if not resolved:
            if os.path.basename(self.config.mcp_command) == 'uvx':
                warnings.append(
                    'uvx not found in PATH. Install uv or set mcp_command to '
                    'the absolute uvx path.'
                )
            else:
                warnings.append(
                    f'MCP command not found or not executable: '
                    f'{self.config.mcp_command}'
                )

        try:
            from mcp import ClientSession as _ClientSession  # noqa: F401
            from mcp import StdioServerParameters as _StdioServerParameters  # noqa: F401
            from mcp.client.stdio import stdio_client as _stdio_client  # noqa: F401
        except ImportError:
            warnings.append(
                'mcp Python SDK is not installed. Install the MCP Python SDK '
                'in the ROS container environment.'
            )
        else:
            self._info('MCP Python SDK import succeeded')

        if not self._rosbridge_reachable():
            warnings.append(
                'rosbridge is not reachable on port 9090. Start '
                'rosbridge_server or enable_rosbridge:=true.'
            )
        else:
            self._info('rosbridge reachable on port 9090')

        return warnings

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
                self._info('MCP stdio client initialized')
                for tool_name, args in self._planned_calls():
                    if tool_name not in self._allowlist:
                        continue
                    out['mcp'][tool_name] = await self._call_tool(
                        session,
                        tool_name,
                        args,
                    )
                self._info(
                    f'MCP read-only tool calls ran: '
                    f'{", ".join(out["mcp"].keys()) or "none"}')
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

    def _resolve_command(self) -> str:
        command = self.config.mcp_command
        if os.path.isabs(command):
            if os.path.isfile(command) and os.access(command, os.X_OK):
                return command
            return ''
        return shutil.which(command) or ''

    def _command_display(self) -> str:
        return ' '.join([self.config.mcp_command, *self.config.mcp_args])

    @staticmethod
    def _rosbridge_reachable() -> bool:
        try:
            with socket.create_connection(('127.0.0.1', 9090), timeout=0.3):
                return True
        except OSError:
            return False

    def _log_warning_list(self, warnings: list[str]):
        if warnings:
            self._warn(f'MCP context warnings: {warnings}')
        else:
            self._info('MCP context warnings: []')

    def _info(self, message: str):
        if self._logger is not None:
            self._logger.info(message)

    def _warn(self, message: str):
        if self._logger is not None:
            self._logger.warn(message)


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
