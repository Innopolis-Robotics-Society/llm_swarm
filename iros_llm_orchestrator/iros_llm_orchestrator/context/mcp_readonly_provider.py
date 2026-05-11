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
    robots_snapshot_dict,
    safe_mcp_allowlist,
    to_jsonable,
    utc_now,
)


class McpReadonlyContextProvider(ChatContextProvider):
    def __init__(
        self,
        config: ChatContextConfig,
        logger: Any | None = None,
        *,
        pose_cache: Any | None = None,
    ):
        super().__init__(config)
        self._logger = logger
        self._pose_cache = pose_cache
        self._allowlist, self._allowlist_warnings = safe_mcp_allowlist(
            config.mcp_tool_allowlist)

    async def get_context(self) -> dict:
        return await self._build_context(
            source='mcp_readonly',
            calls=self._planned_calls(),
        )

    async def get_targeted_context(self, failure: dict) -> dict:
        """Re-snapshot MCP biased toward the leaf that just failed.

        ``failure`` is the dict returned by ``BTLeafSender.last_failure()``
        (leaf_type, action_status, last_error, failed_at_phase). For
        formation/mapf failures we force-enable the topics that diagnose
        the root cause regardless of the include_* flags so the LLM sees
        the freshest possible state for the remediation prompt.
        """
        calls = self._planned_calls_for(failure or {})
        context = await self._build_context(
            source='mcp_readonly_remediation',
            calls=calls,
        )
        context['failure_summary'] = {
            'leaf_type':       (failure or {}).get('leaf_type', ''),
            'failed_at_phase': (failure or {}).get('failed_at_phase', ''),
            'action_status':   (failure or {}).get('action_status', ''),
            'last_error':      (failure or {}).get('last_error', ''),
        }
        return bound_context(context, self.config.max_chars)

    async def _build_context(
        self,
        *,
        source: str,
        calls: list[tuple[str, dict]],
    ) -> dict:
        context = {
            'timestamp': utc_now(),
            'source': source,
            'warnings': list(self._allowlist_warnings),
        }
        if self.config.include_map_summary:
            context['map'] = map_summary(
                self.config.map_name,
                self.config.map_config,
            )
        if self.config.include_robot_positions:
            poses = robots_snapshot_dict(
                self._pose_cache, self.config.pose_stale_ms)
            context['robots'] = poses
            if not poses:
                context['warnings'].append(
                    'No /robot_*/odom received yet — leader pose unavailable')
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
                self._collect_snapshot(calls),
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

    async def _collect_snapshot(
        self,
        calls: list[tuple[str, dict]] | None = None,
    ) -> dict:
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
        planned = self._planned_calls() if calls is None else calls
        out: dict[str, Any] = {'mcp': {}}
        # subscribe_once results are keyed by tool name in the legacy path,
        # but a single tool name can show up multiple times for different
        # topics. Disambiguate using "<tool>:<topic>" so two subscribe_once
        # calls don't collide.
        async with stdio_client(server_params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                self._info('MCP stdio client initialized')
                for tool_name, args in planned:
                    if tool_name not in self._allowlist:
                        continue
                    key = tool_name
                    if tool_name == 'subscribe_once' and args.get('topic'):
                        key = f"subscribe_once:{args['topic']}"
                    out['mcp'][key] = await self._call_tool(
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

    def _planned_calls_for(
        self,
        failure: dict,
    ) -> list[tuple[str, dict]]:
        """Targeted call set keyed on the leaf type that just failed.

        Always pulls the minimum diagnostic surface (topology + bt_state)
        plus the topic that exposes the failed leaf's root cause. Forces
        these in even if the corresponding ``include_*`` flag is off — this
        snapshot is for remediation, not the cheap recurring chat-turn
        snapshot, so we accept slightly more cost for sharper signal.
        """
        leaf = str(failure.get('leaf_type') or '').lower()
        calls: list[tuple[str, dict]] = [
            ('get_topics', {}),
            ('get_nodes', {}),
            ('subscribe_once', {
                'topic': '/bt/state',
                'msg_type': 'iros_llm_swarm_interfaces/msg/BTState',
            }),
        ]
        # Match on the BT leaf type names (lower-case) emitted by the C++
        # nodes via the ``LlmCommand.mode`` field. SetFormation/Disable map
        # to mode == "formation"/"idle"; MapfPlan to "mapf".
        if leaf in {'formation', 'set_formation', 'setformation',
                    'disable_formation', 'disableformation'}:
            calls.append(('subscribe_once', {
                'topic': '/formations/status',
                'msg_type':
                    'iros_llm_swarm_interfaces/msg/FormationsStatus',
            }))
        elif leaf in {'mapf', 'mapfplan', 'mapf_plan'}:
            calls.append(('get_action_status', {
                'action': '/swarm/set_goals',
            }))
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


def summarize_for_remediation(snapshot: dict) -> dict:
    """Slim a runtime-context snapshot for use in a remediation prompt.

    Drops the topology listings (``get_topics`` / ``get_nodes`` /
    ``get_services`` / ``get_actions``) — they are noise once the failed
    leaf is already identified — and keeps the diagnostic surface the LLM
    needs to choose a corrective plan: bt_state, formations, robots,
    recent_events, map name, warnings, and any failure_summary echo.
    """
    if not isinstance(snapshot, dict):
        return {}
    slim: dict = {}
    for key in (
        'timestamp', 'source', 'warnings', 'failure_summary',
        'bt_state', 'formations', 'robots', 'robot_assignment',
        'recent_events',
    ):
        if key in snapshot:
            slim[key] = snapshot[key]
    if isinstance(snapshot.get('map'), dict):
        name = snapshot['map'].get('name')
        if name:
            slim['map'] = {'name': name}
    mcp = snapshot.get('mcp')
    if isinstance(mcp, dict):
        kept = {
            key: value
            for key, value in mcp.items()
            if key.startswith('subscribe_once:')
            or key in {'get_action_status'}
        }
        if kept:
            slim['mcp'] = kept
    return slim


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
