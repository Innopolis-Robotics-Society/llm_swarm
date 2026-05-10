# Phase 2 MCP Context Integration Report

## Summary
- Phase 2 is starting as Channel-3-only read-only context injection for `/llm/chat`. Defaults must preserve Phase 1 behavior: `context_provider: "none"` and `mcp_enabled: false`.
- Implementation order: first `ros_readonly`, then `mcp_readonly`. MCP tools are never exposed to operator text or the LLM planning loop.

## Audit findings
- `chat_server.py` is the correct integration point: it builds `build_user_prompt(...)` immediately before the LLM call and already owns `/bt/state` handling for chat history.
- `build_user_prompt(...)` currently has no runtime context parameter; adding an optional parameter can preserve Phase 1 behavior when absent.
- Available local read topics/types include `/bt/state` (`BTState`), `/llm/events` (`LlmEvent`), and `/formations/status` (`FormationsStatus`).
- `ros-mcp-server` v3.0.1 is distributed as `ros-mcp`, defaults to stdio transport, and connects to ROS through `rosbridge` on port 9090. Its tool set includes read tools and write/control tools, so Phase 2 must enforce its own read-only allowlist.

## Files touched
- `iros_llm_orchestrator/iros_llm_orchestrator/context/__init__.py` — added the context package exports.
- `iros_llm_orchestrator/iros_llm_orchestrator/context/provider.py` — added provider config, factory, safe JSON conversion, bounding, map summary, robot ID helpers, and MCP read/write allowlist constants.
- `iros_llm_orchestrator/iros_llm_orchestrator/context/ros_readonly_provider.py` — added read-only ROS topic cache and normalization for `/bt/state`, `/llm/events`, and `/formations/status`.
- `iros_llm_orchestrator/iros_llm_orchestrator/context/mcp_readonly_provider.py` — added optional lazy stdio MCP snapshot provider with fixed allowlisted read calls and safe unavailable fallback.
- `iros_llm_orchestrator/iros_llm_orchestrator/context/no_execute.py` — added the internal reply-only execution guard helper without changing public plan schema.
- `iros_llm_orchestrator/iros_llm_orchestrator/common/user_prompt.py` — added optional runtime context prompt injection; absent or `source=none` preserves Phase 1 prompt shape.
- `iros_llm_orchestrator/iros_llm_orchestrator/chat_server.py` — wired Channel 3 to the provider factory, bounded context fetch, prompt injection, and internal reply-only execution guard.
- `iros_llm_orchestrator/config/orchestrator.yaml` — added safe default Channel 3 context and MCP read-only parameters; `mcp_enabled` remains false and `ping_robots` is excluded.
- `iros_llm_orchestrator/test/test_chat_context.py` — added non-ROS unit/static coverage for providers, normalization, prompt injection, MCP allowlist/fallback, and reply-only guard.
- `iros_llm_orchestrator/test/test_chat_server_backend.py` — extended static check to assert chat server uses the context provider factory and still avoids direct `OllamaClient`.
- `iros_llm_orchestrator/docs/mcp_context_integration.md` — added read-only MCP/ROS context integration docs, fallback behavior, and manual checks.

## Tests run
- `python3 -m compileall iros_llm_orchestrator/iros_llm_orchestrator iros_llm_orchestrator/test` — passed.
- `git diff --check` — passed.
- `PYTHONPATH=iros_llm_orchestrator python3 - <<'PY' ...` — passed a non-ROS smoke check covering provider factory, ROS normalizer helper, context bounding, MCP blocked allowlist, MCP unavailable fallback, prompt injection, and reply-only guard.
- `python3 -m pytest iros_llm_orchestrator/test/test_chat_context.py iros_llm_orchestrator/test/test_chat_server_backend.py` — blocked in this shell because `pytest` is not installed.

## Current status
- Provider scaffolding, `ros_readonly` normalization, `mcp_readonly` fallback, prompt injection, chat integration, config defaults, docs, and focused non-ROS tests are implemented. Allowed local static/smoke checks passed.

## Risks
- MCP SDK, `uvx`, `ros-mcp`, or rosbridge may be unavailable. This must degrade to warning context, not fail `/llm/chat`.
- Context may become stale if no topics have been received yet. The provider must expose this as unknown/missing state rather than inventing facts.
- `RosReadonlyContextProvider` subscribes independently from existing chat `/bt/state` handling; this is read-only but should be watched for duplicated callback load.
- ROS2/colcon runtime validation was intentionally not run here per instruction; it remains for the docker terminal.

## Next step
- Run docker-terminal ROS validation for `/llm/chat` with `context_provider: "none"`, then `ros_readonly`, and finally `mcp_readonly` with rosbridge/ros-mcp available or intentionally unavailable.

## References audited
- `robotmcp/ros-mcp-server`: https://github.com/robotmcp/ros-mcp-server
- MCP server metadata: https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/server.json
- Programmatic stdio client docs: https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/docs/install/clients/custom-client.md
- Rosbridge setup docs: https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/docs/install/rosbridge.md
- MCP security audit: https://arxiv.org/abs/2504.03767
