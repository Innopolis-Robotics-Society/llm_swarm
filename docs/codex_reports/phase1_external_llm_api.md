# Phase 1 External LLM API Integration Report

## Summary
- Phase 1 source changes are implemented. The target remains `/llm/chat` through an external OpenAI-compatible HTTP API with `execute_after_planning=false`; ClearML, MCP, ROS-MCP, BT XML, MAPF, formation, ROS interfaces, and RViz UI are untouched.

## Files touched
- `iros_llm_orchestrator/iros_llm_orchestrator/web/http_client.py` — added API-key lookup from env/param, Bearer headers, timeout support, chat/completions routing, optional stop tokens, sanitized HTTP errors, robust response extraction, SSE parsing, and safe non-streaming fallback for `stream(...)`.
- `iros_llm_orchestrator/iros_llm_orchestrator/common/llm_factory.py` — added default backend `stream(...)` fallback and passed HTTP auth/routing options through to `HttpClient` without changing mock, Ollama, or local backend construction.
- `iros_llm_orchestrator/iros_llm_orchestrator/chat_server.py` — switched `/llm/chat` from direct `OllamaClient` construction to `get_llm_client(...)`, added backend/auth/routing params, and kept existing reply streaming and preview/execute behavior.
- `iros_llm_orchestrator/iros_llm_orchestrator/user_chat_node.py` — switched the legacy CLI chat path to the same backend factory and replaced direct `_ollama.stream(...)` usage with `_llm.stream(...)`.
- `iros_llm_orchestrator/iros_llm_orchestrator/decision_server.py` — added HTTP API auth/routing params and passed them into the factory for channel 1 compatibility.
- `iros_llm_orchestrator/iros_llm_orchestrator/passive_observer.py` — added the same HTTP API auth/routing params for channel 2 compatibility while keeping the observer disabled by default.
- `iros_llm_orchestrator/config/orchestrator.yaml` — switched LLM-capable nodes to editable OpenAI-compatible HTTP defaults, kept API key values empty, preserved local/Ollama/OpenRouter/vLLM examples as comments, and kept passive observer disabled.
- `iros_llm_orchestrator/launch/orchestrator.launch.py` — explicitly propagates `LLM_API_KEY` to decision, passive observer, and chat nodes.
- `.gitignore` — added ignore patterns for env files, API-key files, and `secrets/`.
- `iros_llm_orchestrator/docs/external_llm_api.md` — added setup, launch, provider notes, preview smoke test, and secret-handling guidance for Phase 1.
- `iros_llm_orchestrator/test/test_http_client.py` — added unit coverage for auth headers, env-key lookup, chat/completions routing, SSE chunk parsing, fallback streaming, response extraction, and error redaction.
- `iros_llm_orchestrator/test/test_chat_server_backend.py` — added a static guard that `/llm/chat` uses the backend factory instead of constructing `OllamaClient` directly.
- `iros_llm_orchestrator/test/test_decision_server.py` — updated stale test field access to the current decision logger attribute.
- `iros_llm_orchestrator/test/test_passive_observer.py` — updated stale test field access and explicitly enables the observer in the fixture.

## Tests run
- `python3 -m compileall iros_llm_orchestrator/iros_llm_orchestrator iros_llm_orchestrator/test` — pass.
- `git diff --check` — pass.
- Static check for direct `OllamaClient` construction in `chat_server.py` and `user_chat_node.py` — pass.
- `rg -n "sk-[A-Za-z0-9]|gsk_[A-Za-z0-9]|OPENAI_API_KEY|GROQ_API_KEY" . -S` — pass, no matches.
- `rg -n "ClearML|clearml|MCP|ROS-MCP|ros-mcp" --glob '!docs/codex_reports/phase1_external_llm_api.md' --glob '!/home/yoba/Downloads/*' . -S` — pass, no matches.
- `python3 -m pytest iros_llm_orchestrator/test` — blocked: `/usr/bin/python3: No module named pytest`.
- `colcon build --packages-select iros_llm_swarm_interfaces iros_llm_orchestrator` — blocked: `colcon: command not found`.

## Current status
- Phase 1 implementation is code-complete at source level. Syntax and static checks pass in this shell; full ROS/pytest verification still needs the ROS/container environment.

## Remaining risks
- Streaming fallback is intentionally conservative; if a provider fails after partial chunks were already emitted, the error is surfaced rather than duplicating partial content.
- No runtime call to a real provider has been made yet.
- `/llm/chat` refactor has not been exercised under ROS yet in this shell.
- `user_chat_node.py` remains legacy/debug tooling and has not been smoke-tested interactively yet.
- Channel 1 and 2 compatibility changes are parameter plumbing only; the main runtime gate remains `/llm/chat`.
- With the active HTTP config, local offline demos should switch `llm_mode` back to `ollama` or `mock` if no external API key is available.
- The new docs are source-tree docs and have not been wired into generated documentation.
- `/llm/decision` now publishes its documented `received`, `thinking`, and `done` feedback stages because the existing integration test already required them.
- Manual `/llm/chat` smoke with a real external provider has not been run in this environment.

## Next step
- Run the `/llm/chat` preview smoke test in the ROS/container environment with `LLM_API_KEY` exported.
