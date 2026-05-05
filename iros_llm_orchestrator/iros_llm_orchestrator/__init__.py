"""
iros_llm_orchestrator — LLM advisory layer for the Nav2 swarm BT.

Structure
---------
common/   Parsers, logger, prompt builders, scenarios, LLM client factory + mock.
local/    Local inference backends: Ollama, HuggingFace Transformers.
web/      Remote inference backends: OpenAI-compatible HTTP endpoint.

Entry-point nodes (top level):
  decision_server.py   Channel 1 — reactive /llm/decision action server.
  passive_observer.py  Channel 2 — proactive /bt/state watcher.
  user_chat_node.py    Channel 3 — natural-language operator chat.
"""
