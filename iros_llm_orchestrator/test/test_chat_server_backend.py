"""Static checks for the /llm/chat backend selection path."""

from pathlib import Path


def test_chat_server_uses_backend_factory_not_direct_ollama():
    package_root = Path(__file__).resolve().parents[1]
    source = (package_root / 'iros_llm_orchestrator' / 'chat_server.py').read_text()

    assert 'get_llm_client' in source
    assert 'self._llm.stream' in source
    assert 'make_context_provider' in source
    assert 'Context provider failed; continuing without live context' in source
    assert 'should_skip_reply_only_execution' in source
    assert 'from iros_llm_orchestrator.local.ollama_client import OllamaClient' not in source
    assert 'OllamaClient(' not in source
