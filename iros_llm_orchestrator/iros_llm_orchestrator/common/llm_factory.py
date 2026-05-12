"""LLM client factory.

  get_llm_client(mode, endpoint, model, max_tokens, temperature) -> LLMClientBase

Selects the backend based on mode string and returns a ready client.
The actual backend classes live in local/ and web/.
"""

from __future__ import annotations


class LLMClientBase:
    """Minimal interface all backends must implement."""

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        raise NotImplementedError

    async def stream(self, prompt: str | list):
        yield await self.generate(prompt, prompt_kind='chat')


def get_llm_client(
    mode: str = 'mock',
    endpoint: str | None = None,
    model: str = '',
    max_tokens: int = 256,
    temperature: float = 0.2,
    api_key: str | None = None,
    api_key_env: str = 'LLM_API_KEY',
    timeout: float = 30.0,
    force_chat: bool | None = None,
    enable_stop: bool = False,
) -> LLMClientBase:
    """Return an LLM client for the requested mode.

    mode:
      'mock'   — heuristic keyword matcher, no network (default for tests)
      'ollama' — local Ollama /api/chat endpoint
      'local'  — HuggingFace Transformers in-process pipeline
      'http'   — OpenAI-compatible /v1/completions endpoint
    """
    mode = mode.strip().lower()

    if mode == 'mock':
        from iros_llm_orchestrator.common.mock_client import MockLLMClient
        return MockLLMClient(max_tokens=max_tokens, temperature=temperature)

    if mode == 'ollama':
        from iros_llm_orchestrator.local.ollama_client import OllamaClient
        return OllamaClient(
            endpoint=endpoint or 'http://localhost:11434/api/chat',
            model=model or 'qwen2.5:14b',
            max_tokens=max_tokens,
            temperature=temperature,
        )

    if mode == 'local':
        from iros_llm_orchestrator.local.transformers_client import TransformersClient
        return TransformersClient(
            model=model or 'sshleifer/tiny-gpt2',
            max_tokens=max_tokens,
            temperature=temperature,
        )

    if mode == 'http':
        from iros_llm_orchestrator.web.http_client import HttpClient
        return HttpClient(
            endpoint=endpoint or 'http://localhost:8000/v1/completions',
            model=model,
            max_tokens=max_tokens,
            temperature=temperature,
            api_key=api_key,
            api_key_env=api_key_env,
            timeout=timeout,
            force_chat=force_chat,
            enable_stop=enable_stop,
        )

    raise ValueError(
        f"Unknown llm_mode: '{mode}'. "
        "Valid options: 'mock', 'ollama', 'local', 'http'."
    )
