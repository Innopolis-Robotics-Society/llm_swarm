"""LLM client factory.

  get_llm_client(mode, endpoint, model, max_tokens, temperature) -> LLMClientBase

Selects the backend based on mode string and returns a ready client.
The actual backend classes live in local/ and web/.
"""

from __future__ import annotations


class LLMClientBase:
    """Minimal interface all backends must implement."""

    async def generate(
        self,
        prompt: str | list,
        prompt_kind: str = 'decision',
        response_format: dict | None = None,
    ) -> str:
        raise NotImplementedError

    async def stream(self, prompt: str | list, response_format: dict | None = None):
        # Backends that support structured outputs override this and honour
        # response_format; the base fallback ignores it.
        yield await self.generate(prompt, prompt_kind='chat')

    async def generate_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """Send messages with tool definitions; return structured result.

        Return shape:
          {"type": "tool_calls", "calls": [{"name": str, "arguments": dict, "call_id": str}]}
          {"type": "text",       "content": str}

        Default implementation falls back to generate() (no tool awareness).
        Backends that support native tool calling override this.
        """
        content = await self.generate(messages)
        return {"type": "text", "content": content}

    async def stream_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ):
        """Async generator yielding event dicts during generation.

        Yields:
          {"type": "chunk",      "content": str}  — text token during generation
          {"type": "tool_calls", "calls": [...]}   — tool call request (terminal)
          {"type": "text",       "content": str}   — full text response (terminal)

        Exactly one terminal event (tool_calls or text) is yielded last.
        Zero or more chunk events may precede the terminal.

        Default implementation falls back to generate_with_tools(); yields the
        terminal event with no preceding chunks. Backends that support streaming
        override this to emit real-time tokens.
        """
        result = await self.generate_with_tools(messages, tools)
        yield result


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
    num_ctx: int = 32768,
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
            num_ctx=num_ctx,
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
            num_ctx=num_ctx,
        )

    raise ValueError(
        f"Unknown llm_mode: '{mode}'. "
        "Valid options: 'mock', 'ollama', 'local', 'http'."
    )
