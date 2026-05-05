"""Ollama /api/chat backend — local inference, streaming and non-streaming.

Recommended for RTX 5060 Ti 16 GB:
  qwen2.5:14b        ~9 GB VRAM  — best quality/speed balance
  mistral-small3.1   ~13 GB VRAM — smarter, ~2x slower
  qwen2.5:7b         ~5 GB VRAM  — fastest, acceptable quality

Install:  curl -fsSL https://ollama.com/install.sh | sh
Pull:     ollama pull qwen2.5:14b
"""

from __future__ import annotations

import json

from iros_llm_orchestrator.common.llm_factory import LLMClientBase


class OllamaClient(LLMClientBase):
    def __init__(
        self,
        endpoint: str  = 'http://localhost:11434/api/chat',
        model:    str  = 'qwen2.5:14b',
        max_tokens:  int   = 256,
        temperature: float = 0.2,
    ):
        self.endpoint    = endpoint
        self.model       = model
        self.max_tokens  = max_tokens
        self.temperature = temperature

    # ------------------------------------------------------------------
    # LLMClientBase interface
    # ------------------------------------------------------------------

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        """Non-streaming call. Returns the full response string.

        prompt may be:
          str  — flat prompt; injected as a single user message
          list — pre-built messages list [{"role":..,"content":..}]
        """
        messages = self._to_messages(prompt)
        return await self._call(messages, stream=False)

    # ------------------------------------------------------------------
    # Streaming helpers (used directly by user_chat_node)
    # ------------------------------------------------------------------

    async def stream(self, messages: list[dict]):
        """Async generator yielding text chunks as they arrive from Ollama."""
        import aiohttp

        payload = self._payload(messages, stream=True)
        async with aiohttp.ClientSession() as session:
            async with session.post(self.endpoint, json=payload) as resp:
                if resp.status != 200:
                    body = await resp.text()
                    raise RuntimeError(f'Ollama HTTP {resp.status}: {body[:300]}')
                async for raw_line in resp.content:
                    raw_line = raw_line.strip()
                    if not raw_line:
                        continue
                    try:
                        data = json.loads(raw_line)
                    except json.JSONDecodeError:
                        continue
                    chunk = data.get('message', {}).get('content', '')
                    if chunk:
                        yield chunk
                    if data.get('done'):
                        return

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_messages(self, prompt: str | list) -> list[dict]:
        if isinstance(prompt, list):
            return prompt
        return [{'role': 'user', 'content': prompt}]

    def _payload(self, messages: list[dict], stream: bool) -> dict:
        return {
            'model':    self.model,
            'messages': messages,
            'stream':   stream,
            'options':  {
                'temperature': self.temperature,
                'num_predict': self.max_tokens,
                'stop':        ['\n## ', '\n# ', '</s>', '<|im_end|>'],
            },
        }

    async def _call(self, messages: list[dict], stream: bool) -> str:
        import aiohttp

        payload = self._payload(messages, stream=stream)
        async with aiohttp.ClientSession() as session:
            async with session.post(self.endpoint, json=payload) as resp:
                if resp.status != 200:
                    body = await resp.text()
                    raise RuntimeError(f'Ollama HTTP {resp.status}: {body[:300]}')
                data = await resp.json()

        try:
            return data['message']['content']
        except (KeyError, TypeError) as exc:
            raise RuntimeError(f'Unexpected Ollama response shape: {data!r}') from exc
