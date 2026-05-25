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
        num_ctx:     int   = 8192,
    ):
        self.endpoint    = endpoint
        self.model       = model
        self.max_tokens  = max_tokens
        self.temperature = temperature
        # Input context window. Ollama defaults to 4096 when unset; the channel-3
        # chat prompt is ~7000 tokens, so leaving it unset truncates the system
        # prompt and forces the model into a tool-call/prose loop.
        self.num_ctx     = num_ctx

    # ------------------------------------------------------------------
    # LLMClientBase interface
    # ------------------------------------------------------------------

    async def generate(
        self,
        prompt: str | list,
        prompt_kind: str = 'decision',
        response_format: dict | None = None,
    ) -> str:
        """Non-streaming call. Returns the full response string.

        prompt may be:
          str  — flat prompt; injected as a single user message
          list — pre-built messages list [{"role":..,"content":..}]
        response_format, when given, is a JSON schema passed to Ollama's
        ``format`` field for constrained (schema-valid) decoding.
        """
        messages = self._to_messages(prompt)
        return await self._call(messages, stream=False, response_format=response_format)

    # ------------------------------------------------------------------
    # Streaming helpers (used directly by user_chat_node)
    # ------------------------------------------------------------------

    async def stream(self, messages: list[dict], response_format: dict | None = None):
        """Async generator yielding text chunks as they arrive from Ollama."""
        import aiohttp

        payload = self._payload(messages, stream=True, response_format=response_format)
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
    # Tool calling
    # ------------------------------------------------------------------

    async def generate_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """Non-streaming call with tool definitions via Ollama /api/chat.

        Returns {"type": "tool_calls", "calls": [...]} or {"type": "text", "content": str}.
        """
        from iros_llm_orchestrator.common.tool_definitions import parse_ollama_tool_calls

        import aiohttp

        payload = {
            'model':    self.model,
            'messages': messages,
            'tools':    tools,
            'stream':   False,
            'options':  {
                'temperature': self.temperature,
                'num_predict': self.max_tokens,
                'num_ctx':     self.num_ctx,
            },
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(self.endpoint, json=payload) as resp:
                if resp.status != 200:
                    body = await resp.text()
                    raise RuntimeError(f'Ollama HTTP {resp.status}: {body[:300]}')
                data = await resp.json()

        message = data.get('message', {})
        calls = parse_ollama_tool_calls(message)
        if calls:
            return {"type": "tool_calls", "calls": calls}
        content = message.get('content', '')
        return {"type": "text", "content": content}

    async def stream_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ):
        """Streaming call with tool definitions via Ollama /api/chat.

        Yields:
          {"type": "chunk",      "content": str}  — text token during generation
          {"type": "tool_calls", "calls": [...]}   — tool call request (terminal)
          {"type": "text",       "content": str}   — full text response (terminal)
        """
        from iros_llm_orchestrator.common.tool_definitions import parse_ollama_tool_calls
        import aiohttp

        payload = {
            'model':    self.model,
            'messages': messages,
            'tools':    tools,
            'stream':   True,
            'options':  {
                'temperature': self.temperature,
                'num_predict': self.max_tokens,
                'num_ctx':     self.num_ctx,
            },
        }

        full_text = ''
        final_message: dict = {}
        # Ollama may put tool_calls in intermediate chunks rather than the done
        # chunk — accumulate across all chunks so they aren't silently dropped.
        accumulated_tool_calls: list = []

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
                    msg = data.get('message', {})
                    chunk = msg.get('content', '')
                    if chunk:
                        full_text += chunk
                        yield {'type': 'chunk', 'content': chunk}
                    chunk_calls = msg.get('tool_calls')
                    if chunk_calls:
                        accumulated_tool_calls = chunk_calls
                    if data.get('done'):
                        final_message = msg
                        break

        # Merge tool_calls found in intermediate chunks into the final message.
        if accumulated_tool_calls and not final_message.get('tool_calls'):
            final_message = {**final_message, 'tool_calls': accumulated_tool_calls}

        calls = parse_ollama_tool_calls(final_message)
        if calls:
            yield {'type': 'tool_calls', 'calls': calls}
        else:
            yield {'type': 'text', 'content': full_text}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_messages(self, prompt: str | list) -> list[dict]:
        if isinstance(prompt, list):
            return prompt
        return [{'role': 'user', 'content': prompt}]

    def _payload(
        self,
        messages: list[dict],
        stream: bool,
        response_format: dict | None = None,
    ) -> dict:
        payload = {
            'model':    self.model,
            'messages': messages,
            'stream':   stream,
            'options':  {
                'temperature': self.temperature,
                'num_predict': self.max_tokens,
                'num_ctx':     self.num_ctx,
                'stop':        ['\n## ', '\n# ', '</s>', '<|im_end|>'],
            },
        }
        if response_format:
            payload['format'] = response_format
        return payload

    async def _call(
        self,
        messages: list[dict],
        stream: bool,
        response_format: dict | None = None,
    ) -> str:
        import aiohttp

        payload = self._payload(messages, stream=stream, response_format=response_format)
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
