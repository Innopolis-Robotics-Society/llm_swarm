"""OpenAI-compatible HTTP backend.

Works with vLLM, TGI, llama.cpp-server with --api-like-openai, etc.
Also handles /v1/chat/completions shape (messages API).
"""

from __future__ import annotations

import json
import os

from iros_llm_orchestrator.common.llm_factory import LLMClientBase


class HttpClient(LLMClientBase):
    def __init__(
        self,
        endpoint: str = 'http://localhost:8000/v1/completions',
        model: str = '',
        max_tokens: int = 256,
        temperature: float = 0.2,
        api_key: str | None = None,
        api_key_env: str = 'LLM_API_KEY',
        timeout: float = 30.0,
        force_chat: bool | None = None,
        enable_stop: bool = False,
    ):
        self.endpoint = endpoint
        self.model = model
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.api_key_env = api_key_env
        self.api_key = (api_key or os.environ.get(api_key_env, '')).strip()
        self.timeout = timeout
        self.force_chat = force_chat
        self.enable_stop = enable_stop

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        if self._should_use_chat(prompt):
            return await self._chat(self._to_messages(prompt))
        return await self._completions(str(prompt))

    async def stream(self, messages: list[dict]):
        """Yield chat-completion chunks, falling back to a full response.

        Some OpenAI-compatible providers or local gateways reject streaming.
        In that case, keep /llm/chat usable by returning the non-streamed
        response as a single chunk.
        """
        emitted = False
        try:
            async for chunk in self._stream_chat(self._to_messages(messages)):
                emitted = True
                yield chunk
        except Exception:
            if emitted:
                raise
            full = await self.generate(messages, prompt_kind='chat')
            if full:
                yield full

    def _headers(self) -> dict:
        headers = {'Content-Type': 'application/json'}
        if self.api_key:
            headers['Authorization'] = f'Bearer {self.api_key}'
        return headers

    def _should_use_chat(self, prompt: str | list) -> bool:
        if isinstance(prompt, list):
            return True
        if self.force_chat is True:
            return True
        if self.force_chat is False:
            return False
        if '/chat/completions' in self.endpoint:
            return True
        if self.api_key:
            return True
        return False

    def _to_messages(self, prompt: str | list) -> list[dict]:
        if isinstance(prompt, list):
            return prompt
        return [{'role': 'user', 'content': prompt}]

    def _chat_endpoint(self) -> str:
        base = self.endpoint.rstrip('/')
        if base.endswith('/chat/completions'):
            return base
        if base.endswith('/completions'):
            return base[:-len('/completions')] + '/chat/completions'
        return base

    def _with_optional_stop(self, payload: dict) -> dict:
        if self.enable_stop:
            payload = dict(payload)
            payload['stop'] = ['\n## ', '\n# ', '</s>']
        return payload

    async def _completions(self, prompt: str) -> str:
        """POST to /v1/completions (flat prompt string)."""
        payload = {
            'model': self.model,
            'prompt': prompt,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }
        data = await self._post_json(self.endpoint, self._with_optional_stop(payload))
        return self._extract(data)

    async def _chat(self, messages: list[dict]) -> str:
        """POST to /v1/chat/completions (messages list)."""
        payload = {
            'model': self.model,
            'messages': messages,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }
        data = await self._post_json(
            self._chat_endpoint(),
            self._with_optional_stop(payload),
        )
        return self._extract(data)

    async def _stream_chat(self, messages: list[dict]):
        import aiohttp

        payload = self._with_optional_stop({
            'model': self.model,
            'messages': messages,
            'stream': True,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        })
        timeout = aiohttp.ClientTimeout(total=self.timeout)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                self._chat_endpoint(),
                json=payload,
                headers=self._headers(),
            ) as resp:
                if resp.status < 200 or resp.status >= 300:
                    body = await resp.text()
                    raise RuntimeError(
                        f'HTTP LLM {resp.status}: {self._sanitize_body(body)[:500]}'
                    )
                async for raw_line in resp.content:
                    text = raw_line.decode('utf-8', errors='replace')
                    for line in text.splitlines():
                        chunk = self._extract_sse_line(line)
                        if chunk == '[DONE]':
                            return
                        if chunk:
                            yield chunk

    async def _post_json(self, endpoint: str, payload: dict) -> dict:
        import aiohttp

        timeout = aiohttp.ClientTimeout(total=self.timeout)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                endpoint,
                json=payload,
                headers=self._headers(),
            ) as resp:
                if resp.status < 200 or resp.status >= 300:
                    body = await resp.text()
                    raise RuntimeError(
                        f'HTTP LLM {resp.status}: {self._sanitize_body(body)[:500]}'
                    )
                return await resp.json()

    @staticmethod
    def _extract(data: dict) -> str:
        if 'choices' in data and data['choices']:
            choice = data['choices'][0]
            if 'message' in choice:
                return (choice.get('message') or {}).get('content') or ''
            if 'delta' in choice:
                return (choice.get('delta') or {}).get('content') or ''
            if 'text' in choice:
                return choice['text'] or ''
        for key in ('text', 'output', 'generated_text'):
            if key in data:
                return data[key] or ''
        raise RuntimeError(f'Unrecognised HTTP response shape: {data!r}')

    @staticmethod
    def _extract_sse_line(line: str) -> str:
        line = line.strip()
        if not line.startswith('data:'):
            return ''
        payload = line[len('data:'):].strip()
        if payload == '[DONE]':
            return '[DONE]'
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return ''
        if 'choices' in data and data['choices']:
            choice = data['choices'][0]
            delta = choice.get('delta') or {}
            if delta.get('content'):
                return delta['content']
            message = choice.get('message') or {}
            if message.get('content'):
                return message['content']
            if choice.get('text'):
                return choice['text']
        for key in ('text', 'output', 'generated_text'):
            if data.get(key):
                return data[key]
        return ''

    def _sanitize_body(self, body: str) -> str:
        if self.api_key:
            return body.replace(self.api_key, '[redacted]')
        return body
