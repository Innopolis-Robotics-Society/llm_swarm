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
        num_ctx: int = 32768,
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
        # OpenAI-compatible APIs do not expose a standard per-request context
        # window knob. Keep this as a local budget hint for diagnostics; the
        # actual window must be configured server-side (for example vLLM
        # --max-model-len).
        self.num_ctx = num_ctx

    async def generate(
        self,
        prompt: str | list,
        prompt_kind: str = 'decision',
        response_format: dict | None = None,
    ) -> str:
        if self._should_use_chat(prompt):
            return await self._chat(self._to_messages(prompt), response_format)
        return await self._completions(str(prompt))

    async def stream(self, messages: list[dict], response_format: dict | None = None):
        """Yield chat-completion chunks, falling back to a full response.

        Some OpenAI-compatible providers or local gateways reject streaming.
        In that case, keep /llm/chat usable by returning the non-streamed
        response as a single chunk. response_format, when given, is a JSON
        schema sent as OpenAI ``response_format: json_schema``.
        """
        emitted = False
        try:
            async for chunk in self._stream_chat(
                self._to_messages(messages), response_format):
                emitted = True
                yield chunk
        except Exception:
            if emitted:
                raise
            full = await self.generate(
                messages, prompt_kind='chat', response_format=response_format)
            if full:
                yield full

    async def generate_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """Non-streaming call with tool definitions via OpenAI /v1/chat/completions.

        Returns {"type": "tool_calls", "calls": [...]} or {"type": "text", "content": str}.
        """
        from iros_llm_orchestrator.common.tool_definitions import parse_openai_tool_calls

        payload = self._with_optional_stop({
            'model':       self.model,
            'messages':    messages,
            'tools':       tools,
            'tool_choice': 'auto',
            'max_tokens':  self.max_tokens,
            'temperature': self.temperature,
        })
        data = await self._post_json(self._chat_endpoint(), payload)

        if not data.get('choices'):
            raise RuntimeError(f'Unexpected HTTP response (no choices): {data!r}')
        choice = data['choices'][0]
        calls = parse_openai_tool_calls(choice)
        if calls:
            return {"type": "tool_calls", "calls": calls}
        content = (choice.get('message') or {}).get('content', '') or ''
        return {"type": "text", "content": content}

    async def stream_with_tools(
        self,
        messages: list[dict],
        tools: list[dict],
    ):
        """Streaming call with tool definitions via OpenAI /v1/chat/completions.

        Yields:
          {"type": "chunk",      "content": str}  — text token during generation
          {"type": "tool_calls", "calls": [...]}   — tool call request (terminal)
          {"type": "text",       "content": str}   — full text response (terminal)
        """
        from iros_llm_orchestrator.common.tool_definitions import parse_openai_tool_calls
        import aiohttp

        payload = self._with_optional_stop({
            'model':       self.model,
            'messages':    messages,
            'tools':       tools,
            'tool_choice': 'auto',
            'stream':      True,
            'max_tokens':  self.max_tokens,
            'temperature': self.temperature,
        })

        full_text = ''
        # {index: {"id": str, "type": str, "function": {"name": str, "arguments": str}}}
        tool_calls_acc: dict[int, dict] = {}
        finish_reason: str | None = None

        await self._pace()
        timeout = aiohttp.ClientTimeout(total=self.timeout)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                self._chat_endpoint(), json=payload, headers=self._headers(),
            ) as resp:
                if resp.status < 200 or resp.status >= 300:
                    body = await resp.text()
                    raise RuntimeError(
                        f'HTTP LLM {resp.status}: {self._sanitize_body(body)[:500]}')
                async for raw_line in resp.content:
                    text = raw_line.decode('utf-8', errors='replace')
                    for line in text.splitlines():
                        line = line.strip()
                        if not line.startswith('data:'):
                            continue
                        payload_str = line[len('data:'):].strip()
                        if payload_str == '[DONE]':
                            break
                        try:
                            data = json.loads(payload_str)
                        except json.JSONDecodeError:
                            continue
                        if not data.get('choices'):
                            continue
                        choice = data['choices'][0]
                        delta = choice.get('delta') or {}
                        fr = choice.get('finish_reason')
                        if fr:
                            finish_reason = fr

                        content = delta.get('content')
                        if content:
                            full_text += content
                            yield {'type': 'chunk', 'content': content}

                        tc_deltas = delta.get('tool_calls')
                        if tc_deltas:
                            for tc in tc_deltas:
                                idx = tc.get('index', 0)
                                if idx not in tool_calls_acc:
                                    tool_calls_acc[idx] = {
                                        'id': '',
                                        'type': 'function',
                                        'function': {'name': '', 'arguments': ''},
                                    }
                                acc = tool_calls_acc[idx]
                                if tc.get('id'):
                                    acc['id'] = tc['id']
                                if tc.get('type'):
                                    acc['type'] = tc['type']
                                fn = tc.get('function') or {}
                                if fn.get('name'):
                                    acc['function']['name'] += fn['name']
                                if fn.get('arguments'):
                                    acc['function']['arguments'] += fn['arguments']

        if finish_reason == 'tool_calls' and tool_calls_acc:
            calls_list = [tool_calls_acc[i] for i in sorted(tool_calls_acc)]
            synthetic_choice = {
                'message': {
                    'role': 'assistant',
                    'content': None,
                    'tool_calls': calls_list,
                }
            }
            calls = parse_openai_tool_calls(synthetic_choice)
            if calls:
                yield {'type': 'tool_calls', 'calls': calls}
                return

        yield {'type': 'text', 'content': full_text}

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
        return self._with_router_hints(payload)

    @staticmethod
    def _with_router_hints(payload: dict) -> dict:
        """Optional OpenRouter provider pin, from OPENROUTER_PROVIDER.

        Purely an operational escape hatch: OpenRouter answers sustained load
        with HTTP 403 "Access denied by security policy" (before generation,
        in under a second), and the only way to finish a sweep is to move the
        remaining calls to a different upstream. Unset -- the normal case, and
        every production path -- leaves the payload untouched, so this cannot
        change how the orchestrator behaves in the field.

        Whichever upstream actually served a call is echoed back in the
        response `provider` field and must be recorded next to the numbers:
        upstreams differ in quantisation, and a run split across two of them is
        not one measurement.
        """
        prov = os.environ.get('OPENROUTER_PROVIDER', '').strip()
        if not prov:
            return payload
        payload = dict(payload)
        payload['provider'] = {
            'order': [p.strip() for p in prov.split(',') if p.strip()],
            'allow_fallbacks': os.environ.get(
                'OPENROUTER_ALLOW_FALLBACKS', '').strip().lower() == 'true',
        }
        return payload

    @staticmethod
    async def _pace() -> None:
        """Sleep between calls when LLM_CALL_DELAY_SEC is set.

        The harness fires the next request the instant the previous one
        returns; that burst rate is what trips the router's throttle. A second
        or two of spacing costs minutes over a sweep and saves re-running it.
        """
        try:
            delay = float(os.environ.get('LLM_CALL_DELAY_SEC', '') or 0)
        except ValueError:
            return
        if delay > 0:
            import asyncio
            await asyncio.sleep(delay)

    @staticmethod
    def _with_response_format(payload: dict, response_format: dict | None) -> dict:
        """Attach an OpenAI json_schema response_format from a raw JSON schema."""
        if not response_format:
            return payload
        payload = dict(payload)
        payload['response_format'] = {
            'type': 'json_schema',
            'json_schema': {'name': 'plan_response', 'schema': response_format},
        }
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

    async def _chat(
        self, messages: list[dict], response_format: dict | None = None) -> str:
        """POST to /v1/chat/completions (messages list)."""
        payload = {
            'model': self.model,
            'messages': messages,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }
        payload = self._with_response_format(
            self._with_optional_stop(payload), response_format)
        data = await self._post_json(self._chat_endpoint(), payload)
        return self._extract(data)

    async def _stream_chat(
        self, messages: list[dict], response_format: dict | None = None):
        import aiohttp

        payload = self._with_response_format(self._with_optional_stop({
            'model': self.model,
            'messages': messages,
            'stream': True,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
        }), response_format)
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
        await self._pace()
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
