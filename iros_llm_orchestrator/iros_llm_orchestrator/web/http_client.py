"""OpenAI-compatible /v1/completions HTTP backend.

Works with vLLM, TGI, llama.cpp-server with --api-like-openai, etc.
Also handles /v1/chat/completions shape (messages API).
"""

from __future__ import annotations

from iros_llm_orchestrator.common.llm_factory import LLMClientBase


class HttpClient(LLMClientBase):
    def __init__(
        self,
        endpoint:    str   = 'http://localhost:8000/v1/completions',
        model:       str   = '',
        max_tokens:  int   = 256,
        temperature: float = 0.2,
    ):
        self.endpoint    = endpoint
        self.model       = model
        self.max_tokens  = max_tokens
        self.temperature = temperature

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        if isinstance(prompt, list):
            return await self._chat(prompt)
        return await self._completions(prompt)

    async def _completions(self, prompt: str) -> str:
        """POST to /v1/completions (flat prompt string)."""
        import aiohttp

        payload = {
            'model':       self.model,
            'prompt':      prompt,
            'max_tokens':  self.max_tokens,
            'temperature': self.temperature,
            'stop':        ['\n## ', '\n# ', '</s>'],
        }
        async with aiohttp.ClientSession() as session:
            async with session.post(self.endpoint, json=payload) as resp:
                resp.raise_for_status()
                data = await resp.json()

        return self._extract(data)

    async def _chat(self, messages: list[dict]) -> str:
        """POST to /v1/chat/completions (messages list)."""
        import aiohttp

        # Auto-detect endpoint — swap /completions → /chat/completions
        base = self.endpoint.rstrip('/')
        if base.endswith('/completions') and not base.endswith('/chat/completions'):
            chat_endpoint = base[:-len('/completions')] + '/chat/completions'
        else:
            chat_endpoint = base

        payload = {
            'model':       self.model,
            'messages':    messages,
            'max_tokens':  self.max_tokens,
            'temperature': self.temperature,
            'stop':        ['\n## ', '\n# ', '</s>'],
        }
        async with aiohttp.ClientSession() as session:
            async with session.post(chat_endpoint, json=payload) as resp:
                resp.raise_for_status()
                data = await resp.json()

        return self._extract(data)

    @staticmethod
    def _extract(data: dict) -> str:
        if 'choices' in data and data['choices']:
            choice = data['choices'][0]
            if 'text' in choice:
                return choice['text']
            if 'message' in choice:
                return choice['message'].get('content', '')
        for key in ('text', 'output', 'generated_text'):
            if key in data:
                return data[key]
        raise RuntimeError(f'Unrecognised HTTP response shape: {data!r}')
