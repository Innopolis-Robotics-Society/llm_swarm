"""HuggingFace Transformers in-process backend.

Only useful for tiny debug models — large models should use Ollama instead.
The `transformers` import is lazy so this package doesn't hard-depend on it.
"""

from __future__ import annotations

import asyncio

from iros_llm_orchestrator.common.llm_factory import LLMClientBase


class TransformersClient(LLMClientBase):
    def __init__(
        self,
        model:       str   = 'sshleifer/tiny-gpt2',
        max_tokens:  int   = 256,
        temperature: float = 0.2,
    ):
        self.model       = model
        self.max_tokens  = max_tokens
        self.temperature = temperature
        self._pipe       = None   # lazy init on first call

    async def generate(self, prompt: str | list, prompt_kind: str = 'decision') -> str:
        flat = prompt if isinstance(prompt, str) else self._flatten(prompt)
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(None, self._sync_generate, flat)

    def _sync_generate(self, prompt: str) -> str:
        if self._pipe is None:
            try:
                from transformers import pipeline
            except ImportError as exc:
                raise RuntimeError(
                    'llm_mode=local requires the `transformers` package. '
                    'Install: pip install transformers --break-system-packages'
                ) from exc
            self._pipe = pipeline('text-generation', model=self.model)

        outputs = self._pipe(
            prompt,
            max_new_tokens=self.max_tokens,
            do_sample=self.temperature > 0.0,
            temperature=max(self.temperature, 1e-3),
            return_full_text=False,
        )
        return outputs[0]['generated_text']

    @staticmethod
    def _flatten(messages: list) -> str:
        """Convert chat messages to a flat prompt string."""
        parts = []
        for m in messages:
            role    = m.get('role', 'user')
            content = m.get('content', '')
            if role == 'system':
                parts.append(f'[SYSTEM]\n{content}')
            elif role == 'assistant':
                parts.append(f'[ASSISTANT]\n{content}')
            else:
                parts.append(f'[USER]\n{content}')
        parts.append('[ASSISTANT]')
        return '\n\n'.join(parts)
