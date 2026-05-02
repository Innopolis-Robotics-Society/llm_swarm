"""
Thin wrapper over the three supported LLM backends.

  mock  — deterministic heuristic on the prompt tail. Runs without any
          model / network, used for unit tests and bring-up on laptops.
  http  — OpenAI-compatible HTTP endpoint (vLLM / TGI / llama.cpp server
          with --api-like-openai, etc). The default `llm_endpoint` already
          targets this shape.
  local — in-process transformers inference. Nemotron-30B is way too big
          for anything other than a beefy server — this mode is for
          debugging with a tiny model. Import is lazy so `transformers`
          isn't a hard dep.
"""

import asyncio
import json


_ABORT_KEYWORDS = (
    'collision',
    'fatal',
    'unreachable',
    'no valid agents',
    'pbs failed',
    'cancelled',
)

_REPLAN_KEYWORDS = (
    'timeout',
    'stall',
    'stalled',
    'stuck',
    'deviated',
    'out of lives',
    'replans_done',
)


class LLMClient:
    def __init__(self, mode='mock', endpoint=None, model='', max_tokens=256, temperature=0.2):
        self.mode = mode
        self.endpoint = endpoint
        self.model = model
        self.max_tokens = max_tokens
        self.temperature = temperature
        self._local_pipe = None  # lazily created

    async def generate(self, prompt, prompt_kind='decision'):
        if self.mode == 'mock':
            if prompt_kind == 'command':
                return await self._generate_mock_command(prompt)
            return await self._generate_mock(prompt)
        if self.mode == 'http':
            return await self._generate_http(prompt)
        if self.mode == 'local':
            return await self._generate_local(prompt)
        raise ValueError(f'unknown llm_mode: {self.mode}')

    # ---- mock -------------------------------------------------------------

    async def _generate_mock(self, prompt):
        # small sleep so tests exercise the async path
        await asyncio.sleep(0.01)
        current = self._extract_current_situation(prompt).lower()

        if any(w in current for w in _ABORT_KEYWORDS):
            return json.dumps({
                'decision': 'abort',
                'reason': 'mock: detected fatal/unreachable keyword',
            })
        if any(w in current for w in _REPLAN_KEYWORDS):
            return json.dumps({
                'decision': 'replan',
                'reason': 'mock: detected stall/timeout/deviation keyword',
            })
        return json.dumps({
            'decision': 'wait',
            'reason': 'mock: no critical keyword in current situation',
        })

    async def _generate_mock_command(self, prompt):
        await asyncio.sleep(0.01)
        current = self._extract_current_situation(prompt)
        lowered = current.lower()

        status = ''
        active_action = ''
        for line in current.splitlines():
            stripped = line.strip()
            if 'status=' in stripped and not status:
                for token in stripped.replace(',', ' ').split():
                    if token.startswith('status='):
                        status = token.split('=', 1)[1].upper().strip("'\"")
                    if token.startswith('action='):
                        active_action = token.split('=', 1)[1].strip("'\"")
            if stripped.startswith('## Триггер'):
                continue

        if status == 'ERROR' or any(w in lowered for w in _ABORT_KEYWORDS):
            return json.dumps({
                'mode': 'idle',
                'reason': 'mock: halt on error',
            })
        if status == 'WARN' and active_action == 'MapfPlan':
            return json.dumps({
                'mode': 'mapf',
                'robot_ids': [0, 1, 2],
                'goals': [[10.0, 10.0], [11.0, 10.0], [12.0, 10.0]],
                'reason': 'mock: replan via MAPF on WARN',
            })
        if status == 'WARN' and active_action in ('SetFormation', 'DisableFormation'):
            return json.dumps({
                'mode': 'formation',
                'formation_id': 'line',
                'leader_ns': 'robot_1',
                'follower_ns': ['robot_2', 'robot_3'],
                'offsets_x': [-1.0, -2.0],
                'offsets_y': [0.0, 0.0],
                'reason': 'mock: retry formation with alternate leader',
            })
        return json.dumps({
            'mode': 'idle',
            'reason': 'mock: fallback',
        })

    @staticmethod
    def _extract_current_situation(prompt):
        marker = '# Текущая ситуация'
        idx = prompt.rfind(marker)
        return prompt if idx == -1 else prompt[idx:]

    # ---- http (OpenAI-compatible) ----------------------------------------

    async def _generate_http(self, prompt):
        # aiohttp is an optional dep — only needed in http mode.
        import aiohttp  # noqa: WPS433

        payload = {
            'model': self.model,
            'prompt': prompt,
            'max_tokens': self.max_tokens,
            'temperature': self.temperature,
            'stop': ['\n## ', '\n# ', '</s>'],
        }
        async with aiohttp.ClientSession() as session:
            async with session.post(self.endpoint, json=payload) as resp:
                resp.raise_for_status()
                data = await resp.json()

        # OpenAI-style: {"choices": [{"text": "..."}]}
        if isinstance(data, dict) and 'choices' in data and data['choices']:
            choice = data['choices'][0]
            if 'text' in choice:
                return choice['text']
            if 'message' in choice and 'content' in choice['message']:
                return choice['message']['content']
        # Fallbacks for bespoke servers
        for key in ('text', 'output', 'generated_text'):
            if isinstance(data, dict) and key in data:
                return data[key]
        raise RuntimeError(f'unrecognized llm http response shape: {data!r}')

    # ---- local (transformers) --------------------------------------------

    async def _generate_local(self, prompt):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, self._generate_local_sync, prompt)

    def _generate_local_sync(self, prompt):
        if self._local_pipe is None:
            try:
                from transformers import pipeline  # noqa: WPS433
            except ImportError as exc:
                raise RuntimeError(
                    'llm_mode=local requires the `transformers` package — install it or '
                    'switch to llm_mode=mock / http'
                ) from exc
            self._local_pipe = pipeline(
                'text-generation',
                model=self.model or 'sshleifer/tiny-gpt2',
            )

        outputs = self._local_pipe(
            prompt,
            max_new_tokens=self.max_tokens,
            do_sample=self.temperature > 0.0,
            temperature=max(self.temperature, 1e-3),
            return_full_text=False,
        )
        return outputs[0]['generated_text']
