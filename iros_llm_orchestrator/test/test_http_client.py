"""Unit tests for the OpenAI-compatible HTTP backend."""

import asyncio

from iros_llm_orchestrator.web.http_client import HttpClient


def _collect(async_iterable):
    async def run():
        out = []
        async for item in async_iterable:
            out.append(item)
        return out
    return asyncio.run(run())


def test_headers_include_explicit_api_key():
    client = HttpClient(api_key='placeholder-value')

    assert client._headers()['Authorization'] == 'Bearer placeholder-value'


def test_headers_read_api_key_from_env(monkeypatch):
    monkeypatch.setenv('LLM_API_KEY', 'placeholder-env-value')
    client = HttpClient(api_key='')

    assert client.api_key == 'placeholder-env-value'
    assert client._headers()['Authorization'] == 'Bearer placeholder-env-value'


def test_chat_routing_for_messages_and_api_key():
    client = HttpClient(
        endpoint='http://localhost:8000/v1/completions',
        api_key='placeholder-value',
    )

    assert client._should_use_chat([{'role': 'user', 'content': 'hi'}])
    assert client._should_use_chat('hi')
    assert client._chat_endpoint() == 'http://localhost:8000/v1/chat/completions'


def test_completions_route_preserved_for_string_without_key():
    client = HttpClient(
        endpoint='http://localhost:8000/v1/completions',
        api_key='',
    )

    assert not client._should_use_chat('flat prompt')


def test_force_chat_false_overrides_api_key_for_string_prompt():
    client = HttpClient(
        endpoint='http://localhost:8000/v1/completions',
        api_key='placeholder-value',
        force_chat=False,
    )

    assert not client._should_use_chat('flat prompt')


def test_sse_line_parses_delta_content():
    line = 'data: {"choices":[{"delta":{"content":"hello"}}]}'

    assert HttpClient._extract_sse_line(line) == 'hello'


def test_sse_line_parses_done_marker():
    assert HttpClient._extract_sse_line('data: [DONE]') == '[DONE]'


def test_extract_supports_common_response_shapes():
    assert HttpClient._extract({'choices': [{'message': {'content': 'chat'}}]}) == 'chat'
    assert HttpClient._extract({'choices': [{'text': 'completion'}]}) == 'completion'
    assert HttpClient._extract({'generated_text': 'generated'}) == 'generated'


def test_error_sanitizer_redacts_api_key():
    client = HttpClient(api_key='placeholder-value')

    assert 'placeholder-value' not in client._sanitize_body('bad placeholder-value bad')
    assert '[redacted]' in client._sanitize_body('bad placeholder-value bad')


def test_stream_falls_back_to_generate_when_sse_unavailable():
    class FallbackClient(HttpClient):
        async def _stream_chat(self, messages):
            if False:
                yield ''
            raise RuntimeError('stream not supported')

        async def generate(self, prompt, prompt_kind='decision'):
            return '{"reply":"ok","plan":{"type":"idle","reason":"x"}}'

    client = FallbackClient()

    assert _collect(client.stream([{'role': 'user', 'content': 'hi'}])) == [
        '{"reply":"ok","plan":{"type":"idle","reason":"x"}}'
    ]
