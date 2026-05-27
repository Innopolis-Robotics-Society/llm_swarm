"""Unit tests for endpoint-only LLM launch resolution."""

import ast
from pathlib import Path
import unittest


def _load_resolver():
    """Load only the pure resolver code from the ROS launch file.

    The local test environment may not have the ROS ``launch`` package
    installed, so importing the whole launch module would fail before the
    resolver tests can run.
    """
    package_root = Path(__file__).resolve().parents[1]
    launch_path = package_root / 'launch' / 'orchestrator.launch.py'
    tree = ast.parse(launch_path.read_text())
    wanted = {
        'LOCAL_OLLAMA_ENDPOINT',
        'DEFAULT_OLLAMA_MODEL',
        '_KNOWN_LLM_ENDPOINT_PROFILES',
        '_KNOWN_LLM_ENDPOINT_DISPLAY',
        '_llm_profile',
        '_known_endpoint_lines',
        '_unknown_openai_endpoint_error',
        'resolve_llm_endpoint',
    }

    body = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in wanted:
            body.append(node)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    body.append(node)
                    break

    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, str(launch_path), 'exec'), namespace)
    return namespace['resolve_llm_endpoint']


resolve_llm_endpoint = _load_resolver()


class LlmEndpointResolverTest(unittest.TestCase):
    def test_empty_string_resolves_to_local_ollama(self):
        profile = resolve_llm_endpoint('')

        self.assertEqual(profile['profile'], 'local-ollama')
        self.assertEqual(profile['llm_mode'], 'ollama')
        self.assertEqual(
            profile['llm_endpoint'], 'http://localhost:11434/api/chat')
        self.assertEqual(profile['llm_model'], 'mistral-small3.1')
        self.assertTrue(profile['llm_force_chat'])
        self.assertFalse(profile['llm_enable_stop'])

    def test_localhost_ollama_endpoint_resolves_to_local_ollama(self):
        profile = resolve_llm_endpoint('http://localhost:11434/api/chat')

        self.assertEqual(profile['profile'], 'local-ollama')
        self.assertEqual(profile['llm_mode'], 'ollama')
        self.assertEqual(
            profile['llm_endpoint'], 'http://localhost:11434/api/chat')
        self.assertEqual(profile['llm_model'], 'mistral-small3.1')

    def test_loopback_ollama_endpoint_resolves_to_local_ollama(self):
        profile = resolve_llm_endpoint('http://127.0.0.1:11434/api/chat')

        self.assertEqual(profile['profile'], 'local-ollama')
        self.assertEqual(profile['llm_mode'], 'ollama')
        self.assertEqual(
            profile['llm_endpoint'], 'http://localhost:11434/api/chat')
        self.assertEqual(profile['llm_model'], 'mistral-small3.1')

    def test_qwen32b_endpoint_resolves_to_http_profile(self):
        endpoint = 'http://10.100.11.191:8000/v1/chat/completions'

        profile = resolve_llm_endpoint(endpoint)

        self.assertEqual(profile['profile'], 'team-qwen32b')
        self.assertEqual(profile['llm_mode'], 'http')
        self.assertEqual(profile['llm_endpoint'], endpoint)
        self.assertEqual(profile['llm_model'], 'qwen32b')

    def test_qwen72b_endpoint_resolves_to_http_profile(self):
        endpoint = 'http://10.100.11.191:8001/v1/chat/completions'

        profile = resolve_llm_endpoint(endpoint)

        self.assertEqual(profile['profile'], 'team-qwen72b')
        self.assertEqual(profile['llm_mode'], 'http')
        self.assertEqual(profile['llm_endpoint'], endpoint)
        self.assertEqual(profile['llm_model'], 'qwen72b')

    def test_groq_endpoint_resolves_to_http_profile(self):
        endpoint = 'https://api.groq.com/openai/v1/chat/completions'

        profile = resolve_llm_endpoint(endpoint)

        self.assertEqual(profile['profile'], 'groq-llama70b')
        self.assertEqual(profile['llm_mode'], 'http')
        self.assertEqual(profile['llm_endpoint'], endpoint)
        self.assertEqual(profile['llm_model'], 'llama-3.3-70b-versatile')

    def test_unknown_api_chat_endpoint_resolves_to_custom_ollama(self):
        endpoint = 'http://10.0.0.23:11434/api/chat'

        profile = resolve_llm_endpoint(endpoint)

        self.assertEqual(profile['profile'], 'custom-ollama')
        self.assertEqual(profile['llm_mode'], 'ollama')
        self.assertEqual(profile['llm_endpoint'], endpoint)
        self.assertEqual(profile['llm_model'], 'mistral-small3.1')

    def test_unknown_chat_completions_endpoint_raises_clear_error(self):
        endpoint = 'http://example.com:8000/v1/chat/completions'

        with self.assertRaises(ValueError) as cm:
            resolve_llm_endpoint(endpoint)

        message = str(cm.exception)
        self.assertIn('Unknown OpenAI-compatible llm_endpoint', message)
        self.assertIn(endpoint, message)
        self.assertIn(
            'Add this endpoint to resolve_llm_endpoint() with its model name.',
            message,
        )
        self.assertIn(
            'http://10.100.11.191:8000/v1/chat/completions', message)
        self.assertIn(
            'https://api.groq.com/openai/v1/chat/completions', message)


if __name__ == '__main__':
    unittest.main()
