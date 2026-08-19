"""Both wire dialects for recorded tool calls, and neither accepting the other.

Run 20260819_073009 sent the Ollama shape to OpenRouter and the provider
answered with pydantic errors naming the two missing pieces: `id` and a
string `arguments`. It only surfaced there because the provider whitelist
rotates and lenient providers had been serving the earlier runs.

    python3 -m pytest test/test_tool_messages.py
"""

import json

from iros_llm_orchestrator.common.tool_definitions import (
    build_tool_result_message,
    build_tool_use_assistant_message,
    parse_openai_tool_calls,
)


def _calls():
    return [
        {'name': 'find_group_placement_in_room',
         'arguments': {'room': 'cafeteria', 'min_clearance_m': 2},
         'call_id': 'call_abc'},
        {'name': 'get_robot_position',
         'arguments': {'robot_id': 0},
         'call_id': 'call_def'},
    ]


# --- OpenAI dialect --------------------------------------------------------

def test_openai_call_carries_an_id_and_a_type():
    msg = build_tool_use_assistant_message(_calls(), 'openai')
    assert [tc['id'] for tc in msg['tool_calls']] == ['call_abc', 'call_def']
    assert all(tc['type'] == 'function' for tc in msg['tool_calls'])


def test_openai_arguments_are_a_json_string_not_a_dict():
    """The exact complaint from the provider: `Input should be a valid string`."""
    msg = build_tool_use_assistant_message(_calls(), 'openai')
    raw = msg['tool_calls'][0]['function']['arguments']
    assert isinstance(raw, str)
    assert json.loads(raw) == {'room': 'cafeteria', 'min_clearance_m': 2}


def test_openai_result_is_addressed_to_the_call_it_answers():
    msg = build_tool_result_message('call_abc', '{"ok": true}', 'openai')
    assert msg == {'role': 'tool', 'tool_call_id': 'call_abc',
                   'content': '{"ok": true}'}


def test_openai_synthesises_an_id_when_the_model_omitted_one():
    calls = [{'name': 'get_robot_position', 'arguments': {}},
             {'name': 'get_robot_position', 'arguments': {}}]
    msg = build_tool_use_assistant_message(calls, 'openai')
    ids = [tc['id'] for tc in msg['tool_calls']]
    assert len(set(ids)) == 2, 'two calls to one tool must not share an id'


# --- Ollama dialect --------------------------------------------------------

def test_ollama_keeps_arguments_as_a_dict():
    """Re-serialising breaks Ollama: "Value looks like object, but can't
    find closing '}'"."""
    msg = build_tool_use_assistant_message(_calls(), 'ollama')
    assert msg['tool_calls'][0]['function']['arguments'] == {
        'room': 'cafeteria', 'min_clearance_m': 2}


def test_ollama_omits_the_id_and_type_wrappers():
    msg = build_tool_use_assistant_message(_calls(), 'ollama')
    for tc in msg['tool_calls']:
        assert set(tc) == {'function'}


def test_ollama_result_has_no_tool_call_id():
    msg = build_tool_result_message('call_abc', '{}', 'ollama')
    assert msg == {'role': 'tool', 'content': '{}'}


# --- Round trip ------------------------------------------------------------

def test_an_openai_response_survives_being_rebuilt_into_a_request():
    """Parse a provider response, then echo it back: ids must line up."""
    choice = {'message': {'tool_calls': [
        {'id': 'call_xyz', 'type': 'function',
         'function': {'name': 'check_occupancy',
                      'arguments': '{"x": 2.7, "y": 10.1}'}},
    ]}}
    parsed = parse_openai_tool_calls(choice)
    assert parsed[0]['call_id'] == 'call_xyz'
    echoed = build_tool_use_assistant_message(parsed, 'openai')
    result = build_tool_result_message(parsed[0]['call_id'], '{}', 'openai')
    assert echoed['tool_calls'][0]['id'] == result['tool_call_id']


def test_parser_gives_repeated_tools_distinct_ids_when_provider_omits_them():
    choice = {'message': {'tool_calls': [
        {'function': {'name': 'get_robot_position', 'arguments': '{"robot_id": 0}'}},
        {'function': {'name': 'get_robot_position', 'arguments': '{"robot_id": 1}'}},
    ]}}
    parsed = parse_openai_tool_calls(choice)
    assert parsed[0]['call_id'] != parsed[1]['call_id']


def test_the_default_dialect_is_openai():
    assert (build_tool_use_assistant_message(_calls())
            == build_tool_use_assistant_message(_calls(), 'openai'))
