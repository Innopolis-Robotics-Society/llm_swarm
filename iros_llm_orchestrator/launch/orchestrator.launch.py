import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LOCAL_OLLAMA_ENDPOINT = 'http://localhost:11434/api/chat'
DEFAULT_OLLAMA_MODEL = 'qwen2.5:14b'


def _llm_profile(
    profile: str,
    llm_mode: str,
    llm_endpoint: str,
    llm_model: str,
) -> dict:
    return {
        'profile': profile,
        'llm_mode': llm_mode,
        'llm_endpoint': llm_endpoint,
        'llm_model': llm_model,
        'llm_force_chat': True,
        'llm_enable_stop': False,
    }


_KNOWN_LLM_ENDPOINT_PROFILES = {
    '': _llm_profile(
        'local-ollama', 'ollama', LOCAL_OLLAMA_ENDPOINT, DEFAULT_OLLAMA_MODEL),
    LOCAL_OLLAMA_ENDPOINT: _llm_profile(
        'local-ollama', 'ollama', LOCAL_OLLAMA_ENDPOINT, DEFAULT_OLLAMA_MODEL),
    'http://127.0.0.1:11434/api/chat': _llm_profile(
        'local-ollama', 'ollama', LOCAL_OLLAMA_ENDPOINT, DEFAULT_OLLAMA_MODEL),
    'http://10.100.11.182:8000/v1/chat/completions': _llm_profile(
        'team-qwen32b-aiagent01',
        'http',
        'http://10.100.11.182:8000/v1/chat/completions',
        'qwen32b',
    ),
    'http://10.100.11.191:8000/v1/chat/completions': _llm_profile(
        'team-qwen32b',
        'http',
        'http://10.100.11.191:8000/v1/chat/completions',
        'qwen32b',
    ),
    'http://10.100.11.191:8001/v1/chat/completions': _llm_profile(
        'team-qwen72b',
        'http',
        'http://10.100.11.191:8001/v1/chat/completions',
        'qwen72b',
    ),
    'https://api.groq.com/openai/v1/chat/completions': _llm_profile(
        'groq-llama70b',
        'http',
        'https://api.groq.com/openai/v1/chat/completions',
        'llama-3.3-70b-versatile',
    ),
    # E3 runs the campaign against a hosted model so the experiment does not
    # depend on the operator owning a GPU. The model named here is only a
    # fallback: every E3 run passes llm_model:= explicitly, and that override
    # is applied after this profile (see ablation_overrides below).
    #
    # OpenRouter is a router, not a model. Which provider serves the request
    # decides quantisation, latency and whether tool calling works at all, and
    # it changes minute to minute. Pin one with OPENROUTER_PROVIDER (read by
    # web/http_client.py) for anything whose numbers get compared.
    'https://openrouter.ai/api/v1/chat/completions': _llm_profile(
        'openrouter',
        'http',
        'https://openrouter.ai/api/v1/chat/completions',
        'qwen/qwen3.5-397b-a17b',
    ),
}

_KNOWN_LLM_ENDPOINT_DISPLAY = (
    LOCAL_OLLAMA_ENDPOINT,
    'http://10.100.11.182:8000/v1/chat/completions',
    'http://10.100.11.191:8000/v1/chat/completions',
    'http://10.100.11.191:8001/v1/chat/completions',
    'https://api.groq.com/openai/v1/chat/completions',
    'https://openrouter.ai/api/v1/chat/completions',
)


def _known_endpoint_lines() -> str:
    return '\n'.join(f'  {endpoint}' for endpoint in _KNOWN_LLM_ENDPOINT_DISPLAY)


def _unknown_openai_endpoint_error(endpoint: str) -> str:
    return (
        'Unknown OpenAI-compatible llm_endpoint:\n\n'
        f'  {endpoint}\n\n'
        'This launch accepts only one public LLM parameter, so the model name '
        'must be known from the endpoint profile.\n\n'
        'Add this endpoint to resolve_llm_endpoint() with its model name.\n\n'
        'Known endpoints:\n'
        f'{_known_endpoint_lines()}'
    )


def resolve_llm_endpoint(endpoint_spec: str) -> dict:
    """Resolve the single public LLM endpoint into internal node parameters."""
    endpoint = (endpoint_spec or '').strip()

    known_profile = _KNOWN_LLM_ENDPOINT_PROFILES.get(endpoint)
    if known_profile:
        return dict(known_profile)

    if '/api/chat' in endpoint:
        return _llm_profile(
            'custom-ollama', 'ollama', endpoint, DEFAULT_OLLAMA_MODEL)

    if '/chat/completions' in endpoint:
        raise ValueError(_unknown_openai_endpoint_error(endpoint))

    raise ValueError(
        'Unknown llm_endpoint:\n\n'
        f'  {endpoint}\n\n'
        'Use an Ollama /api/chat endpoint, or add an OpenAI-compatible '
        '/chat/completions endpoint to resolve_llm_endpoint() with its '
        'model name.\n\n'
        'Known endpoints:\n'
        f'{_known_endpoint_lines()}'
    )


def _resolve_map_name(context) -> str:
    """Read scenarios YAML, return the bare stem of the active scenario's
    map_description (e.g. ``warehouse.yaml`` -> ``warehouse``). Falls back to
    ``cave`` if the file or field is missing — same default as the Python
    nodes' declare_parameter calls."""
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios_file = LaunchConfiguration('scenarios_file').perform(context)
    data = {}
    try:
        with open(scenarios_file, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}
    scenario_data = (data.get('scenarios') or {}).get(scenario) or {}
    md = scenario_data.get('map_description') or 'cave.yaml'
    return os.path.splitext(md)[0]


def setup(context, *args, **kwargs):
    enable_passive = LaunchConfiguration('enable_passive_observer')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge')
    enable_llm_mapf_proxy = LaunchConfiguration('enable_llm_mapf_proxy')

    config = os.path.join(
        get_package_share_directory('iros_llm_orchestrator'),
        'config',
        'orchestrator.yaml',
    )

    map_name = _resolve_map_name(context)
    map_param = {'map_name': map_name}
    endpoint_spec = LaunchConfiguration('llm_endpoint').perform(context).strip()
    llm_profile = resolve_llm_endpoint(endpoint_spec)
    llm_overrides = {
        key: value for key, value in llm_profile.items() if key != 'profile'
    }
    llm_num_ctx = LaunchConfiguration('llm_num_ctx').perform(context).strip()
    if llm_num_ctx:
        llm_overrides['llm_num_ctx'] = int(llm_num_ctx)
    # Session recording: when session_dir is given, every channel's JSONL
    # lands under it instead of the per-channel default in ~/.ros, so one
    # operator run produces exactly one self-contained folder alongside the
    # rosbag. Empty (the default) keeps the historical paths untouched.
    session_dir = LaunchConfiguration('session_dir').perform(context).strip()
    def _ds(channel: str) -> list:
        if not session_dir:
            return []
        return [{'dataset_path': os.path.join(
            os.path.expanduser(session_dir), f'llm_{channel}')}]

    # ── E3 ablation factors ────────────────────────────────────────────
    # These are chat_server parameters and previously could only be changed by
    # editing orchestrator.yaml between runs — which left no trace in the
    # recording, so a folder of bags was indistinguishable by configuration.
    # Exposed as launch arguments and echoed into session.json by
    # scripts/record_session.sh. Empty string means "leave the YAML value".
    _ABLATION_BOOL_ARGS = (
        'remediation_enabled',
        'llm_repair_enabled',
        'llm_mission_supervision_enabled',
        'tool_calling_enabled',
        'structured_output_enabled',
    )
    ablation_overrides = {}
    for _name in _ABLATION_BOOL_ARGS:
        _raw = LaunchConfiguration(_name).perform(context).strip().lower()
        if _raw:
            if _raw not in ('true', 'false'):
                raise RuntimeError(
                    f"{_name} must be 'true', 'false' or empty, got {_raw!r}")
            ablation_overrides[_name] = (_raw == 'true')
    _model = LaunchConfiguration('llm_model').perform(context).strip()
    if _model:
        ablation_overrides['llm_model'] = _model
    if ablation_overrides:
        llm_overrides.update(ablation_overrides)

    ablation_notice = LogInfo(
        msg=('E3 ablation overrides: '
             + (', '.join(f'{k}={v}' for k, v in sorted(ablation_overrides.items()))
                if ablation_overrides else '(none — orchestrator.yaml as shipped)')))

    llm_env = {'LLM_API_KEY': os.environ.get('LLM_API_KEY', '')}
    llm_notice = LogInfo(
        msg=(
            'Resolved LLM endpoint: '
            f'profile={llm_profile["profile"]} '
            f'mode={llm_profile["llm_mode"]} '
            f'endpoint={llm_profile["llm_endpoint"]} '
            f'model={llm_profile["llm_model"]}'
        )
    )

    rosbridge_condition = IfCondition(enable_rosbridge)
    rosbridge_notice = LogInfo(
        msg='Starting rosbridge_server on port 9090 for MCP read-only context...',
        condition=rosbridge_condition,
    )
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server',
             'rosbridge_websocket_launch.xml'],
        output='screen',
        condition=rosbridge_condition,
    )

    return [
        ablation_notice,
        rosbridge_notice,
        rosbridge,
        llm_notice,
        Node(
            package='iros_llm_orchestrator',
            executable='decision_server',
            name='llm_decision_server',
            parameters=[config, llm_overrides, map_param, *_ds('decisions')],
            output='screen',
            additional_env=llm_env,
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='passive_observer',
            name='llm_passive_observer',
            parameters=[
                config,
                llm_overrides,
                {'enabled': enable_passive},
                map_param,
                *_ds('commands'),
            ],
            output='screen',
            additional_env=llm_env,
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='chat_server',
            name='llm_chat_server',
            parameters=[config, llm_overrides, map_param, *_ds('chat')],
            output='screen',
            additional_env=llm_env,
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='execute_server',
            name='llm_execute_server',
            parameters=[config, map_param],
            output='screen',
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='mapf_proxy',
            name='llm_mapf_proxy',
            parameters=[{
                'proxy_action_name': '/llm/swarm/set_goals_proxy',
                'target_action_name': '/swarm/set_goals',
                'decision_action_name': '/llm/decision',
            }],
            output='screen',
            condition=IfCondition(enable_llm_mapf_proxy),
        ),
    ]


def generate_launch_description():
    llm_endpoint_arg = DeclareLaunchArgument(
        'llm_endpoint',
        default_value='',
        description='Only public LLM selection parameter. Empty resolves to '
                    'local Ollama; known OpenAI-compatible endpoints resolve '
                    'their model names from the launch profile map.',
    )

    enable_passive_arg = DeclareLaunchArgument(
        'enable_passive_observer',
        default_value='false',
        description='Enable proactive LLM observer (channel 2). '
                    'When false, only reactive /llm/decision (channel 1) is active.',
        choices=['true', 'false'],
    )

    enable_rosbridge_arg = DeclareLaunchArgument(
        'enable_rosbridge',
        default_value='false',
        description='Start rosbridge_server on port 9090. Required by the '
                    'mcp_readonly context provider; disable when an external '
                    'rosbridge is already running.',
        choices=['true', 'false'],
    )

    enable_llm_mapf_proxy_arg = DeclareLaunchArgument(
        'enable_llm_mapf_proxy',
        default_value='true',
        description='Start the read-only LLM decision proxy for MAPF feedback. '
                    'BT runners must remap /swarm/set_goals to '
                    '/llm/swarm/set_goals_proxy to use it.',
        choices=['true', 'false'],
    )

    llm_num_ctx_arg = DeclareLaunchArgument(
        'llm_num_ctx',
        default_value='32768',
        description='Requested local context window for Ollama backends. For '
                    'OpenAI-compatible HTTP endpoints this is logged as a '
                    'budget hint only; set the server max_model_len separately.',
    )

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='amongus',
        description='Scenario name from common_scenarios.yaml; selects the '
                    'map_description used by chat/execute servers.',
        choices=['cave', 'large_cave', 'warehouse_2', 'warehouse_4', 'amongus'],
    )

    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'scenario', 'common_scenarios.yaml',
        ]),
        description='YAML with scenarios (must match the one used by swarm_lns).',
    )

    ablation_args = [
        DeclareLaunchArgument(
            name, default_value='',
            description=f"E3 ablation override for {name} ('true'/'false'; "
                        "empty keeps the orchestrator.yaml value).")
        for name in ('remediation_enabled', 'llm_repair_enabled',
                     'llm_mission_supervision_enabled', 'tool_calling_enabled',
                     'structured_output_enabled')
    ] + [DeclareLaunchArgument(
        'llm_model', default_value='',
        description='Override the model name (empty keeps orchestrator.yaml).')]

    session_dir_arg = DeclareLaunchArgument(
        'session_dir', default_value='',
        description='If set, all three channels write their JSONL under this '
                    'directory instead of ~/.ros/llm_*. Set by '
                    'scripts/record_session.sh.',
    )

    return LaunchDescription([
        *ablation_args,
        session_dir_arg,
        llm_endpoint_arg,
        enable_passive_arg,
        enable_rosbridge_arg,
        enable_llm_mapf_proxy_arg,
        llm_num_ctx_arg,
        scenario_arg,
        scenarios_file_arg,
        OpaqueFunction(function=setup),
    ])
