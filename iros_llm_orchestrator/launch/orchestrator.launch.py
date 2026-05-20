import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


OLLAMA_DEFAULT_ENDPOINT = 'http://localhost:11434/api/chat'


def infer_llm_mode(endpoint: str) -> tuple[str, str]:
    """Infer the internal LLM mode from the endpoint shape."""
    endpoint = endpoint.strip()
    if not endpoint:
        return 'ollama', OLLAMA_DEFAULT_ENDPOINT
    if '/api/chat' in endpoint:
        return 'ollama', endpoint
    if '/chat/completions' in endpoint:
        return 'http', endpoint
    raise RuntimeError(
        'Unsupported llm_endpoint. Supported endpoint forms:\n'
        f'  Ollama: {OLLAMA_DEFAULT_ENDPOINT}\n'
        '  OpenAI-compatible API: https://.../v1/chat/completions'
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


def _resolve_llm_overrides(context) -> tuple[dict, list]:
    """Return launch-time LLM overrides inferred from endpoint/model args."""
    deprecated_backend = LaunchConfiguration('llm_backend').perform(context).strip()
    endpoint = LaunchConfiguration('llm_endpoint').perform(context).strip()
    model = LaunchConfiguration('llm_model').perform(context).strip()
    api_key_env = LaunchConfiguration('llm_api_key_env').perform(context).strip()

    if not model:
        raise RuntimeError(
            'llm_model is required. Example:\n'
            '  local Ollama: llm_model:=mistral-small3.1\n'
            '  API: llm_endpoint:=https://.../chat/completions llm_model:=...'
        )

    mode, resolved_endpoint = infer_llm_mode(endpoint)
    notices = []
    if deprecated_backend:
        notices.append(LogInfo(
            msg='llm_backend is deprecated and ignored; use llm_endpoint + '
                f'llm_model instead. Inferred llm_mode={mode}.'
        ))

    overrides = {
        'llm_mode': mode,
        'llm_endpoint': resolved_endpoint,
        'llm_model': model,
        'llm_api_key': '',
        'llm_api_key_env': api_key_env or 'LLM_API_KEY',
    }
    return overrides, notices


def setup(context, *args, **kwargs):
    enable_passive = LaunchConfiguration('enable_passive_observer')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge')

    config = os.path.join(
        get_package_share_directory('iros_llm_orchestrator'),
        'config',
        'orchestrator.yaml',
    )

    map_name = _resolve_map_name(context)
    map_param = {'map_name': map_name}
    llm_overrides, llm_notices = _resolve_llm_overrides(context)

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
        *llm_notices,
        rosbridge_notice,
        rosbridge,
        Node(
            package='iros_llm_orchestrator',
            executable='decision_server',
            name='llm_decision_server',
            parameters=[config, llm_overrides, map_param],
            output='screen',
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
            ],
            output='screen',
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='chat_server',
            name='llm_chat_server',
            parameters=[config, llm_overrides, map_param],
            output='screen',
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='execute_server',
            name='llm_execute_server',
            parameters=[config, map_param],
            output='screen',
        ),
    ]


def generate_launch_description():
    llm_backend_arg = DeclareLaunchArgument(
        'llm_backend',
        default_value='',
        description='Deprecated and ignored. LLM mode is inferred from '
                    'llm_endpoint; use llm_endpoint + llm_model instead.',
    )

    llm_endpoint_arg = DeclareLaunchArgument(
        'llm_endpoint',
        default_value='',
        description='LLM endpoint. Empty selects local Ollama at '
                    f'{OLLAMA_DEFAULT_ENDPOINT}.',
    )

    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='',
        description='LLM model name. Required for Ollama and HTTP API modes.',
    )

    llm_api_key_env_arg = DeclareLaunchArgument(
        'llm_api_key_env',
        default_value='LLM_API_KEY',
        description='Environment variable name used by HTTP API clients.',
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
        default_value='true',
        description='Start rosbridge_server on port 9090. Required by the '
                    'mcp_readonly context provider; disable when an external '
                    'rosbridge is already running.',
        choices=['true', 'false'],
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

    return LaunchDescription([
        llm_backend_arg,
        llm_endpoint_arg,
        llm_model_arg,
        llm_api_key_env_arg,
        enable_passive_arg,
        enable_rosbridge_arg,
        scenario_arg,
        scenarios_file_arg,
        OpaqueFunction(function=setup),
    ])
