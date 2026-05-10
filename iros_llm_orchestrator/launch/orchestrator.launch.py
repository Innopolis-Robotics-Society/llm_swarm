import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def _resolve_llm_overrides(context) -> dict:
    """Return launch-time LLM backend overrides.

    The YAML keeps the external HTTP default. This helper lets demos switch
    back to local Ollama with one launch argument instead of editing all three
    LLM node sections by hand.
    """
    backend = LaunchConfiguration('llm_backend').perform(context).strip().lower()
    endpoint = LaunchConfiguration('llm_endpoint').perform(context).strip()
    model = LaunchConfiguration('llm_model').perform(context).strip()

    overrides = {'llm_mode': backend}
    if backend == 'ollama':
        overrides.update({
            'llm_endpoint': endpoint or 'http://localhost:11434/api/chat',
            'llm_model': model or 'qwen2.5:14b',
            'llm_api_key': '',
            'llm_force_chat': True,
            'llm_enable_stop': False,
        })
    elif backend == 'http':
        if endpoint:
            overrides['llm_endpoint'] = endpoint
        if model:
            overrides['llm_model'] = model
    elif backend == 'local':
        if model:
            overrides['llm_model'] = model

    return overrides


def setup(context, *args, **kwargs):
    enable_passive = LaunchConfiguration('enable_passive_observer')

    config = os.path.join(
        get_package_share_directory('iros_llm_orchestrator'),
        'config',
        'orchestrator.yaml',
    )

    map_name = _resolve_map_name(context)
    map_param = {'map_name': map_name}
    llm_overrides = _resolve_llm_overrides(context)
    llm_env = {'LLM_API_KEY': os.environ.get('LLM_API_KEY', '')}

    return [
        Node(
            package='iros_llm_orchestrator',
            executable='decision_server',
            name='llm_decision_server',
            parameters=[config, llm_overrides, map_param],
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
            ],
            output='screen',
            additional_env=llm_env,
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='chat_server',
            name='llm_chat_server',
            parameters=[config, llm_overrides, map_param],
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
    ]


def generate_launch_description():
    llm_backend_arg = DeclareLaunchArgument(
        'llm_backend',
        default_value='http',
        description='LLM backend override for decision/passive/chat nodes. '
                    'Use "ollama" for local Ollama without editing YAML.',
        choices=['http', 'ollama', 'mock', 'local'],
    )

    llm_endpoint_arg = DeclareLaunchArgument(
        'llm_endpoint',
        default_value='',
        description='Optional LLM endpoint override. Empty keeps YAML for '
                    'http and uses the local Ollama default for ollama.',
    )

    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='',
        description='Optional LLM model override. Empty keeps YAML for http '
                    'and uses qwen2.5:14b for ollama.',
    )

    enable_passive_arg = DeclareLaunchArgument(
        'enable_passive_observer',
        default_value='false',
        description='Enable proactive LLM observer (channel 2). '
                    'When false, only reactive /llm/decision (channel 1) is active.',
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
        enable_passive_arg,
        scenario_arg,
        scenarios_file_arg,
        OpaqueFunction(function=setup),
    ])
