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


def setup(context, *args, **kwargs):
    enable_passive = LaunchConfiguration('enable_passive_observer')

    config = os.path.join(
        get_package_share_directory('iros_llm_orchestrator'),
        'config',
        'orchestrator.yaml',
    )

    map_name = _resolve_map_name(context)
    map_param = {'map_name': map_name}
    llm_env = {'LLM_API_KEY': os.environ.get('LLM_API_KEY', '')}

    return [
        Node(
            package='iros_llm_orchestrator',
            executable='decision_server',
            name='llm_decision_server',
            parameters=[config, map_param],
            output='screen',
            additional_env=llm_env,
        ),
        Node(
            package='iros_llm_orchestrator',
            executable='passive_observer',
            name='llm_passive_observer',
            parameters=[
                config,
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
            parameters=[config, map_param],
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
        enable_passive_arg,
        scenario_arg,
        scenarios_file_arg,
        OpaqueFunction(function=setup),
    ])
