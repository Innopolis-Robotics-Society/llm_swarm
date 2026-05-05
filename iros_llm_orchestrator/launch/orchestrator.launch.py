import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_passive_arg = DeclareLaunchArgument(
        'enable_passive_observer',
        default_value='false',
        description='Enable proactive LLM observer (channel 2). '
                    'When false, only reactive /llm/decision (channel 1) is active.',
        choices=['true', 'false'],
    )
    enable_passive = LaunchConfiguration('enable_passive_observer')

    config = os.path.join(
        get_package_share_directory('iros_llm_orchestrator'),
        'config',
        'orchestrator.yaml',
    )

    return LaunchDescription([
        enable_passive_arg,

        Node(
            package='iros_llm_orchestrator',
            executable='decision_server',
            name='llm_decision_server',
            parameters=[config],
            output='screen',
        ),

        Node(
            package='iros_llm_orchestrator',
            executable='passive_observer',
            name='llm_passive_observer',
            parameters=[
                config,
                {'enabled': enable_passive},
            ],
            output='screen',
        ),
    ])