from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iros_llm_swarm_bt',
            executable='bt_runner',
            output='screen'
        )
    ])
