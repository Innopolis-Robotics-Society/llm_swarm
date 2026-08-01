#!/usr/bin/env python3
# Launch the LNS2 MAPF planner node.
#
# Usage:
#   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py
#   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py num_robots:=10

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('iros_llm_swarm_mapf_lns')

    params_file = LaunchConfiguration('params_file')
    log_level   = LaunchConfiguration('log_level')
    footprint_topic_template = LaunchConfiguration('footprint_topic_template')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'mapf_lns2.yaml']),
        description='YAML parameter file for mapf_lns2 node.'
    )

    log_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logger level (debug|info|warn|error).'
    )

    footprint_topic_arg = DeclareLaunchArgument(
        'footprint_topic_template',
        default_value='/robot_{id}/local_costmap/published_footprint',
        description="Per-robot footprint topic ('{id}' -> robot index) the "
                    'planner reads each agent radius from. Override to route '
                    'through the formation footprint proxy.'
    )

    lns2_node = Node(
        package='iros_llm_swarm_mapf_lns',
        executable='mapf_lns2_node',
        name='mapf_lns2',
        output='screen',
        parameters=[
            params_file,
            {'footprint_topic_template': footprint_topic_template},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        params_arg,
        log_arg,
        footprint_topic_arg,
        lns2_node,
    ])