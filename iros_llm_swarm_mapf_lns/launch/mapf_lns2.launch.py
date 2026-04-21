#!/usr/bin/env python3
# Launch the LNS2 MAPF planner node.
#
# Usage:
#   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py
#   ros2 launch iros_llm_swarm_mapf_lns mapf_lns2.launch.py num_robots:=10

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _spawn_followers(context, *args, **kwargs):
    num          = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    return [
        Node(
            package='iros_llm_swarm_mapf_lns',
            executable='path_follower_node',
            name=f'path_follower_{i}',
            output='screen',
            parameters=[{
                'robot_id':               i,
                'path_frame':             'map',
                'schedule_tolerance_sec': 5.0,
                'use_sim_time':           use_sim_time,
            }],
        )
        for i in range(num)
    ]

def generate_launch_description():

    num_robots_arg = DeclareLaunchArgument('num_robots',   default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    pkg_share = FindPackageShare('iros_llm_swarm_mapf_lns')

    params_file = LaunchConfiguration('params_file')
    log_level   = LaunchConfiguration('log_level')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'mapf_lns2.yaml']),
        description='YAML parameter file for mapf_lns2 node.')
    log_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logger level (debug|info|warn|error).')

    lns2_node = Node(
        package='iros_llm_swarm_mapf_lns',
        executable='mapf_lns2_node',
        name='mapf_lns2',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        params_arg,
        log_arg,
        lns2_node,
        OpaqueFunction(function=_spawn_followers),
        ])