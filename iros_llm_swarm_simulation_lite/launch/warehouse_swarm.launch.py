"""Launch 20 robots in Stage warehouse simulation."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    
    pkg_dir =  FindPackageShare("iros_llm_swarm_simulation_lite")

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution(
            [
                pkg_dir, "stage_sim", "warehouse.world",
            ]
        ))

    world_file = LaunchConfiguration('world_file')
    
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage_sim',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'one_tf_tree': True,
            'enforce_prefixes': True,
            'use_sim_time': True,
            'is_depth_canonical': True,
            'base_watchdog_timeout': 0.2,
        }],
    )
    
    return LaunchDescription([
        world_file_arg,
        LogInfo(msg='Launching Stage simulator...'),
        stage_node,
    ])