"""Launch 20 robots in Stage warehouse simulation."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import yaml
from launch.actions import OpaqueFunction


def setup(context, *args, **kwargs):
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios_file = LaunchConfiguration('scenarios_file').perform(context)
    data = {}
    try:
        with open(scenarios_file, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}
    scenario_data = (data.get('scenarios') or {}).get(scenario) or {}
    world_rel = scenario_data.get('world')

    if world_rel:
        world_file = PathJoinSubstitution([FindPackageShare("iros_llm_swarm_simulation_lite"), "stage_sim", world_rel])
    else:
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

    return [stage_node]

def generate_launch_description():
    
    pkg_dir =  FindPackageShare("iros_llm_swarm_simulation_lite")

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='cave',
        description='Scenario name'
    )

    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([FindPackageShare('iros_llm_swarm_simulation_lite'), 'scenario', 'common_scenarios.yaml']),
        description='YAML with scenarios'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution(
            [
                pkg_dir, "stage_sim", "cave.world",
            ]
        ))
    
    return LaunchDescription([
        scenario_arg,
        scenarios_file_arg,
        world_file_arg,
        LogInfo(msg='Launching Stage simulator...'),
        OpaqueFunction(function=setup),
    ])