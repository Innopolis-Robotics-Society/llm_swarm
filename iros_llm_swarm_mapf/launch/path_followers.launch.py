from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_followers(context, *args, **kwargs):
    num          = int(LaunchConfiguration('num_robots').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    return [
        Node(
            package='iros_llm_swarm_mapf',
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
    return LaunchDescription([
        DeclareLaunchArgument('num_robots',   default_value='20'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_spawn_followers),
    ])