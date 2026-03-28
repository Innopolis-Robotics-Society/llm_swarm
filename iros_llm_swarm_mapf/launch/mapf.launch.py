from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots', default_value='20',
            description='Количество роботов'
        ),
        DeclareLaunchArgument(
            'time_step_sec', default_value='0.1',
            description='Секунд на один grid-шаг PBS пути'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'
        ),

        Node(
            package='iros_llm_swarm_mapf',
            executable='mapf_planner_node',
            name='mapf_planner',
            output='screen',
            parameters=[{
                'num_robots':    LaunchConfiguration('num_robots'),
                'time_step_sec': LaunchConfiguration('time_step_sec'),
                'use_sim_time':  LaunchConfiguration('use_sim_time'),
                'map_topic':     '/map',
                'goals_topic':   '/swarm_goals',
            }],
        ),
    ])