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
        DeclareLaunchArgument(
            'default_robot_radius', default_value='0.22',
            description='Радиус робота по умолчанию если Nav2 ещё не опубликовал footprint'
        ),
        DeclareLaunchArgument(
            'replan_check_hz', default_value='2.0',
            description='Schedule monitor frequency (0 to disable)'
        ),
        DeclareLaunchArgument(
            'replan_threshold_m', default_value='1.0',
            description='Deviation from schedule to trigger replan (metres)'
        ),
        DeclareLaunchArgument(
            'replan_cooldown_sec', default_value='10.0',
            description='Cooldown between end of last replan and next replan (seconds)'
        ),
        DeclareLaunchArgument(
            'replan_predict_sec', default_value='-1.0',
            description='Prediction horizon for on-schedule robots (<0 = adaptive)'
        ),
        DeclareLaunchArgument(
            'replan_stop_mode', default_value='deviated',
            description='Which robots to stop before replan: none / deviated / all'
        ),
        DeclareLaunchArgument(
            'goal_reached_m', default_value='0.5',
            description='Distance to consider robot arrived at goal'
        ),
        DeclareLaunchArgument(
            'max_speed', default_value='0.5',
            description='Max robot speed (m/s), used with time_step_sec to compute movement reach'
        ),
        DeclareLaunchArgument(
            'urgency', default_value='1.0',
            description='Urgency coefficient: how much robots prefer moving over waiting. '
                        '0 = time is free (shortest path only), 1.0 = balanced, >1 = rush'
        ),
        DeclareLaunchArgument(
            'planner_type', default_value='euclidean',
            description='A* planner type: euclidean (N-connected) or classic (4-connected)'
        ),

        Node(
            package='iros_llm_swarm_mapf',
            executable='mapf_planner_node',
            name='mapf_planner',
            output='screen',
            parameters=[{
                'num_robots':             LaunchConfiguration('num_robots'),
                'time_step_sec':          LaunchConfiguration('time_step_sec'),
                'use_sim_time':           LaunchConfiguration('use_sim_time'),
                'map_topic':              '/map',
                'default_robot_radius':   LaunchConfiguration('default_robot_radius'),
                'replan_check_hz':        LaunchConfiguration('replan_check_hz'),
                'replan_threshold_m':     LaunchConfiguration('replan_threshold_m'),
                'replan_cooldown_sec':    LaunchConfiguration('replan_cooldown_sec'),
                'replan_predict_sec':     LaunchConfiguration('replan_predict_sec'),
                'replan_stop_mode':       LaunchConfiguration('replan_stop_mode'),
                'goal_reached_m':         LaunchConfiguration('goal_reached_m'),
                'max_speed':              LaunchConfiguration('max_speed'),
                'urgency':        LaunchConfiguration('urgency'),
                'planner_type':           LaunchConfiguration('planner_type'),
            }],
        ),
    ])