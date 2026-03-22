from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


# Порядок запуска:
#
#  t=0s   Stage симулятор
#  t=1s   Nav2 (map_server + controller_server на каждого робота)
#  t=8s   mapf_planner  ← ждём пока map_server поднимется и опубликует /map
#
# После запуска отправляй цели:
#   ros2 topic pub --once /swarm_goals std_msgs/msg/String \
#     "data: '{\"goals\": [{\"id\":0,\"sx\":2.0,\"sy\":2.0,\"gx\":15.0,\"gy\":15.0}, ...]}'"


def generate_launch_description():

    # ------------------------------------------------------------------ args
    num_robots_arg = DeclareLaunchArgument('num_robots',    default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    time_step_arg = DeclareLaunchArgument(
        'time_step_sec', default_value='0.4',
        description='Секунд на один grid-шаг PBS пути')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'stage_sim', 'warehouse.world',
        ]))
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_bringup'),
            'rviz', 'swarm_20.rviz',
        ]))

    num_robots    = LaunchConfiguration('num_robots')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    time_step_sec = LaunchConfiguration('time_step_sec')
    world_file    = LaunchConfiguration('world_file')
    rviz_cfg      = LaunchConfiguration('rviz_cfg')

    # ---------------------------------------------------------- Stage (t=0s)
    stage_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_simulation_lite'),
                'launch', 'warehouse_swarm.launch.py',
            ])
        ]),
        launch_arguments=[('world_file', world_file)],
    )

    # --------------------------------------------------- Nav2 + map (t=1s)
    local_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_local_nav'),
                'launch', 'robot_local_nav.launch.py',
            ])
        ]),
        launch_arguments=[('num_robots', num_robots)],
    )

    # ------------------------------------------------- PBS planner (t=8s)
    # Ждём 8 секунд чтобы map_server успел опубликовать /map
    # с QoS transient_local — mapf_planner получит карту сразу при подписке.
    mapf_planner = Node(
        package='iros_llm_swarm_mapf',
        executable='mapf_planner_node',
        name='mapf_planner',
        output='screen',
        parameters=[{
            'num_robots':    num_robots,
            'time_step_sec': time_step_sec,
            'use_sim_time':  use_sim_time,
            'map_topic':     '/map',
            'goals_topic':   '/swarm_goals',
        }],
    )

    # ------------------------------------------------------- RViz (t=0s)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    path_followers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_mapf'),
                'launch', 'path_followers.launch.py',
            ])
        ]),
        launch_arguments=[
            ('num_robots',   num_robots),
            ('use_sim_time', use_sim_time),
        ],
    )

    # ----------------------------------------- path followers (t=10s)
    path_followers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_mapf'),
                'launch', 'path_followers.launch.py',
            ])
        ]),
        launch_arguments=[
            ('num_robots',   num_robots),
            ('use_sim_time', use_sim_time),
        ],
    )

    return LaunchDescription([
        # аргументы
        num_robots_arg,
        use_sim_time_arg,
        time_step_arg,
        world_file_arg,
        rviz_cfg_arg,

        # запуск по порядку
        stage_sim,
        TimerAction(period=1.0, actions=[local_nav2]),   # ждём Stage
        TimerAction(period=8.0,  actions=[mapf_planner]),  # ждём map_server
        TimerAction(period=10.0, actions=[path_followers]), # ждём mapf_planner
        TimerAction(period=9.0, actions=[path_followers]), # ждём mapf_planner
        rviz,
    ])