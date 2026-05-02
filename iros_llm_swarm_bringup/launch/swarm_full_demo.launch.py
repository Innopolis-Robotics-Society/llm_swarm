from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument('num_robots', default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_passive_arg = DeclareLaunchArgument(
        'enable_passive_observer',
        default_value='false',
        description='Enable proactive LLM channel 2 (passive observer)',
        choices=['true', 'false'],
    )

    num_robots = LaunchConfiguration('num_robots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_passive = LaunchConfiguration('enable_passive_observer')

    swarm_mapf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_bringup'),
                'launch', 'swarm_mapf.launch.py',
            ])
        ]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('use_sim_time', use_sim_time),
        ],
    )

    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_orchestrator'),
                'launch', 'orchestrator.launch.py',
            ])
        ]),
        launch_arguments=[('enable_passive_observer', enable_passive)],
    )

    bt_runner = Node(
        package='iros_llm_swarm_bt',
        executable='test_bt_runner',
        output='screen',
    )

    return LaunchDescription([
        num_robots_arg,
        use_sim_time_arg,
        enable_passive_arg,

        swarm_mapf,
        TimerAction(period=18.0, actions=[
            LogInfo(msg='Starting LLM orchestrator (decision_server + passive_observer)...'),
            orchestrator,
        ]),
        TimerAction(period=20.0, actions=[
            LogInfo(msg='Starting test_bt_runner...'),
            bt_runner,
        ]),
        TimerAction(period=22.0, actions=[
            LogInfo(msg='==== Full demo ready. Send fleet command: '
                        'ros2 run iros_llm_swarm_bt fleet_cmd --scenario simple ===='),
        ]),
    ])
