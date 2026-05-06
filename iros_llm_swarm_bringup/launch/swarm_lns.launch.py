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
# Launch order:
#
#  t=0s   Stage simulator + RViz
#  t=1s   Nav2 (map_server + controller_server per robot)
#  t=10s  mapf_planner  -- wait for map_server to publish /map
#  t=12s  path_followers -- wait for mapf_planner
#
# After launch, send goals via service:
#   ros2 run iros_llm_swarm_mapf test_send_goals --goal-x 15.0 --goal-y 15.0

def generate_launch_description():

    # ------------------------------------------------------------------ args
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='amongus',
        description='Scenario name',
        choices=['cave', 'large_cave', 'warehouse_2', 'warehouse_4', 'amongus']
    )

    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([FindPackageShare('iros_llm_swarm_simulation_lite'), 'scenario', 'common_scenarios.yaml']),
        description='YAML with scenarios'
    )
    num_robots_arg = DeclareLaunchArgument('num_robots',    default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_bringup'),
            'rviz', 'swarm_20.rviz',
        ]))

    num_robots    = LaunchConfiguration('num_robots')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    rviz_cfg      = LaunchConfiguration('rviz_cfg')
    scenario = LaunchConfiguration('scenario')
    scenarios_file = LaunchConfiguration('scenarios_file')

    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ PathJoinSubstitution([FindPackageShare('iros_llm_swarm_simulation_lite'),'launch','warehouse_swarm.launch.py']) ]),
        launch_arguments=[
            ('scenario', scenario),
            ('scenarios_file', scenarios_file),
        ],
    )

    # --------------------------------------------------- Nav2 + map (t=1s)
    local_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_local_nav'),
                'launch', 'robot_local_nav.launch.py',
            ])
        ]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('scenario', scenario)],
    )

    # ------------------------------------------------- PBS planner (t=10s)
    # Wait 10 seconds for map_server to publish /map with transient_local
    # QoS -- mapf_planner will receive the map immediately on subscribe.
    mapf_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_mapf_lns'),
                'launch', 'mapf_lns2.launch.py',
            ])
        ]),
        launch_arguments=[
            ('num_robots',   num_robots),
            ('use_sim_time', use_sim_time),
        ],
    )

    # ------------------------------------------------------- RViz (t=0s)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg, '--ros-args', '--log-level', 'WARN'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    return LaunchDescription([
        # arguments
        scenario_arg,
        scenarios_file_arg,
        num_robots_arg,
        use_sim_time_arg,
        rviz_cfg_arg,

        # launch in sequence
        stage,
        TimerAction(period=1.0,  actions=[LogInfo(msg='Starting Nav2...'), local_nav2]),
        TimerAction(period=10.0, actions=[LogInfo(msg='Starting MAPF stack...'), mapf_planner]),
        rviz,
    ])
