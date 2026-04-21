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
    num_robots_arg = DeclareLaunchArgument('num_robots',    default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    time_step_arg = DeclareLaunchArgument(
        'time_step_sec', default_value='0.4',
        description='Seconds per PBS grid step')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'stage_sim', 'warehouse_four.world',
        ]))
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_bringup'),
            'rviz', 'swarm_20.rviz',
        ]))
    formations_cfg_arg = DeclareLaunchArgument(
        'formations_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_formation'),
            'config', 'formations_2.yaml',
        ]),
        description='Path to formations.yaml')

    num_robots    = LaunchConfiguration('num_robots')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    time_step_sec = LaunchConfiguration('time_step_sec')
    world_file    = LaunchConfiguration('world_file')
    rviz_cfg      = LaunchConfiguration('rviz_cfg')
    formations_cfg = LaunchConfiguration('formations_cfg')

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

        # ----------------------------------------- formation manager (t=12s)
    formation_manager = Node(
        package='iros_llm_swarm_formation',
        executable='formation_manager_node',
        name='formation_manager',
        output='screen',
        parameters=[{
            'config_file':       formations_cfg,
            'auto_activate':     True,
            'footprint_padding': 0.2,
            'robot_radius':      0.3,
            'use_sim_time':      use_sim_time,
        }],
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
        num_robots_arg,
        use_sim_time_arg,
        time_step_arg,
        world_file_arg,
        rviz_cfg_arg,
        formations_cfg_arg,

        # launch in sequence
        stage_sim,
        TimerAction(period=1.0,  actions=[LogInfo(msg='Starting Nav2...'), local_nav2]),
        TimerAction(period=10.0, actions=[LogInfo(msg='Starting MAPF stack...'), mapf_planner]),
        formation_manager,
        rviz,
    ])
