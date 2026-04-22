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
#  t=10s  mapf_planner      ← wait for map_server to publish /map
#  t=12s  path_followers    ← wait for mapf_planner
#  t=12s  formation_manager ← starts alongside path_followers, loads formation from YAML
#                             and activates it immediately (auto_activate: true)

def generate_launch_description():

    # ------------------------------------------------------------------ args
    num_robots_arg = DeclareLaunchArgument('num_robots',     default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    time_step_arg = DeclareLaunchArgument(
        'time_step_sec', default_value='0.4',
        description='Seconds per grid step in PBS path')
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
    formations_cfg_arg = DeclareLaunchArgument(
        'formations_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_formation'),
            'config', 'formations_1.yaml',
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

    # ------------------------------------------------- MAPF planner (t=10s)
    mapf_planner = Node(
        package='iros_llm_swarm_mapf',
        executable='mapf_planner_node',
        name='mapf_planner',
        output='screen',
        parameters=[{
            'num_robots':           num_robots,
            'time_step_sec':        time_step_sec,
            'use_sim_time':         use_sim_time,
            'map_topic':            '/map',
            'default_robot_radius': 0.22,
        }],
    )

    # ----------------------------------------- path followers (t=12s)
    motion_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iros_llm_swarm_robot'),
                'launch', 'motion_controllers.launch.py',
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

    # ---------------------------------------------------------- RViz (t=0s)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    return LaunchDescription([
        # args
        num_robots_arg,
        use_sim_time_arg,
        time_step_arg,
        world_file_arg,
        rviz_cfg_arg,
        formations_cfg_arg,

        # t=0s
        stage_sim,
        rviz,

        # t=1s
        TimerAction(period=1.0, actions=[
            LogInfo(msg='Starting Nav2...'),
            local_nav2,
        ]),

        # t=10s
        TimerAction(period=10.0, actions=[
            LogInfo(msg='Starting MAPF planner...'),
            mapf_planner,
        ]),

        # t=12s — path followers and formation manager start together so that
        # robots are subscribed before the latched config message is published
        TimerAction(period=12.0, actions=[
            LogInfo(msg='Starting motion controllers and formation manager...'),
            motion_controllers,
            formation_manager,
        ]),
    ])