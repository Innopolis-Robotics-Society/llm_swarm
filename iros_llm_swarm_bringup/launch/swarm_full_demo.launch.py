"""Self-contained full-demo launch.

Composes stage simulator + per-robot Nav2 + chosen MAPF planner
(LNS2 or PBS) + optional formation control + LLM orchestrator + BT runner.
Driven by a single `scenario` arg and a `planner` switch. This file does
NOT chain through `swarm_lns_formation.launch.py` / `swarm_lns.launch.py`
etc.; it includes the foundational launches directly so this file is the
demo's source of truth.

Launch order (sim seconds):
  t=0   Stage simulator + RViz
  t=1   Per-robot Nav2 stack (map_server + controller per robot)
  t=10  Chosen MAPF planner
          - lns: mapf_lns2 (planner + path_followers bundled)
          - pbs: mapf (PBS planner) + motion_controllers (per-robot drivers)
  t=12  Formation manager + monitor (if enable_formation:=true)
  t=18  LLM orchestrator (decision/passive/chat/execute servers)
  t=20  BT runner

Examples:
  ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py
  ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py scenario:=warehouse_2 planner:=pbs
  ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py enable_formation:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------ launch args
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='amongus',
        choices=['cave', 'large_cave', 'warehouse_2', 'warehouse_4', 'amongus'],
        description='Scenario name from common_scenarios.yaml. Drives world, '
                    'map, initial poses, and LLM map description.',
    )
    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'scenario', 'common_scenarios.yaml',
        ]),
        description='YAML with scenarios.',
    )
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='pbs',
        choices=['lns', 'pbs'],
        description='MAPF planner: "lns" (LNS2, scalable) or "pbs" '
                    '(Priority-Based Search, more deterministic).',
    )
    num_robots_arg = DeclareLaunchArgument('num_robots', default_value='20')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    enable_passive_arg = DeclareLaunchArgument(
        'enable_passive_observer',
        default_value='false',
        choices=['true', 'false'],
        description='Enable proactive LLM channel 2 (passive observer).',
    )
    enable_formation_arg = DeclareLaunchArgument(
        'enable_formation',
        default_value='true',
        choices=['true', 'false'],
        description='Start formation_manager + formation_monitor nodes.',
    )
    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_bringup'),
            'rviz', 'swarm_20.rviz',
        ]),
    )

    scenario         = LaunchConfiguration('scenario')
    scenarios_file   = LaunchConfiguration('scenarios_file')
    planner          = LaunchConfiguration('planner')
    num_robots       = LaunchConfiguration('num_robots')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    enable_passive   = LaunchConfiguration('enable_passive_observer')
    enable_formation = LaunchConfiguration('enable_formation')
    rviz_cfg         = LaunchConfiguration('rviz_cfg')

    is_lns        = IfCondition(PythonExpression(["'", planner, "' == 'lns'"]))
    is_pbs        = IfCondition(PythonExpression(["'", planner, "' == 'pbs'"]))
    use_formation = IfCondition(enable_formation)

    # ----------------------------------------------------- foundational layers
    stage = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'launch', 'warehouse_swarm.launch.py',
        ])]),
        launch_arguments=[
            ('scenario', scenario),
            ('scenarios_file', scenarios_file),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_local_nav'),
            'launch', 'robot_local_nav.launch.py',
        ])]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('scenario', scenario),
            ('scenarios_file', scenarios_file),
        ],
    )

    lns_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_mapf_lns'),
            'launch', 'mapf_lns2.launch.py',
        ])]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('use_sim_time', use_sim_time),
        ],
        condition=is_lns,
    )

    pbs_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_mapf'),
            'launch', 'mapf.launch.py',
        ])]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('use_sim_time', use_sim_time),
        ],
        condition=is_pbs,
    )

    motion_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_robot'),
            'launch', 'motion_controllers.launch.py',
        ])]),
        launch_arguments=[
            ('num_robots', num_robots),
            ('use_sim_time', use_sim_time),
            ('controller_type', planner),
        ],
    )

    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('iros_llm_orchestrator'),
            'launch', 'orchestrator.launch.py',
        ])]),
        launch_arguments=[
            ('enable_passive_observer', enable_passive),
            ('scenario', scenario),
            ('scenarios_file', scenarios_file),
        ],
    )

    # ------------------------------------------------------------ inline nodes
    formation_manager = Node(
        package='iros_llm_swarm_formation',
        executable='formation_manager_node',
        name='formation_manager',
        output='screen',
        parameters=[{
            'config_file':       '',
            'auto_activate':     False,
            'footprint_padding': 0.2,
            'robot_radius':      0.3,
            'position_tolerance': 0.5,
        }],
        condition=use_formation,
    )

    formation_monitor = Node(
        package='iros_llm_swarm_formation',
        executable='formation_monitor_node',
        name='formation_monitor',
        output='screen',
        parameters=[{
            'monitor_hz':        10.0,
            'stable_thresh_m':    0.15,
            'degraded_thresh_m':  0.35,
            'odom_timeout_s':     1.0,
            'stuck_window_s':     3.0,
            'stuck_delta_m':      0.05,
            'use_sim_time':       use_sim_time,
        }],
        condition=use_formation,
    )

    bt_runner = Node(
        package='iros_llm_swarm_bt',
        executable='test_bt_runner',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg, '--ros-args', '--log-level', 'WARN'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )

    # ---------------------------------------------------------------- compose
    return LaunchDescription([
        scenario_arg,
        scenarios_file_arg,
        planner_arg,
        num_robots_arg,
        use_sim_time_arg,
        enable_passive_arg,
        enable_formation_arg,
        rviz_cfg_arg,

        # t=0
        stage,
        rviz,

        TimerAction(period=1.0, actions=[
            LogInfo(msg='Starting per-robot Nav2 stack...'),
            nav2,
        ]),

        TimerAction(period=10.0, actions=[
            LogInfo(msg=['Starting MAPF planner (', planner, ')...']),
            lns_planner,
            pbs_planner,
            motion_controllers,
        ]),

        TimerAction(period=12.0, actions=[
            LogInfo(msg='Starting formation manager + monitor...'),
            formation_manager,
            formation_monitor,
        ]),

        TimerAction(period=18.0, actions=[
            LogInfo(msg='Starting LLM orchestrator (decision/passive/chat/execute)...'),
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
