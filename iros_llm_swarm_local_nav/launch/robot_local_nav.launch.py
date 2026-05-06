import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import ReplaceString, RewrittenYaml

def _parse_initial_poses(s: str):
    # format: "x,y,yaw; x,y,yaw; ..."
    out = []
    s = (s or "").strip()
    if not s:
        return out
    parts = [p.strip() for p in s.split(';') if p.strip()]
    for p in parts:
        vals = [v.strip() for v in p.split(',')]
        if len(vals) != 3:
            continue
        try:
            out.append((float(vals[0]), float(vals[1]), float(vals[2])))
        except ValueError:
            continue
    return out


def _setup_robots(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    initial_poses_s = LaunchConfiguration('initial_poses').perform(context)
    nav2_params_file = LaunchConfiguration('nav2_params_file').perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios_path = LaunchConfiguration('scenarios_file').perform(context)

    if initial_poses_s == "None":
        data = {}
        try:
            with open(scenarios_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            data = {}

        scenario_data = (data.get('scenarios') or {}).get(scenario) or {}
        poses = scenario_data.get('initial_poses') or []

        map_rel = scenario_data.get('map')
        if map_rel:
            map_file = PathJoinSubstitution( [FindPackageShare("iros_llm_swarm_simulation_lite"), "stage_sim", map_rel])
        else:
            map_file = LaunchConfiguration('map_file')

        initial_poses_s = ';'.join([f'{x},{y},{yaw}' for (x, y, yaw) in poses])


    poses = _parse_initial_poses(initial_poses_s)
    if len(poses) < num_robots:
        poses = poses + [(0.0, 0.0, 0.0)] * (num_robots - len(poses))

    # общий /tf и /tf_static
    tf_remaps = [('/tf', '/tf'), ('/tf_static', '/tf_static')]

    actions = [
        # Map server global
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='log',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='log',
            parameters=[{'autostart': True, 'use_sim_time': True, 'node_names': ['map_server']}],
            arguments=['--ros-args', '--log-level', 'WARN'],
        )
    ]

    for i in range(num_robots):
        ns = f'robot_{i}'
        x, y, yaw = poses[i]

        # подставляем <robot_namespace> -> robot_i внутрь yaml
        params_with_ns = ReplaceString(
            source_file=nav2_params_file,
            replacements={'<robot_namespace>': ns},
        )

        configured = ParameterFile(
            RewrittenYaml(
                source_file=params_with_ns,
                root_key=ns,
                param_rewrites={},
                convert_types=True,
            ),
            allow_substs=True,
        )

        robot_group = GroupAction([
            PushRosNamespace(ns),

            # TO-DO: Требутся более агресивная фильтрация\иные подходы, из-за 19 других роботов локализация уплывает быстро
            # Node(
            #     package='nav2_amcl',
            #     executable='amcl',
            #     name='amcl',
            #     output='log',
            #     parameters=[
            #         configured,
            #         {
            #             'set_initial_pose': True,
            #             'initial_pose.x': x,
            #             'initial_pose.y': y,
            #             'initial_pose.yaw': yaw,
            #         }
            #     ],
            #     remappings=tf_remaps + [('/map', '/map')],
            # ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom',
                namespace=ns,
                output='log',
                arguments=[
                    '--frame-id', 'map',
                    '--child-frame-id', f'{ns}/odom',
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--ros-args', '--log-level', 'WARN',
                ],
                parameters=[{'use_sim_time': True}],
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='log',
                parameters=[configured],
                remappings=tf_remaps,
                arguments=['--ros-args', '--log-level', 'WARN'],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='log',
                parameters=[configured],
                remappings=tf_remaps,
                arguments=['--ros-args', '--log-level', 'WARN'],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_nav',
                output='log',
                parameters=[{
                    'autostart': True,
                    'use_sim_time': True,
                    'node_names': ['controller_server', 'behavior_server'],
                    #'node_names': ['amcl', 'controller_server', 'behavior_server'],
                }],
                arguments=['--ros-args', '--log-level', 'WARN'],
            ),
        ])

        actions.append(TimerAction(period=0.3 * i, actions=[robot_group]))

    return actions


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='amongus',
        description='Scenario name'
    )

    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([FindPackageShare('iros_llm_swarm_simulation_lite'), 'scenario', 'common_scenarios.yaml']),
        description='YAML with scenarios'
    )

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='20',
        description='Number of robots to start Nav2 for'
    )

    initial_poses_arg = DeclareLaunchArgument(
        'initial_poses',
        default_value="None",
        description='Semicolon-separated list of x,y,yaw tuples',
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("iros_llm_swarm_simulation_lite"), "stage_sim", "cave.yaml"]
        ),
        description='Full path to map yaml'
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("iros_llm_swarm_local_nav"), "config", "robot_nav2_params.yaml"]
        ),
        description='Full path to Nav2 params yaml'
    )

    ld = LaunchDescription([
        scenario_arg,
        scenarios_file_arg,
        num_robots_arg,
        initial_poses_arg,
        map_file_arg,
        nav2_params_file_arg,
    ])

    # роботы создаются через OpaqueFunction чтобы можно было прочитать LaunchConfiguration
    ld.add_action(OpaqueFunction(function=_setup_robots))

    return ld