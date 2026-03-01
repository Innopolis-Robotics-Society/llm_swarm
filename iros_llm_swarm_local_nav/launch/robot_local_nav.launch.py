import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import ReplaceString, RewrittenYaml


DEFAULT_INITIAL_POSES = [
    (2.0, 2.0, 0.0), (3.5, 2.0, 0.0), (2.0, 3.5, 0.0), (3.5, 3.5, 0.0),
    (2.0, 5.0, 0.0), (3.5, 5.0, 0.0), (2.0, 6.5, 0.0), (3.5, 6.5, 0.0),
    (2.0, 8.0, 0.0), (3.5, 8.0, 0.0),
    (26.0, 22.0, math.pi), (27.5, 22.0, math.pi), (26.0, 23.5, math.pi), (27.5, 23.5, math.pi),
    (26.0, 25.0, math.pi), (27.5, 25.0, math.pi), (26.0, 26.5, math.pi), (27.5, 26.5, math.pi),
    (26.0, 28.0, math.pi), (27.5, 28.0, math.pi),
]


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

    poses = _parse_initial_poses(initial_poses_s)
    if not poses:
        poses = DEFAULT_INITIAL_POSES[:]
    if len(poses) < num_robots:
        poses = poses + [(0.0, 0.0, 0.0)] * (num_robots - len(poses))

    # общий /tf и /tf_static
    tf_remaps = [('/tf', '/tf'), ('/tf_static', '/tf_static')]

    actions = []
    for i in range(num_robots):
        ns = f'robot_{i}'
        x, y, yaw = poses[i]

        # подставляем <robot_namespace> -> robot_i внутрь yaml
        params_with_ns = ReplaceString(
            source_file=nav2_params_file,
            replacements={'<robot_namespace>': ns},
        )

        # В Humble param_rewrites ОБЯЗАТЕЛЕН (может быть пустым)
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
                arguments=['0', '0', '0', '0', '0', '0',
                        'map', f'{ns}/odom'],
                parameters=[{'use_sim_time': True}],
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='log',
                parameters=[configured],
                remappings=tf_remaps,
                #arguments=['--ros-args', '--log-level', 'DEBUG'],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='log',
                parameters=[configured],
                remappings=tf_remaps,
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
            ),
        ])

        actions.append(TimerAction(period=0.3 * i, actions=[robot_group]))

    return actions


def generate_launch_description():
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='20',
        description='Number of robots to start Nav2 for'
    )

    initial_poses_arg = DeclareLaunchArgument(
        'initial_poses',
        default_value=';'.join([f'{x},{y},{yaw}' for (x, y, yaw) in DEFAULT_INITIAL_POSES]),
        description='Semicolon-separated list of x,y,yaw tuples'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare("iros_llm_swarm_simulation_lite"), "stage_sim", "warehouse.yaml"]
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

    map_file = LaunchConfiguration('map_file')

    ld = LaunchDescription([
        num_robots_arg,
        initial_poses_arg,
        map_file_arg,
        nav2_params_file_arg,
    ])

    # Map server global
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='log',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='log',
        parameters=[{'autostart': True, 'use_sim_time': True, 'node_names': ['map_server']}],
    ))

    # роботы создаются через OpaqueFunction чтобы можно было прочитать LaunchConfiguration
    ld.add_action(OpaqueFunction(function=_setup_robots))

    return ld