"""Launch 20 robots in Stage warehouse simulation."""

import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from iros_llm_swarm_simulation_lite.world_colors import parse_robot_colors


def _resolve_world_file(context) -> str:
    """Найти абсолютный путь к .world файлу из scenario или явного аргумента."""
    pkg_share = get_package_share_directory('iros_llm_swarm_simulation_lite')
    scenario = LaunchConfiguration('scenario').perform(context)
    scenarios_file = LaunchConfiguration('scenarios_file').perform(context)

    data = {}
    try:
        with open(scenarios_file, 'r') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        data = {}
    scenario_data = (data.get('scenarios') or {}).get(scenario) or {}
    world_rel = scenario_data.get('world')

    if world_rel:
        return os.path.join(pkg_share, 'stage_sim', world_rel)
    return LaunchConfiguration('world_file').perform(context)


def setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('iros_llm_swarm_simulation_lite')
    world_file = _resolve_world_file(context)

    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage_sim',
        output='screen',
        parameters=[{
            'world_file': world_file,
            'one_tf_tree': True,
            'enforce_prefixes': True,
            'use_sim_time': True,
            'is_depth_canonical': True,
            'base_watchdog_timeout': 0.2,
        }],
    )

    # Парсим цвета из .world и поднимаем robot_state_publisher на каждого робота
    # с собственным URDF (отличается только цветом тела).
    urdf_xacro = os.path.join(pkg_share, 'robot_description', 'swarm_robot.urdf.xacro')
    colors = parse_robot_colors(world_file)

    rsp_nodes = []
    for robot_name, rgba in colors.items():
        urdf_xml = xacro.process_file(
            urdf_xacro,
            mappings={'robot_name': robot_name, 'robot_color': rgba},
        ).toxml()

        rsp_nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='log',
            parameters=[{
                'robot_description': urdf_xml,
                'use_sim_time': True,
            }],
        ))

    return [stage_node, *rsp_nodes]


def generate_launch_description():
    pkg_dir = FindPackageShare("iros_llm_swarm_simulation_lite")

    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='cave',
        description='Scenario name'
    )

    scenarios_file_arg = DeclareLaunchArgument(
        'scenarios_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('iros_llm_swarm_simulation_lite'),
            'scenario', 'common_scenarios.yaml',
        ]),
        description='YAML with scenarios'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_dir, "stage_sim", "cave.world"]),
    )

    return LaunchDescription([
        scenario_arg,
        scenarios_file_arg,
        world_file_arg,
        LogInfo(msg='Launching Stage simulator...'),
        OpaqueFunction(function=setup),
    ])
