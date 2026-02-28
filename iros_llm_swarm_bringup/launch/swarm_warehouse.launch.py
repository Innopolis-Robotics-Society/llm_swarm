from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("iros_llm_swarm_bringup"), "rviz", "swarm_20.rviz",
            ]
        ))

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("iros_llm_swarm_simulation_lite"), "stage_sim", "warehouse.world",
            ]
        ))

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value="20")

    rviz_cfg = LaunchConfiguration('rviz_cfg')
    num_robots = LaunchConfiguration('num_robots')
    world_file = LaunchConfiguration('world_file')

    stage_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("iros_llm_swarm_simulation_lite"), "launch", "warehouse_swarm.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ('world_file', world_file),

        ]
    )

    local_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("iros_llm_swarm_local_nav"), "launch", "robot_local_nav.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ('num_robots', num_robots),

        ]
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
        )

    return LaunchDescription([
        world_file_arg,
        num_robots_arg,
        rviz_cfg_arg,

        stage_sim,
        TimerAction(period=1.0, actions=[local_nav2]),
        rviz,
    ])