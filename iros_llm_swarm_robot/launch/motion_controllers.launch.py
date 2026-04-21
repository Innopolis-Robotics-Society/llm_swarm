from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _spawn_controllers(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    config_file = os.path.join(
        get_package_share_directory("iros_llm_swarm_robot"),
        "config",
        "motion_controller.yaml"
    )

    return [
        Node(
            package="iros_llm_swarm_robot",
            executable="motion_controller_node",
            name=f"motion_controller_{i}",
            output="screen",
            parameters=[
                config_file,
                {"robot_id":     i,
                 'path_frame':   'map',
                 'use_sim_time': use_sim_time,
                }
            ],
        )
        for i in range(num_robots)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots",   default_value="20"),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        OpaqueFunction(function=_spawn_controllers),
    ])
