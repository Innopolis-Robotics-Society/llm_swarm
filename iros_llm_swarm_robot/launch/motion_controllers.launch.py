"""
motion_controllers.launch.py
=============================
Spawns one motion_controller_node per robot.

Launch arguments
----------------
  num_robots   : int    — number of robots  (default: 20)
  use_sim_time : bool   — use simulation clock (default: true)
  kp           : float  — PD proportional gain (default: 1.2)
  kd           : float  — PD derivative gain   (default: 0.3)
  max_v        : float  — max linear speed      (default: 0.5)
  max_omega    : float  — max angular speed     (default: 1.0)
  control_hz   : float  — PD loop rate          (default: 20.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _spawn_controllers(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))

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
                {"robot_id": i}
            ],
        )
        for i in range(num_robots)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots",   default_value="20"),

        OpaqueFunction(function=_spawn_controllers),
    ])
