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


def _spawn_controllers(context, *args, **kwargs):
    num_robots   = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    kp           = float(LaunchConfiguration("kp").perform(context))
    kd           = float(LaunchConfiguration("kd").perform(context))
    max_v        = float(LaunchConfiguration("max_v").perform(context))
    max_omega    = float(LaunchConfiguration("max_omega").perform(context))
    control_hz   = float(LaunchConfiguration("control_hz").perform(context))

    return [
        Node(
            package="iros_llm_swarm_robot",
            executable="motion_controller_node",
            name=f"motion_controller_{i}",
            output="screen",
            parameters=[{
                "robot_id":               i,
                "use_sim_time":           use_sim_time,
                "schedule_tolerance_sec": 0.5,
                "kp":                     kp,
                "kd":                     kd,
                "max_v":                  max_v,
                "max_omega":              max_omega,
                "control_hz":             control_hz,
            }],
        )
        for i in range(num_robots)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots",   default_value="20"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("kp",           default_value="1.2"),
        DeclareLaunchArgument("kd",           default_value="0.3"),
        DeclareLaunchArgument("max_v",        default_value="0.5"),
        DeclareLaunchArgument("max_omega",    default_value="1.0"),
        DeclareLaunchArgument("control_hz",   default_value="20.0"),

        OpaqueFunction(function=_spawn_controllers),
    ])
