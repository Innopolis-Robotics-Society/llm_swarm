"""
formation.launch.py
===================
Launches the formation_manager_node and one formation_controller_node
per follower defined in the active formation.

Launch arguments
----------------
  formation_id   : str   — which formation from formations.yaml to activate
  config_file    : str   — path to formations.yaml
  kp             : float — PD proportional gain (default 1.2)
  kd             : float — PD derivative gain   (default 0.3)
  max_v          : float — max linear speed      (default 0.5)
  max_omega      : float — max angular speed     (default 1.0)
  control_hz     : float — controller update rate (default 20.0)

Example
-------
  ros2 launch iros_llm_swarm_formation formation.launch.py \\
      formation_id:=wedge config_file:=/path/to/formations.yaml
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_formation(config_file: str, formation_id: str):
    """Return (leader_ns, [(follower_ns, dx, dy), ...]) or raise."""
    with open(config_file, "r") as f:
        data = yaml.safe_load(f)
    for entry in data.get("formations", []):
        if str(entry["id"]) == formation_id:
            leader = str(entry["leader"])
            followers = [
                (str(fw["ns"]), float(fw["dx"]), float(fw["dy"]))
                for fw in entry.get("followers", [])
            ]
            return leader, followers
    raise ValueError(f"Formation '{formation_id}' not found in {config_file}")


def _generate_nodes(context, *args, **kwargs):
    formation_id = LaunchConfiguration("formation_id").perform(context)
    config_file  = LaunchConfiguration("config_file").perform(context)
    kp           = LaunchConfiguration("kp").perform(context)
    kd           = LaunchConfiguration("kd").perform(context)
    max_v        = LaunchConfiguration("max_v").perform(context)
    max_omega    = LaunchConfiguration("max_omega").perform(context)
    control_hz   = LaunchConfiguration("control_hz").perform(context)

    nodes = []

    # --- formation_manager_node -------------------------------------------
    nodes.append(Node(
        package="iros_llm_swarm_formation",
        executable="formation_manager_node",
        name="formation_manager",
        output="screen",
        parameters=[{
            "config_file":   config_file,
            "auto_activate": True,          # activate the loaded formation immediately
        }],
    ))

    # --- one formation_controller_node per follower -----------------------
    try:
        leader_ns, followers = _load_formation(config_file, formation_id)
    except Exception as e:
        print(f"[formation.launch] WARNING: {e} — no controller nodes spawned")
        return nodes

    for follower_ns, dx, dy in followers:
        nodes.append(Node(
            package="iros_llm_swarm_formation",
            executable="formation_controller_node",
            name=f"formation_ctrl_{follower_ns}",
            output="screen",
            parameters=[{
                "follower_ns":   follower_ns,
                "leader_ns":     leader_ns,
                "formation_id":  formation_id,
                "offset_x":      dx,
                "offset_y":      dy,
                "kp":            float(kp),
                "kd":            float(kd),
                "max_v":         float(max_v),
                "max_omega":     float(max_omega),
                "control_hz":    float(control_hz),
            }],
        ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("iros_llm_swarm_formation")
    default_cfg = os.path.join(pkg_share, "config", "formations.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("formation_id", default_value="wedge",
                              description="Formation to activate"),
        DeclareLaunchArgument("config_file",  default_value=default_cfg,
                              description="Path to formations.yaml"),
        DeclareLaunchArgument("kp",           default_value="1.2"),
        DeclareLaunchArgument("kd",           default_value="0.3"),
        DeclareLaunchArgument("max_v",        default_value="0.5"),
        DeclareLaunchArgument("max_omega",    default_value="1.0"),
        DeclareLaunchArgument("control_hz",   default_value="20.0"),

        OpaqueFunction(function=_generate_nodes),
    ])