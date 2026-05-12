from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# ros2 launch iros_llm_swarm_robot motion_controller.launch.py controller_type:=lns
# ros2 launch iros_llm_swarm_robot motion_controller.launch.py controller_type:=pbs

def _spawn_controllers(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    controller_type = LaunchConfiguration("controller_type").perform(context)

    executable_map = {
        "pbs": "pbs_motion_controller",
        "lns": "lns_motion_controller",
    }

    if controller_type not in executable_map:
        raise RuntimeError(
            f"Unknown controller_type: '{controller_type}'. "
            f"Available: {list(executable_map.keys())}"
        )

    executable_name = executable_map[controller_type]

    config_file = os.path.join(
        get_package_share_directory("iros_llm_swarm_robot"),
        "config",
        "motion_controller.yaml"
    )

    return [
        Node(
            package="iros_llm_swarm_robot",
            executable=executable_name,
            name=f"{controller_type}_motion_controller_{i}",
            output="screen",
            parameters=[
                config_file,
                {
                    "robot_id": i,
                    "path_frame": "map",
                    "use_sim_time": use_sim_time,
                }
            ],
        )
        for i in range(num_robots)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_robots", default_value="20"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        DeclareLaunchArgument(
            "controller_type",
            default_value="lns",
            description="Controller type: lns or pbs"
        ),

        OpaqueFunction(function=_spawn_controllers),
    ])