"""
swarm_simulation.launch.py
==========================
Launches N differential-drive robots in Ignition Gazebo (Fortress).

Parameters
----------
n_robots : int (default 3)
    Number of robots to spawn.

positions_yaml : str (default '')
    Path to a YAML file containing a list of [x, y, yaw] triples,
    one per robot.  If empty, robots are placed automatically in a
    square grid with 1.5 m spacing.

z_spawn : float (default '0.3')
    The height at which the robots will be added during automatic placement


Topic layout (per robot_N)
--------------------------
  /robot_N/cmd_vel       geometry_msgs/msg/Twist        - user publishes here
  /robot_N/odom          nav_msgs/msg/Odometry          - odometry output
  /robot_N/scan          sensor_msgs/msg/LaserScan      - 2-D LiDAR
  /robot_N/imu           sensor_msgs/msg/Imu            - IMU
  /robot_N/joint_states  sensor_msgs/msg/JointState     - joint state broadcaster
  /clock                 rosgraph_msgs/msg/Clock        - sim time

Architecture
------------
* One Ignition Gazebo process (world: swarm_world.sdf).
* One /clock bridge.
* Per robot:
    - controllers YAML written to /tmp/ at launch time
    - robot_state_publisher   in namespace robot/N
    - ros_gz_sim create       spawns the Ignition model
    - parameter_bridge        bridges LiDAR + IMU from IGN to ROS2
    - topic_tools relay       exposes /robot/N/cmd_vel and /robot/N/odom
    - controller spawners     for joint_state_broadcaster + diff_drive_controller
"""

import math
import os
import tempfile

import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PKG = 'iros_llm_swarm_simulation'

def _auto_grid_positions(n: int, spacing: float = 1.5):
    """Return [[x, y, yaw], ...] arranged in a square grid."""
    cols = math.ceil(math.sqrt(n))
    return [
        [float((i % cols) * spacing), float((i // cols) * spacing), 0.0]
        for i in range(n)
    ]


def _write_swarm_controllers_yaml(tmp_dir: str, n_robots: int) -> str:
    cfg = {}

    for i in range(n_robots):
        ros_ns = f'robot_{i}'
        model  = f'robot_{i}'

        cm  = f'/{ros_ns}/controller_manager'
        ddc = f'/{ros_ns}/diff_drive_controller'

        cfg[cm] = {
            'ros__parameters': {
                'update_rate': 50,
                'use_sim_time': True,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
                'diff_drive_controller': {
                    'type': 'diff_drive_controller/DiffDriveController',
                },
            }
        }

        cfg[ddc] = {
            'ros__parameters': {
                'left_wheel_names':  [f'{model}/left_wheel_joint'],
                'right_wheel_names': [f'{model}/right_wheel_joint'],
                'wheel_separation': 0.25,
                'wheel_radius': 0.05,

                # чтобы TF/odom не конфликтовали между роботами
                'odom_frame_id': f'{model}/odom',
                'base_frame_id': f'{model}/base_link',
                'enable_odom_tf': True,

                'publish_rate': 50.0,
                'use_stamped_vel': False,
                'use_sim_time': True,
            }
        }

    path = os.path.join(tmp_dir, 'controllers_swarm.yaml')
    with open(path, 'w') as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False)
    return path


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction — builds per-robot actions at launch time
# ─────────────────────────────────────────────────────────────────────────────

def _launch_swarm(context, *args, **kwargs):
    pkg_share       = get_package_share_directory(PKG)
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    n_robots       = int(LaunchConfiguration('n_robots').perform(context))
    positions_yaml = LaunchConfiguration('positions_yaml').perform(context)
    z_spawn        = float(LaunchConfiguration('z_spawn').perform(context))
    # ── Resolve spawn positions ──────────────────────────────────────────────
    if positions_yaml:
        with open(positions_yaml, 'r') as fh:
            positions = yaml.safe_load(fh)
        if len(positions) < n_robots:
            raise ValueError(
                f'positions_yaml has {len(positions)} entries '
                f'but n_robots={n_robots}'
            )
        positions = [list(p) for p in positions[:n_robots]]
    else:
        positions = _auto_grid_positions(n_robots)

    # ── One Ignition Gazebo instance ────────────────────────────────────────
    world_file = os.path.join(pkg_share, 'world', 'swarm_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── One /clock bridge ───────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── Shared temp directory for per-robot controller YAMLs ────────────────
    tmp_dir = tempfile.mkdtemp(prefix='swarm_ctrl_')

    xacro_file = os.path.join(pkg_share, 'robot_description', 'robot.urdf.xacro')

    # ── Per-robot actions ────────────────────────────────────────────────────
    robot_actions = []

    tmp_dir = tempfile.mkdtemp(prefix='swarm_ctrl_')
    swarm_yaml = _write_swarm_controllers_yaml(tmp_dir, n_robots)

    for i in range(n_robots):
        model_name    = f'robot_{i}'          # Ignition model name
        ros_namespace = f'robot_{i}'           # ROS2 namespace  (robot_0, robot_1…)
        x, y, yaw     = positions[i]

        # 2. Process xacro → URDF string
        robot_desc = xacro.process_file(
            xacro_file,
            mappings={
                'robot_name':       model_name,
                'ros_namespace':    ros_namespace,
                'controllers_yaml': swarm_yaml,
            },
        ).toxml()

        # 3. robot_state_publisher (namespaced under robot/N)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ros_namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
        )

        # 4. Spawn the robot in Ignition Gazebo
        #    Reads robot_description from the ROS2 topic published by rsp.
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{model_name}',
            arguments=[
                '-name',  model_name,
                '-topic', f'/{ros_namespace}/robot_description',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z_spawn),
                '-Y', str(yaw),
            ],
            output='screen',
        )

        # 5. Bridge: Ignition sensors → ROS2
        #    Direction syntax:  topic@ros_type[gz_type  means IGN→ROS2
        #
        #    Ignition topic names come from <topic> in the xacro:
        #      /{model_name}/scan_raw   (LaserScan)
        #      /{model_name}/imu_raw    (IMU)
        #
        #    Remapped to the canonical user-facing names:
        #      /robot/N/scan
        #      /robot/N/imu
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{model_name}',
            arguments=[
                f'/{model_name}/scan_raw'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{model_name}/imu_raw'
                '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            remappings=[
                (f'/{model_name}/scan_raw', f'/{ros_namespace}/scan'),
                (f'/{model_name}/imu_raw',  f'/{ros_namespace}/imu'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # 8. Controller spawners
        #    ign_ros2_control starts controller_manager at
        #    /{ros_namespace}/controller_manager inside Gazebo.
        #    We must pass --controller-manager explicitly so that spawners
        #    target the correct instance when multiple robots are running.
        jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=f'jsb_spawner_{model_name}',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', f'/{ros_namespace}/controller_manager',
                '--controller-manager-timeout', '60',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        ddc_spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=f'ddc_spawner_{model_name}',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', f'/{ros_namespace}/controller_manager',
                '--controller-manager-timeout', '60',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )

        # Stagger robot bringup: add a per-robot offset so that controller
        # manager instances don't all race at startup.
        base_delay  = 3.0               # wait for Gazebo to be ready
        robot_delay = base_delay + i * 5.0   # stagger each robot by 1 s
        ctrl_delay  = robot_delay + 8.0      # extra wait for controller_manager

        robot_group = [
            TimerAction(period=robot_delay, actions=[rsp, spawn, bridge]),
            TimerAction(period=ctrl_delay, actions=[jsb_spawner]),
            TimerAction(period=ctrl_delay + 1.0, actions=[ddc_spawner]),
        ]

        robot_actions.extend(robot_group)

    return [gazebo, clock_bridge] + robot_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots',
            default_value='5',
            description='Number of robots to spawn in the simulation.',
        ),
        DeclareLaunchArgument(
            'z_spawn',
            default_value='0.3',
            description='The height at which the robots will be added during automatic placement',
        ),
        DeclareLaunchArgument(
            'positions_yaml',
            default_value='',
            description=(
                'Absolute path to a YAML file with spawn positions: '
                '[[x0,y0,yaw0], [x1,y1,yaw1], ...].  '
                'If empty, robots are placed in an auto-grid (1.5 m spacing).'
            ),
        ),
        OpaqueFunction(function=_launch_swarm),
    ])
