iros_llm_swarm_simulation
=========================

3D Ignition Gazebo (Fortress) simulator for the swarm. Builds robots
from a xacro-generated URDF, spawns them into an empty Gazebo world
through ``ros_gz_sim``, runs ``ros2_control`` with ``diff_drive_controller``
inside Gazebo, and bridges sensor data to ROS 2 via
``ros_gz_bridge``.

.. warning::

   **Preview only — not yet recommended for daily use.** The launch
   file boots up to 20 robots in principle, but stable end-to-end
   operation with the rest of the swarm stack (MAPF, formation, BT)
   is **not** guaranteed at this scale. The supported simulator is
   ``iros_llm_swarm_simulation_lite`` (2D Stage); use this package
   for individual experimentation with 3D physics or as the
   integration target for future work, not as the day-to-day
   simulator.

   Several aspects of the integration with the rest of the stack are
   currently incompatible by default — see the *Compatibility notes*
   section below.

.. contents::
   :local:
   :depth: 2

What's in the package
---------------------

* ``launch/swarm_simulation.launch.py`` — boots Gazebo and spawns N
  differential-drive robots. Default ``n_robots = 5`` (vs 20 in the
  Stage launch).
* ``robot_description/robot.urdf.xacro`` — the robot model.
* ``world/swarm_world.sdf`` — a deliberately empty world: physics,
  required Gazebo plugins, lighting, ground plane. Robots are
  **not** declared in the world; they are spawned dynamically by the
  launch file.

Architecture
------------

::

    Once globally:
      gazebo (one process, swarm_world.sdf)
      clock_bridge   /clock @ rosgraph_msgs ↔ gz.msgs.Clock

    Per robot (staggered, namespace robot_<id>):
      robot_state_publisher
      ros_gz_sim create        (spawn the URDF model in Gazebo)
      parameter_bridge         (LiDAR + IMU bridges)
      controller spawners      (joint_state_broadcaster + diff_drive_controller)

The ``ros2_control`` ``controller_manager`` is **not** spawned by the
launch as a separate node — it is hosted inside Gazebo by the
``gz_ros2_control-system`` plugin embedded in the URDF. The
controller spawners launched in ROS 2 connect to that controller
manager via the namespaced action server
``/robot_<id>/controller_manager``.

Per-robot xacro processing
~~~~~~~~~~~~~~~~~~~~~~~~~~

For each robot the launch:

#. Writes a per-fleet ``controllers_swarm.yaml`` to a fresh
   ``/tmp/swarm_ctrl_*/`` directory containing one
   ``controller_manager`` + one ``diff_drive_controller`` block per
   robot.
#. Calls ``xacro.process_file`` with three mappings:
   ``robot_name`` (Gazebo model name, ``robot_<id>``),
   ``ros_namespace`` (matching ROS 2 namespace), and
   ``controllers_yaml`` (path to the YAML written above).
#. Feeds the resulting URDF string to ``robot_state_publisher`` in
   the per-robot namespace.
#. Calls ``ros_gz_sim/create -topic /robot_<id>/robot_description``
   to materialise the model in Gazebo at the requested pose.
#. Starts a ``parameter_bridge`` that bridges the model's Ignition
   sensor topics to ROS 2 (see *Sensor bridges* below).
#. Starts two controller spawners against
   ``/robot_<id>/controller_manager``: the joint state broadcaster
   first, then the diff-drive controller a second later.

The xacro embeds ``gz_ros2_control::GazeboSimROS2ControlPlugin`` in
the URDF itself with ``<namespace>`` set to ``robot_<id>``. Gazebo
loads the plugin per spawned model and starts a controller manager
in that namespace, reading parameters from the ``controllers_yaml``
path passed in via xacro.

Staggering
~~~~~~~~~~

Robot bringup is heavily staggered to avoid the controller-manager
ABI race that occurs when many ros2_control instances start
simultaneously::

    base_delay  = 3.0                 # wait for Gazebo to be ready
    robot_delay = base_delay + i * 5  # 5 s between robots
    ctrl_delay  = robot_delay + 8     # extra wait for the per-robot
                                      # controller_manager to come up
    spawn jsb_spawner   at t = ctrl_delay
    spawn ddc_spawner   at t = ctrl_delay + 1

For ``n_robots = 5`` the last spawner fires at ``t = 3 + 4*5 + 8 + 1
= 32 s``. Scaling to 20 robots pushes the last spawner past
``t = 100 s``. The startup cost is the dominant compatibility issue
with the integrated bringup, which is tuned for the few-second
startup of the Stage simulator.

Robot model
-----------

Defined in ``robot_description/robot.urdf.xacro``:

* **Body:** 30 × 30 × 15 cm box, mass 5 kg, inertia computed from
  the box dimensions.
* **Drive:** two ``continuous`` revolute joints
  (``left_wheel_joint`` / ``right_wheel_joint``) with 5 cm radius,
  4 cm width, 25 cm separation. ``mu1 = mu2 = 1.0`` plus
  ``kp = 1e6 / kd = 100`` per Gazebo wheel for high traction.
* **Casters:** two passive spheres at the front and back of the
  body, 2.5 cm radius, ``mu1 = mu2 = 0`` — pure sliders, no
  steering geometry.
* **LiDAR:** GPU lidar mounted on top of the body. 360° FOV, 181
  samples, 0.10 – 10 m range, 0.01 m resolution, Gaussian noise
  ``stddev = 0.01``. Publishes to Ignition topic
  ``/<rname>/scan_raw`` at 10 Hz.
* **IMU:** at body centre. 100 Hz, Gaussian noise on angular
  velocity (``stddev = 0.009``) and linear acceleration
  (``stddev = 0.021``). Publishes to ``/<rname>/imu_raw``.
* **TF root:** ``${rname}/base_footprint`` (zero-inertia ground
  anchor) → ``${rname}/base_link`` (lifted by the wheel radius).

The diff-drive controller publishes ``odom → base_link`` itself
(``enable_odom_tf: true``) and uses
``odom_frame_id = "${rname}/odom"``,
``base_frame_id = "${rname}/base_link"``.

Sensor bridges
--------------

The Ignition topics produced by the URDF's ``<sensor>`` blocks are
remapped to canonical ROS 2 names via ``ros_gz_bridge``:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Ignition topic
     - ROS 2 topic
     - Type
   * - ``/robot_<id>/scan_raw``
     - ``/robot_<id>/scan``
     - ``sensor_msgs/LaserScan``
   * - ``/robot_<id>/imu_raw``
     - ``/robot_<id>/imu``
     - ``sensor_msgs/Imu``

The clock bridge is a separate, single-instance node bridging
``/clock`` (``rosgraph_msgs/Clock`` ↔ ``gz.msgs.Clock``).

Per-robot ROS 2 topics
----------------------

After all spawners settle:

* ``/robot_<id>/cmd_vel`` — ``geometry_msgs/Twist``, consumed by the
  diff-drive controller.
* ``/robot_<id>/odom`` — ``nav_msgs/Odometry``, published by the
  diff-drive controller.
* ``/robot_<id>/scan`` — ``sensor_msgs/LaserScan``.
* ``/robot_<id>/imu`` — ``sensor_msgs/Imu``.
* ``/robot_<id>/joint_states`` — ``sensor_msgs/JointState``,
  published by the joint state broadcaster.
* ``/robot_<id>/robot_description`` —
  ``std_msgs/String`` (latched URDF), published by
  ``robot_state_publisher``.
* ``/clock`` — ``rosgraph_msgs/Clock`` (shared).

Launch arguments
----------------

.. list-table::
   :header-rows: 1
   :widths: 22 18 60

   * - Argument
     - Default
     - Meaning
   * - ``n_robots``
     - 5
     - Number of robots to spawn. Note: the launch file uses
       ``n_robots`` (not ``num_robots`` like the rest of the stack).
   * - ``positions_yaml``
     - ``""``
     - Absolute path to a YAML containing
       ``[[x0, y0, yaw0], ...]``. Empty string means the launch
       generates a square auto-grid (``ceil(sqrt(N))`` columns,
       1.5 m spacing).
   * - ``z_spawn``
     - ``0.3``
     - Height at which robots are added during automatic placement.
       Spawning at zero would clip the wheels into the ground plane.

Usage::

    # 5 robots, auto-grid
    ros2 launch iros_llm_swarm_simulation swarm_simulation.launch.py

    # Specific count and positions
    ros2 launch iros_llm_swarm_simulation swarm_simulation.launch.py \
      n_robots:=10 positions_yaml:=/abs/path/to/positions.yaml

Compatibility notes
-------------------

This package is **not** plug-compatible with the rest of the swarm
stack out of the box. Known divergences from the Stage backend:

#. **Sensor topic name.** Gazebo bridges to ``/robot_<id>/scan``;
   the Stage robot publishes ``/robot_<id>/base_scan``. The Nav2
   parameter file in ``iros_llm_swarm_local_nav`` is wired to
   ``base_scan``. Switching to Gazebo requires changing the
   ``obstacle_layer.base_scan.topic`` entry in
   ``robot_nav2_params.yaml`` (or remapping at launch time).
#. **Sensor characteristics.** The Gazebo lidar reaches 10 m and is
   noisy; the Stage lidar reaches 3.5 m and is noise-free.
   ``ResettingObstacleLayer`` was introduced for the Stage sparse-
   scanner ghost-trail problem — its trade-offs are different on a
   denser, longer-range Gazebo scan, and may not be needed (or may
   need re-tuning).
#. **Spawn names vs world list.** The Stage worlds hardcode 20
   robot names with explicit colors and poses. The Gazebo launch
   names robots ``robot_0`` … ``robot_<n_robots-1>`` from the launch
   argument; there is no static "fleet of 20" baked into the world
   file.
#. **Map server has no map.** The bundled
   ``swarm_world.sdf`` is empty (just ground plane). Nav2's
   ``map_server`` expects an occupancy grid; without one, the
   global costmap collapses to "everything free" and obstacle
   information has to come purely from the per-robot lidars and
   their local costmaps. For now ``iros_llm_swarm_local_nav``
   defaults its map argument to a Stage map yaml, which is
   geometrically meaningless against the empty Gazebo world.
#. **Startup time scales with N.** With ``i * 5 s`` per-robot
   stagger, 20 robots take ~110 s to bring up, well beyond the 1 s
   delays the bringup files use between layers.
#. **Per-robot frames.** Gazebo's URDF prefixes frames as
   ``robot_<id>/base_link`` etc., matching the Stage convention,
   but ``base_footprint`` is added (and is the URDF's root). The
   static TF in ``iros_llm_swarm_local_nav`` connects
   ``map → robot_<id>/odom``; the diff-drive controller fills in
   ``odom → base_link``. ``base_footprint`` to ``base_link`` is a
   fixed joint internal to the URDF.

Treat this package as the integration target rather than a daily
driver. When the swarm stack is ready to switch to 3D physics, the
list above is the punch-list.

Build, install, run
-------------------

Pure Python; runtime dependencies are listed in ``package.xml``
(``ros_gz_sim``, ``ros_gz_bridge``, ``ros2_control``, friends).

::

    colcon build --packages-select iros_llm_swarm_simulation
    ros2 launch iros_llm_swarm_simulation swarm_simulation.launch.py n_robots:=3

The integrated bringup files do **not** include this launch — they
include ``iros_llm_swarm_simulation_lite/warehouse_swarm.launch.py``
instead.

Cross-references
----------------

* The supported simulator that the bringup files actually use:
  ``iros_llm_swarm_simulation_lite``.
* Nav2 stack tuned for the Stage backend (will need topic / map
  changes to drive Gazebo): ``iros_llm_swarm_local_nav``.
* Roadmap: the project's ``README`` lists "Transition to Gazebo
  Harmonic for 3D simulation" as a future item; this package is the
  Fortress-era starting point.
