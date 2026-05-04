iros_llm_swarm_formation
========================

Centralized **formation registry and health monitor** for the swarm.
Two Python nodes:

* ``formation_manager_node`` — owns the in-memory formation registry,
  exposes the ``/formation/*`` CRUD services, and publishes the
  latched ``/formations/config`` snapshot.
* ``formation_monitor_node`` — subscribes to the registry, observes
  leader and follower odometry, computes per-follower errors, and
  publishes ``/formations/status`` with a per-formation state machine.

.. note::

   **No PD controllers live in this package.** The package's
   ``<description>`` mentions "leader-follower with PD controllers",
   but the actual PD math is implemented in the per-robot followers:

   * ``iros_llm_swarm_robot/motion_controller_node`` for the PBS
     stack.
   * ``iros_llm_swarm_mapf_lns/path_follower_node`` for the LNS2
     stack.

   This package only declares **who** follows **whom** and **at what
   offset**, then watches the result. The followers translate that
   declaration into ``cmd_vel`` themselves.

.. contents::
   :local:
   :depth: 2

Role in the stack
-----------------

The formation layer is a thin overlay on top of the existing follower
nodes. Activating a formation flips affected followers from AUTONOMOUS
mode (consuming MAPF paths / plans) to FORMATION_FOLLOWER mode (PD
tracking of a leader's body-frame offset). The flip is event-driven by
``/formations/config``; the followers do not restart.

Data flow::

    YAML on disk
       │
       ▼
    formation_manager_node            ──▶  /formations/config (latched)
       ▲      ▲                                   │
       │      │ /formation/* services             │
       │                                          ▼
    BT / CLI / scripted client       formation_monitor_node + per-robot followers
                                                  │
                                                  ▼
                                          /formations/status

The manager owns the canonical state. The monitor is read-only — it
sees the same snapshot every consumer sees and adds telemetry on top.

formation_manager_node
----------------------

Single-instance Python node. Holds the formation registry as a
dictionary indexed by ``formation_id``. After every mutation it
republishes the **full** registry on ``/formations/config`` (latched
``TRANSIENT_LOCAL`` + ``RELIABLE`` + ``depth=1``), so late subscribers
immediately see the current state.

Internal record
~~~~~~~~~~~~~~~

A formation is stored as a small ``_Formation`` object with these
fields:

* ``formation_id`` — unique string key.
* ``leader_ns`` — e.g. ``"robot_0"``.
* ``follower_ns`` — list of follower namespaces.
* ``offsets`` — list of ``(dx, dy)`` tuples in the leader's body frame
  (``+x`` forward, ``+y`` left), parallel to ``follower_ns``.
* ``active`` — whether followers should currently be tracking.

Conversion to ``FormationConfig`` adds a computed ``footprint``
polygon (see below) and stamps the message.

Footprint computation
~~~~~~~~~~~~~~~~~~~~~

Every published ``FormationConfig`` carries a ``footprint`` polygon
that the MAPF planner can use to plan the formation as a single
compound entity. The manager computes it as a 16-point circle
approximation:

#. ``(cx, cy)`` is the centroid of the follower offsets.
#. ``radius`` is the maximum distance from any offset to the centroid,
   plus ``robot_radius`` (per-robot envelope), plus
   ``footprint_padding`` (extra safety margin).
#. The circle is rasterized to 16 ``Point32`` samples around
   ``(cx, cy)`` with that radius.

Both ``robot_radius`` and ``footprint_padding`` are node parameters.
The leader's own footprint contribution is implicit: offsets are in
the leader body frame, so ``(0, 0)`` is the leader's centre, and the
circle's centroid is the geometric mean of just the followers — the
leader is at the origin and is automatically inside the circle for
the typical "leader at front" formation.

If ``offsets`` is empty, the polygon is a circle of radius
``robot_radius`` at the origin (leader-only "formation").

Services
~~~~~~~~

All services live under the ``/formation/*`` namespace. Type
definitions are in ``iros_llm_swarm_interfaces``; only
manager-specific behavior is documented here.

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Service
     - Behavior
   * - ``/formation/set``
     - Create or overwrite. Validates that ``offsets_x`` /
       ``offsets_y`` lengths match ``follower_ns``. ``activate=true``
       activates atomically with the registration. Republishes
       ``/formations/config``.
   * - ``/formation/remove``
     - Refuses to remove an **active** formation — disband it first
       via ``/formation/deactivate`` to avoid orphan trackers in the
       monitor or in followers. Republishes on success.
   * - ``/formation/activate`` / ``/formation/deactivate``
     - Toggle the ``active`` flag. Idempotent — already-active /
       already-inactive transitions return success without
       republishing changes (still no-ops).
   * - ``/formation/get``
     - Returns one ``FormationConfig`` with the freshly computed
       footprint at the current time.
   * - ``/formation/list``
     - Returns all formations (active and inactive) with computed
       footprints.
   * - ``/formation/load``
     - Loads from YAML (see schema below). ``clear_existing=true``
       wipes the registry before loading. ``activate`` is applied to
       every loaded formation. Returns ``loaded_count`` (entries that
       parsed successfully — malformed entries are logged and skipped,
       not fatal).
   * - ``/formation/save``
     - Persists the registry to a YAML file in the same schema.
       ``only_active=true`` skips inactive entries. Returns
       ``saved_count``.

Side-effect summary: every mutation
(``/formation/set``, ``/formation/remove``,
``/formation/activate``, ``/formation/deactivate``,
``/formation/load``) republishes the latched
``/formations/config``. ``get``, ``list``, ``save`` are pure reads.

YAML schema
~~~~~~~~~~~

The YAML format used by ``config_file`` (startup load) and by
``/formation/load`` / ``/formation/save`` is **not** a 1:1 copy of the
ROS service request fields:

.. code-block:: yaml

   formations:
     - id: "wedge"
       leader: "robot_1"
       followers:
         - ns: "robot_2"
           dx: -1.0
           dy:  0.6
         - ns: "robot_0"
           dx: -1.0
           dy: -0.6

Field mapping:

* ``id`` ↔ ``formation_id``
* ``leader`` ↔ ``leader_ns``
* ``followers[].ns`` ↔ entry in ``follower_ns``
* ``followers[].dx`` / ``followers[].dy`` ↔ entries in
  ``offsets_x`` / ``offsets_y``

Offsets are in the leader **body** frame: ``+x`` forward, ``+y`` left,
units in metres.

Two example files ship with the package: ``config/formations_1.yaml``
(wedge of three robots around ``robot_1``, line of four robots around
``robot_5``) and ``config/formations_2.yaml`` (similar layout shifted
across the warehouse).

Initial-load behavior
~~~~~~~~~~~~~~~~~~~~~

On startup, if ``config_file`` is non-empty, the manager loads it
synchronously and stamps every formation with ``active = auto_activate``
(parameter, default ``false``). After loading it always publishes the
registry once (so a freshly-launched monitor or follower immediately
sees the snapshot, even if it is empty).

Parameters
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 26 14 60

   * - Parameter
     - Default
     - Meaning
   * - ``config_file``
     - ``""``
     - YAML file loaded at startup. Empty string skips the load.
   * - ``auto_activate``
     - ``false``
     - Whether YAML-loaded formations should start active. Runtime
       ``/formation/load`` ignores this parameter and uses its own
       ``activate`` field.
   * - ``footprint_padding``
     - 0.2
     - Extra metres added to the footprint circle radius.
   * - ``robot_radius``
     - 0.3
     - Per-robot envelope added to the footprint circle radius.

formation_monitor_node
----------------------

Single-instance Python node. Subscribes to the latched
``/formations/config`` and to per-robot ``/<ns>/odom`` of every
participant; publishes ``/formations/status`` at ``monitor_hz`` with a
per-formation state machine and per-follower error metrics.

Tracking state
~~~~~~~~~~~~~~

The monitor maintains:

* ``self._formations: dict[formation_id, _FormationState]`` — the
  shadow registry (config + leader/follower state + state machine).
* ``self._ns_to_formations: dict[ns, list[formation_id]]`` — reverse
  index used to route incoming odom to every formation a robot
  participates in. Built so a robot can theoretically belong to
  multiple formations at once; in the typical use case each robot
  participates in zero or one.
* ``self._odom_subs: dict[ns, Subscription]`` — odom subscriptions
  reused across formations. Created lazily on registry updates and
  dropped only when a robot leaves all formations it was in.

When the registry changes, the monitor diffs old vs new membership
per formation and:

* Subscribes to odom for newly added robots.
* Drops the subscription for robots that no longer appear anywhere.
* Preserves accumulated ``_RobotState`` (last position, error,
  stuck-sample) for robots that remain — so an in-flight stuck
  detection isn't reset just because the registry was republished.

State machine
~~~~~~~~~~~~~

::

    INACTIVE  ──(formation activated)─────────▶  FORMING
    FORMING   ──(all errors < stable_thresh)──▶  STABLE
    STABLE    ──(any error > degraded_thresh)─▶  DEGRADED
    DEGRADED  ──(all errors < stable_thresh)──▶  STABLE
    DEGRADED  ──(stuck or lost detected)──────▶  BROKEN
    BROKEN    ──(disband then re-activate)────▶  INACTIVE → FORMING
    any state ──(formation disbanded)─────────▶  INACTIVE

Two key properties:

* **Hysteresis between STABLE and DEGRADED.** While ``max_error`` sits
  in the band ``(stable_thresh, degraded_thresh]`` the monitor keeps
  the previous state instead of oscillating. Crossing the upper
  threshold goes to DEGRADED; crossing back below the lower one goes
  to STABLE.
* **BROKEN is sticky.** Once a formation enters BROKEN
  (follower lost, follower stuck, or leader lost), the monitor will
  not transition back to STABLE / DEGRADED on its own. The only way
  out is a registry change — typically a deactivate / activate
  cycle from the manager — which resets the state to INACTIVE and
  then FORMING.

Failure detection
~~~~~~~~~~~~~~~~~

* **LEADER_LOST** — leader's last odom is older than
  ``odom_timeout_s``, or never arrived. Reported with
  ``failure_code = FAILURE_LEADER_LOST``. All follower errors are
  reported as ``-1.0`` for that tick (the world-frame target cannot
  be computed without leader pose).
* **FOLLOWER_LOST** — at least one follower's odom timed out. The
  affected followers are listed in ``failure_reason``; their entries
  in ``follower_errors_m`` are ``-1.0``. The remaining followers
  still get errors computed for them.
* **FOLLOWER_STUCK** — the per-follower stuck check triggers when:

  * the follower's error exceeds ``degraded_thresh_m``, **and**
  * the error has improved by less than ``stuck_delta_m`` over the
    last ``stuck_window_s``.

  A robot sitting at a small steady error never trips this, since the
  ``error > degraded`` precondition gates the check.

Per-follower error
~~~~~~~~~~~~~~~~~~

For each follower with valid odom, the monitor computes the expected
world-frame target

.. code-block:: text

   T = leader_pos + R(leader_yaw) · offset

and reports the Euclidean distance from the follower's current
position to ``T`` in ``follower_errors_m[i]``. Aggregates
``max_error_m`` and ``mean_error_m`` are computed only over valid
errors (``e ≥ 0``); they are ``-1.0`` if no follower has valid odom.

Topics, services, parameters
----------------------------

manager_node
~~~~~~~~~~~~

Published:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - QoS
   * - ``/formations/config``
     - ``iros_llm_swarm_interfaces/FormationsConfig``
     - ``KeepLast(1) + transient_local + reliable`` (latched).

Services exposed (request / response types in
``iros_llm_swarm_interfaces``):

* ``/formation/set``, ``/formation/remove``,
  ``/formation/activate``, ``/formation/deactivate``,
  ``/formation/get``, ``/formation/list``, ``/formation/load``,
  ``/formation/save``.

monitor_node
~~~~~~~~~~~~

Subscribed:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - QoS / notes
   * - ``/formations/config``
     - ``iros_llm_swarm_interfaces/FormationsConfig``
     - Latched (matches the manager).
   * - ``/<ns>/odom``
     - ``nav_msgs/Odometry``
     - ``depth=10``. One subscription per participating robot,
       dynamically created and dropped as registry membership
       changes.

Published:

.. list-table::
   :header-rows: 1
   :widths: 38 32 30

   * - Topic
     - Type
     - QoS
   * - ``/formations/status``
     - ``iros_llm_swarm_interfaces/FormationsStatus``
     - ``depth=10``, default reliability. Published at
       ``monitor_hz``.

Parameters:

.. list-table::
   :header-rows: 1
   :widths: 26 14 60

   * - Parameter
     - Default
     - Meaning
   * - ``monitor_hz``
     - 10.0
     - Status publication rate.
   * - ``stable_thresh_m``
     - 0.15
     - All errors below this ⇒ STABLE.
   * - ``degraded_thresh_m``
     - 0.35
     - Any error above this ⇒ DEGRADED.
   * - ``odom_timeout_s``
     - 1.0
     - Maximum age of last odom before LOST.
   * - ``stuck_window_s``
     - 3.0
     - Window over which a follower's error must improve to avoid
       being flagged stuck.
   * - ``stuck_delta_m``
     - 0.05
     - Minimum improvement (m) within ``stuck_window_s`` to count as
       progress.

Build and launch
----------------

The package is Python; only ``iros_llm_swarm_interfaces`` is a hard
build prerequisite (its message and service types are imported at
runtime)::

    colcon build --packages-select iros_llm_swarm_interfaces
    colcon build --packages-select iros_llm_swarm_formation

There is **no launch file** in this package. Both nodes are spawned
by the integrated bringup launches:

* ``swarm_mapf_formation.launch.py`` (PBS + formation).
* ``swarm_lns_formation.launch.py`` (LNS2 + formation).

Each one starts ``formation_manager_node`` (with a YAML
``config_file``) and ``formation_monitor_node`` alongside the per-robot
followers at ``t = 12 s``.

To run either node manually for debugging::

    ros2 run iros_llm_swarm_formation formation_manager_node
    ros2 run iros_llm_swarm_formation formation_monitor_node

Then poke at the registry::

    ros2 service call /formation/set iros_llm_swarm_interfaces/srv/SetFormation "
    formation_id: 'line'
    leader_ns: 'robot_0'
    follower_ns: ['robot_1', 'robot_2']
    offsets_x: [-1.0, -2.0]
    offsets_y: [0.0, 0.0]
    activate: true
    "

    ros2 topic echo /formations/config
    ros2 topic echo /formations/status

Cross-references
----------------

* Message and service definitions: ``iros_llm_swarm_interfaces``
  (``FormationConfig``, ``FormationsConfig``, ``FormationStatus``,
  ``FormationsStatus``, plus the ``/formation/*`` service types).
* Per-robot consumers of ``/formations/config``:
  ``iros_llm_swarm_robot`` (``motion_controller_node``, PBS) and
  ``iros_llm_swarm_mapf_lns`` (``path_follower_node``, LNS2).
* BT nodes that wrap formation services: ``iros_llm_swarm_bt``.
