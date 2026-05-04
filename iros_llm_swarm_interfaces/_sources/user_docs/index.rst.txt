iros_llm_swarm_interfaces
=========================

Type-only ROS 2 package. Defines every ``msg``, ``srv``, and ``action``
shared across the swarm stack — fleet-level MAPF goals, formation
configuration and status, per-robot follower telemetry, and the LLM
decision channel exposed to the behavior tree.

This package contains no nodes and no executables. Other packages depend
on its generated headers (C++) and modules (Python).

.. contents::
   :local:
   :depth: 2

Build order
-----------

Because every C++ and Python package in the workspace depends on the
types declared here, build this package **first** whenever you add or
change a type::

    colcon build --packages-select iros_llm_swarm_interfaces
    colcon build

Skipping the first command after editing a ``.msg`` / ``.srv`` /
``.action`` file usually surfaces as missing-header or
``ImportError`` failures in downstream packages.

Inventory
---------

Actions (2):

* **SetGoals.action** — fleet-wide MAPF mission control.
* **LlmDecision.action** — LLM advisory decisions for the BT.

Messages (7):

* **MAPFPlan.msg** / **MAPFStep.msg** — per-robot path the planner
  publishes to each follower.
* **FollowerStatus.msg** — per-robot execution telemetry consumed by
  the planner.
* **FormationConfig.msg** / **FormationsConfig.msg** — formation
  definition and the fleet-wide snapshot.
* **FormationStatus.msg** / **FormationsStatus.msg** — formation
  runtime state and the fleet-wide snapshot.

Services (8):

* **SetFormation.srv** / **RemoveFormation.srv**
* **ActivateFormation.srv** / **DeactivateFormation.srv**
* **GetFormation.srv** / **ListFormations.srv**
* **LoadFormations.srv** / **SaveFormations.srv**

Actions
-------

SetGoals.action
~~~~~~~~~~~~~~~

Long-lived fleet navigation action. A single goal covers the entire
mission lifecycle:

::

    validating → planning → executing → replanning (on deviation) → succeeded
                                                                  ↘ failed
                                                                  ↘ cancelled

The action server lives inside the active MAPF planner — currently
either ``mapf_planner_node`` (PBS) or ``mapf_lns2_node`` (LNS2). External
clients (BT, scripted tests, future LLM agent) treat the whole mission
as one atomic operation and read progress from the periodic feedback.

**Goal**

.. code-block:: text

   uint32[]              robot_ids
   geometry_msgs/Point[] goals       # parallel to robot_ids; map frame

Robot start positions are read by the server from each robot's
``/robot_*/odom`` — the client only sends targets.

**Result**

.. list-table::
   :header-rows: 1
   :widths: 30 18 52

   * - Field
     - Type
     - Meaning
   * - ``success``
     - ``bool``
     - Whether the mission completed successfully.
   * - ``message``
     - ``string``
     - Human-readable summary or error.
   * - ``error_code``
     - ``uint16``
     - One of the constants below.
   * - ``planning_time_ms``
     - ``float64``
     - Wall-clock planning time of the **last** plan (ms).
   * - ``num_agents_planned``
     - ``uint32``
     - Number of agents actually planned.
   * - ``pbs_expansions``
     - ``uint32``
     - PBS tree node expansions (PBS planner only).
   * - ``max_path_length`` / ``path_lengths[]``
     - ``uint32``
     - Longest and per-agent path length (steps).
   * - ``astar_ok_count`` / ``astar_fail_count``
     - ``uint32``
     - Successful and capped A* invocations.
   * - ``astar_avg_exp`` / ``astar_max_exp``
     - ``uint32``
     - A* expansion statistics across all calls.
   * - ``total_replans``
     - ``uint32``
     - Replans triggered during execution.
   * - ``total_execution_sec``
     - ``float64``
     - Total time from initial planning to all-arrived (s).

Error code constants:

.. code-block:: text

   NONE             = 0
   UNKNOWN          = 200
   NO_VALID_AGENTS  = 201
   PBS_FAILED       = 202
   TIMEOUT          = 203
   CANCELLED        = 204

The ``PBS_FAILED`` constant is also used by the LNS2 backend to signal
generic planner failure, so it is not literally PBS-specific.

**Feedback** (published periodically, ~2 Hz)

.. list-table::
   :header-rows: 1
   :widths: 25 18 57

   * - Field
     - Type
     - Meaning
   * - ``status``
     - ``string``
     - Phase: ``"validating"``, ``"planning"``, ``"executing"``,
       ``"replanning"``, ``"failed"``.
   * - ``elapsed_ms``
     - ``uint32``
     - Milliseconds since the goal was accepted.
   * - ``robots_arrived`` / ``robots_active`` / ``robots_deviated``
     - ``uint32``
     - Execution-phase counters; zero before execution starts.
   * - ``replans_done``
     - ``uint32``
     - Replan count so far.
   * - ``robot_stall``
     - ``uint32``
     - Robots considered dead / unresponsive.
   * - ``info``
     - ``string``
     - Periodic informational text (low rate).
   * - ``warning``
     - ``string``
     - One-shot warnings (e.g. ``"N unplanable skipped"``,
       ``"Robot N out of lives"``).

Cancellation stops every robot immediately by clearing assigned plans
(see ``MAPFPlan.msg`` below).

LlmDecision.action
~~~~~~~~~~~~~~~~~~

Asynchronous advisory channel between the behavior tree and an external
LLM agent. The BT requests guidance when ``WARN`` / ``ERROR`` events
appear during a MAPF mission, or on a periodic batch of ``INFO`` logs.
The LLM responds with a discrete decision the BT can act on.

**Goal**

.. code-block:: text

   string   level        # "WARN" | "ERROR" | "INFO"
   string   event        # short description
   string[] log_buffer   # last N info lines accumulated before this event

**Result**

.. code-block:: text

   string decision   # "wait" | "abort" | "replan"
   string reasoning  # explanation for the user (not used by BT logic)

**Feedback**

.. code-block:: text

   string reasoning_stream

The ``reasoning_stream`` field is used to forward the LLM's
intermediate reasoning to the user while the request is in flight. The
BT itself only acts on ``decision``.

Messages
--------

MAPFPlan.msg
~~~~~~~~~~~~

Full per-robot plan, published by the active planner on
``/<ns>/mapf_plan``.

.. code-block:: text

   std_msgs/Header header
   uint32 plan_id
   iros_llm_swarm_interfaces/MAPFStep[] steps

Semantics:

* ``header.stamp`` — when the plan was issued.
* ``header.frame_id`` — frame for every step's target (typically
  ``"map"``).
* ``plan_id`` — monotonically increasing token. Followers echo this in
  their ``FollowerStatus`` so the planner can ignore stale statuses
  across replan transitions.
* ``steps`` — empty list = **CANCEL**: the follower aborts any in-flight
  Nav2 goal and goes IDLE. Planners use this to stop unplanable robots
  or to reset before a fresh ``set_goals`` mission.

MAPFStep.msg
~~~~~~~~~~~~

One waypoint inside a ``MAPFPlan``.

.. code-block:: text

   geometry_msgs/Point target
   float32 hold_sec

Semantics: drive to ``target``; once arrived, hold position for
``hold_sec`` seconds before moving to the next step.
``hold_sec = 0`` means proceed immediately. ``target`` is in the frame
declared by ``MAPFPlan.header.frame_id``.

FollowerStatus.msg
~~~~~~~~~~~~~~~~~~

Per-robot execution telemetry. Published on ``/<ns>/follower_status``
both on every state transition and at a heartbeat rate (default 2 Hz),
so the planner can distinguish a *legitimately holding* follower from a
*frozen / crashed* one.

.. code-block:: text

   std_msgs/Header header
   uint32 plan_id
   uint8  state
   uint32 current_step_index
   uint32 num_steps
   float32 hold_remaining_sec
   uint32 nav2_failures
   uint8  last_failure_reason

State machine (``state`` field):

.. code-block:: text

   STATE_IDLE          = 0    # no plan / cancelled / not in AUTONOMOUS mode
   STATE_NAVIGATING    = 1    # Nav2 driving toward current step's target
   STATE_HOLDING       = 2    # waiting hold_sec at current step's target
   STATE_PLAN_COMPLETE = 3    # all steps executed successfully
   STATE_FAILED        = 4    # Nav2 aborted; stays here until a new plan

Failure reasons (``last_failure_reason``, valid when ``state ==
STATE_FAILED``):

.. code-block:: text

   FAIL_NONE             = 0
   FAIL_NAV2_ABORTED     = 1   # Nav2 action aborted mid-execution
   FAIL_NAV2_REJECTED    = 2   # Nav2 action server rejected the goal
   FAIL_NAV2_SERVER_DOWN = 3   # Nav2 action server unavailable

The fine-grained "no valid trajectories" vs "failed to make progress"
distinction is not exposed — Nav2 only signals ``ABORTED`` at this
layer. Counter ``nav2_failures`` is cumulative within a single
``plan_id`` and resets on every new plan.

FormationConfig.msg
~~~~~~~~~~~~~~~~~~~

Definition of one formation. Published as part of
``FormationsConfig`` by ``formation_manager_node``.

.. code-block:: text

   std_msgs/Header header
   string   formation_id        # unique name e.g. "wedge"
   string   leader_ns           # e.g. "robot_0"
   string[] follower_ns         # e.g. ["robot_1", "robot_2"]
   geometry_msgs/Point[] offsets    # parallel to follower_ns
   geometry_msgs/Polygon footprint  # bounding shape in leader body frame
   bool active

Conventions:

* ``offsets[i]`` is in the **leader body frame** (``+x`` forward,
  ``+y`` left), not the map frame.
* ``offsets`` length must equal ``follower_ns`` length.
* ``footprint`` is the bounding polygon of the assembled formation in
  the leader's body frame; used by the MAPF planner when planning the
  formation as a single compound entity (when supported).
* ``active = false`` means the formation is dissolved and followers
  should fall back to autonomous mode.

FormationsConfig.msg
~~~~~~~~~~~~~~~~~~~~

Fleet-wide snapshot of all formations. Published on
``/formations/config`` with QoS ``TRANSIENT_LOCAL`` + ``RELIABLE`` +
``depth=1`` (latched), so late subscribers immediately receive the
current state.

.. code-block:: text

   std_msgs/Header header
   FormationConfig[] formations

FormationStatus.msg
~~~~~~~~~~~~~~~~~~~

Runtime status of one formation. Published as part of
``FormationsStatus`` by ``formation_monitor_node``.

.. code-block:: text

   std_msgs/Header header
   string   formation_id
   string   leader_ns
   string[] follower_ns
   uint8    state
   uint8    failure_code
   string   failure_reason
   float32[] follower_errors_m
   float32  max_error_m
   float32  mean_error_m

State machine (``state`` field):

.. code-block:: text

   STATE_INACTIVE  = 0   # formation not active
   STATE_FORMING   = 1   # active, followers converging to offsets
   STATE_STABLE    = 2   # all followers within cohesion threshold
   STATE_DEGRADED  = 3   # some followers out of tolerance but recovering
   STATE_BROKEN    = 4   # formation cannot be maintained

Failure codes (``failure_code``, valid when ``state == STATE_BROKEN``):

.. code-block:: text

   FAILURE_NONE           = 0
   FAILURE_FOLLOWER_LOST  = 1   # follower odom stopped arriving
   FAILURE_FOLLOWER_STUCK = 2   # follower error not decreasing
   FAILURE_LEADER_LOST    = 3   # leader odom stopped arriving

Conventions:

* ``follower_errors_m`` is parallel to ``follower_ns``. ``-1.0`` means
  data is not available for that follower.
* ``max_error_m`` / ``mean_error_m`` are aggregates over
  ``follower_errors_m``; ``-1.0`` if no data.

FormationsStatus.msg
~~~~~~~~~~~~~~~~~~~~

Fleet-wide snapshot of formation runtime status. Published on
``/formations/status`` (QoS ``depth=10``).

.. code-block:: text

   std_msgs/Header header
   FormationStatus[] formations

Services
--------

All formation services live under the ``/formation/*`` namespace and
are served by ``formation_manager_node``. ``Get*`` / ``List*`` are
read-only; the rest mutate the formation registry and trigger a fresh
``FormationsConfig`` publication.

SetFormation.srv
~~~~~~~~~~~~~~~~

Create a new formation or overwrite an existing one.

.. code-block:: text

   string    formation_id
   string    leader_ns
   string[]  follower_ns
   float64[] offsets_x
   float64[] offsets_y
   bool      activate
   ---
   bool   success
   string message

``offsets_x`` / ``offsets_y`` are parallel arrays in the leader body
frame; their length must equal ``follower_ns``. Setting ``activate =
true`` activates the formation atomically with the update.

Example::

   ros2 service call /formation/set iros_llm_swarm_interfaces/srv/SetFormation "
   formation_id: 'line'
   leader_ns: 'robot_0'
   follower_ns: ['robot_1', 'robot_2']
   offsets_x: [1.0, 2.0]
   offsets_y: [0.0, 0.0]
   activate: true
   "

RemoveFormation.srv
~~~~~~~~~~~~~~~~~~~

Drop a formation from the registry. Active formations are deactivated
first.

.. code-block:: text

   string formation_id
   ---
   bool   success
   string message

ActivateFormation.srv
~~~~~~~~~~~~~~~~~~~~~

Activate a previously defined formation. Followers begin tracking their
offsets relative to the leader.

.. code-block:: text

   string formation_id
   ---
   bool   success
   string message

DeactivateFormation.srv
~~~~~~~~~~~~~~~~~~~~~~~

Deactivate a formation. Followers fall back to autonomous mode and
resume consuming their own ``mapf_plan`` topics.

.. code-block:: text

   string formation_id
   ---
   bool   success
   string message

GetFormation.srv
~~~~~~~~~~~~~~~~

Fetch a single formation by ID, including its computed footprint.

.. code-block:: text

   string formation_id
   ---
   bool             success
   string           message
   FormationConfig  formation

ListFormations.srv
~~~~~~~~~~~~~~~~~~

Return every formation currently registered (active and inactive).

.. code-block:: text

   ---
   FormationConfig[] formations

LoadFormations.srv
~~~~~~~~~~~~~~~~~~

Load formations from a YAML file on disk. Optionally clear the existing
registry first and / or activate freshly loaded formations.

.. code-block:: text

   string file_path
   bool   clear_existing
   bool   activate
   ---
   bool   success
   string message
   uint32 loaded_count

SaveFormations.srv
~~~~~~~~~~~~~~~~~~

Persist the current registry to a YAML file. ``only_active = true``
skips inactive formations.

.. code-block:: text

   string file_path
   bool   only_active
   ---
   bool   success
   string message
   uint32 saved_count

Cross-references
----------------

For the high-level pipeline that produces and consumes these types, see
the **Architecture** page in the ``iros_llm_swarm_docs`` aggregator.

Per-type consumers in the workspace:

* ``SetGoals.action`` — served by the active MAPF planner
  (``iros_llm_swarm_mapf_lns`` for LNS2; the legacy
  ``iros_llm_swarm_mapf`` for PBS). Called by ``iros_llm_swarm_bt`` and
  the bundled ``test_send_goals`` script.
* ``LlmDecision.action`` — invoked from ``iros_llm_swarm_bt``; served
  by an external LLM agent (out of tree).
* ``MAPFPlan.msg`` / ``MAPFStep.msg`` — published by the planner;
  consumed by per-robot path followers in ``iros_llm_swarm_mapf_lns``
  and ``iros_llm_swarm_robot``.
* ``FollowerStatus.msg`` — published by per-robot followers in
  ``iros_llm_swarm_mapf_lns`` (and ``iros_llm_swarm_robot``); consumed
  by the planner for replan triggering.
* ``FormationsConfig.msg`` / ``FormationsStatus.msg`` — published by
  ``iros_llm_swarm_formation`` (manager and monitor respectively);
  consumed by motion controllers / path followers.
* All ``/formation/*`` services — served by ``formation_manager_node``
  in ``iros_llm_swarm_formation``; called from ``iros_llm_swarm_bt`` and
  ad-hoc via ``ros2 service call``.
