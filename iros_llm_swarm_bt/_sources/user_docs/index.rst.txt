iros_llm_swarm_bt
=================

Behavior-tree front-end for the swarm. Provides four
**BehaviorTree.CPP v3** action / condition node types that wrap the
swarm's ROS interfaces, a reference tree implementing a mode-dispatch
mission loop, and a runner executable that can either ingest commands
from a topic or run an embedded 20-robot integration scenario.

The package is also the integration point for an out-of-tree LLM
agent: the ``MapfPlan`` node opens an asynchronous side channel via
``/llm/decision`` (``LlmDecision`` action) where the LLM can return a
``"wait" | "abort" | "replan"`` verdict in response to MAPF feedback
events.

.. note::

   **State of the BT layer.** The project ``README`` lists "BT
   integration" as roadmap work not yet done. Read that as
   *expansion* — the four BT nodes documented here already build, are
   exercised by the embedded scenario, and the main tree drives a full
   mode-dispatch loop end to end. Outstanding items (a dedicated
   ``SwarmBtNavigator`` akin to ``nav2_bt_navigator``, dedicated
   ``ExecuteFormation`` / ``EmergencyStop`` nodes, and richer
   mode-switching control flow) are layered on top of this base, not
   blocked by it.

.. contents::
   :local:
   :depth: 2

What's in the package
---------------------

* ``iros_llm_swarm_bt_nodes`` (shared library) — the BT plugin.
  Registers four node types via ``BT_REGISTER_NODES``.
* ``test_bt_runner`` (executable) — minimal BT runner / scenario
  harness. Loads the plugin, ticks the reference tree at 10 Hz, and
  optionally drives an embedded 20-robot scenario.
* ``behavior_trees/`` — XML trees:

  * ``swarm_navigate_to_pose.xml`` — main reference tree (used by
    the runner).
  * ``test_*.xml`` — minimal trees that exercise each BT node in
    isolation.
  * ``disable_formation.xml`` / ``test_disable_formation.xml`` —
    trees that drive a formation disband from the BT side.
* ``launch/test_bt_runner.launch.py`` — one-line launch wrapper for
  the runner.

The plugin and the runner can be used independently of each other —
any BT.CPP v3 host that loads
``libiros_llm_swarm_bt_nodes.so`` gets the four node types.

BT node catalog
---------------

All four types live in namespace ``iros_llm_swarm_bt`` and register
under their unqualified class name (``MapfPlan``, ``SetFormation``,
``DisableFormation``, ``CheckMode``).

MapfPlan
~~~~~~~~

``BT::StatefulActionNode`` wrapping the ``/swarm/set_goals`` action
plus an asynchronous ``/llm/decision`` advisory channel.

**Ports**

.. list-table::
   :header-rows: 1
   :widths: 22 12 16 50

   * - Name
     - Direction
     - Type
     - Meaning
   * - ``robot_ids``
     - input
     - ``vector<int>``
     - Dense list of robot IDs (cast to ``uint32`` for the action).
   * - ``goals``
     - input
     - ``vector<geometry_msgs/Point>``
     - Parallel to ``robot_ids``; map-frame target positions.
   * - ``mapf_ok``
     - output
     - ``bool``
     - ``true`` on action success.
   * - ``mapf_info``
     - output
     - ``string``
     - Result summary line or failure reason.
   * - ``mapf_warn``
     - output
     - ``string``
     - Last warning observed in feedback (only set when feedback
       carried ``warning != ""``).

**Lifecycle**

* ``onStart()`` validates inputs (presence, equal lengths, non-empty),
  waits up to 2 s for the action server, then sends the goal with a
  feedback callback bound to ``on_feedback``.
* ``onRunning()`` is purely poll-based — it never blocks the BT
  executor:

  #. If the LLM has produced a decision, act on it
     (``"abort"`` → cancel + FAILURE, ``"replan"`` → cancel + FAILURE
     and write ``@mapf_decision = "replan"`` to the blackboard,
     ``"wait"`` → continue RUNNING).
  #. If the LLM future produced a goal handle but not yet a result,
     start polling the result future.
  #. If the MAPF goal handle is still pending, return RUNNING.
  #. If the MAPF result future is still pending, return RUNNING.
  #. Otherwise the action is done — populate ``mapf_info``,
     ``mapf_ok``, return SUCCESS or FAILURE.
* ``onHalted()`` cancels the MAPF goal, cancels any pending LLM goal,
  clears the buffered decision.

**LLM feedback path**

For every feedback message the node builds a one-line summary (status,
elapsed, arrived/active/stall counters, replans, plus any ``info`` /
``warning`` text) and pushes it onto a ring buffer of size
``llm_info_buffer_size`` (default 50). Then:

* If the feedback carries ``warning`` — fire an immediate
  ``LlmDecision`` goal with ``level = "WARN"`` and the buffered logs.
* If the feedback carries ``info`` and ``llm_log_interval_sec > 0`` —
  fire a periodic ``LlmDecision`` goal with ``level = "INFO"`` no more
  often than once every ``llm_log_interval_sec`` seconds.

Only one LLM call is in flight at a time. While one is pending,
further LLM events are skipped (logged at WARN level). The result
``decision`` field flows back into ``onRunning()`` via a mutex-guarded
``pending_decision_`` slot.

If ``/llm/decision`` is not available, the node logs and continues
without LLM input — MAPF execution is not blocked by LLM unavailability.

**Failure modes** that complete the BT node with FAILURE:

* Missing ``robot_ids`` / ``goals`` ports.
* ``robot_ids.size() != goals.size()`` or ``robot_ids.empty()``.
* ``/swarm/set_goals`` action server unavailable for 2 s.
* Goal rejected by the action server.
* Action transport error (any ``ResultCode != SUCCEEDED``).
* Result with ``num_agents_planned == 0``.
* LLM verdict ``"abort"`` or ``"replan"``.

A *partial* result (``success = false`` but
``num_agents_planned > 0``) is logged at WARN level and still returns
SUCCESS — the node treats "some robots planned" as a forward-progress
event. Read ``mapf_info`` if the caller cares about the distinction.

**Node-level ROS parameters** (declared at construction on the
host node):

.. list-table::
   :header-rows: 1
   :widths: 26 14 60

   * - Parameter
     - Default
     - Meaning
   * - ``llm_log_interval_sec``
     - 0.0
     - Periodic LLM-call cadence on ``info`` feedback. ``0.0``
       disables periodic calls; ``WARN`` events are always sent.
   * - ``llm_info_buffer_size``
     - 50
     - Ring-buffer size for accumulated ``info`` lines that ride
       along with each LLM call.

SetFormation
~~~~~~~~~~~~

``BT::StatefulActionNode`` wrapping the ``/formation/set`` service.

**Ports**

.. list-table::
   :header-rows: 1
   :widths: 24 12 14 50

   * - Name
     - Direction
     - Type
     - Meaning
   * - ``formation_id``
     - input
     - ``string``
     -
   * - ``leader_ns``
     - input
     - ``string``
     -
   * - ``follower_ns``
     - input
     - ``vector<string>``
     -
   * - ``offsets_x``
     - input
     - ``vector<double>``
     - Parallel to ``follower_ns``.
   * - ``offsets_y``
     - input
     - ``vector<double>``
     - Parallel to ``follower_ns``.
   * - ``activate``
     - input
     - ``bool``
     - Default ``true`` (activate immediately).
   * - ``formation_enabled``
     - output
     - ``bool``
     - ``service.success && activate``. ``false`` if either the
       service failed or ``activate`` was ``false``.
   * - ``active_formation``
     - output
     - ``string``
     - Echo of ``formation_id`` for downstream branches.

**Behavior**

* ``onStart()`` validates port presence and array-length consistency,
  waits up to 2 s for the service, sends the request asynchronously.
* ``onRunning()`` polls the service future; on completion writes the
  outputs and returns SUCCESS / FAILURE based on ``response.success``.
* ``onHalted()`` clears the future (request keeps flying server-side
  but the BT no longer cares about the result).

DisableFormation
~~~~~~~~~~~~~~~~

``BT::StatefulActionNode`` wrapping ``/formation/deactivate``.

**Ports**

.. list-table::
   :header-rows: 1
   :widths: 24 12 14 50

   * - Name
     - Direction
     - Type
     - Meaning
   * - ``formation_id``
     - input
     - ``string``
     -
   * - ``formation_enabled``
     - output
     - ``bool``
     - Always written as ``false`` on completion.

Same poll-and-complete pattern as ``SetFormation``. The output is
unconditionally ``false`` because a successful disband always means
the formation is no longer enabled.

CheckMode
~~~~~~~~~

Synchronous ``BT::ConditionNode``. Reads two string inputs, returns
SUCCESS if equal, FAILURE otherwise (or if either input is missing).

**Ports**

.. list-table::
   :header-rows: 1
   :widths: 22 12 14 52

   * - Name
     - Direction
     - Type
     - Meaning
   * - ``mode``
     - input
     - ``string``
     - Typically bound to a blackboard entry, e.g. ``{@mode}``.
   * - ``expected``
     - input
     - ``string``
     - Typically a literal in the XML, e.g. ``"mapf"``.

This is the only BT node in the set that does no I/O of its own — it's
the building block of the mode-dispatch idiom (see below).

Reference tree
--------------

The runner loads ``behavior_trees/swarm_navigate_to_pose.xml`` (the
``main_tree_to_execute = "FleetMain"``):

.. code-block:: text

   <root BTCPP_format="4" main_tree_to_execute="FleetMain">
     <BehaviorTree ID="FleetMain">
       <ReactiveFallback name="ModeDispatch">
         <CheckMode mode="{@mode}" expected="idle"/>

         <Sequence name="MapfBranch">
           <CheckMode mode="{@mode}" expected="mapf"/>
           <MapfPlan robot_ids="{@robot_ids}" goals="{@goals}"
                     mapf_ok="{@mapf_ok}" mapf_info="{@mapf_info}"/>
         </Sequence>

         <Sequence name="FormationBranch">
           <CheckMode mode="{@mode}" expected="formation"/>
           <SetFormation formation_id="{@formation_id}"
                         leader_ns="{@leader_ns}"
                         follower_ns="{@follower_ns}"
                         offsets_x="{@offsets_x}"
                         offsets_y="{@offsets_y}"
                         activate="true"
                         formation_enabled="{@formation_enabled}"
                         active_formation="{@active_formation}"/>
         </Sequence>
       </ReactiveFallback>
     </BehaviorTree>
   </root>

The ``ReactiveFallback`` re-evaluates every child on every tick, so a
mid-execution mode change halts the currently running action via
``onHalted()`` and starts the new branch. A common usage pattern:

#. Set ``@mode = "mapf"`` plus ``@robot_ids`` / ``@goals`` →
   ``MapfPlan`` runs.
#. While the MAPF action is in progress, set ``@mode = "idle"`` →
   ``MapfPlan::onHalted()`` cancels the goal and the tree settles on
   the idle branch.

.. note::

   The reference tree uses ``BTCPP_format="4"`` while CMake links
   against ``behaviortree_cpp_v3``. BT.CPP v3 builds in the project's
   container accept format-4 XML; if you swap to a different BT.CPP
   release, expect to revisit this header.

   Also: the tree does **not** include a ``DisableFormation`` branch.
   Disbanding happens implicitly via blackboard mode flips
   (formation → idle); ``DisableFormation`` is currently exercised
   only by the standalone ``disable_formation.xml`` /
   ``test_disable_formation.xml`` trees, kept for future expansion.

Blackboard convention
~~~~~~~~~~~~~~~~~~~~~

The runner pre-creates the blackboard with one root-shared entry:
the host ROS node under ``"node"`` (used by every BT class to create
ROS clients), plus default values for ``@mode``, ``@robot_ids``,
``@goals``. Other entries are populated by the operator (CLI / scenario
thread) or written back by the BT nodes:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Blackboard key
     - Owner / use
   * - ``@mode``
     - Operator input — ``"idle"`` / ``"mapf"`` / ``"formation"``.
   * - ``@robot_ids`` / ``@goals``
     - Operator input — MAPF mission targets.
   * - ``@formation_id`` / ``@leader_ns`` / ``@follower_ns`` /
       ``@offsets_x`` / ``@offsets_y``
     - Operator input — formation definition.
   * - ``@mapf_ok`` / ``@mapf_info`` / ``@mapf_warn``
     - ``MapfPlan`` outputs — mission result and last warning.
   * - ``@formation_enabled`` / ``@active_formation``
     - ``SetFormation`` / ``DisableFormation`` outputs.
   * - ``@mapf_decision``
     - ``MapfPlan`` writes ``"replan"`` when the LLM returns that
       verdict. Consumed by an outer controller that sets new
       goals and flips ``@mode`` back to ``"mapf"``.
   * - ``@mapf_failed`` / ``@formation_failed``
     - Set by the runner on terminal FAILURE; read by the embedded
       scenario thread to abort.

test_bt_runner
--------------

Two roles selectable via the ``scenario`` ROS parameter:

Idle runner (``scenario = false``, default)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Subscribes to ``/fleet/cmd`` (``std_msgs/String``, format
``"key=value"``) and writes the corresponding entry into the
blackboard. Recognized keys: ``mode``, ``formation_id``, ``leader_ns``,
``robot_ids`` (CSV of ints), ``goals`` (CSV of ``x,y,x,y,…``),
``follower_ns`` (CSV), ``offsets_x``, ``offsets_y`` (CSV of doubles).

Example interaction::

    # Send 20 robots to the warehouse centre
    ros2 topic pub --once /fleet/cmd std_msgs/msg/String \
      "data: 'robot_ids=0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19'"
    ros2 topic pub --once /fleet/cmd std_msgs/msg/String \
      "data: 'goals=15,15,15,15,...'"  # one (x,y) per id
    ros2 topic pub --once /fleet/cmd std_msgs/msg/String \
      "data: 'mode=mapf'"

    # Cancel mid-mission
    ros2 topic pub --once /fleet/cmd std_msgs/msg/String \
      "data: 'mode=idle'"

Scenario mode (``scenario = true``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Runs an embedded 20-robot integration scenario in a side thread:

#. **MAPF center** — all 20 robots to a 4×5 grid centred on
   ``(15, 15)``. Wait up to 240 s.
#. **Formation WEDGE_20** — orange squad ``robot_0..9`` with
   ``robot_1`` as leader, alternating left/right wings stepping back
   1 m per row.
#. **Formation LINE_BLUE** — blue squad ``robot_10..14`` with
   ``robot_10`` as leader, ``robot_11..14`` lined up behind.
#. **MAPF cross-swap** — orange squad to the upper-right unloading
   zone, blue squad to the lower-left loading zone. Two clusters
   crossing the full warehouse — the stress test for replan.
#. **MAPF home** — return everyone to their starting positions.
#. **Idle** — scenario complete.

Each step toggles ``@mode`` between the active branch and ``idle``,
seeds the relevant blackboard inputs, and waits for the ``@mapf_ok``
or ``@formation_enabled`` flag (or its ``_failed`` counterpart). Step
timeouts are 240 s (initial MAPF, generous), 300 s (cross-swap and
home), 15 s (formation set-up).

Tick loop
~~~~~~~~~

The runner's main loop runs at 10 Hz:

* On every iteration it spins ``rclcpp`` (``spin_some``), reads
  ``@mode``, publishes it on ``/fleet/mode``.
* When ``@mode`` transitions **into** ``"idle"``, ``haltTree()`` is
  called immediately so any in-flight action gets cancelled.
* When ``@mode == "idle"``, the tree is **not** ticked at all.
* When the tree ticks to a terminal state (SUCCESS / FAILURE),
  ``@mapf_ok`` is published on ``/fleet/mapf_ok``,
  ``@formation_enabled`` on ``/fleet/formation_enabled``, the tree
  is halted, and on FAILURE the runner sets
  ``@mapf_failed = @formation_failed = true`` and flips ``@mode`` to
  ``"idle"``.

Published topics
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 32 30 38

   * - Topic
     - Type
     - Notes
   * - ``/fleet/mode``
     - ``std_msgs/String``
     - QoS ``KeepLast(1) + transient_local``. Mirror of ``@mode`` —
       useful for an external operator UI.
   * - ``/fleet/mapf_ok``
     - ``std_msgs/String``
     - ``"true"`` / ``"false"``. Published once per terminal MAPF
       result.
   * - ``/fleet/formation_enabled``
     - ``std_msgs/Bool``
     - Published once per terminal formation result.

ROS parameters of the runner node:

.. list-table::
   :header-rows: 1
   :widths: 26 14 60

   * - Parameter
     - Default
     - Meaning
   * - ``scenario``
     - ``false``
     - Run the embedded 20-robot integration scenario.
   * - ``llm_log_interval_sec``
     - 0.0
     - See ``MapfPlan``. Declared by the BT node when its first
       instance is constructed.
   * - ``llm_info_buffer_size``
     - 50
     - See ``MapfPlan``.

Build, install layout, plugin loading
-------------------------------------

Required dependencies:

* ``behaviortree_cpp_v3`` from the system / ``ros-${ROS_DISTRO}-…``
* ``iros_llm_swarm_interfaces`` (must be built first when adding
  types).

Built artefacts:

* ``libiros_llm_swarm_bt_nodes.so`` (shared library) —
  installed under ``lib/`` (and the
  exported ``include/`` headers).
* ``test_bt_runner`` (executable) — installed under
  ``lib/iros_llm_swarm_bt/``.
* ``behavior_trees/`` and ``launch/`` — installed under
  ``share/iros_llm_swarm_bt/``.

The runner loads the plugin at startup with::

    factory.registerFromPlugin("libiros_llm_swarm_bt_nodes.so");

This relies on ``LD_LIBRARY_PATH`` resolving the plugin name. Sourcing
the ROS workspace's ``setup.bash`` sets that up correctly. Standalone
hosts (custom BT runners) need to point ``LD_LIBRARY_PATH`` at the
install ``lib/`` directory or pass the absolute path.

Run the runner directly::

    # Idle mode — listens on /fleet/cmd
    ros2 launch iros_llm_swarm_bt test_bt_runner.launch.py

    # Embedded integration scenario
    ros2 run iros_llm_swarm_bt test_bt_runner --ros-args -p scenario:=true

Both modes assume the rest of the swarm stack (planner + per-robot
followers + optional formation manager) is already running. The runner
itself does not bring the stack up.

Cross-references
----------------

* Action server consumed by ``MapfPlan``:
  ``iros_llm_swarm_mapf_lns/mapf_lns2_node`` (LNS2) or the legacy
  ``iros_llm_swarm_mapf/mapf_planner_node`` (PBS). Either one can sit
  behind ``/swarm/set_goals``.
* Service backend for ``SetFormation`` / ``DisableFormation``:
  ``iros_llm_swarm_formation/formation_manager_node``.
* LLM action server backing ``MapfPlan``'s advisory channel
  (``/llm/decision``): out of tree. The expected goal/result/feedback
  shape is ``LlmDecision.action`` in ``iros_llm_swarm_interfaces``.
* All message and action types: ``iros_llm_swarm_interfaces``.
