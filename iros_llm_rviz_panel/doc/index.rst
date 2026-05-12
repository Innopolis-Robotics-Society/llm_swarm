iros_llm_rviz_panel
===================

A single docked RViz2 ``Panel`` plugin
(``iros_llm_rviz_panel/LLM-Panel``) that is the operator's primary
surface on the swarm. Five tabs plus a sticky status bar with a
**STOP ALL** button.

Loading the panel
-----------------

In RViz2: **Panels → Add New Panel → ``iros_llm_rviz_panel/LLM-Panel``**.
The panel is preloaded by ``swarm_full_demo.launch.py`` via the bundled
``swarm_20.rviz`` config.

Layout
------

* **Status bar** (top, always visible)

  * Mode badge — ``mapf | formation | idle`` (background colour from
    ``modeColor()``).
  * Action badge — ``OK | WARN | ERROR | HALTED`` (colour from
    ``statusColor()``).
  * ``LLM thinking`` indicator — driven by the ``llm_thinking`` field
    of ``/bt/state``.
  * Sticky last-error label — lingers for 5 s after the error clears.
  * **STOP ALL** button — publishes
    ``LlmCommand{mode=idle}`` directly to ``/llm/command``. No LLM in
    the loop.

* **Chat tab** — the channel-3 entry point.

  * Read-only conversation view (``QTextEdit``) streamed chunk-by-chunk
    from ``/llm/chat`` action feedback.
  * Input line + **Send** button + **Preview** checkbox.
  * When *Preview* is on, the parsed plan JSON is rendered as a
    ``QTreeWidget`` and the **Cancel** / **Execute** buttons activate.
    *Execute* sends the plan to ``/llm/execute_plan``.

* **MAPF tab** — fleet progress.

  * Progress bar (arrived / total) + per-row labels (active, stalled,
    replans, last event tail).
  * Two custom ``Sparkline`` widgets (60-sample ring buffer) for active
    and arrived counts.
  * Per-robot status table.

* **Events tab** — live tail of ``/llm/events``.

  * Five-column table: ``time``, ``channel``, ``trigger``, ``output``,
    ``reason``.
  * Per-channel filter checkboxes (decision, observer, user).

* **BT tab** — current ``/bt/state`` and a transition log.

  * Eleven-row property table: ``mode``, ``action_status``,
    ``active_action``, ``last_error``, ``formation_id``, ``leader_ns``,
    ``robots``, ``goals``, ``llm_thinking``, ``stamp``,
    ``action_summary``.
  * Transition log (max 200 rows).

* **Info tab** — async-refreshed parameter snapshot of the running
  stack.

ROS interfaces
--------------

**Subscribers:**

* ``/bt/state`` (``iros_llm_swarm_interfaces/BTState``, reliable, depth
  20).
* ``/llm/events`` (``iros_llm_swarm_interfaces/LlmEvent``, reliable,
  depth 50).

**Publishers:**

* ``/llm_panel/markers`` (``visualization_msgs/MarkerArray``) — goal
  spheres, text labels, TF-aware arrows, formation polygon. Refreshed
  on a 30 Hz timer when ``mode == mapf``.

**Action clients:**

* ``/llm/chat`` (``LlmChat``) — chat tab.
* ``/llm/command`` (``LlmCommand``) — STOP ALL button (and the
  chat-with-execute path).
* ``/llm/execute_plan`` (``LlmExecutePlan``) — *Execute* button on the
  pending plan.

Threading model
---------------

All ROS callbacks fire on the executor thread; **all widget access
happens on the Qt GUI thread**. Cross-thread bridging is done with Qt
signals declared in ``llm_panel.hpp`` (``btStateReceived``,
``chatChunkReceived``, ``chatStageReceived``, ``chatFinished``,
``execStageReceived``, ``execFinished``, ``eventReceived``,
``systemInfoChanged``) and connected with
``Qt::QueuedConnection`` in ``llm_panel.cpp``. Two mutexes protect
state shared between the executor thread and the GUI thread:

* ``cached_state_mutex_`` — last BT state used by the marker timer.
* ``info_mutex_`` — async parameter-client results.

Touching widgets directly from a ROS callback will segfault under load
even when it appears to work on a quiet workstation.

Source layout
-------------

* ``include/llm_panel.hpp`` / ``src/llm_panel.cpp`` — main panel.
* ``include/action_summary.hpp`` / ``src/action_summary.cpp`` — pure-C++
  parser for the ``action_summary`` field of ``BTState``
  (``[t=Xms status=Y arrived=A active=B stall=S replans=R]`` style).
* ``include/sparkline.hpp`` / ``src/sparkline.cpp`` — minimal custom
  ``QWidget`` for the MAPF tab sparklines.
* ``test/test_action_summary.cpp`` — gtest cases for the parser
  (well-formed, malformed, reordered, missing fields).

Build
-----

Standard ament + CMake. ``CMAKE_AUTOMOC`` is ON to drive Qt's MOC over
the ``Q_OBJECT`` classes. The plugin is installed via
``pluginlib_export_plugin_description_file(rviz_common
plugin_description.xml)``.
