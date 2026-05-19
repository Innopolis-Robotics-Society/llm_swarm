Examples and Usage Guide
========================

This page is a tour of the operator-facing surface of the swarm: how to
talk to the LLM through the RViz panel, what the system actually does
with each command, the patterns the LLM is trained against, and the
fallbacks (click-to-command tools, CLI, dataset inspection) when the
panel is not the right tool.

It assumes the stack is already built and you can launch the full demo.
For the build / launch mechanics see :doc:`getting_started`. For the
component-level reference see :doc:`architecture` and :doc:`packages`.

.. contents:: On this page
   :local:
   :depth: 2

Bringing the panel up
---------------------

Inside the dev container::

    ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
        scenario:=amongus \
        planner:=lns \
        enable_passive_observer:=false \
        llm_backend:=http

Wait for the ``==== Full demo ready ====`` line (~20 s). RViz comes up
with the bundled ``swarm_20.rviz`` config, which preloads the operator
panel and the three click-to-command tools.

If the panel is not visible: **Panels → Add New Panel →
``iros_llm_rviz_panel/LLM-Panel``**. Drag it to the right side of the
RViz window; the dock split that the bundled config uses puts the panel
beside the 3D view.

Channel 1 (reactive ``/llm/decision``) is always on. Add
``enable_passive_observer:=true`` to also enable channel 2 — both run
in parallel without colliding (channel 2 checks the ``llm_thinking``
flag on ``/bt/state`` and skips while channel 1 is busy). Channel 3
(operator chat) is the panel's *Chat* tab and is always available.

Anatomy of a chat turn
----------------------

When you press **Send** in the *Chat* tab, this is what happens
end-to-end:

::

    ┌─────────────┐  text  ┌────────────┐  prompt   ┌──────────┐
    │ RViz panel  │───────▶│ chat_server│──────────▶│   LLM    │
    │ (Chat tab)  │        │ (channel 3)│           │ backend  │
    └─────────────┘        └────────────┘           └────┬─────┘
           ▲                      │ stream chunks         │ token stream
           │ feedback             │                       │
           │  (stage, chunk,      │ parse two-line        │
           │   detail)            │ output → reply +      │
           │                      │ plan JSON             │
           │                      ▼
           │              ┌──────────────────┐  /llm/command
           │              │  PlanExecutor    │──────────────▶  BT runner
           │              │  (sequence /     │
           │              │   parallel /     │
           │              │   leaves)        │
           │              └──────────────────┘
           │                      │
           │                      │ on failure: refresh runtime
           │                      │ context (MCP read-only) + retry
           │                      │ up to max_remediation_attempts
           │                      ▼
           │              result: final_reply,
           │              plan_json, success,
           │              plan_executed
           ▼
    LlmEvent on /llm/events
    JSONL append in ~/.ros/llm_commands/

Feedback stages on ``/llm/chat`` (visible in the Events tab and consumed
by the panel for the streaming reply):

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Stage
     - Meaning
   * - ``thinking``
     - Request sent, waiting for the first token from the LLM.
   * - ``streaming``
     - Each ``streaming`` feedback carries one ``chunk`` of the reply
       text. The Chat tab appends the chunk to the conversation view.
   * - ``parsed``
     - Plan JSON ready. ``detail`` carries the JSON; the Chat tab fills
       the plan tree if *Preview* is on.
   * - ``executing``
     - ``PlanExecutor`` is walking the plan tree. ``detail`` describes
       the current leaf (``mapf robot_ids=[...]``,
       ``formation cyan_line``, ``idle``).
   * - ``done`` / ``error``
     - Terminal. ``Result.success`` mirrors the stage.

Robot vocabulary (Among Us scenario)
------------------------------------

The ``amongus`` scenario maps the 20 robots into 5 colour groups, each
spawned in a different room. The system prompt and the runtime context
both reference these groups, so the LLM understands "yellow",
"жёлтые", "циан", and so on without further hints. Definitions live in
``iros_llm_swarm_simulation_lite/map_descriptions/amongus_description.yaml``.

.. list-table::
   :header-rows: 1
   :widths: 12 12 26 26 24

   * - Group
     - Robot ids
     - Spawn / home
     - Aliases
     - Notes
   * - cyan
     - 0, 1, 2, 3
     - Upper Engine ``(-21.9, 9.9)``
     - cyan, голубые, циан
     - Top-left dead-end spur.
   * - magenta
     - 4, 5, 6, 7
     - Electrical ``(-8.7, -5.9)``
     - magenta, розовые, магента
     - Lower-center-left.
   * - green
     - 8, 9, 10, 11
     - Navigation ``(28.5, 1.9)``
     - green, зелёные, зеленые
     - Far east dead-end spur.
   * - orange
     - 12, 13, 14, 15
     - Storage ``(0.6, -10.9)``
     - orange, оранжевые
     - Bottom-center; ~3 m formation room.
   * - yellow
     - 16, 17, 18, 19
     - Weapons ``(17.6, 10.7)``
     - yellow, жёлтые, желтые
     - Top-right dead-end spur.

Named locations (subset — see the YAML for the full 14):
``cafeteria (2.7, 10.1)``, ``reactor (-28.4, 0.9)``,
``medbay (-9.5, 4.0)``, ``admin (4.0, 0.0)``,
``navigation (29.5, 1.9)``, ``electrical (-8.7, -5.9)``,
``storage (0.6, -10.9)``, ``communications (17.6, -9.7)``.

English aliases include ``cafe``, ``caf``, ``med bay``, ``nav``,
``comms``, ``elec``, ``upper``, ``lower``. Russian aliases include
``кафетерий``, ``столовая``, ``оружейная``, ``навигация``, ``связь``,
``склад``, ``админ``, ``медотсек``, ``реактор``.

Other scenarios (``cave``, ``warehouse_2``, ``warehouse_4``,
``large_cave``) define their own groups, locations, and aliases in the
matching ``*_description.yaml``.

Your first chat command
-----------------------

Type into the Chat tab::

    yellow to weapons

Press **Send**. Within ~1–3 s of "thinking", the conversation view
streams::

    Sending yellow group to Weapons.

Open the *BT* tab — the ``mode`` row flips from ``idle`` to ``mapf`` and
``active_action`` becomes ``MapfPlan``. The *MAPF* tab shows the four
yellow robots as ``active``; the progress bar grows as they arrive.
Inspect the actual plan emitted by the LLM with::

    ros2 topic echo --once /llm/events

The ``output`` field carries the parsed JSON::

    {"type":"mapf",
     "robot_ids":[16,17,18,19],
     "goals":[[17.6,10.7],[17.6,10.7],[17.6,10.7],[17.6,10.7]],
     "reason":"yellow to weapons"}

The PlanExecutor walks the tree (here a single ``mapf`` leaf), packages
it as an ``LlmCommand{mode=mapf, robot_ids=[16,17,18,19], goals=[…]}``
and calls ``/llm/command``. ``LlmCommandReceiver`` (in the BT runner)
applies it to the BT blackboard, ``ModeDispatch`` ticks ``MapfPlan``,
and ``MapfPlan`` calls ``/swarm/set_goals`` on the active planner
(``mapf_lns2_node`` for ``planner:=lns``). The planner spreads the four
identical goals into a 1 m grid around the click and produces
collision-free time-indexed plans on ``/robot_*/mapf_plan``. The LNS2
follower (``lns_motion_controller``) starts publishing
``FollowerStatus``; the planner's schedule monitor watches deviation;
the action stays alive until every robot is within ``goal_reached_m``.

Plan structure cheat sheet
--------------------------

Leaf nodes:

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Type
     - Body
   * - ``mapf``
     - ``{"type":"mapf", "robot_ids":[...], "goals":[[x,y],...], "reason":"..."}``
   * - ``formation``
     - ``{"type":"formation", "formation_id":"<name>",
       "leader_ns":"robot_N", "follower_ns":[...],
       "offsets_x":[...], "offsets_y":[...], "reason":"..."}``
   * - ``idle``
     - ``{"type":"idle", "reason":"..."}``

Containers:

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Type
     - Body
   * - ``sequence``
     - ``{"type":"sequence", "steps":[...]}`` — execute children one by
       one in order.
   * - ``parallel``
     - ``{"type":"parallel", "steps":[...]}`` — execute children at the
       same time, wait for all.

Nesting is allowed. The system prompt also defines a few reason
prefixes on ``idle`` leaves that have special meaning:

* ``reply_only:`` — factual answer, no robot motion.
* ``operator stop`` — operator-requested stop.
* ``needs_help: <q>`` — the LLM cannot decide; the question is in
  ``reply``.
* ``clarify: <q>`` — the LLM picked a default but wants confirmation.

Common patterns
---------------

The system prompt is trained on the patterns below. Adapt them to your
own scenarios; the LLM understands the underlying intent (parallel vs.
sequence, single-group vs. multi-group, formation vs. autonomous, stop)
in both English and Russian.

.. list-table::
   :header-rows: 1
   :widths: 32 68

   * - Operator phrase
     - Plan
   * - ``yellow to weapons``
     - Single ``mapf`` leaf, four ids at one cluster goal.
   * - ``yellow to east, green to west``
     - ``parallel`` with two ``mapf`` leaves.
   * - ``cyan to hub, then form a wedge``
     - ``sequence``: cluster ``mapf`` (no formation yet, all 4 ids), then
       ``formation`` leaf with ``cyan_wedge``.
   * - ``cyan and magenta to hub, then cyan forms a line while magenta goes home``
     - ``sequence`` of two ``parallel`` blocks.
   * - ``magenta домой``
     - Single ``mapf`` leaf with ``goals`` set to spawn coordinates from
       ``robot_groups.magenta.home``.
   * - ``stop`` / ``стоп``
     - ``{"type":"idle", "reason":"operator stop"}``.
   * - ``where is robot 7?``
     - Reply-only; ``{"type":"idle", "reason":"reply_only: ..."}``. The
       reply field carries the answer; no robots move.

The system prompt also contains worked examples for the trickier
formation cases — see the next section.

Formation deep dive
-------------------

Formations are leader-follower. The ``formation_manager`` activates a
formation **only when every follower is within 0.5 m of**
``leader_xy + (offset_x, offset_y)``. A bare ``formation`` leaf sent
while followers are out of position will be **rejected**, no robot
moves, and the chat turn fails. The system prompt enforces a mandatory
pre-check that the LLM applies before emitting any formation leaf.

Common offset patterns (leader faces +x by default; +x = forward,
+y = left in the leader's body frame):

.. list-table::
   :header-rows: 1
   :widths: 24 38 38

   * - Pattern
     - offsets_x
     - offsets_y
   * - Line behind (3 followers)
     - ``[-1.5, -3.0, -4.5]``
     - ``[0.0, 0.0, 0.0]``
   * - Wedge (2 followers)
     - ``[-1.0, -1.0]``
     - ``[0.6, -0.6]``
   * - Echelon left (2 followers)
     - ``[-1.5, -3.0]``
     - ``[1.0, 2.0]``
   * - Abreast (2 followers)
     - ``[0.0, 0.0]``
     - ``[1.5, -1.5]``

**Staging rule.** Worked example with magenta (ids 4–7) currently
spread out::

    Operator: "magenta form a line"

    runtime_context.robots:
      "4": { "x":-9.45, "y":-6.65, "yaw":0.0, "stale_ms":48 }
      "5": { "x":-7.95, "y":-6.65, ... }
      "6": { "x":-9.45, "y":-5.15, ... }
      "7": { "x":-7.95, "y":-5.15, ... }

    Pre-check (offsets behind robot_4 = leader):
      target_5 = (-9.45 - 1.5, -6.65) = (-10.95, -6.65)   distance 3.00 m
      target_6 = (-9.45 - 3.0, -6.65) = (-12.45, -6.65)   distance 3.35 m
      target_7 = (-9.45 - 4.5, -6.65) = (-13.95, -6.65)   distance 6.18 m

    All > 0.5 m → stage first.

The LLM emits a sequence::

    {"reply":"Staging magenta followers behind robot_4, then forming a line.",
     "plan":{"type":"sequence","steps":[
       {"type":"mapf","robot_ids":[5,6,7],
        "goals":[[-10.95,-6.65],[-12.45,-6.65],[-13.95,-6.65]],
        "reason":"stage magenta followers at line offsets"},
       {"type":"formation","formation_id":"magenta_line","leader_ns":"robot_4",
        "follower_ns":["robot_5","robot_6","robot_7"],
        "offsets_x":[-1.5,-3.0,-4.5],"offsets_y":[0.0,0.0,0.0],
        "reason":"line behind robot_4"}
     ]}}

**Movement rule.** Once a formation is active, followers run in
``FORMATION_FOLLOWER`` mode and ignore their own MAPF goals. To move the
formation as a unit, the ``mapf`` leaf must contain **only the
leader's id**. The followers chase the leader's pose via
``/formations/config``. Operator phrasings that mean this:
"send the line to X", "move the wedge to Y", "формация magenta к
центру", "<color> line, head to <place>"::

    Operator: "send the magenta line to the central hub"
    runtime_context.formations:
      [ {formation_id:"magenta_line", leader_ns:"robot_4",
         followers:["robot_5","robot_6","robot_7"], status:"STABLE"} ]

    {"reply":"Moving robot_4 to the central hub — the magenta line follows.",
     "plan":{"type":"mapf","robot_ids":[4],"goals":[[-6.0,2.0]],
             "reason":"move magenta_line leader to central_hub"}}

If the operator asks to *form X and go to Y* in the same turn, the LLM
emits a three-step sequence: stage followers → activate formation →
move leader.

Preview, Execute, Cancel
------------------------

The Chat tab has a **Preview** checkbox above the input. When it is on,
the panel still calls ``/llm/chat`` but with
``execute_after_planning=false``. The orchestrator generates the reply
and the plan, but **does not** dispatch leaves; instead it returns the
plan JSON in the action result.

The panel parses the JSON and renders it as a tree under the
conversation view (one row per node, with type and key fields shown
inline). The bottom row exposes:

* **Execute** — sends the buffered plan JSON to ``/llm/execute_plan``.
  ``execute_server`` replays the plan deterministically (no second LLM
  call). Use this when you want to read the LLM's intent before it
  touches the robots.
* **Cancel** — drops the buffered plan and clears the tree. The robots
  do nothing.

When *Preview* is off, the LLM call and execution are atomic — the
plan dispatches as soon as the LLM emits ``parsed`` and the
PlanExecutor walks it; the tree is shown only as a record.

STOP ALL vs. ``idle``
---------------------

Two ways to halt the swarm; they look similar but go through different
code paths.

* **STOP ALL button** (top-right of the panel) — bypasses the LLM
  entirely. The panel publishes
  ``LlmCommand{mode="idle", reason="rviz_stop_all"}`` to ``/llm/command``
  directly. ``LlmCommandReceiver`` applies it to the BT blackboard;
  ``ModeDispatch`` switches to the idle branch on the next tick;
  ``DisableFormation`` deactivates any active formation; running MAPF
  goals are cancelled and ``/swarm/set_goals`` returns ``CANCELLED``.
  No JSONL record is written for channel 3 because no LLM call was
  made.
* **Typing "stop" / "стоп" in chat** — goes through the LLM, which
  returns
  ``{"reply":"Stopping all robots.", "plan":{"type":"idle","reason":"operator stop"}}``.
  The PlanExecutor turns the ``idle`` leaf into the same
  ``LlmCommand{mode=idle}`` call as STOP ALL, plus a JSONL record on
  ``~/.ros/llm_commands/``. Slower (one LLM round-trip), but creates a
  training datum. Prefer STOP ALL for emergencies.

Click-to-command tools (no LLM)
-------------------------------

When you want a deterministic, scriptable, non-LLM command path —
demos, ground-truth comparisons, tests — use the three RViz tools
from ``iros_llm_rviz_tool``:

.. list-table::
   :header-rows: 1
   :widths: 14 16 70

   * - Key
     - Tool
     - Workflow
   * - ``g``
     - ``SendLlmGoalTool``
     - Click on the map → group picker dialog → N goals are spread
       around the click point at 1.5 m spacing → dispatched as
       ``LlmCommand{mode=mapf}`` to ``/llm/command`` → BT picks it up.
       No ``/llm/chat`` call, no JSONL record on the chat dataset.
   * - ``b``
     - ``PlaceObstacleTool``
     - Choose **Shape** (Circle / Rectangle / Door) and the size in the
       Tool Properties panel. Left-click adds the obstacle at the
       clicked ground-plane point; right-click removes the nearest
       obstacle. Talks to ``/obstacles/{add_*, list, remove}``.
   * - ``d``
     - ``DoorTool``
     - Set the **Door ID** in the Tool Properties panel. Left-click
       calls ``/doors/open``; right-click calls ``/doors/close``.

These tools require ``iros_llm_swarm_obstacles`` to be running for
``b`` / ``d``. The full demo does not yet wire it in by default —
launch ``dynamic_obstacle_manager`` separately or compose it into a
custom launch::

    ros2 run iros_llm_swarm_obstacles dynamic_obstacle_manager \
        --ros-args \
            -p raw_map_topic:=/raw_map \
            -p scenario_file:=$(ros2 pkg prefix iros_llm_swarm_simulation_lite)/share/iros_llm_swarm_simulation_lite/scenario/common_scenarios.yaml \
            -p scenario_name:=amongus

(Repoint the static map server to publish on ``/raw_map`` first;
``zone_map_server`` and ``dynamic_obstacle_manager`` cannot both own
``/map`` simultaneously — see :doc:`packages` for the integration
note.)

Dynamic obstacles and doors via CLI
-----------------------------------

If RViz is not the right tool — scripted scenarios, automated tests,
remote operation — call the services directly::

    # Block a corridor with a 0.5 m circle
    ros2 service call /obstacles/add_circle \
        iros_llm_swarm_interfaces/srv/AddCircle \
        "{id: 'box1', position: {x: 10.0, y: 5.0, z: 0.0}, radius: 0.5}"

    # Drop a 1 × 2 m wall
    ros2 service call /obstacles/add_rectangle \
        iros_llm_swarm_interfaces/srv/AddRectangle \
        "{id: 'wall1', position: {x: -3.0, y: 0.0, z: 0.0}, width: 1.0, height: 2.0}"

    # Close a door declared in the scenario
    ros2 service call /doors/close \
        iros_llm_swarm_interfaces/srv/CloseDoor \
        "{door_id: 'corridor_center'}"

    # Read state
    ros2 service call /obstacles/list \
        iros_llm_swarm_interfaces/srv/ListObstacles "{}"

    # Remove
    ros2 service call /obstacles/remove \
        iros_llm_swarm_interfaces/srv/RemoveObstacle "{id: 'box1'}"

The merged map is republished on ``/map`` with ``TRANSIENT_LOCAL`` QoS
so any planner / costmap that joins late picks up the latest state.

Channel 1 — reactive
--------------------

Channel 1 fires when a BT action node produces a WARN or ERROR while it
is active. The most frequent triggers are MAPF planner failures and
formation rejection.

Trace one through the system::

    # Provoke a stress scenario from another shell
    ros2 run iros_llm_swarm_bt fleet_cmd --scenario stress

You should see in the launch log::

    [test_bt_runner] MapfPlan: action accepted
    [mapf_lns2_node] LNS2 timed out at iter 12 (4500 ms)
    [test_bt_runner] MapfPlan: status=WARN replans_done=2
    [decision_server] /llm/decision call (channel 1) — trigger: stress, status=WARN
    [decision_server] reply: replan
    [test_bt_runner] applying decision: replan

In the panel's *BT* tab, ``llm_thinking`` flips to ``true`` for the
duration of the LLM call, then back to ``false`` once a verdict
arrives. The *Events* tab shows one row with ``channel=1``,
``trigger="stress / status=WARN ..."``, ``output="replan"``, and the
LLM's ``reason`` text. The decision is also appended to
``~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl``.

Channel 2 — proactive
---------------------

Channel 2 watches ``/bt/state`` continuously. It fires when:

* ``action_status`` transitions OK → WARN/ERROR, **and**
* the cooldown (``cooldown_sec``, default 10 s) has elapsed since the
  last channel-2 call, **and**
* ``llm_thinking`` is ``false`` (channel 1 is not currently consulting
  the LLM).

Enable it with::

    ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
        enable_passive_observer:=true

Confirm the node is awake (not silent-mode) in the launch log::

    [llm_passive_observer] PassiveObserver up: mode=http, cooldown=10.0s,
        trigger=warn_or_error_or_formation_unhealthy

When it fires, it pushes an ``LlmCommand`` to ``/llm/command`` (mode
choice and arguments depend on the LLM's reasoning). The panel's
*Events* tab shows ``channel=2`` rows; the JSONL goes to
``~/.ros/llm_commands/`` (same directory as channel-3 chat).

Useful contrast with channel 1: channel 1 is a *point verdict* on a
specific BT node ("what should I, ``MapfPlan``, do about this WARN —
wait, abort, or replan?"). Channel 2 is a *mode change* on the whole
tree ("the formation has been DEGRADED for 12 s, take everyone idle
and resend MAPF goals"). They never collide — the cooldown plus the
``llm_thinking`` gate keep them in different time windows.

CLI alternatives
----------------

For repeatable scenarios — CI, demos, regression — bypass the panel:

* ``ros2 run iros_llm_orchestrator user_chat`` — the same channel-3
  pipeline driven from stdin. Reads a prompt, prints the streamed
  reply, executes the plan exactly like the panel. Useful for headless
  servers and scripted prompts (``echo 'yellow to weapons' | user_chat``).
* ``ros2 run iros_llm_swarm_bt fleet_cmd --scenario {simple|stress|unreachable|idle}``
  — direct ``LlmCommand`` injection without the LLM. Used to provoke
  channel-1 / channel-2 reactions deterministically.
* ``ros2 run iros_llm_swarm_mapf test_send_goals --json-file goals.json``
  — direct ``/swarm/set_goals`` action call. Bypasses the BT and the
  LLM entirely. The lowest layer you can drive from outside.

Inspecting the dataset
----------------------

Every LLM call lands in a JSONL file, one record per call, one file per
UTC day::

    ~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl   # channel 1
    ~/.ros/llm_commands/decisions_YYYYMMDD.jsonl    # channels 2 + 3

Schema (each line is a JSON object)::

    {
      "timestamp": "2026-05-12T14:23:11.481Z",
      "level":     "WARN",                   # channel 1: BT level; ch 2/3: trigger label
      "event":     "stress: replans=3 stalled=4",
      "log_buffer": "...recent log lines, channel 1 only...",
      "decision":  "replan",                 # ch 1: verdict; ch 2/3: command mode
      "reason":    "robots stalled in central corridor; replan around"
    }

Quick inspection::

    # Count today's decisions by verdict
    jq -r .decision ~/.ros/llm_decisions/decisions_*.jsonl | sort | uniq -c

    # Pull the last 5 chat-driven commands
    tail -n 5 ~/.ros/llm_commands/decisions_$(date -u +%Y%m%d).jsonl | jq .

Channel 3 also publishes the parsed plan JSON to
``/llm/events.output``, so you can stream live commands without
touching the JSONL files::

    ros2 topic echo /llm/events --filter "m.channel == 3"

Backend selection
-----------------

The orchestrator chooses the LLM client by the ``llm_mode`` parameter
(or the equivalent ``llm_backend`` launch argument).

.. list-table::
   :header-rows: 1
   :widths: 14 86

   * - Mode
     - When to use
   * - ``mock``
     - Offline development. Heuristic keyword matcher returns a
       hard-coded plan for a small set of known phrases. No network.
   * - ``ollama``
     - Local Ollama server (``http://localhost:11434``) running a
       model like ``llama3.2`` or ``qwen2.5``. Streaming chat. No API
       key.
   * - ``http``
     - Any OpenAI-compatible endpoint —
       ``https://api.openai.com/v1/chat/completions``,
       ``https://generativelanguage.googleapis.com``, vLLM,
       LM Studio, etc. Reads the API key from the env var named by
       ``llm_api_key_env`` (default ``LLM_API_KEY``).
   * - ``local``
     - In-process HuggingFace Transformers inference. Heaviest;
       suitable for benchmarking, not for interactive operation on
       modest hardware.

Examples::

    # Talk to a local Ollama server
    ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
        llm_backend:=ollama \
        llm_endpoint:=http://localhost:11434 \
        llm_model:=qwen2.5:7b-instruct

    # Talk to OpenAI
    export LLM_API_KEY=sk-...
    ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
        llm_backend:=http \
        llm_endpoint:=https://api.openai.com/v1/chat/completions \
        llm_model:=gpt-4o-mini

    # Offline smoke test
    ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
        llm_backend:=mock

The MCP context provider
------------------------

The chat channel can ground its prompts in live system state through a
read-only MCP subprocess (``uvx ros-mcp --transport=stdio``). It is on
by default (``context_provider: mcp_readonly``). What it adds to each
prompt:

* Map summary (bounds, named locations, obstacles).
* Live robot poses (``robots[id] = {x, y, yaw, stale_ms, stale}``).
  Stale entries are flagged so the LLM does not act on them.
* Active formations from ``/formations/status`` (state, leader,
  followers, errors).
* Recent ``LlmEvent`` tail from ``/llm/events``.

The provider is locked to a read-only allowlist
(``mcp_tool_allowlist`` in ``orchestrator.yaml``):
``get_topics``, ``get_topic_type``, ``subscribe_once``, ``get_nodes``,
``get_services``, ``get_actions`` and similar. **No execute or write
tools** ever reach the LLM — the LLM never sees the MCP tool list,
only the bounded snapshot the provider builds on its behalf.

On execution failure the chat channel calls
``_get_targeted_runtime_context(failure)``, which refreshes only the
tools relevant to the failure (e.g. formation issues re-subscribe to
``/formations/status``). If the targeted refresh times out, the
provider falls back to the cached snapshot with an
``mcp_stale: true`` warning so the LLM knows the data is old.

Disable the provider entirely with::

    ros2 launch iros_llm_orchestrator orchestrator.launch.py \
        --ros-args -p chat_server.context_provider:=none

Failure modes and what to do
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Symptom
     - Likely cause / fix
   * - Chat tab streams nothing for > 30 s, no error.
     - LLM backend is unreachable. Check ``llm_endpoint``, network,
       ``LLM_API_KEY``. Channel 3 timeout is 60 s; after that the
       Result returns ``success=False`` with the underlying error in
       ``info``.
   * - Reply arrives but plan is empty (``plan_json: ""``).
     - LLM returned malformed output (missing the JSON line, broken
       JSON). The Events tab shows ``output: ""``. Re-prompt with a
       simpler phrasing or switch the model.
   * - Plan parses, leaves dispatch, but BT goes ``ERROR`` immediately
       on a ``formation`` leaf with ``formation_max_error_m > 0.5``.
     - Followers were not pre-staged. Either (a) the LLM skipped the
       staging step (re-prompt), or (b) the followers' poses were stale
       and the staging math used wrong positions. Re-issue the command
       once odom updates land.
   * - Channel 2 never fires even with ``enable_passive_observer:=true``.
     - Either the cooldown is hiding it (wait > 10 s), or
       ``llm_thinking`` is stuck high (channel 1 hung; check
       ``decision_server`` log), or no WARN/ERROR ever transitions
       ``action_status``. Drive a deliberate failure with
       ``fleet_cmd --scenario stress``.
   * - Click-to-command tools have no effect.
     - For ``g``: the BT must be running. For ``b``/``d``: the
       ``dynamic_obstacle_manager`` must be running. Check
       ``ros2 node list``.
   * - "STOP ALL" button is greyed / no effect.
     - The panel cannot connect to the ``/llm/command`` action server.
       ``LlmCommandReceiver`` lives in ``test_bt_runner``; restart the
       BT runner.
   * - MCP context times out (``mcp_stale: true`` in events).
     - The subprocess is slow or misconfigured. Reduce
       ``context_max_chars`` or set ``context_provider: ros_readonly``
       to skip MCP and use direct ROS introspection instead.

See also
--------

* :doc:`getting_started` — bring-up steps and launch arguments.
* :doc:`architecture` — full topic / action / service map.
* :doc:`packages` — the package map and per-package roles.
* The per-package generated documentation linked from :doc:`index`
  (``iros_llm_orchestrator``, ``iros_llm_rviz_panel``,
  ``iros_llm_rviz_tool``, ``iros_llm_swarm_obstacles``).
