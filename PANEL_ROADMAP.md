# `iros_llm_rviz_panel` — development roadmap

This document is a phased roadmap for `iros_llm_rviz_panel`, written for an
autonomous coding agent (Claude Code). Each phase is self-contained, with
clear deliverables and acceptance criteria. Execute phases in order unless
the human says otherwise.

The MVP is already in the repo. Read [`README.md`](./iros_llm_rviz_panel/README.md)
and the source files before touching anything — most context lives there.

---

## 0. Repository state assumptions

The panel package already contains:

```
iros_llm_rviz_panel/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml
├── README.md
├── include/iros_llm_rviz_panel/llm_panel.hpp
└── src/llm_panel.cpp
```

Sibling packages this depends on:

* `iros_llm_swarm_interfaces` — already extended with `LlmChat.action` and
  `LlmEvent.msg`. Re-build it before any phase if you've just pulled.
* `iros_llm_orchestrator` — exposes `/llm/chat` (Python action server in
  `chat_server.py`), `/llm/command` (legacy), and publishes `/llm/events`.
* `iros_llm_swarm_bt` — publishes `/bt/state` (BEST_EFFORT QoS, ~10 Hz from
  the BT tick loop). Don't modify this package.

If `colcon build --packages-up-to iros_llm_rviz_panel` doesn't succeed
*before* you start a phase, fix the build first and only then proceed.

---

## 1. Architecture invariants — DO NOT BREAK

Anything that violates these creates random-seeming RViz crashes that are
nightmare to debug. Treat as hard rules.

### 1.1 No widget access from ROS callbacks

ROS callbacks (subscriptions, action client feedback/result) run on the
executor thread. Qt widgets are GUI-thread-only. Bridge ALWAYS via Qt
signals with `Qt::QueuedConnection`. The MVP code already does this — keep
adding new ROS handlers the same way:

```cpp
// In a lambda passed to create_subscription / SendGoalOptions:
Q_EMIT mySignal(QString::fromStdString(msg->field));

// In a private slot, runs on GUI thread:
void onMySignal(QString data) { someWidget_->setText(data); }

// Wired up once in buildUi() / setupRos():
connect(this, &LlmPanel::mySignal, this, &LlmPanel::onMySignal,
        Qt::QueuedConnection);
```

Never call `someWidget_->...` from inside a lambda passed to ROS. Even if
"it works on my machine".

The single exception is `setupRos()` itself — it's called from
`onInitialize()` on the GUI thread, before any ROS callbacks fire.

### 1.2 ROS node is borrowed, not owned

The panel grabs RViz's shared node via
`getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()`.
Never call `rclcpp::init`, never construct your own node, never spin an
executor — RViz spins.

### 1.3 No long-running work on the GUI thread

If a feature needs >50ms of computation (parsing a large file, big regex
loop, etc.), do it in a worker `QThread` and emit results via signals.
Currently nothing in MVP does this; keep it that way.

### 1.4 Markers are published, not rendered in-panel

The panel publishes `MarkerArray` on `/llm_panel/markers`. Users add a
`MarkerArray` Display themselves. Don't bring `Ogre`, `rviz_rendering`,
or any 3D-scene API into this package.

### 1.5 Don't move logic out of the orchestrator

`PlanExecutor`, prompt building, plan parsing, plan post-processing — all
live in `iros_llm_orchestrator`. The panel asks the orchestrator to do
work via `/llm/chat` or `/llm/command`. Don't reimplement any of that in
C++.

### 1.6 Don't refactor the MVP for the sake of it

If a phase doesn't require touching MVP code, leave MVP code alone. Add
new files/widgets alongside, don't restructure the panel's internals.

---

## 2. Code style

* C++17 (do not bump to C++20 without asking the human).
* `-Wall -Wextra -Wpedantic` clean — zero warnings.
* Match existing style: 2-space indent, brace placement and spacing as in
  `src/llm_panel.cpp`.
* Qt: function-pointer signal/slot syntax (no `SIGNAL()/SLOT()` macros).
  Use `Q_EMIT`, not bare `emit`.
* ROS 2: standard rclcpp idioms. Use `rclcpp::QoS::best_effort()` for any
  topic that the BT side publishes BEST_EFFORT.
* One feature = one phase. No drive-by refactors.
* Comments in English. Sparse — explain *why*, not *what*. The MVP source
  is the style template.

---

## 3. Phases

Each phase below has: **Goal**, **Files**, **Notes**, **Acceptance**.

---

### Phase 1 — Polish chat streaming (filter to "reply" field)

**Goal.** Currently `chat_server.py` streams the *raw JSON* from Ollama;
the panel sees `{"reply":"...", "plan":{...` accumulating live. Port the
state machine from `user_chat_node._stream_command` (see
`iros_llm_orchestrator/iros_llm_orchestrator/user_chat_node.py`,
function `_stream_command`) into `chat_server.py` so feedback chunks only
carry the human-readable reply text.

**Files.**

* `iros_llm_orchestrator/iros_llm_orchestrator/chat_server.py` — replace
  the inner `async for chunk in self._ollama.stream(messages)` loop with
  the BEFORE_REPLY / IN_REPLY / AFTER_REPLY state machine. Continue
  accumulating `full_raw` (used for `parse_plan`) the same way.
* No panel changes.

**Notes.**

* The state machine is a few dozen lines and is already commented in the
  source. Copy it; don't redesign.
* Behaviour: feedback `chunk` should never contain JSON braces, escapes,
  or the substring `"plan"`. After the closing `"` of the reply value,
  stop emitting chunks, but keep accumulating `full_raw` for parsing.

**Acceptance.**

* `ros2 action send_goal /llm/chat iros_llm_swarm_interfaces/action/LlmChat \
   "{user_message: 'cyan to hub'}" --feedback`
  shows feedback chunks that look like prose, no JSON syntax.
* Final result still has populated `final_reply` and valid `plan_json`.

---

### Phase 2 — MAPF Live tab

**Goal.** Parse `BTState.action_summary` and show live MAPF stats:
progress bar, four counters, two sparklines, per-robot table.

**Files.**

* `include/iros_llm_rviz_panel/llm_panel.hpp` — declare new widgets and
  new private signals/slots.
* `src/llm_panel.cpp` — replace `mapf_tab` placeholder with real layout.
* New: `include/iros_llm_rviz_panel/action_summary.hpp` and
  `src/action_summary.cpp` — pure C++ parser, no Qt or ROS dependencies,
  unit-testable.
* New: `include/iros_llm_rviz_panel/sparkline.hpp` and
  `src/sparkline.cpp` — small `QWidget` subclass with `QPainter`-rendered
  sparkline. Ring buffer of last N samples, redraw on `update()`.
* New: `test/test_action_summary.cpp` — gtest unit tests for the parser.
* `CMakeLists.txt` — compile the new sources, add an `ament_add_gtest`
  block guarded by `if(BUILD_TESTING)`.
* `package.xml` — add `<test_depend>ament_cmake_gtest</test_depend>`.

**Notes.**

* Parser input examples:
  ```
  [t=1200ms status=executing arrived=5 active=15 stall=0 replans=0]
  [t=2400ms status=executing arrived=8 active=12 stall=0 replans=0] INFO: progressing normally
  [t=3600ms status=executing arrived=8 active=12 stall=1 replans=0] WARN: robot_3 stalled for 5s at (12.4, 8.1)
  ```
* Output struct:
  ```cpp
  struct ActionSummary {
    std::optional<int64_t>     elapsed_ms;
    std::optional<std::string> status;     // "executing" / "validating" / ...
    std::optional<int>         arrived;
    std::optional<int>         active;
    std::optional<int>         stall;
    std::optional<int>         replans;
    std::string                event_tail; // raw tail (INFO/WARN), or empty
  };
  ActionSummary parseActionSummary(std::string_view line);
  ```
* The parser must be tolerant: any field may be missing → `std::nullopt`.
  Do not throw on malformed input.
* Sparkline: ring buffer of last 60 samples (~6 s at 10 Hz). Custom
  `QWidget`, `paintEvent` draws polyline via `QPainter`. **Don't** pull in
  QtCharts — it's an extra dep we don't need.
* Robot table: `QTableWidget` with columns `id`, `goal`, `dist (m)`. The
  `dist` column stays empty in this phase — Phase 6 wires TF.
* Don't redraw the sparkline on every BTState. Use a `QTimer` at 30 Hz
  that calls `update()` if data changed.

**Acceptance.**

* With BT runner in scenario mode (`ros2 launch iros_llm_swarm_bt
  test_bt_runner.launch.py`), the MAPF tab updates in real time. Counters
  move; sparklines scroll.
* Switching `mode` to `idle` freezes counters at last value (intentional
  — no spam clearing).
* `colcon test --packages-select iros_llm_rviz_panel` runs the parser
  tests; all green. Cover at least: well-formed line, missing fields,
  reordered fields, empty string, trailing INFO/WARN.

---

### Phase 3 — LLM Events tab

**Goal.** Subscribe `/llm/events` and show a chronological log:
timestamp, channel pill, trigger, output, reason. Channel filter
checkboxes at the top.

**Files.**

* Header: declare `events_table_`, `event_filter_decision_`,
  `event_filter_observer_`, `event_filter_user_`, signal
  `eventReceived(...)`, slot `onEventReceived(...)`.
* Source: replace `events_tab` placeholder.

**Notes.**

* Cap the log at last ~500 entries (auto-evict from top). At 1 event/sec
  for an hour that's >3000; don't grow unboundedly.
* Channel codes are constants in `LlmEvent.msg`
  (`CHANNEL_DECISION = 1`, etc.). Render with the same colors as the
  status badges (DECISION blue, OBSERVER orange, USER green).
* Use a `QTableWidget` with truncated cell text. Click on a row → expand
  inline (or open a side panel) showing full `trigger` + `reason`.
* Subscribe with default reliable QoS — events are low-volume, missing
  one is bad.

**Acceptance.**

* Triggering MapfPlan WARN → row appears in Events tab within ~1 s,
  channel = DECISION, output one of `wait`/`abort`/`replan`.
* Sending a chat command → row appears with channel = USER, output =
  plan JSON, reason = LLM reply.
* Filter checkboxes hide/show channels live without losing buffered
  events.

---

### Phase 4 — Plan preview & confirm (human-in-the-loop)

**Goal.** Add a "Preview mode" toggle below the chat input. When on,
panel sends `LlmChat{execute_after_planning=false}`, renders the parsed
plan as a tree, shows preview markers, lets operator hit **Execute** or
**Cancel**.

**Files.**

* Header: add `preview_mode_checkbox_`, `plan_tree_view_`,
  `execute_button_`, `cancel_button_`, member `pending_plan_json_`.
* Source: extend `onSendChat` to read the checkbox, extend `onChatStage`
  to populate the tree on stage `parsed`, implement
  `onExecuteClicked` / `onCancelClicked`.
* Source: extend `publishMarkers` — when `pending_plan_json_` is
  non-empty, draw additional preview markers (alpha 0.4) for those
  goals in a separate ns `llm_panel/preview`.

**Notes — design decision required.**

Two paths to actually execute the previewed plan:

1. **Re-send option.** On Execute, panel sends a *second* `LlmChat` goal
   with the same `user_message` and `execute_after_planning=true`.
   Orchestrator re-generates and executes. Wasteful (one extra LLM call,
   different temp may give different plan) but no interface changes.

2. **New action option.** Add `LlmExecutePlan.action`:
   ```
   string plan_json
   ---
   bool   success
   string info
   ---
   string stage
   string detail
   ```
   Served by the orchestrator: parses `plan_json`, runs `PlanExecutor`,
   reports progress. Panel calls this on Execute with the plan it
   already has. Clean, deterministic.

Pick option (2). Option (1) is tempting because no interfaces change,
but the determinism is worth it — operator approves *exactly* the plan
that runs.

**Files (option 2).**

* `interfaces_additions/action/LlmExecutePlan.action` (new file in
  `iros_llm_swarm_interfaces`).
* `iros_llm_orchestrator/iros_llm_orchestrator/execute_server.py` (new
  action server, ~50 lines, reuses `parse_plan`, `_postprocess_plan`,
  `PlanExecutor`).
* `iros_llm_orchestrator/setup.py` — register the new entry point.
* `iros_llm_orchestrator/launch/orchestrator.launch.py` — start it.
* Panel changes as above.

**Notes — plan tree rendering.**

* Use `QTreeWidget`. Each node = sequence/parallel/mapf/formation/idle,
  mirroring the structure of `_describe_plan` in `user_chat_node.py`.
* For `mapf` leaves: show `n robots → reason`, expandable to per-robot
  rows `robot_<id> → (x.xx, y.yy)`.

**Acceptance.**

* Preview mode on: type a command → reply streams → tree appears →
  preview markers appear → Execute sends actual command → robots move.
* Cancel → no execution, preview markers disappear, chat goes idle.
* Preview mode off: behaves exactly like MVP (`execute_after_planning =
  true`, auto-execute).
* Modifying the chat reply by typing while previewing does nothing
  destructive — the pending plan is whatever was last `parsed`.

---

### Phase 5 — Quick actions strip

**Goal.** A horizontal button row beneath the tabs: hard-coded common
operations like "Send orange home", "Form wedge at center", "Reset all".
Reads map YAML from
`iros_llm_orchestrator/prompts/maps/<map_name>.yaml` to get robot groups
and named locations.

**Files.**

* Header: declare `quick_action_bar_`, slots per generated action.
* Source: parse YAML at panel construction. Build button row dynamically
  from groups in YAML.
* `package.xml`: add `<depend>libyaml-cpp-dev</depend>`.
* `CMakeLists.txt`: `find_package(yaml-cpp REQUIRED)`, link `yaml-cpp`.

**Notes.**

* Locate the YAML via `ament_index_cpp::get_package_share_directory(
  "iros_llm_orchestrator")` + `"/prompts/maps/" + map_name_ + ".yaml"`.
  Don't hardcode paths.
* Map name is a panel parameter. Add `map_name_` member, default
  `"cave"`. Persist via `save()` / `load()` — ties into Phase 8.
* For "Send <color> home", build an `LlmCommand` with `mode=mapf`,
  `robot_ids` from group's ids, `goals` from spawn positions (or repeat
  home centroid `len(ids)` times if no spawn dict). Skip the LLM
  entirely. The orchestrator's `_spread_goals` will not run for this
  path — that's fine, spawn positions are already non-overlapping.
* For "Form X at Y", use the formation_zones list from YAML to pick Y;
  group ids → leader is first id, others are followers in line offsets
  `[-1.5, -3.0, ...]`.
* Buttons should be visually grouped: leftmost a "🛑 Stop" duplicate of
  the top STOP ALL (for muscle-memory), then per-group home buttons,
  then formation buttons. Use `QToolButton` with text + emoji icon.

**Acceptance.**

* On launch with `map_name=warehouse`: two home buttons appear.
* On launch with `map_name=cave`: five home buttons appear.
* Clicking moves robots to home positions without invoking the LLM
  (verify by checking `/llm/events` — no event with channel=USER).
* Switching `map_name` parameter and reloading the panel layout
  regenerates buttons.

---

### Phase 6 — TF-aware visualization

**Goal.** Show arrows from each robot's current pose to its current
goal. Show a polygon connecting leader+followers when `mode=formation`.

**Files.**

* Header: add `tf_buffer_`, `tf_listener_`, `marker_timer_`. Cache last
  `BTState` fields needed for the timer.
* Source: instantiate `tf2_ros::Buffer` + `tf2_ros::TransformListener`
  in `setupRos`. Add a `QTimer` (5 Hz, so 200 ms) that calls a new
  `publishMarkersTimed()` looking up `map → robot_<id>/base_link` for
  each robot in `last_robot_ids_`, drawing an arrow from there to
  `last_goals_[i]`.
* `package.xml` / `CMakeLists.txt`: add `tf2_ros`, `tf2_geometry_msgs`.

**Notes.**

* Cache `last_robot_ids_`, `last_goals_`, `last_mode_`,
  `last_formation_id_`, `last_leader_ns_`, `last_follower_ns_`,
  `last_offsets_x_`, `last_offsets_y_` in the BTState callback. Read
  them under a `std::mutex` — the timer fires on the GUI thread, the
  cache is written from the executor thread.
* Use a different marker `ns` per layer:
  - `llm_panel/goals` — existing goal spheres (Phase MVP)
  - `llm_panel/arrows` — new pose→goal arrows
  - `llm_panel/formation` — new formation polygon
  Keep them visible alongside each other; users toggle via Display.
* Arrow scale: shaft 0.05, head_diameter 0.1, head_length 0.15.
  Color = `robotColor(id)`, alpha 1.0.
* Each TF lookup is in a try/catch (`tf2::TransformException`) — skip
  that arrow on miss, never log per-frame, never crash.
* Formation polygon: when `mode=formation`, look up TF for `leader_ns`
  and each `follower_ns[i]`. If all succeed, publish a `LINE_STRIP`
  marker connecting them in order leader → follower[0] → follower[1]
  → ... → leader (closed polygon).
* TF frames: the BT side uses `robot_<id>/base_link`. Confirm by
  inspecting `ros2 run tf2_tools view_frames` against a running stack
  before assuming.

**Acceptance.**

* Robots moving across map: arrows update in real time, follow each
  robot's actual position. No log spam from missing TFs at startup.
* Changing goal (new chat command) → arrow head jumps; tail follows
  the moving robot.
* Formation mode → polygon appears connecting leader and followers.
* Killing a robot's TF (e.g. stopping its odom publisher) → that
  robot's arrow simply disappears within 1 s; the rest unchanged.

---

### Phase 7 — Send-LLM-Goal Tool plugin (separate package)

**Goal.** A separate package `iros_llm_rviz_tool` providing a
`rviz_common::Tool` that lets the operator click on the map → small
popup chooses the robot group → an `LlmCommand` is sent.

This is a *Tool* plugin, not a *Panel* plugin — different extension
point, different `<class>` base type in `plugin_description.xml`.

**Files (new package).**

* `iros_llm_rviz_tool/CMakeLists.txt`
* `iros_llm_rviz_tool/package.xml`
* `iros_llm_rviz_tool/plugin_description.xml` — Tool manifest, separate
  from Panel manifest.
* `iros_llm_rviz_tool/include/iros_llm_rviz_tool/send_llm_goal_tool.hpp`
* `iros_llm_rviz_tool/src/send_llm_goal_tool.cpp`
* `iros_llm_rviz_tool/icons/send_llm_goal.svg` — ~32×32 icon.

**Notes.**

* Subclass `rviz_default_plugins::tools::PoseTool` (gives free 2D-click
  on the ground plane). Override `onPoseSet(double x, double y, double
  theta)` — `theta` is unused for our case.
* Group selection: spawn a `QInputDialog::getItem` with the list of
  groups from the same map YAML as Phase 5. Cache the YAML — don't
  reload on every click.
* After click + group selection: build an `LlmCommand{mode=mapf,
  robot_ids=group ids, goals=[(x,y)] * len(ids)}`, send to
  `/llm/command`. Don't spread goals — the orchestrator's
  `_spread_goals` runs on the chat path, not on `/llm/command` directly,
  so add a small client-side spread here (copy the algorithm from
  `iros_llm_orchestrator/iros_llm_orchestrator/user_chat_node.py`,
  function `_spread_goals`).
* Out-of-bounds check: read `bounds` from the YAML; if click is outside,
  call `setStatus("out of bounds")` and don't send anything.

**Acceptance.**

* In RViz, after building the new package, "Send LLM Goal" appears in
  the toolbar with a recognizable icon.
* Click on map → popup with groups → pick "orange" → orange robots
  drive there.
* Click outside `bounds` from YAML → status bar shows "out of bounds",
  no command is sent.
* Coexists with the panel — both can be active at once.

---

### Phase 8 — Settings persistence

**Goal.** `Panel::load` / `save` actually persist things across RViz
sessions (currently stubbed): tab index, preview mode default, map
name, events history size.

**Files.**

* `src/llm_panel.cpp` — `load(const Config &)` reads typed values via
  `config.mapGetString` etc. `save(Config)` writes them.

**Notes.**

* RViz config IO uses `rviz_common::Config` map-style API. See
  `rviz_default_plugins`' `image_display.cpp` or similar for examples.
* Don't persist the chat scrollback — it's transient.
* Don't persist sparkline data — also transient.

**Acceptance.**

* Set preview mode on, switch to Events tab, save RViz layout
  (`File → Save Config As`), restart RViz, layout reloads → both
  choices are persisted.

---

## 4. Cross-cutting concerns

### 4.1 Testing strategy

* Unit-test pure logic only (parsers, color pickers, plan tree
  formatters). Don't try to test Qt widgets — adds complexity for
  marginal value.
* For each phase that introduces parsing or non-trivial formatting,
  add a `test/test_*.cpp` and an `ament_add_gtest` target.
* Optional integration test: a small `examples/` ROS launch that brings
  up the panel + a Python fake `/llm/chat` server that echoes a canned
  plan. Lets a human exercise the chat flow without Ollama.

### 4.2 Error handling philosophy

* Panel must never crash on missing/malformed data. Empty `BTState` →
  "—" everywhere. Action server unreachable → red text in the chat
  view, no exception.
* Don't `throw` from any ROS callback or Qt slot. Catch and log via
  `RCLCPP_WARN`.

### 4.3 Performance budget

* `BTState` arrives at ~10 Hz. Each tick: status update (cheap) + marker
  publish (cheap, ~40 markers max) + a few hash lookups. Target <1 ms
  per tick on the GUI thread.
* Sparkline / arrow redraw: cap at 5–30 Hz via `QTimer`. Don't redraw
  on every `BTState`. Buffer values, redraw on timer.

### 4.4 Logging

* Use `RCLCPP_INFO` / `RCLCPP_WARN` / `RCLCPP_ERROR` from
  `node_->get_logger()` for ROS-side events.
* For panel internal debug, use `qDebug()` — shows up in RViz stderr.
* No `std::cout` / `printf`. Ever.

### 4.5 Build hygiene

* After every phase: `colcon build --packages-select iros_llm_rviz_panel`
  followed by `colcon test --packages-select iros_llm_rviz_panel` must
  succeed with zero new warnings or test failures.
* If a phase changes interfaces, rebuild interfaces first
  (`colcon build --packages-select iros_llm_swarm_interfaces`) and
  source `install/setup.bash` before re-building the panel.

---

## 5. Out of scope (do NOT do)

* Don't build a 3D rendering layer in the panel. `MarkerArray` only.
* Don't add a Python wrapper or rqt fallback. RViz2 doesn't load Python
  panels reliably; we picked C++ for a reason.
* Don't reimplement `PlanExecutor` in C++. Always go through the
  orchestrator.
* Don't add an internal LLM call in the panel. The panel never talks to
  Ollama directly.
* Don't make the panel depend on Stage / Gazebo / nav2 specifically. It
  must work with any backend that publishes `/bt/state` and serves
  `/llm/chat` + `/llm/command`.
* Don't bundle orchestrator changes into this package. Orchestrator
  changes go in `iros_llm_orchestrator`.
* Don't replace `rviz_common::Panel` with a `QDockWidget` to "make it
  detachable" — RViz already supports detaching panels via right-click.

---

## 6. Glossary — interfaces the panel touches

### `/bt/state` — `iros_llm_swarm_interfaces/msg/BTState`

BEST_EFFORT QoS, ~10 Hz. Published from `BTStatePublisher` in
`iros_llm_swarm_bt`.

| field             | type                       | meaning                                      |
|-------------------|----------------------------|----------------------------------------------|
| `mode`            | string                     | `idle` / `mapf` / `formation`                |
| `action_status`   | string                     | `OK` / `WARN` / `ERROR`                      |
| `active_action`   | string                     | `MapfPlan` / `SetFormation` / `DisableFormation` / `none` |
| `action_summary`  | string                     | parsed in Phase 2 (see format above)         |
| `last_error`      | string                     | empty unless `action_status != OK`           |
| `formation_id`    | string                     |                                              |
| `leader_ns`       | string                     | e.g. `"robot_0"`                             |
| `robot_ids[]`     | uint32[]                   | ids in the current mission                   |
| `goals[]`         | geometry_msgs/Point[]      | one per `robot_ids[i]`                       |
| `stamp_ms`        | int64                      | publish time, ms since epoch                 |
| `llm_thinking`    | bool                       | true while channel-1 LLM call is in flight   |

### `/llm/chat` — `iros_llm_swarm_interfaces/action/LlmChat`

* Goal: `string user_message`, `bool execute_after_planning`.
* Result: `string final_reply`, `string plan_json`, `bool plan_executed`,
  `bool success`, `string info`.
* Feedback stages: `thinking`, `streaming`, `parsed`, `executing`,
  `done`, `error`. `chunk` is non-empty during `streaming`. `detail`
  carries plan JSON during `parsed`, current step description during
  `executing`, error text during `error`.

### `/llm/command` — `iros_llm_swarm_interfaces/action/LlmCommand`

Goal:

```
string mode                         # "idle" | "mapf" | "formation"
string reason
uint32[] robot_ids                  # mapf only
geometry_msgs/Point[] goals         # mapf only, length == robot_ids
string formation_id                 # formation only
string leader_ns                    # formation only
string[] follower_ns                # formation only
float64[] offsets_x                 # formation only
float64[] offsets_y                 # formation only
```

Result: `bool success`, `string info`.

### `/llm/events` — `iros_llm_swarm_interfaces/msg/LlmEvent`

Reliable QoS, low volume. Published by orchestrator on every channel-1
verdict, channel-2 command, channel-3 chat turn.

```
uint8 CHANNEL_DECISION = 1   # /llm/decision (BT → wait/abort/replan)
uint8 CHANNEL_OBSERVER = 2   # PassiveObserver → /llm/command
uint8 CHANNEL_USER     = 3   # /llm/chat or RViz panel

int64  stamp_ms
uint8  channel
string trigger     # what caused this event
string output      # LLM output (verdict / command / plan JSON)
string reason      # LLM-provided justification
```

### Map YAMLs

`iros_llm_orchestrator/prompts/maps/{warehouse,cave}.yaml`. Source of
truth for: `bounds`, `named_locations`, `robot_groups[*].{ids, color,
home, spawn, aliases}`, `formation_zones[*]`, `heuristics`. Phase 5
and Phase 7 both read these.
