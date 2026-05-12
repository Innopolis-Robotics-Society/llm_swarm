# iros_llm_rviz_panel

RViz2 panel for the **iros_llm_swarm** operator UI.

## MVP scope (this version)

* **Status bar** — live `mode` / `active_action` / `action_status` /
  `llm_thinking` / `last_error` from `/bt/state`. Includes a **STOP ALL**
  button that publishes `LlmCommand{mode=idle}` directly to `/llm/command`,
  bypassing the LLM.
* **Chat tab** — sends operator text to `/llm/chat` (action). Streams the
  reply chunk-by-chunk into the chat view via action feedback.
* **Marker publisher** — when `mode=mapf`, publishes goal spheres + text
  labels on `/llm_panel/markers`. Add a `MarkerArray` Display in RViz to
  see them.
* **MAPF Live** and **LLM Events** tabs are placeholders for v2.

## Prerequisites

This package depends on two new things in `iros_llm_swarm_interfaces`:

* `action/LlmChat.action`
* `msg/LlmEvent.msg`  (used by v2 — not consumed by the MVP panel, but keep
  them in the same patch so you don't have to rebuild interfaces twice)

The orchestrator package needs an action server on `/llm/chat`. See
`orchestrator_changes.md` next to this README for the wiring.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select iros_llm_swarm_interfaces \
                               iros_llm_orchestrator \
                               iros_llm_rviz_panel
source install/setup.bash
```

## Use

```bash
rviz2
```

In RViz: **Panels → Add New Panel → iros_llm_rviz_panel/LLM-Panel**.

To see goal markers in the 3D scene:
**Add Display → MarkerArray → Topic = /llm_panel/markers**.

## Threading note

ROS callbacks fire on the executor thread; Qt widgets must only be touched
from the GUI thread. The panel bridges the two via Qt signals with
`Qt::QueuedConnection` (see `buildUi()`). Don't call `chat_view_->append()`
or any other widget method from inside a ROS callback — it will segfault
in the wild and pass on a quiet workstation, which is the worst-case mix.

## File map

```
iros_llm_rviz_panel/
├── CMakeLists.txt
├── package.xml
├── plugin_description.xml         # pluginlib manifest
├── include/iros_llm_rviz_panel/
│   └── llm_panel.hpp              # LlmPanel class declaration
└── src/
    └── llm_panel.cpp              # implementation + PLUGINLIB_EXPORT_CLASS
```
