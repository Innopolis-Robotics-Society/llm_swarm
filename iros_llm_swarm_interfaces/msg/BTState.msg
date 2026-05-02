# Snapshot of the Nav2 BT state, published each tick by BTStatePublisher.
# Consumed by PassiveObserver (Python) which decides whether to emit a
# /llm/command goal in response to WARN/ERROR transitions.

# Current mode driven by @mode on the blackboard.
string mode                         # "idle" | "mapf" | "formation"

# Status of the currently active BT action node.
string action_status                # "OK" | "WARN" | "ERROR"

# Which BT node is currently active (or "none").
string active_action                # "MapfPlan" | "SetFormation" | "DisableFormation" | "none"

# Human-readable one-line summary of the last feedback / error.
string action_summary

# Last error surfaced by the active action (empty when OK).
string last_error

# Context of the current task — populated whenever relevant.
uint32[] robot_ids                  # mapf context
geometry_msgs/Point[] goals         # mapf context
string formation_id                 # formation context
string leader_ns                    # formation context

# Timestamp (ms since epoch).
int64 stamp_ms
