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

# True while a BT node has an active /llm/decision request in flight.
# PassiveObserver checks this to avoid double-triggering channel 2
# when channel 1 is already consulting the LLM for the same event.
bool llm_thinking

# Formation monitor status — filled from /formations/status every tick
# whenever formation_id is non-empty. Mirrors FormationStatus constants:
#   STATE_INACTIVE=0  STATE_FORMING=1  STATE_STABLE=2
#   STATE_DEGRADED=3  STATE_BROKEN=4
# DEGRADED escalates action_status → WARN.
# BROKEN  escalates action_status → ERROR.
# This allows PassiveObserver and the user chat to react to formation
# health changes even after SetFormation has already returned SUCCESS.
uint8   formation_state           # FormationStatus.STATE_*
uint8   formation_failure_code    # FormationStatus.FAILURE_*
string  formation_failure_reason  # human-readable, empty if no failure
float32 formation_max_error_m     # max follower position error [m], -1 = n/a
float32 formation_mean_error_m    # mean follower position error [m], -1 = n/a

# Timestamp (ms since epoch).
int64 stamp_ms
