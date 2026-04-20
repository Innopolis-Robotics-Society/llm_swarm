# Execution status of one robot's path_follower, published on
# /<ns>/follower_status.
#
# Published on every state transition AND at a heartbeat rate
# (default 2 Hz) so MAPF can tell a "frozen/crashed follower" apart
# from a "legitimately holding" one.
#
# `plan_id` is the MAPFPlan.plan_id currently being executed. Set to 0
# when the follower is IDLE (no plan) or hasn't yet processed any plan.
# MAPF compares this to its own last-published plan_id to detect stale
# statuses during replan transitions.

std_msgs/Header header
uint32 plan_id

# State machine values
uint8 STATE_IDLE          = 0    # no plan / cancelled / not in AUTONOMOUS mode
uint8 STATE_NAVIGATING    = 1    # Nav2 is driving toward some step's target
uint8 STATE_HOLDING       = 2    # waiting hold_sec at current step's target
uint8 STATE_PLAN_COMPLETE = 3    # all steps of current plan executed successfully
uint8 STATE_FAILED        = 4    # Nav2 aborted; follower stays here until new plan
uint8 state

# Progress within current plan
uint32 current_step_index      # which MAPFStep we're on
uint32 num_steps               # echo of plan.steps.size() for convenience

# Hold countdown (only meaningful when state == STATE_HOLDING)
float32 hold_remaining_sec

# Cumulative count of Nav2 aborts since the current plan started.
# Resets on every new plan.
uint32 nav2_failures

# Reason codes for STATE_FAILED. Nav2's action server only signals
# ABORTED at this layer — the "No valid trajectories" vs "Failed to make
# progress" distinction lives inside controller_server logs and doesn't
# reach us. Fine-grained reasons can be added later if needed.
uint8 FAIL_NONE             = 0
uint8 FAIL_NAV2_ABORTED     = 1    # Nav2 action aborted mid-execution
uint8 FAIL_NAV2_REJECTED    = 2    # Nav2 action server rejected the goal
uint8 FAIL_NAV2_SERVER_DOWN = 3    # Nav2 action server not available
uint8 last_failure_reason
