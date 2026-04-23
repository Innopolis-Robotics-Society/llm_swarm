# State constants
uint8 STATE_INACTIVE  = 0   # formation not active
uint8 STATE_FORMING   = 1   # active, followers converging to offsets
uint8 STATE_STABLE    = 2   # all followers within cohesion threshold
uint8 STATE_DEGRADED  = 3   # some followers out of tolerance but recovering
uint8 STATE_BROKEN    = 4   # formation cannot be maintained (follower lost/stuck)

# Failure code constants
uint8 FAILURE_NONE        = 0
uint8 FAILURE_FOLLOWER_LOST    = 1   # odom stopped arriving
uint8 FAILURE_FOLLOWER_STUCK   = 2   # error not decreasing
uint8 FAILURE_LEADER_LOST      = 3   # leader odom stopped arriving

# Header 
std_msgs/Header header
string          formation_id
string          leader_ns
string[]        follower_ns

# Discrete state 
uint8  state           # one of STATE_* constants above
uint8  failure_code    # one of FAILURE_* constants, FAILURE_NONE if state != BROKEN
string failure_reason  # human-readable, empty if no failure

# Per-follower errors
# Parallel to follower_ns. Position error [m] from assigned offset.
# -1.0 = data not available for this follower.
float32[] follower_errors_m

# Aggregate metrics
float32 max_error_m   # max of follower_errors_m, -1 if unavailable
float32 mean_error_m  # mean of follower_errors_m, -1 if unavailable