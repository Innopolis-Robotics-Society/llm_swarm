# Full MAPF plan for one robot, published on /<ns>/mapf_plan.
#
# `plan_id` is a monotonically-increasing token assigned by the planner.
# Followers echo it in their FollowerStatus so MAPF can tell which plan
# a status refers to and ignore stale statuses during replan transitions.
#
# An empty `steps` list is a CANCEL signal: the follower cancels any
# in-flight Nav2 goal and transitions to IDLE. Planners use this to stop
# robots marked unplanable or to reset before a fresh set_goals action.
#
# header.stamp is when the plan was issued by the planner.
# header.frame_id is the frame of every step's target (typically "map").

std_msgs/Header header
uint32 plan_id
iros_llm_swarm_interfaces/MAPFStep[] steps
