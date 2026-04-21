# One step in a MAPF plan.
#
# Semantics: "drive to `target` and, once there, wait `hold_sec` seconds
# before moving on to the next step". A step with hold_sec = 0 has no
# wait; the follower proceeds to the next step immediately.
#
# `target` is in the map frame (matches MAPFPlan.header.frame_id).

geometry_msgs/Point target
float32 hold_sec
