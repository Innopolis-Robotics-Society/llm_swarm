# Formation configuration published by formation_manager_node
# One message per formation, published on /formations/<id>/config

std_msgs/Header header

string   formation_id        # unique name e.g. "wedge"
string   leader_ns           # e.g. "robot_0"
string[] follower_ns         # e.g. ["robot_1", "robot_2"]

# Per-follower offset in leader BODY frame (forward=+x, left=+y).
# Length must match follower_ns.
geometry_msgs/Point[] offsets

# Dummy bounding footprint for MAPF planner (convex polygon in leader body frame).
# For PoC this is a circle approximation; real hull computed later.
geometry_msgs/Polygon footprint

bool active  # false = formation dissolved, followers go autonomous