# Formation configuration published by formation_manager_node

std_msgs/Header header

string   formation_id        # unique name e.g. "wedge"
string   leader_ns           # e.g. "robot_0"
string[] follower_ns         # e.g. ["robot_1", "robot_2"]

# Per-follower offset in leader BODY frame (forward=+x, left=+y).
# Length must match follower_ns.
geometry_msgs/Point[] offsets

# Bounding footprint for MAPF planner (Polygon in leader body frame).
geometry_msgs/Polygon footprint

bool active  # false = formation dissolved, followers go autonomous