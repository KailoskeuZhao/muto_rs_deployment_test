# One typed preflight result; Nav2 remains authoritative for execution.
uint8 UNKNOWN=0
uint8 REACHABLE=1
uint8 UNREACHABLE=2

uint8 state
string reason_code
float32 path_length_m
float32 estimated_time_s
uint64 costmap_revision
string freshness
bool projected
geometry_msgs/PoseStamped selected_pose
