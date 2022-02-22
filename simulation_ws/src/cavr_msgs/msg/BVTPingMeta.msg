# CAVR Blue View Sonar messages with some additional data
Header header
int8 head_num
int16 orig_xy_col
int16 orig_xy_row
int16 orig_rt_col
int16 orig_rt_row
string head_orientation # horizontal, vertical, down
float64 ping_num
float64 range_resolution
float64 stop_range
float64 start_range
float64 bearing_resolution
geometry_msgs/PoseStamped sonar_pose
nav_msgs/Odometry state
cavr_msgs/StateMeta state_meta
