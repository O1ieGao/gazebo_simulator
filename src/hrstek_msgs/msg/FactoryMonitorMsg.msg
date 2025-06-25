std_msgs/Int32[] all_reserved_node_vec
std_msgs/Int32[] all_occupied_node_vec
std_msgs/Int32[] all_instock_node_vec

# 0 = normal, 1 = off, 2 = paused
int32 hanxin_state

# Reserved flag for testing purpose
bool error_flag_0
string owner_0

# Register an agent that is not close enough to nearest node
bool error_flag_1
string owner_1

# Plan path for an agent that is not close enough to starting node
bool error_flag_2
string owner_2

# Try to register and agent that is not exist(not detected)
bool error_flag_3
string owner_3

# Transportation task logical start or goal fail to be interpreted
bool error_flag_4
string owner_4

# Hanxin get no nodes or no edges from roadmap database
bool error_flag_5
string owner_5

# Path planning failed for one task
bool error_flag_6
string owner_6
