# header
std_msgs/Header header

# joint name
string[] joint_name

# command_type
# 0 - the command not for this joint
# 1 - velocity control for this joint
# 2 - position control for this joint
int8[] command_type

# velocity, unit in m/s or rad/s
float32[] velocity

# position, unit in m or rad
float32[] position

