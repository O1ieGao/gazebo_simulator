std_msgs/Header header
std_msgs/String ipaddr
nav_msgs/Odometry gloca

hrstek_msgs/BatteryInfo battery_info
std_msgs/Float32 odom_cov

# trajectory controller and agent status
std_msgs/Int32 robot_status
std_msgs/Int32 signal_strength
uint64[] dtc
uint64[] persistent_dtc
sensor_msgs/JointState joint_states

int8[] cargo_status
float32 odometer
string current_floor
string current_map
string extend_param