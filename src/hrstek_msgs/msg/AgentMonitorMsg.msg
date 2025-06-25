std_msgs/Header header
std_msgs/String ipaddr
nav_msgs/Odometry gloca
std_msgs/Int32 exec_status
std_msgs/Int32 robot_node_id
geometry_msgs/Twist cmd_vel
hrstek_msgs/BatteryInfo battery_info
uint64[] dtc
uint64[] persistent_dtc
std_msgs/Bool manual
int16 estop
float32 cpu_usage_percent
float32 ram_usage_percent
float32[] temperature_zone
float32 disk_usage_percent

# -1 lose connection, 0 reconnecting, 1 normal
int8[] lidar_status
string extend_param
