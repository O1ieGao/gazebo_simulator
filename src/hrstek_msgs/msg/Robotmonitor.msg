
std_msgs/Header header

std_msgs/String ip

# current pose\cov
nav_msgs/Odometry gloca

# current line velocity\angel velocity 
geometry_msgs/Twist cmd_vel

# battery info
hrstek_msgs/BatteryInfo battery_info

# dtc 
uint64[] dtc

# uint64[] persistent_dtc

# check if take control
std_msgs/Bool manual

# int16 estop

float32 cpu_usage_percent 

float32 ram_usage_percent 

float32 temperature 

float32 disk_usage_percent 

float32 lidar_status
