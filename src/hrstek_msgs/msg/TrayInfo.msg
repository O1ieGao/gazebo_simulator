# header
std_msgs/Header header

# tray status
# 1 - follow
# 2 - stop
# 3 - only rotate
int32 tray_status

#current tray position, unit rad
float32 tray_current_position

#current tray velocity, unit rad/s
float32 tray_velocity

uint64[] error_code 
