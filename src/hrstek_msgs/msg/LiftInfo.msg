# header
std_msgs/Header header

# current lift position, unit mm
int32 lift_motor_current_position

# whether lift motor is idle
bool lift_motor_idle

# lift task status
# 0  - idle
# -1 - normal pause
# -2 - soft estop pause
# -3 - joystick interruption
# +n - req id for lift task
int32 lift_task_status

# lift velocity in mm/s
int16 lift_velocity

# cargo weight, unit in kg
int16 cargo_weight

uint64[] error_code
