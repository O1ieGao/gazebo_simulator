# header of the command
std_msgs/Header header

# priority
int8 priority

# action other than lifting
int16 action

# 0 - stop lift
# 1 - position contrl, lift up to lift_position at lift_speed 
# 2 - speed control, maintain lift_speed until command timeout
# 3 - speed direction control, lift_speed indicates direction, no accurate speed control
int16 lift

# lift position in mm
int16 lift_position

# lift speed in mm/s
int16 lift_speed