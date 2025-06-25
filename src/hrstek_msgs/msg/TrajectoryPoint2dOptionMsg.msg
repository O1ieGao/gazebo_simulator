# 0: forward motion
# 1: lateral motion
# 2: follow up
# 3: follow up and lateral motion
uint8 motion_mode
# direction angle of agent in global coordinates
float64 direction_angle
int16 detection_field
#
string floor

Action[] pure_action

Action[] move_action

float32 cargo_angle

string node_user_info

string edge_user_info
