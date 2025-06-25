std_msgs/Header header
float64 time_to_fail
int32 failure_level
## failure_level = 1 
## emergency bumper triggered, immediate stop, obstacle (almost) touch
## failure_level = 2
## virtual bumper triggered, predicted stop soon, need deceleration
## failure_level = 3
## virtual bumper triggered, may stop in the future, no deceleration
## failure_level = 10
## nothing happened, safe to go with full speed
