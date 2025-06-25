
# have this field to make the timestamp compatible with python script
# there is no direct to set universal time(int64) in python
std_msgs/Header header
#
int64 timestamp
#
float64 x
#
float64 y
#
float64 theta
#
float64 x_dot
#
float64 y_dot
#
float64 theta_dot

Action[] actions    # 到达此路径点后要执行的动作序列