#header
std_msgs/Header header

#sensor roi mode
#order: point_cloud_lidar, laser_scan_lidar, detection_lidar, optronics, ultrasonics
int8 footprint
string[] sensor_name
int32[] roi 
int32[] obstacle_state
