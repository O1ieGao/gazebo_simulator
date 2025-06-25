
# pointcloud information
sensor_msgs/PointCloud2 pointcloud

# this param indicate the type of input pointcloud
# type = 0, reserved, default value
# type = 1, 3d planar surface pointcloud
# type = 2, 2d planar contour pointcloud
# type = 3, 2d planar surface pointcloud
# type = ...
int16 type

# A location that help identify the location of the pointcloud
# To be defined
geometry_msgs/Pose reference_pose

# This field shows the mode of the point cloud
# mode = 1, global pointcloud, one-time request
# mode = 2, local pointcloud, continued with previously sent message
# mode = ...
int16 mode

