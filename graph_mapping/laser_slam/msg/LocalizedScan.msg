# The reference frame and time point w.r.t which sensor pose is stored
# Note that header.stamp might be different from scan.header.stamp
Header header

# Original scan
sensor_msgs/LaserScan scan

# Point cloud in sensor frame at timepoint header.stamp, corrected for robot movement
sensor_msgs/PointCloud cloud

# Pose of sensor in reference frame
geometry_msgs/Pose sensor_pose

# Barycenter of cloud in ref frame
geometry_msgs/Point barycenter