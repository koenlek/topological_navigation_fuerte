# Node Parameters
uint32 index

# Contains a translation and rotation
geometry_msgs/Transform transform

# Camera parameters from the K matrix
float64 fx
float64 fy
float64 cx
float64 cy

# Only relevant for a stereo camera
float64 baseline

# Whether the camera is fixed in space: i.e., its position is known
bool fixed
