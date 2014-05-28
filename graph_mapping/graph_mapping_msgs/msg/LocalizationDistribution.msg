# Represents a probability distribution over the robot's pose
# w.r.t the pose graph

time stamp

# Each sample has a frame_id of the form 'node42', and an offset
# wrt that node.  The stamp field of the poses is not used: they're
# assumed to share the stamp field of this object.
geometry_msgs/PoseStamped[] samples



