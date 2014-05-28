# Info stored with an edge in 2d topological map

# Id of this edge in the topological map
uint32 id

# Id of the source node
uint32 src

# Id of the destination node
uint32 dest

# Pose of origin of destination node's coordinate frame
# represented in the source node's coordinate frame
geometry_msgs/Pose offset
