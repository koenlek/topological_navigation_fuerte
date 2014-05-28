# Info stored with a waypoint node in the topological roadmap

# Id of this node in the roadmap
uint32 id

# Id of the topological map grid wrt which this waypoint position is defined
uint32 grid

# Relative position in the grid frame
geometry_msgs/Point position