# Info stored with a node in a 2d topological map

# Id of this node in the topological map
uint32 id

# Dimensions of local grid.  The local grid covers the rectangle between
# (0,0) and (x_size, y_size) in the grid's frame
float32 x_size
float32 y_size

# Resolution of the local occupancy grid
float32 resolution

