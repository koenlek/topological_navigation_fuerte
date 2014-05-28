# A 2d topological map consists of a set of local occupancy grids together with
# the transformations between coordinate frames of overlapping grids.  This ROS
# message represents the structure of the topological map, i.e., all the 
# information except the grids themselves.  You can get individual grids by
# calling the GetLocalGrid service on the mapper.

TopologicalMapNode[] nodes
TopologicalMapEdge[] edges