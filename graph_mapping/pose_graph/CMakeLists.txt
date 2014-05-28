
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_add_boost_directories()
# rosbuild_genmsg()
#rosbuild_gensrv()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_library(pose_graph 
                     src/constraint_graph.cpp
                     src/diff_publisher.cpp
                     src/diff_subscriber.cpp
                     src/message_conversion.cpp 
                     src/spa_2d_conversion.cpp
                     src/visualization.cpp
                     src/graph_search.cpp)




# rosbuild_add_executable(bin/test_pose_graph test/test_pose_graph.cpp)
# target_link_libraries(bin/test_pose_graph pose_graph)
# rosbuild_add_gtest_build_flags(bin/test_pose_graph)

# rosbuild_add_rostest(test/pose_graph_test.launch)
