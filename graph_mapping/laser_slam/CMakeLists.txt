cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(laser_slam src/util.cpp

                                src/point_cloud_snapshotter.cpp
                                src/global_map.cpp

                                src/scan_match_localization.cpp
                                src/scan_matching.cpp
                                src/scan_match_constraints.cpp

                                src/scan_intersection.cpp
                                )
target_link_libraries(laser_slam pose_graph)
rosbuild_add_compile_flags(laser_slam ${SSE_FLAGS})

rosbuild_add_executable(bin/odom_constraint_node src/odom_constraint_node.cpp)
target_link_libraries(bin/odom_constraint_node graph_slam)

rosbuild_add_executable(bin/laser_slam_node src/laser_slam_node.cpp)
target_link_libraries(bin/laser_slam_node laser_slam graph_slam)

rosbuild_add_executable(bin/cloud_snapshotter src/snapshotter_node.cpp)
target_link_libraries(bin/cloud_snapshotter laser_slam)

rosbuild_add_executable(bin/scan_match_constraint_generator src/scan_match_constraint_node.cpp)
target_link_libraries(bin/scan_match_constraint_generator laser_slam)

rosbuild_add_executable(bin/scan_match_localizer src/scan_match_localizer.cpp)
target_link_libraries(bin/scan_match_localizer laser_slam)

rosbuild_add_gtest(test_laser_slam test/test_laser_slam.cpp)
target_link_libraries(test_laser_slam laser_slam)

