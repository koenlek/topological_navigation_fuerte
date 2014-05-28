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

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(topological_roadmap
                     src/roadmap.cpp
                     src/ros_conversion.cpp
                     src/shortest_paths.cpp)

rosbuild_add_executable(bin/roadmap_builder src/roadmap_builder.cpp)
target_link_libraries(bin/roadmap_builder topological_roadmap)

rosbuild_add_gtest(test_roadmap test/test_roadmap.cpp)
target_link_libraries(test_roadmap topological_roadmap)

rosbuild_add_executable(bin/move_base_topo src/move_base_topo.cpp)
target_link_libraries(bin/move_base_topo topological_roadmap)