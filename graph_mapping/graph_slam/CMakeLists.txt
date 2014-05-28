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

#set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(ROS_COMPILE_FLAGS "-pg")
#set(ROS_LINK_FLAGS "-pg")

#uncomment if you have defined messages
# rosbuild_genmsg()
#uncomment if you have defined services
# rosbuild_gensrv()

rosbuild_add_library(graph_slam src/graph_mapper.cpp
                                src/localization_buffer.cpp
                                src/odometer.cpp)

rosbuild_download_data(http://pr.willowgarage.com/data/graph_slam/wg-cafe.bag test/wg-cafe.bag df27e61cb839a384d552ad0cfc710613)

# rosbuild_add_rostest(test/graph_hz_test.launch)
# rosbuild_add_rostest(test/localization_hz_test.launch)
