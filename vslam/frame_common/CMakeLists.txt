cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILT_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_check_for_sse()

add_definitions(-Wall -Wno-missing-field-initializers -fopenmp)

#####################################################################
# library
find_package(Eigen)
find_package(OpenCV)
find_package(PCL)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)

rosbuild_add_library(frame_common src/frame.cpp src/stereo.cpp src/draw.cpp src/stereolib.c)
rosbuild_add_compile_flags(frame_common ${SSE_FLAGS})
target_link_libraries(frame_common ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#####################################################################
# unit tests
#



######################################################################
# executables
#
rosbuild_add_executable (test_frame src/test_frame.cpp)
target_link_libraries (test_frame gomp
                                  ${OpenCV_LIBRARIES}
)
######################################################################
# visualization
#


