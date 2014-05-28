cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#add_definitions(-D HOWARD=1)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_boost_directories()

rosbuild_check_for_sse()

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_definitions(-pg)
find_package(Boost REQUIRED signals)
find_package(ROS COMPONENTS rosbag)

#####################################################################
# download data

rosbuild_download_data(http://pr.willowgarage.com/data/calonder_descriptor/current.rtc data/calonder.rtc c2a1d9cb567f2a89d672c59a856549a8)

#####################################################################
# libraries
rosbuild_add_library(vo src/vo.cpp src/place_recognizer.cpp src/vslam.cpp src/vslam_mono.cpp)
rosbuild_add_compile_flags(vo ${SSE_FLAGS})

rosbuild_add_library(vo_ptcloud src/vo_ptcloud.cpp src/place_recognizer.cpp src/vslam_ptcloud.cpp)
rosbuild_add_compile_flags(vo_ptcloud ${SSE_FLAGS})

#####################################################################
# unit tests
#


######################################################################
# test for run_mono pipeline using simulated data
rosbuild_add_executable(run_simulated_mono test/run_simulated_mono.cpp)
target_link_libraries(run_simulated_mono vo)

######################################################################
# executables
#

# read stereo sequence and SBA it
rosbuild_add_executable(run_stereo src/run_stereo.cpp)
target_link_libraries(run_stereo vo)

rosbuild_add_executable(comp src/po_comparison.cpp)
target_link_libraries(comp vo)

# read stereo sequence from PS device and SBA it
rosbuild_add_executable(run_ps src/run_ps.cpp)
target_link_libraries(run_ps vo)

# read stereo sequence from PS device and SBA it
rosbuild_add_executable(run_ps_bag src/run_ps_bag.cpp)
target_link_libraries(run_ps_bag vo ${rosbag_LIBRARIES})

# read stereo sequence for an object and SBA it
rosbuild_add_executable(run_object src/run_object.cpp)
target_link_libraries(run_object vo)

# read stereo sequence and run mono VO
rosbuild_add_executable(run_mono src/run_mono.cpp)
target_link_libraries(run_mono vo)

# stereo VSLAM as a ROS node
rosbuild_add_executable(stereo_vslam_node src/nodes/stereo_vslam_node.cpp)
target_link_libraries(stereo_vslam_node vo ${Boost_LIBRARIES})

# monocular VSLAM as a ROS node
rosbuild_add_executable(mono_vslam_node src/nodes/mono_vslam_node.cpp)
target_link_libraries(mono_vslam_node vo ${Boost_LIBRARIES})


# stereo & ptcloud VSLAM as a ROS node
rosbuild_add_executable(ptcloud_vslam_node src/nodes/ptcloud_vslam_node.cpp)
target_link_libraries(ptcloud_vslam_node vo ${Boost_LIBRARIES})

# stereo VO as a ROS node
rosbuild_add_executable(vo_node src/nodes/vo_node.cpp)
target_link_libraries(vo_node vo ${Boost_LIBRARIES})

# publish NewCollege (or other) static dataset over ROS
rosbuild_add_executable(fake_stereo_cam src/nodes/fake_stereo_cam.cpp)

######################################################################
# visualization
#

# SBA on intel files
rosbuild_add_executable(vis-intel src/vis-intel.cpp src/read_spa.cpp)
target_link_libraries(vis-intel)
