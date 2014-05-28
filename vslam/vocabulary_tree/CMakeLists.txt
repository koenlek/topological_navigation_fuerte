cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_check_for_sse()
add_definitions(${SSE_FLAGS})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(${PROJECT_NAME} src/database.cpp src/generic_tree.cpp)

# Download vocabulary and weights trained on Holidays image dataset
rosbuild_download_data(http://pr.willowgarage.com/data/vocabulary_tree/holidays.tree holidays.tree bf1d267dfc0b3b1953e4e82c8b2d2b7e)
rosbuild_download_data(http://pr.willowgarage.com/data/vocabulary_tree/holidays.weights holidays.weights 9705a970c83999258a57bacea105fea7)

# Various test programs, need to distill into useful apps and tests
find_package(Eigen)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
rosbuild_add_executable(make_tree src/make_tree.cpp)

rosbuild_add_executable(train_weights src/train_weights.cpp)
target_link_libraries(train_weights ${PROJECT_NAME})

rosbuild_add_executable(make_tree_gaussian src/make_tree_gaussian.cpp)

rosbuild_add_executable(test_kmeans test/test_kmeans.cpp)
rosbuild_add_executable(test_kmeans_hier test/test_kmeans_hier.cpp)

rosbuild_add_executable(test_distance test/test_distance.cpp)

rosbuild_add_executable(make_test_tree test/make_test_tree.cpp)

rosbuild_add_executable(find_word test/find_word.cpp)

rosbuild_add_executable(test_database test/test_database.cpp)
target_link_libraries(test_database ${PROJECT_NAME})

rosbuild_add_executable(test_generic_tree test/test_generic_tree.cpp)
target_link_libraries(test_generic_tree ${PROJECT_NAME})
