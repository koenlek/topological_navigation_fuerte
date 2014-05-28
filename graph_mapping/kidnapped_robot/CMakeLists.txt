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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} signals)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(PkgConfig)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
include_directories(${SQLITE3_INCLUDE_DIRS})
link_directories(${SQLITE3_LIBRARY_DIRS})

rosbuild_add_library(${PROJECT_NAME} src/place_database.cpp)
target_link_libraries(${PROJECT_NAME} ${SQLITE3_LIBRARIES})

rosbuild_add_executable(kidnapped_node src/kidnapped_node.cpp)
rosbuild_add_boost_directories() #KL: to fix error: undefined reference to symbol 'boost::signals::connection::~connection()'
rosbuild_link_boost(kidnapped_node signals) #KL: to fix error: undefined reference to symbol 'boost::signals::connection::~connection()'
target_link_libraries(kidnapped_node ${PROJECT_NAME})

rosbuild_add_executable(point_head src/point_head.cpp)

rosbuild_add_executable(test_place_db src/test_place_db.cpp)
target_link_libraries(test_place_db ${PROJECT_NAME})

#rosbuild_add_executable(db_test src/db_test.cpp)
#target_link_libraries(db_test ${SQLITE3_LIBRARIES})

rosbuild_add_executable(place_recognition_node src/place_recognition_node.cpp)
rosbuild_add_boost_directories() #KL: to fix error: undefined reference to symbol 'boost::signals::connection::~connection()'
rosbuild_link_boost(place_recognition_node signals) #KL: to fix error: undefined reference to symbol 'boost::signals::connection::~connection()'
target_link_libraries(place_recognition_node ${PROJECT_NAME})

# download Calonder descriptor trees file
rosbuild_download_data(http://pr.willowgarage.com/data/calonder_descriptor/current.rtc calonder.rtc c2a1d9cb567f2a89d672c59a856549a8)
