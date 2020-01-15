project(compareMaps)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)

find_package(GFlags REQUIRED)
find_package(glog REQUIRED)
find_package(roscpp REQUIRED)
find_package(rosbag REQUIRED)
find_package(PCL REQUIRED)

list(APPEND LINK_LIBS
  ${CERES_LIBRARIES}
  ${GTEST_LIBRARIES}
  ${rosbag_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
)

include_directories(
  ${GFLAGS_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${glog_DIRS}
)

add_executable(compareMaps compareMaps.cpp)
target_link_libraries(compareMaps ${LINK_LIBS})
