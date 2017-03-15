cmake_minimum_required(VERSION 2.8.3)
project(gmap_test)
find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gmap_test src/odom_compute.cpp)
target_link_libraries(gmap_test ${catkin_LIBRARIES})
add_dependencies(gmap_test gmap_test)
