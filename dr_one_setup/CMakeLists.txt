cmake_minimum_required(VERSION 2.8.3)
project(dr_one_setup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
 )

catkin_package()

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_broadcaster dr_one_setup)

add_executable(laser_tf_listener src/laser_tf_listener.cpp)
target_link_libraries(laser_tf_listener ${catkin_LIBRARIES})
add_dependencies(laser_tf_listener dr_one_setup)


