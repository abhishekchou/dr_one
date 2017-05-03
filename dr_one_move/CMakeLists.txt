cmake_minimum_required(VERSION 2.8.3)
project(dr_one_move)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY 
   action
  FILES
   move_drone.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dr_one_move
  CATKIN_DEPENDS actionlib actionlib_msgs #message_generation roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_drone_server src/move_drone_server.cpp)
target_link_libraries(move_drone_server ${catkin_LIBRARIES})
add_dependencies(move_drone_server dr_one_move_gencpp ${dr_one_move_EXPORTED_TARGETS})


