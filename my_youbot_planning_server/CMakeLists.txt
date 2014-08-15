cmake_minimum_required(VERSION 2.8.3)
project(my_youbot_planning_server)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  my_youbot_msgs
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Instructions.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  my_youbot_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_youbot_planning_server
 CATKIN_DEPENDS geometry_msgs my_youbot_msgs std_msgs
#  DEPENDS system_lib
)

