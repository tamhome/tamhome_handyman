cmake_minimum_required(VERSION 3.0.2)
project(tamhome_handyman)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  HandymanMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

