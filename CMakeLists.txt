cmake_minimum_required(VERSION 2.8.3)
project(ardrone_altitude)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ardrone_as
  genmsg
  geometry_msgs
  rospy
  std_msgs
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
   DIRECTORY action
   FILES
   ArdroneAltitude.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   actionlib
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ardrone_altitude
  CATKIN_DEPENDS rospy actionlib ardrone_as geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
