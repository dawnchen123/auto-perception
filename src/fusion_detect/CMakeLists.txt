cmake_minimum_required(VERSION 2.8.3)
project(fusion_detect)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
)

## Generate messages in the 'msg' folder
# add_message_files(
# 	FILES
#     CustomPoint.msg
#     CustomMsg.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs  # Or other packages containing msgs
# )

## Specify additional locations of header files
## Your package locations should be listed before other locations

catkin_package(
  CATKIN_DEPENDS autoware_msgs  roscpp rospy std_msgs sensor_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(fusion_detect 
  src/fusion_detect.cpp
)
target_link_libraries(fusion_detect ${catkin_LIBRARIES})




