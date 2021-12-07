cmake_minimum_required(VERSION 3.0.2)
project(joy_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
  message_generation
  dynamixel_sdk
)

add_message_files(
  FILES
    SetPosition.msg
)

add_service_files(
  FILES
    GetPosition.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  LIBRARIES joy_control
  CATKIN_DEPENDS joy roscpp rospy std_msgs dynamixel_sdk
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(joy_control_src
  src/joy_control.cpp
  src/joy_control.h
)

add_executable(joy_control ${joy_control_src})
target_link_libraries(joy_control ${catkin_LIBRARIES})