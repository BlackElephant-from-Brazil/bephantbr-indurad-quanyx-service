cmake_minimum_required(VERSION 2.8.3)
project(omega_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ROS compatible only with UNIX system
if(UNIX)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  message_runtime
  roscpp
  rospy
)

add_message_files(
  DIRECTORY 
    msg
  FILES
    OmegaMetadata.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    std_msgs
)

endif()