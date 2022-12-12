cmake_minimum_required(VERSION 2.8.3)
project(disp_to_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ROS compatible only with UNIX system
if(UNIX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  omega_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    rospy
    omega_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node disp_to_pcl.cpp disp_to_pcl.h)
# Link executable with library
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
# Add msg dependency
add_dependencies(${PROJECT_NAME}_node omega_msgs_generate_messages_cpp)

endif()

