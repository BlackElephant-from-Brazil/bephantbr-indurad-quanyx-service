# Send PCL data from ROS to OPCDA
cmake_minimum_required(VERSION 2.8)

project(find_objects)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(UNIX)

find_package(PCL 1.7.1 REQUIRED)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Tell CMake we are creating an executable with those files
add_executable(find_objects main.cpp)
# And link this executable with the required libraries for AutonomousSensorHead
target_link_libraries(find_objects ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# Add msg dependency
add_dependencies(${PROJECT_NAME} omega_msgs_generate_messages_cpp)

endif()
