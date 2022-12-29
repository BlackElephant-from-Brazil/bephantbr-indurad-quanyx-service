# Send PCL data from ROS to OPCDA
cmake_minimum_required(VERSION 2.8)

project(find_objects)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(UNIX)

find_package(PCL 1.7.1 REQUIRED)
find_package(AutonomousSensorHead)
find_package(JetsonGPIO)

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
include_directories(${AutonomousSensorHead_INCLUDE_DIRS})
link_directories(${AutonomousSensorHead_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT AutonomousSensorHead_FOUND)
    pkg_check_modules(ASH REQUIRED autonomoussensorhead)
    include_directories(${ASH_INCLUDE_DIRS})
    link_directories(${ASH_LIBRARY_DIRS})
else()
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
    set(ASH_LIBRARIES AutonomousSensorHead)
    link_directories(${GST_APP_LIBRARY_DIRS})
endif()

# Tell CMake we are creating an executable with those files
add_executable(find_objects main.cpp)
# And link this executable with the required libraries for AutonomousSensorHead
target_link_libraries(find_objects ${ASH_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} JetsonGPIO::JetsonGPIO)
# Add msg dependency
add_dependencies(${PROJECT_NAME} omega_msgs_generate_messages_cpp)

endif()
