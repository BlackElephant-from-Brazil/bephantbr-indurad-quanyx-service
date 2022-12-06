cmake_minimum_required(VERSION 2.8.3)
project(omega_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ROS compatible only with UNIX system
if(UNIX)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  omega_msgs
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs 
    sensor_msgs 
    omega_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/omega_node.cpp
  src/ImageProcessing.cpp
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} AutonomousSensorHead CalibProcessor gst_meta_disparity gst_meta_sensor)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node DESTINATION bin)

endif()
