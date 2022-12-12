cmake_minimum_required(VERSION 2.8.11)

project(pcl_viewer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  pcl_ros
)
find_package(VTK REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package( OpenCV REQUIRED )
# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge
)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp pclViewer.cpp pclReceiver.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Qt5::Widgets ${catkin_LIBRARIES}) 
