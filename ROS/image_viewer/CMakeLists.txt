cmake_minimum_required(VERSION 2.8.11)

project(image_viewer)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package( OpenCV REQUIRED )
# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets)
# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
catkin_package(
  CATKIN_DEPENDS roscpp cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(project_SOURCES main.cpp imageViewer.cpp imageReceiver.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
