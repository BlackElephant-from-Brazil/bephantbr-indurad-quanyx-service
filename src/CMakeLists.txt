# Stream videos from a given Autonous Sensor Head (ASH)
# 

project(SERVICE)
cmake_minimum_required(VERSION 2.8)

include(FindPkgConfig)
include(${CMAKE_CURRENT_SOURCE_DIR}/WinDeployQt5.cmake)

find_package( OpenCV REQUIRED )
find_package(CalibProcessor)
find_package(gst_meta_disparity)
find_package(gst_meta_sensor)
find_package(AutonomousSensorHead)

# Set the required CXX flags to compile this sample with the ASH library
if(UNIX)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${CalibProcessor_INCLUDE_DIRS})
include_directories(${AutonomousSensorHead_INCLUDE_DIRS})
link_directories(${AutonomousSensorHead_LIBRARY_DIRS})

if(NOT AutonomousSensorHead_FOUND)
    pkg_check_modules(ASH REQUIRED autonomoussensorhead)
    include_directories(${ASH_INCLUDE_DIRS})
    link_directories(${ASH_LIBRARY_DIRS})
else()
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
    set(ASH_LIBRARIES AutonomousSensorHead)
    link_directories(${GST_APP_LIBRARY_DIRS})
endif()

# Search for required Qt packages 
find_package(Qt5Widgets REQUIRED)
find_package(Qt5 COMPONENTS Core Gui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Tell CMake we are creating an executable with those files
add_executable(ASHViewer main.cpp Viewer.cpp ImageProcessing.cpp SensorControlThread.cpp)
# And link this executable with the required libraries for AutonomousSensorHead
target_link_libraries(ASHViewer ${ASH_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${OpenCV_LIBS})
qt5_use_modules(ASHViewer Core Gui)
        
install(TARGETS ${PROJECT_NAME}
        DESTINATION bin)

if(WIN32)
    WinDeployQt(windeploy ${PROJECT_NAME})

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/windeploy/
            DESTINATION bin)
endif()
