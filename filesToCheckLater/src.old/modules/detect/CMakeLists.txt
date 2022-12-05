# Detect all Autonomous Sensor Head (ASH)
# ----------------------------------------
# Just print all detected camera
#
project(ASHDetect)
cmake_minimum_required(VERSION 2.8)

include(FindPkgConfig)

find_package(CalibProcessor)
find_package(gst_meta_disparity)
find_package(gst_meta_sensor)
find_package(AutonomousSensorHead)

# Set the required CXX flags to compile this sample with the ASH library
if(UNIX)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
endif()

include_directories(${CalibProcessor_INCLUDE_DIRS})
include_directories(${AutonomousSensorHead_INCLUDE_DIRS})
link_directories(${AutonomousSensorHead_LIBRARY_DIRS})

if(NOT AutonomousSensorHead_FOUND)
    pkg_check_modules(ASH REQUIRED autonomoussensorhead)
    include_directories(${ASH_INCLUDE_DIRS})
    link_directories(${ASH_LIBRARY_DIRS})
else()
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
    include_directories(${ASH_INCLUDE_DIRS})
    set(ASH_LIBRARIES AutonomousSensorHead)
    link_directories(${GST_APP_LIBRARY_DIRS})
endif()


# Tell CMake we are creating an executable with those files
add_executable(${PROJECT_NAME} main.cpp)

# And link this executable with the required libraries for AutonomousSensorHead
target_link_libraries(${PROJECT_NAME} AutonomousSensorHead)

find_package(Qt5 COMPONENTS Core Gui)
qt5_use_modules(${PROJECT_NAME} Core Gui)
        
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

