cmake_minimum_required(VERSION 2.8.3)
# Ros pcl example
# This example contains three nodes:
#   omega_camera : connect to omega sensor, stream frames and publish it to the ros
#   image_viewer : subscribe to all the topics published by the omega node and display the frames
#   pcl_viewer   : subscribe to pcl topics and display the frames


# build omega_camera node
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/omega_camera)
# build image viewer node
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/image_viewer)
# build pcl viewer node
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pcl_viewer)