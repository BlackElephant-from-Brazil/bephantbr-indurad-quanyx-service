#!/bin/bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
rosrun pcl_to_clp_data_sender pcl_to_clp_data_sender