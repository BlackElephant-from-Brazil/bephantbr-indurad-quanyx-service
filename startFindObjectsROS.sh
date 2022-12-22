#!/bin/bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
# rosrun find_objects find_objects