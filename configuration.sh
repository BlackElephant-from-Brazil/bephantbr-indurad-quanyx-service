#!/bin/bash
source /opt/ros/melodic/setup.bash
cd /opt/catkin_ws
catkin build
source devel/setup.bash
roslaunch omega_camera omega.launch
