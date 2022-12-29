#!/bin/bash
source /opt/ros/melodic/setup.bash
cd /opt/catkin_ws
catkin build
source devel/setup.bash
systemctl daemon-reload
rosrun find_objects find_objects