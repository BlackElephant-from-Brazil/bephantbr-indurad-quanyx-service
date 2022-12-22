#!/bin/bash
source /opt/ros/melodic/setup.bash
cd /home/cmak/catkin_ws
catkin build
echo "finished build"
source devel/setup.bash
systemctl daemon-reload
echo "finished source"
rosrun find_objects find_objects
echo "launched"