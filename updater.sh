#!/bin/bash
# Pull updates from git
echo "BEGIN: Pull updates from git"
git config --system http.sslVerify false
git config --system user.email "gui.sartori96@gmail.com"
git config --global --unset-all http.proxy
git config --global --unset-all https.proxy
cd /usr/bephantbr-indurad-quanyx-service
git pull
echo "FINISH: Pull updates from git"

# Reload systemctl
systemctl daemon-reload

# Install ROS Updates
echo "BEGIN: Install ROS Updates"
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws/src
cp -r /usr/bephantbr-indurad-quanyx-service/ROS/* .
echo "FINISH: Install ROS Updates"

# Install src and build updates
echo "BEGIN: Install src and build updates"
cd /usr/bephantbr-indurad-quanyx-service/src/build
cmake ../
make
echo "FINISH: Install src and build updates"

# Start environment configurations up
echo "BEGIN: Start environment configurations up"
systemctl start configuration-boot.service
echo "FINISH: Start environment configurations up"

# Start ROS find_objects
echo "BEGIN: Start ROS find_objects"
systemctl start start-find-objects-ros.service
echo "FINISH: Start ROS find_objects"

# Start SERVICE
echo "BEGIN: Start SERVICE"
systemctl start startup-boot.service
echo "FINISH: Start SERVICE"

# journalctl --since "2022-12-16 13:36:00" --no-pager