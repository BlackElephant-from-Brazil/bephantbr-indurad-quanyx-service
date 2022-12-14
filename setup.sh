#!/bin/bash
# Install all Packages to run ROS 
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update -y
apt-key adv --keyserver hkps://keyserver.ubuntu.com --refresh-keys
apt install cmake python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-desktop-full ros-melodic-pcl-ros -y
apt-cache policy libopencv-dev
apt-get install libopencv-dev=3.2.0+dfsg-4ubuntu0.1 -V -y

# Configurating environment
source /opt/ros/melodic/setup.bash
apt-get install python-rosdep python-catkin-tools python-rospkg -y
rosdep init
rosdep update -y
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd src
cp -r /usr/bephantbr-indurad-quanyx-service/ROS/* .
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin config --extend /opt/ros/melodic
catkin build
source devel/setup.bash

# Set git configuration
git config --global --unset http.proxy
git config --global --unset https.proxy
git config --global user.name "Guilherme Sartori"
git config --global user.email "gui.sartori96@gmail.com"

# Reload SystemCTL daemon
systemctl daemon-reload

# Create updater boot
cd /usr/bephantbr-indurad-quanyx-service
cp ./updater.service /etc/systemd/system
systemctl enable updater.service

# Create configuration boot on ubuntu
cd /usr/bephantbr-indurad-quanyx-service
cp ./configuration-boot.service /etc/systemd/system
systemctl enable configuration-boot.service

# Create startup boot on ubuntu
cd /usr/bephantbr-indurad-quanyx-service
cp ./startup-boot.service /etc/systemd/system
systemctl enable startup-boot.service

# Create the data sender startup configuration
cd /usr/bephantbr-indurad-quanyx-service
cp ./data-sender-startup.service /etc/systemd/system
systemctl enable data-sender-startup.service

# Build module
cd /usr/bephantbr-indurad-quanyx-service/src
mkdir build && cd build
cmake ../
make

# Allow any changes in folder (segmentation fault resolution)
cd /usr
chmod 777 -R bephantbr-indurad-quanyx-service