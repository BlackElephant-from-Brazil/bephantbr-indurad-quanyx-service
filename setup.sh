#!/bin/bash
# Install all Packages to run ROS 
echo "BEGIN: Install all Packages to run ROS"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update -y
apt-key adv --keyserver hkps://keyserver.ubuntu.com --refresh-keys
apt install cmake python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-desktop-full ros-melodic-pcl-ros -y
apt-cache policy libopencv-dev
apt-get install libopencv-dev=3.2.0+dfsg-4ubuntu0.1 -V -y
echo "FINISH: Install all Packages to run ROS"

# Configurating environment
echo "BEGIN: Configurating environment"
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
echo "FINISH: Configurating environment"

# Set git configuration
echo "BEGIN: Set git configuration"
cd /usr/bephantbr-indurad-quanyx-service
cp ./ssh/bephantbr-indurad-quanyx-service ~/.ssh
cp ./ssh/bephantbr-indurad-quanyx-service.pub ~/.ssh
eval $(ssh-agent -s)
ssh-add ~/.ssh/bephantbr-indurad-quanyx-service
git config --global user.name "Guilherme Sartori"
git config --global user.email "gui.sartori96@gmail.com"
echo "FINISH: Set git configuration"

# Reload SystemCTL daemon
echo "BEGIN: Reload SystemCTL daemon"
systemctl daemon-reload
echo "FINISH: Reload SystemCTL daemon"

# Create updater boot
echo "BEGIN: Create updater boot"
cd /usr/bephantbr-indurad-quanyx-service
cp ./updater.service /etc/systemd/system
systemctl enable updater.service
echo "FINISH: Create updater boot"

# Create configuration boot on ubuntu
echo "BEGIN: Create configuration boot on ubuntu"
cd /usr/bephantbr-indurad-quanyx-service
cp ./configuration-boot.service /etc/systemd/system
systemctl enable configuration-boot.service
echo "FINISH: Create configuration boot on ubuntu"

# Create startup boot on ubuntu
echo "BEGIN: Create startup boot on ubuntu"
cd /usr/bephantbr-indurad-quanyx-service
cp ./startup-boot.service /etc/systemd/system
systemctl enable startup-boot.service
echo "FINISH: Create startup boot on ubuntu"

# Create the data sender startup configuration
echo "BEGIN: Create the data sender startup configuration"
cd /usr/bephantbr-indurad-quanyx-service
cp ./data-sender-startup.service /etc/systemd/system
systemctl enable data-sender-startup.service
echo "FINISH: Create the data sender startup configuration"

# Build module
echo "BEGIN: Build module"
cd /usr/bephantbr-indurad-quanyx-service/src
mkdir build && cd build
cmake ../
make
echo "FINISH: Build module"

# Allow any changes in project
echo "BEGIN: Allow any changes in project"
cd /usr
chmod 777 -R bephantbr-indurad-quanyx-service
echo "FINISH: Allow any changes in project"