#!/bin/bash

# Install OMEGA ASH
echo "BEGIN: Install OMEGA ASH"
apt-get update -y
dpkg -i ASH_UBUNTU-18-04_ARM64_v2.1.1.deb
apt-get -f install -y
dpkg -i ASH_UBUNTU-18-04_ARM64_v2.1.1.deb
echo "FINISH: Install OMEGA ASH"

# Install all Packages to run ROS 
echo "BEGIN: Install all Packages to run ROS"
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update -y
apt install ros-melodic-desktop-full -y
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
apt install python-rosdep
rosdep init
rosdep update -y
# apt-key adv --keyserver hkps://keyserver.ubuntu.com --refresh-keys
# apt install cmake python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-desktop-full ros-melodic-pcl-ros -y
# apt-cache policy libopencv-dev
# apt-get install libopencv-dev=3.2.0+dfsg-4ubuntu0.1 -V -y
echo "FINISH: Install all Packages to run ROS"

# Setup catkin to 3.11
echo "BEGIN: Setup catkin to 3.11"
apt remove --purge --auto-remove cmake -y
apt update -y
apt install -y software-properties-common lsb-release
apt clean all
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
apt update -y
apt install cmake -y
echo "FINISH: Setup catkin to 3.11"

# Configurating environment
echo "BEGIN: Configurating environment"
chown -R $USER ~/.ros
apt-get install python-rosdep python-catkin-tools python-rospkg -y
mkdir -p /home/cmak/catkin_ws/src
cd /home/cmak/catkin_ws
catkin init
cd src
cp -r /usr/bephantbr-indurad-quanyx-service/ROS/* .
cd ..
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic
catkin config --extend /opt/ros/melodic
cp /usr/bephantbr-indurad-quanyx-service/cv_bridgeConfig.cmake /opt/ros/melodic/share/cv_bridge/cmake/
catkin build
source devel/setup.bash
echo "FINISH: Configurating environment"

# Set git configuration
echo "BEGIN: Set git configuration"
git config --system http.sslVerify false
git config --system user.email "gui.sartori96@gmail.com"
git config --global --unset-all http.proxy
git config --global --unset-all https.proxy
git config --global user.name "Guilherme Sartori"
git config --global user.email "gui.sartori96@gmail.com"
echo "FINISH: Set git configuration"

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
echo "FINISH: Create configuration boot on ubuntu"

# Create startup boot on ubuntu
echo "BEGIN: Create startup boot on ubuntu"
cd /usr/bephantbr-indurad-quanyx-service
cp ./startup-boot.service /etc/systemd/system
systemctl enable startup-boot.service
echo "FINISH: Create startup boot on ubuntu"

# Create the data sender startup configuration
echo "BEGIN: Create the find objects ros"
cd /usr/bephantbr-indurad-quanyx-service
cp ./start-find-objects-ros.service /etc/systemd/system
echo "FINISH: Create the find objects ros"

# Reload SystemCTL daemon
echo "BEGIN: Reload SystemCTL daemon"
systemctl daemon-reload
echo "FINISH: Reload SystemCTL daemon"

# Build module
echo "BEGIN: Build module"
echo "Defaults env_keep += \"DISPLAY XAUTHORITY\"" >> /etc/sudoers.d/local
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