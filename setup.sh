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
catkin build
source devel/setup.bash

# Create autorun on ubuntu
cd /usr/bephantbr-indurad-quanyx-service
cp ./configuration-boot /etc/systemd/system
systemctl enable configuration-boot

# Build module
cd ./src
mkdir build && cd build
cmake ../
make