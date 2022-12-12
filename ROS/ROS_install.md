# ROS installation for OMEGA

This small tutorial describes installation of ROS middleware which can be used with Omega.

Currently compatible versions of ROS for the latest version of Omega:
- Melodic (1.14)
- Kinetic (1.12) - end-of-life is on April 2021 (and therefore this version is NOT recommended).

Therefore, all the tutorials in ROS section will concern version "Melodic".

The OS to be used is Ubuntu 16.04 or 18.04.

## Installation
Set up your computer (or SBC like Jetson Nano) to accept software from packages.ros.org:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Add a new apt key:
```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update the Debian packages index:
```bash
sudo apt update
```

If at this moment you have a problem (with message of something like ``the key has expired``), you can renew your keys:
```bash
sudo apt-key adv --keyserver hkps://keyserver.ubuntu.com --refresh-keys
```

Install the ROS Desktop package and dependencies
```bash
sudo apt install cmake python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-desktop-full ros-melodic-pcl-ros
```

## Melodic environment

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```

## OpenCV3.2

Opencv3.2 is mandatory for all the subpackages, used in ROS environment by Omega. You can check the version:
```bash
dpkg -l | grep opencv
```
If it's not the version 3.2 you need to find the good 3.2 version name available for your distribution
```bash
apt-cache policy libopencv-dev
```
Then install it. For example, one of the versions available for NVidia Jetson Nano SBC at the moment of writing, the correct version is
```bash
sudo apt-get install libopencv-dev=3.2.0+dfsg-4ubuntu0.1 -V
```

