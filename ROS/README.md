# ROS Packages for Omega stereo camera

## Prerequisites
These packages are intended to be used in following conditions:
* Ubuntu 16.04 (and works on Ubuntu 18.04) with drivers (and dependencies) provided by the vendor;
* with ROS middleware;
* Omega drivers are assumed to be installed;

If you have no existing ROS installation, see a supplementary Readme file in this directory (ROS_Omega.md).

Also requires a service to start and maintain the Avahi manager required for camera discovery. If the camera is in a DHCP or fixed IP then Avahi is not required.
​
We consider here that you are using ROS Melodic. If you are using ROS Kinetic, you simply should replace "melodic" by "kinetic" in the following commands.
​
ROS Dependencies:
*roscpp*
​
ROS environment must be loaded:
```sh
source /opt/ros/melodic/setup.bash
```
​
## Tools
*Catkin_tools* and *rosdep* should be installed to compile any package. To install them:
```sh
sudo apt-get install python-rosdep python-catkin-tools python-rospkg
sudo rosdep init
rosdep update
```
​
## Initialization

Initialize your catkin workspace. In following snippets it will be represented as ``/path/to/my_catkin_ws``.
Such workspace can be situated anywhere you wish. Standard convention says, e.g., ``~/catkin_ws``:
```sh
mkdir -p /path/to/my_catkin_ws/src
cd /path/to/my_catkin_ws
catkin init
```
​
## Source code

Copy the source code of Omega ROS packages:

```sh
cd /path/to/my_catkin_ws/src 
cp -r /path/to/Omega_ROS_examples_directory/* .
```
​
## Dependencies

Install the dependencies:

```sh
cd /path/to/my_catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
​
## Build

```sh
cd /path/to/my_catkin_ws/
catkin build
source devel/setup.bash
```
​
## ROS master

ROS master must be running if you are not using ``roslauch`` (see below).
```sh
roscore &
```
​
## Run nodes

The principal current ROS node for omega is ``omega_camera``. Other two nodes (``pcl_viewer`` or ``image_viewer``) can serve as different means of visualisation.

Your catkin workspace should be sourced. See README of ``omega_camera`` to have more informations.
​
### Omega camera

See Omega camera README for more details ("Usage" subsection at the end).
```sh
roslauch omega_camera omega.launch camera_name:=<camera_hostname>
```
​
### pcl_viewer
This is the ROS package that contains the rosnode that display the point cloud.
```sh
rosrun pcl_viewer pcl_viewer
```
​
### image_viewer
This is the ROS package that contains the rosnode that display raw images, rectified images and disparity map.
```sh
rosrun image_viewer image_viewer
```

### disp_to_pcl
This is the ROS package that converts the disparity to a point cloud.
```sh
roslaunch disp_to_pcl disp_to_pcl.launch

