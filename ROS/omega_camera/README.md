# omega_camera ROS package

## Prerequisites

This package is intended to be used with an already exsisting ROS workspace, and should be pulled in as a submodule in a ROS workspace (see other READMEs in root dir for more info on this).

This package also assumes that the Omega drivers are installed, this node was developed in Ubuntu 18.04 with drivers (and dependencies) provided by the vendor.
Also requires a service to start and maintain the Avahi manager required for camera discovery. If the camera is in a DHCP or fixed IP then Avahi is not required.

In the following instruction, ROS 1.14 Melodic is assumed to be installed. Also, we assume the ``python-catkin-tools`` package installed as well: this enables more flexible catkin-tools-based compilation. However, you can still use the older ``catkin_make`` tool for building the libraries - replacing the ``catkin {tool}`` invocations further in text by ``catkin_make``.

*ROS Dependencies: roscpp, catkin, std_msgs, sensor_msgs, pcl_ros, message_generation, message_runtime*

## Omega ROS using RVIZ

### ROS environment

Before anything, ROS environment must be loaded (this string might already be in your ``~/.bash_rc`` file):
```sh
source /opt/ros/melodic/setup.bash
```

## Init, build, run

The following subsections are needed **ONLY IF** you install this package separately, and did not do it already by following instructions of other READMEs.

### Init project

In following snippets, we assume that your catkin workspace is located at ``/path/to/catkin_ws``. It can be, really, wherever you want.

```sh
export CATKIN_OMEGA_HOME=/path/to/catkin_ws
rm -rf ${CATKIN_OMEGA_HOME} # If necessary
mkdir ${CATKIN_OMEGA_HOME}/src/ -p
cd ${CATKIN_OMEGA_HOME}
catkin init # or catkin_make
```

### Build

In following snippet, we assume that the source code's root directory of this package is located at ``/path/to/Omega_ROS_examples_directory``.
This node needs the node ``omega_msgs`` to compile.

```sh
export CATKIN_OMEGA_HOME=/path/to/catkin_ws
cp /path/to/Omega_ROS_examples_directory/omega_camera ${CATKIN_OMEGA_HOME}/src/ -rf
cp /path/to/Omega_ROS_examples_directory/omega_msgs ${CATKIN_OMEGA_HOME}/src/ -rf
cd ${CATKIN_OMEGA_HOME}
catkin build # or catkin_make
```

### Usage

The only argument which is needed for this package is ``camera_name``. There exists a default value, and by default (if argument is not provided in command line), this default value will be used. If no camera by default is found, the node will try to connect itself to any detected camera.

Following snippets are given with the argument ``camera_name``, where you should insert your ``camera_hostname`` (like ``head-H-SY-19-11-009.local``).

Omega node only:
```sh
export CATKIN_OMEGA_HOME=~/catkin_ws_omega
source ${CATKIN_OMEGA_HOME}/devel/setup.bash
roslaunch ${CATKIN_OMEGA_HOME}/src/omega_camera/launch/omega.launch camera_name:=<camera_hostname>
```

Omega node with visualization by RVIZ:

```sh
export CATKIN_OMEGA_HOME=~/catkin_ws_omega
source ${CATKIN_OMEGA_HOME}/devel/setup.bash
roslaunch ${CATKIN_OMEGA_HOME}/src/omega_camera/launch/omega_rviz.launch camera_name:=<camera_hostname>
```

## Node

if ``camera_name`` (provided as argument in ``*.launch`` file) is empty or invalid, the node looks for available cameras on the network.
If a camera is identified, then it is configured using the ROS parameters defined for this node.
Stream is started right after configuration.
Out of every frame, depending on current configuration of the camera,the following is extracted:
1) Raw Image (right camera)
2) Disparity Map / Raw Image (left camera)
3) Metadata
4) Point Cloud

The rectified image (for one or both cameras) is obtained using the ``ImageProcessing`` library provided by the vendor.

Note that disparity image is published in ``Mono8`` format (grayscale).

## Topics
``/omega_raw_image`` - Raw image in standard ROS image message
``/omega_rec_image`` - Rectified image in standard ROS image message
``/omega_disp_image`` - Disparity image in standard ROS image message
``/omega_metadata`` - Metadata from sensor in custom message (see below)
``/omega_pcl`` - pcl cloud in standard ``pcl::PointCloud<pcl::PointXYZRGB>`` message

## Messages
* Standard ROS [Image message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
* Standard PCL ``pcl::PointCloud<pcl::PointXYZRGB>`` message:
	- http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html
	- http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html

* Custom Metadata Message (type ``omega_camera/OmegaMetadata``):
	int16 rightwidth
	int16 rightheight
	int16 rightwidthrectified
	int16 rightheightrectified
	int16 dispwidth
	int16 dispheight
	float64 focallength
	float64 baseline
	float64 rightopticalcenterx
	float64 rightopticalcentery
	int16 temperature
	int16 fps
	int16 maxexposuretime
	int16 greylevelpercentage
