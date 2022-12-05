# ROS Package for disparity to pcl converter
### Build 
This node needs the node ``omega_msgs`` to compile.
```sh
cd /path/to/my_catkin_ws/src 
cp -r /path/to/Omega_ROS_examples_directory/disp_to_pcl/ .
cp -r /path/to/Omega_ROS_examples_directory/omega_msgs/ .
cd /path/to/my_catkin_ws/
catkin build
source devel/setup.bash
```
Follow the global Omega ROS examples for more detail.

### Usage
To start node, run
```sh
roslaunch disp_to_pcl disp_to_pcl.launch
```

### Example
This node is really useful when playing a rosbag that does not contain the point cloud.
The launch file ``without_omega.launch`` of the node ``omega_camera`` could be used to display the result of this node.

For instance, in a first terminal, open a rosbag :
```sh
rosbag play <rosbag_file.bag>
```

In a second terminal, open the rviz interface of ``omega_camera`` :
```sh
roslaunch omega_camera without_omega.launch
```

In a last terminal, run ``disp_to_pcl`` to compute the point cloud :
```sh
roslaunch disp_to_pcl disp_to_pcl.launch
```

### Argument
There is two arguments that could be used with this node :
(if wrong argument are send to the node with rosrun, the node will display an help).

## useColorMap
The argument useColorMap can have two values : true or false.
If it is true, the color map in Colormap.h will be used to color the generated point cloud.
If not, the node will subscribe to /omega_rec_image and use the color in the rectified image to color the point cloud.
```sh
roslaunch disp_to_pcl disp_to_pcl.launch useColorMap:=true
```
The default value is false.

## filterLevel
The argument filterLevel should be an integer between 0 and 255.
All the pixel that have values of chanel red, green and blue greater than filterLevel in the rectified image will be removed.
```sh
roslaunch disp_to_pcl disp_to_pcl.launch useColorMap:=true filterLevel:=250
```
The default value of filterLevel is 255 (no filtering).

### Node
According to the argument, the node could subscribe to two topics :
- /omega_disp_image (always)
- /omega_rec_image (unless we use the color map without filtering)
The node also needs to collect information one time on the topic /omega_metadata to start.
If it can not (if no metadata are published), the node will stop with error code -1.
The node will compute the point cloud according to the disparity values and camera parameters.