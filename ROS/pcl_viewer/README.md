# ROS Package for disparity to pcl converter
### Build
```sh
cd /path/to/my_catkin_ws/src 
cp -r /path/to/Omega_ROS_examples_directory/pcl_viewer/ .
cd /path/to/my_catkin_ws/
catkin build
source devel/setup.bash
```
Follow the global Omega ROS examples for more detail.

### Usage
To start node, run
```sh
roslaunch pcl_viewer pcl_viewer.launch
```

### Argument
There is four arguments that could be used with this node :

## useDisp2Pcl
The argument useDisp2Pcl can have two values : true or false.
If it is true, the node disp_to_pcl will be launched to generate the point cloud thanks to the disparity map.
It is useful when playing a ros bag that only contains the disparity to visualize the point cloud.
```sh
roslaunch pcl_viewer pcl_viewer.launch useDisp2Pcl:=true
```
The default value is false.

## disp_to_pcl arguments
The three other arguments are the arguments of the disp_to_pcl node. 
See the disp_to_pcl readme for more details.

### Node
The node subscribe to /omega_pcl topic to show the point cloud of omega camera.