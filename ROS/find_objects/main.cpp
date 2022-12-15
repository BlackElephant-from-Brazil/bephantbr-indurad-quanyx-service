
#include <iostream>

// ROS 
#include "ros/ros.h"

// Point Cloud Messages
#include <sensor_msgs/PointCloud2.h>
// Point Cloud Library
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;

void analizeFrame()
{
    cout << "Analizing frame" << endl;
}

void subCallback(const sensor_msgs::PointCloud2& ros_msg)
{
    pcl::PointCloud<PointT> pcl_cloud;
    pcl::fromROSMsg(ros_msg, pcl_cloud);
    
    cout << "*************************** NEW FRAME HERE ***************************" << endl;
    // for (auto point : pcl_cloud.points) {
    //     cout << point << endl;
    // }
    // cout << pcl_cloud.points.size() << endl;
    analizeFrame();
    cout << "*************************** END OF FRAME HERE ***************************" << endl;

}

void receiveDataFromROS(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n_;
    ros::Subscriber sub = n_.subscribe("/omega_pcl", 10, &subCallback);

    ros::spin();

}

/*****************************************************************
 * MAIN FUNCTION
*****************************************************************/

int main(int argc, char *argv[])
{
    receiveDataFromROS(argc, argv);
    return 0;
}
