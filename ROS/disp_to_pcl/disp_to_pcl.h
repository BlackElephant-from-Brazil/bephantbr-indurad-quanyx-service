#ifndef DISP2PCL_H
#define DISP2PCL_H

// NVM : not sure that all theses pcl headers are used.
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <string>
#include <sstream>
#include <iomanip>  
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include <queue>

#include <iostream>

typedef sensor_msgs::PointCloud2 PointCloud;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish(bool _useColorMap=false, int _filterLevel=255);

  void dispCallback(const sensor_msgs::Image& input);

  void recCallback(const sensor_msgs::Image& input);

  void computationCallback(const ros::TimerEvent& event);

  bool hasInitSuccessfully() const ;

private:

  ros::NodeHandle n_; 
  ros::Publisher omg_pcl_;

  ros::Subscriber subDisp_;
  ros::Subscriber subRec_;

  ros::Timer computationTimer_;

  float focallength_;
  float baseline_;
  float rightopticalcenterx_;
  float rightopticalcentery_;

  std::queue<sensor_msgs::Image> dispBuffer_;
  std::queue<sensor_msgs::Image> recBuffer_;

  bool useColorMap_;

  uint8_t filterLevel_ ;

  bool skipRectified_ ;

  bool initSuccessfully_;

};//End of class SubscribeAndPublish

#endif // DISP2PCL_H