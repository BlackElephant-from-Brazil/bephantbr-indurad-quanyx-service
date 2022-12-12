#include "pclReceiver.h"

/**
 * @brief PclReceiver::PclReceiver
 */
PclReceiver::PclReceiver():QObject()
    ,isSpinning(true)
    ,isRectImgColor(true)
{
    sub_pcl = n_.subscribe("/omega_pcl", 10, &PclReceiver::rosPCLCallback,this);
}
/**
 * @brief PclReceiver::startSpinning start reveiving frames from ROS
 */
void PclReceiver::startSpinning()
{
    if(isSpinning)
        ros::spin();
    else
        isSpinning = true;
}

/**
 * @brief PclReceiver::stopSpinning stop reveiving frames from ROS
 */
void PclReceiver::stopSpinning()
{
    isSpinning = false;
}

/**
 * @brief PclReceiver::~PclReceiver
 */
PclReceiver::~PclReceiver()
{
}

void PclReceiver::rosPCLCallback(const sensor_msgs::PointCloud2& pcl)
{
    if(isSpinning)
    {
        pcl::PointCloud<PointT> omg_pcl;
        pcl::fromROSMsg(pcl, omg_pcl);
        emit rosPclSignal(boost::make_shared<pcl::PointCloud<PointT>>(omg_pcl));
    }
}
