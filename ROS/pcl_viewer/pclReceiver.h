#ifndef PCLRECEIVER_H
#define PCLRECEIVER_H

// Qt
#include <QObject>
#include <QDebug>
// ros include
#include <ros/ros.h>
// Point Cloud Library
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
// Point Cloud msgs
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
Q_DECLARE_METATYPE(PointCloudT);

class PclReceiver : public QObject
{
    Q_OBJECT
public:
    PclReceiver();

    ~PclReceiver();
    void rosPCLCallback(const sensor_msgs::PointCloud2& input);

public slots:
    void startSpinning();
    void stopSpinning();

signals:
    void rosPclSignal(PointCloudT);

private:
    // ros
    ros::NodeHandle n_;
    ros::Subscriber sub_pcl;
    bool isSpinning;
    bool isRectImgColor;
};

#endif
