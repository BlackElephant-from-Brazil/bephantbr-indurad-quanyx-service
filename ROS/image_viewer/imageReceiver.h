#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

// Qt
#include <QObject>

// ros include
#include <ros/ros.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

typedef cv::Mat Image;
Q_DECLARE_METATYPE(Image);

class ImageReceiver : public QObject
{
    Q_OBJECT
public:
    ImageReceiver();

    ~ImageReceiver();
    void rosDispCallback(const sensor_msgs::Image &msg);
    void rosRawRightCallback(const sensor_msgs::Image &msg);
    void rosRectRightCallback(const sensor_msgs::Image &msg);
    void rosRawLeftCallback(const sensor_msgs::Image &msg);
    void rosRectLeftCallback(const sensor_msgs::Image &msg);

public slots:
    void startSpinning();
    void stopSpinning();

signals:
    void rosDisparitySignal(Image);
    void rosRawRightSignal(Image);
    void rosRectRightSignal(Image);
    void rosRawLeftSignal(Image);
    void rosRectLeftSignal(Image);

private:
    void imageCallback(const sensor_msgs::Image& msg, cv::Mat& mat);

    // ros
    ros::NodeHandle n_;
    ros::Subscriber sub_disp;
    ros::Subscriber sub_rect_right;
    ros::Subscriber sub_raw_right;
    ros::Subscriber sub_rect_left;
    ros::Subscriber sub_raw_left;
    bool isSpinning;
};

#endif
