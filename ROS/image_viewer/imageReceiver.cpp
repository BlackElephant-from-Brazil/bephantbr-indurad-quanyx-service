#include "imageReceiver.h"
#include <QDebug>

/**
 * @brief ImageReceiver::ImageReceiver : constructor
 */
ImageReceiver::ImageReceiver():QObject()
    ,isSpinning(true)
{
    sub_disp = n_.subscribe("/omega_disp_image", 1, &ImageReceiver::rosDispCallback,this);
    sub_rect_right = n_.subscribe("/omega_rec_image", 1, &ImageReceiver::rosRectRightCallback,this);
    sub_raw_right = n_.subscribe("/omega_raw_image", 1, &ImageReceiver::rosRawRightCallback,this);
    sub_rect_left = n_.subscribe("/omega_rec_left_image", 1, &ImageReceiver::rosRectLeftCallback,this);
    sub_raw_left = n_.subscribe("/omega_raw_left_image", 1, &ImageReceiver::rosRawLeftCallback,this);
}

/**
 * @brief ImageReceiver::rosPclCallback callback function rof omega_disp_image topic
 * @param cloud message given from the Ros master
 */
void ImageReceiver::rosDispCallback(const sensor_msgs::Image& msg)
{
    if (isSpinning)
    {
        cv::Mat cameraFeed;
        if (msg.encoding == sensor_msgs::image_encodings::MONO16)
        {
            cameraFeed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
            cameraFeed = 0.25 * cameraFeed; // Go back to the same subpixel_scaling value
            cameraFeed.convertTo(cameraFeed, CV_8U); // Disparity 11 bits is not supported -> convert to 8bits
        }
        else
        {
            cameraFeed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        }
        emit rosDisparitySignal(cameraFeed);
    }
}

/**
 * @brief Convert ros img to openCV image accordong to the image encoding
 * @param msg is the input ros image message
 * @param mat is the ouput opencv image
 */
void ImageReceiver::imageCallback(const sensor_msgs::Image& msg, cv::Mat& mat)
{
    if (msg.encoding == "rgb8")
    {
        mat = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    else
    {
        mat = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    }
}

/**
 * @brief ImageReceiver::rosPclCallback callback function rof omega_raw_image topic
 * @param cloud message given from the Ros master
 */
void ImageReceiver::rosRawRightCallback(const sensor_msgs::Image& msg)
{
    if (isSpinning)
    {
        cv::Mat cameraFeed;
        imageCallback(msg, cameraFeed);
        emit rosRawRightSignal(cameraFeed);
    }
}

/**
 * @brief ImageReceiver::rosPclCallback callback function rof omega_rect_image topic
 * @param cloud message given from the Ros master
 */
void ImageReceiver::rosRectRightCallback(const sensor_msgs::Image& msg)
{
    if (isSpinning)
    {
        cv::Mat cameraFeed;
        imageCallback(msg, cameraFeed);
        emit rosRectRightSignal(cameraFeed);
    }
}

/**
 * @brief ImageReceiver::rosPclCallback callback function rof omega_raw_left_image topic
 * @param cloud message given from the Ros master
 */
void ImageReceiver::rosRawLeftCallback(const sensor_msgs::Image& msg)
{
    if (isSpinning)
    {
        cv::Mat cameraFeed;
        imageCallback(msg, cameraFeed);
        emit rosRawLeftSignal(cameraFeed);
    }
}

/**
 * @brief ImageReceiver::rosPclCallback callback function rof omega_rect_left_image topic
 * @param cloud message given from the Ros master
 */
void ImageReceiver::rosRectLeftCallback(const sensor_msgs::Image& msg)
{
    if (isSpinning)
    {
        cv::Mat cameraFeed;
        imageCallback(msg, cameraFeed);
        emit rosRectLeftSignal(cameraFeed);
    }
}

/**
 * @brief ImageReceiver::startSpinning start receiving message from ROS
 */
void ImageReceiver::startSpinning()
{
    if (isSpinning)
    {
        ros::spin();
    }
    isSpinning = true;
}


/**
 * @brief ImageReceiver::stopSpinning stop receiving message from ROS
 */
void ImageReceiver::stopSpinning()
{
    isSpinning = false;
}


ImageReceiver::~ImageReceiver()
{
}

