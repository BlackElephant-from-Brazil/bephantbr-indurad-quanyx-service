#include <IDeviceInformation.h>
#include <IDeviceManager.h>
#include <ILUT.h>

#include "omega_camera/ImageProcessing.h"
#include <omega_msgs/OmegaMetadata.h>

#include "ros/ros.h"
#include <ros/console.h>
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
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include "std_msgs/String.h"
#include <sensor_msgs/fill_image.h>

#include <syslog.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <math.h>

#define PROJECT "OMEGA_ROS"
#define PROGRAM_NAME "omega_camera"

void log_info(const char *error_info, int code){

    /*
     * To have syslog capabilites in your code just copy-paste this function in it.
    Be sure to include <syslog.h>
    to call the function:  log_info("some text here",error_code<int>);
    */
    char buffer[140]; //as twitter used to do

    const char *Error_code[3]; //Standar convention of error codes
    Error_code[0] = "INFO";
    Error_code[1] = "WARNING";
    Error_code[2] = "ERROR";

    snprintf (buffer,sizeof(buffer),"[%s][%s][%s]: %s\n", PROJECT,Error_code[code],PROGRAM_NAME,error_info); //build the message
    openlog("slog", LOG_PID|LOG_CONS, LOG_USER); //opens log file
    syslog(LOG_INFO,"%s", buffer); // write the buffer
    closelog(); //close the logfile
    ROS_INFO_STREAM(error_info);
}

void getPointCloud(const double & focallength,
                   const double & baseline,
                   const double & rightopticalcenterx,
                   const double & rightopticalcentery,
                   const int & disp_subpixel_coef,
                   const sensor_msgs::Image & disp,
                   const sensor_msgs::Image & recimg,
                   sensor_msgs::PointCloud2 & ros_msg_pcl){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr omg_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Assigning all points to single row to keep PointCloud unordered
    omg_pcl->width = disp.width * disp.height;
    omg_pcl->height = 1;
    omg_pcl->points.resize (omg_pcl->width * omg_pcl->height);

    float Z;
    const uint16_t *data_16_bits = (const uint16_t *)disp.data.data();
    for (size_t i = 0; i < omg_pcl->points.size (); ++i)
    {
        int column = (int) fmod(i,disp.width) + 1;
        int row = floor(i/disp.width) + 1;

        omg_pcl->points[i].x = 0;
        omg_pcl->points[i].y = 0;
        omg_pcl->points[i].z = 0;
        float disp_value = (disp.encoding == "mono16") ? (float) data_16_bits[i] : (float) disp.data[i];
        if (disp_value > 0)
        {
            Z = (baseline * focallength) / (disp_value / disp_subpixel_coef);
            omg_pcl->points[i].x = (Z*(column - rightopticalcenterx))/focallength;
            omg_pcl->points[i].y = (Z*(row - rightopticalcentery))/focallength;
            omg_pcl->points[i].z = Z;
        }

        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
        if (recimg.encoding == "rgb8") {
            // pack r/g/b into rgb
            r = recimg.data[((i*3))];
            g = recimg.data[((i*3)+1)];
            b = recimg.data[((i*3)+2)];
        }
        else if (recimg.encoding == "mono8")
        {
            r = recimg.data[i];
            g = recimg.data[i];
            b = recimg.data[i];
        }
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        omg_pcl->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }

    pcl::toROSMsg(*omg_pcl,ros_msg_pcl);
}

int main(int argc, char **argv)
{
    /* ---------------------
     *  Get hostname
     * ---------------------
     */
    std::string hostname = "";
        //if we have one more element at the end, it is our head hostname
    if (optind < argc) {
        hostname = argv[optind++];
    }

    /* ---------------------
     *  Sensors Detection
     * ---------------------
     */
    IDeviceManager manager;
    // Get the detected head by ip
    log_info("Detecting sensor ...", 0);
    shared_ptr<IDevice> _head;
    if(!hostname.empty())
    {
        _head = manager.detectDevice(hostname);
    }

    // Detect all sensors
    if (!_head) {
        // Get the list of detected heads
        log_info("Detecting all sensors ...", 0);
        vector<shared_ptr<IDevice>> heads = manager.detectDevices();
        shared_ptr<IDevice> first_detected_head;

        // return the head matching the hostname we are looking for
        for (shared_ptr<IDevice> head: heads) {
            if (!first_detected_head) {
                first_detected_head = head;
            }
            if ((!hostname.empty()) && (head->getDeviceInformation()->getName() == hostname)) {
                _head = head;
                break;
            }
        }
        if (!_head) {
            _head = first_detected_head;
            log_info("A default camera as been found", 1);
        }
    }

    if (!_head) {
            log_info("No Sensor Detected", 2);
            return 1;
        }

    /* ----------------------------------------
     *  Configure Camera
     * ----------------------------------------
     */
     /* Retrieve size of frames */
    int _raw_img_width = _head->getCameraParameters()->getNumberOfColumns();
    int _raw_img_height = _head->getCameraParameters()->getNumberOfRows();
    int _rect_img_width = _head->getCameraParameters()->getCalibrationLUT()->getRectifyImgWidth();
    int _rect_img_height = _head->getCameraParameters()->getCalibrationLUT()->getRectifyImgHeight();

    /* Retrieve LUT (for rectified image computation) */
    pixelCoord* _lut_right = _head->getCameraParameters()->getCalibrationLUT()->getRightLut();
    pixelCoord* _lut_left = _head->getCameraParameters()->getCalibrationLUT()->getLeftLut();

    /* Pre-allocate buffers */
    std::string stream_type = _head->getDeviceInformation()->getStreamType();
    std::string streaming_raw_format = "mono8";
    int raw_buffer_size_factor = 1;
    if (stream_type.compare("I420") == 0)
    {
        raw_buffer_size_factor = 3;
        streaming_raw_format = "rgb8";
    }
    unsigned char * raw_frame = (unsigned char *) malloc(raw_buffer_size_factor * _raw_img_width * _raw_img_height);
    unsigned char * rect_frame = (unsigned char *) malloc(raw_buffer_size_factor * _rect_img_width * _rect_img_height);

    /* Configure Streaming */
    _head->configureImageAcquisition();

    /* starting ros */
    log_info("Starting ROS ...", 0);
    ros::init(argc, argv, "omega_camera");
    ros::NodeHandle node_handle;

    ros::Publisher omega_raw_img = node_handle.advertise<sensor_msgs::Image>("omega_raw_image", 1);
    ros::Publisher omega_rec_img = node_handle.advertise<sensor_msgs::Image>("omega_rec_image", 1);
    ros::Publisher omega_raw_left_img = node_handle.advertise<sensor_msgs::Image>("omega_raw_left_image", 1);
    ros::Publisher omega_rec_left_img = node_handle.advertise<sensor_msgs::Image>("omega_rec_left_image", 1);
    ros::Publisher omega_disp_img = node_handle.advertise<sensor_msgs::Image>("omega_disp_image", 1);
    ros::Publisher omega_metadata = node_handle.advertise<omega_msgs::OmegaMetadata>("omega_metadata", 1);
    ros::Publisher omega_pcl = node_handle.advertise<sensor_msgs::PointCloud2>("omega_pcl",1);

    sensor_msgs::Image img_right_;
    sensor_msgs::Image rec_img_right_;
    sensor_msgs::Image img_left_;
    sensor_msgs::Image rec_img_left_;
    sensor_msgs::Image disp_;
    omega_msgs::OmegaMetadata metadata_;
    sensor_msgs::PointCloud2 pcl_;

    std::string header_frame_id = "/world";

    img_right_.header.frame_id = header_frame_id;
    rec_img_right_.header.frame_id = header_frame_id;

    img_left_.header.frame_id = header_frame_id;
    rec_img_left_.header.frame_id = header_frame_id;

    disp_.header.frame_id = header_frame_id;


    std::string message = "Getting/Setting Sensor Parameters from sensor: " + _head->getDeviceInformation()->getSerialNumber();
    log_info(message.c_str(), 1);

    /* If no frame is received within 2s, exit application */
    _head->setBlockingReadTimeout(2000);

    /* Pack Metadata Message */
    metadata_.rightwidth =_raw_img_width;
    metadata_.rightheight = _raw_img_height;
    metadata_.rightwidthrectified = _rect_img_width;
    metadata_.rightheightrectified = _rect_img_height;
    metadata_.dispwidth = _head->getWidthDisparity();
    metadata_.dispheight = _head->getHeightDisparity();
    metadata_.focallength = _head->getCameraParameters()->getFocalLength();
    metadata_.baseline = _head->getCameraParameters()->getBaseLine();
    metadata_.rightopticalcenterx = _head->getCameraParameters()->getRightOpticalCenterX();
    metadata_.rightopticalcentery = _head->getCameraParameters()->getRightOpticalCenterY();
    metadata_.temperature = _head->getDeviceStatus()->getTemperatureLevel();
    metadata_.fps = _head->getFps();
    metadata_.maxexposuretime = _head->getMaxExposureTime();
    metadata_.greylevelpercentage = _head->getGreyLevelPercentage();
    metadata_.blockingreadtimeout = _head->getBlockingReadTimeout();
    metadata_.modeofstreaming = _head->getModeOfStreaming();
    metadata_.serialnumber = _head->getDeviceInformation()->getSerialNumber();

    /* Start Streaming */
    _head->startImageAcquisition();

    ros::Rate loop_rate(10); // 10 messages by seconds
    log_info("Starting ROS loop", 0);

    // Check if
    if(!ros::master::check())
    {
        log_info("No running master found", 2);
        _head->stopImageAcquisition();
        return 1;
    }

    int return_code = 0;
    while(ros::ok())
    {
        /* Get a frame */
        shared_ptr<IData> frame = _head->blockingRead();
        if (!frame || !frame->isValid()) {
            log_info("No frame received. Waiting...", 2);
            /* Stop ROS */
            // ros::shutdown();
            // return_code = 1;
            continue;
        }
        int int_seq_id = frame->getFrameNumber();
        uint32_t uint_seq_id = (int_seq_id >= 0 ? static_cast<uint32_t>(int_seq_id) : 0);

        /*
        2021-06-03 (JIRA TICKET OMG-364):

        There is non-fixable bug inside third-party software which causes non-stability of timestamp --
        the values, provided by getSHAcquisitionTimestamp(), can be inconsistent over time, abruptly
        decreasing or even becoming invalid values.

        The simplest workaround is to provide getSHDataOutTimestamp() as timestamp of acquisition,
        which brings ~70ms of latency between acquisition and data output into equation, but provides
        much more steady monotinic clock.
        */
       // struct timespec timestamp_data_acqui = frame->getSHAcquisitionTimestamp();
        struct timespec timestamp_data_out = frame->getSHDataOutTimestamp();
        ros::Time time_data_out = ros::Time(timestamp_data_out.tv_sec, timestamp_data_out.tv_nsec);

        if (metadata_.modeofstreaming.find("right") != std::string::npos)
        {
            /* Raw Right Frame */
            raw_frame = (unsigned char *) frame->getRightImageData();

            fillImage(img_right_, streaming_raw_format, _raw_img_height, _raw_img_width, raw_buffer_size_factor * _raw_img_width, raw_frame);

            img_right_.header.seq = uint_seq_id;
            img_right_.header.stamp = time_data_out;

            /* Rectified Right Frame */
            if (stream_type.compare("I420") == 0)
            {
                ImageProcessing::getRectifyImgRgb(rect_frame, _rect_img_width, _rect_img_height, _lut_right, raw_frame, _raw_img_width);
            }
            else
            {
                ImageProcessing::getRectifyImg(rect_frame, _rect_img_width, _rect_img_height, _lut_right, raw_frame, _raw_img_width);
            }

            fillImage(rec_img_right_, streaming_raw_format, _rect_img_height, _rect_img_width, raw_buffer_size_factor * _rect_img_width, rect_frame);

            rec_img_right_.header.seq = uint_seq_id;
            rec_img_right_.header.stamp = time_data_out;

            omega_raw_img.publish(img_right_);
            omega_rec_img.publish(rec_img_right_);
        }

        if (metadata_.modeofstreaming.find("left") != std::string::npos)
        {
            /* Raw Left Frame */
            raw_frame = (unsigned char *) frame->getLeftImageData();
            fillImage(img_left_, streaming_raw_format, _raw_img_height, _raw_img_width, raw_buffer_size_factor * _raw_img_width, raw_frame);

            img_left_.header.seq = uint_seq_id;
            img_left_.header.stamp = time_data_out;


            /* Rectified Left Frame */
            if (stream_type.compare("I420") == 0)
            {
                ImageProcessing::getRectifyImgRgb(rect_frame, _rect_img_width, _rect_img_height, _lut_left, raw_frame, _raw_img_width);
            }
            else
            {
                ImageProcessing::getRectifyImg(rect_frame, _rect_img_width, _rect_img_height, _lut_left, raw_frame, _raw_img_width);
            }

            fillImage(rec_img_left_, streaming_raw_format, _rect_img_height, _rect_img_width, raw_buffer_size_factor * _rect_img_width, rect_frame);

            rec_img_left_.header.seq = uint_seq_id;
            rec_img_left_.header.stamp = time_data_out;

            omega_raw_left_img.publish(img_left_);
            omega_rec_left_img.publish(rec_img_left_);
        }

        if (metadata_.modeofstreaming.find("disp") != std::string::npos)
        {
            /* Disparity Matrix */
            std::string streaming_disp_format = "mono8";
            int disp_buffer_size_coef = 1;
            int disp_subpixel_coef = 4;
            if (strcmp(frame->getStreamingDispFormat(), "GRAY16") == 0) {
                streaming_disp_format = "mono16";
                disp_buffer_size_coef = 2;
                disp_subpixel_coef = 16;
            }

            fillImage(disp_, streaming_disp_format, frame->getHeightDisparityData(), frame->getWidthDisparityData(), disp_buffer_size_coef*frame->getWidthDisparityData(), frame->getDisparityData());

            disp_.header.seq = uint_seq_id;
            disp_.header.stamp = time_data_out;

            omega_disp_img.publish(disp_);

            /* PCL */
            getPointCloud(metadata_.focallength,
                          metadata_.baseline * 0.001, // baseline is in mm,
                          metadata_.rightopticalcenterx,
                          metadata_.rightopticalcentery,
                          disp_subpixel_coef,
                          disp_,
                          rec_img_right_,
                          pcl_);

            pcl_.header = disp_.header;
            pcl_.header.stamp = ros::Time::now();
            pcl_.header.frame_id = "omega_cam";
            omega_pcl.publish(pcl_);
        }

        omega_metadata.publish(metadata_);
        loop_rate.sleep();

    }
    _head->stopImageAcquisition();
    return return_code;
}
