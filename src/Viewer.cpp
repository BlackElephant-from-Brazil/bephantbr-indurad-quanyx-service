#include <iostream>
#include <QDebug>
#include "Viewer.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

Viewer::Viewer(std::string hostname, int& argc, char** argv,
           bool verbose, 
           std::string _displayVideoName, cv::VideoWriter _displayVideoWriter,
           std::string _disparityVideoName, cv::VideoWriter _disparityVideoWriter)
    : QApplication(argc, argv)
    , _sensorControl(hostname, verbose)
    , _sceneRight (NULL)
    , _sceneLeft (NULL)
    , _sceneDisparity (NULL)
    , _sceneDisparityOverlay (NULL)
    , _sceneRightRectified (NULL)
    , _sceneLeftRectified (NULL)
    , _viewRight (NULL)
    , _viewLeft (NULL)
    , _viewDisparity (NULL)
    , _viewDisparityOverlay (NULL)
    , _viewRightRectified (NULL)
    , _viewLeftRectified (NULL)
    , _right (NULL)
    , _left (NULL)
    , _disparity (NULL)
    , _disparityOverlay (NULL)
    , _rightRectified (NULL)
    , _leftRectified (NULL)
    , _displayVideoName (_displayVideoName)
    , _disparityVideoName (_disparityVideoName)
    , _displayVideoWriter (_displayVideoWriter)
    , _disparityVideoWriter (_disparityVideoWriter)
{
    qRegisterMetaType< QImage::Format >("QImage::Format");
    connect(&_sensorControl, SIGNAL(newFrames(const unsigned char*,int,int,QImage::Format,
                                              const unsigned char*,int,int,QImage::Format,
                                              const unsigned char*,int,int,QImage::Format,
                                              const unsigned char*,int,int,QImage::Format,
                                              const unsigned char*,int,int,QImage::Format)),
            this, SLOT(displayFrames(const unsigned char*,int,int,QImage::Format,
                                     const unsigned char*,int,int,QImage::Format,
                                     const unsigned char*,int,int,QImage::Format,
                                     const unsigned char*,int,int,QImage::Format,
                                     const unsigned char*,int,int,QImage::Format)));


    connect(this, SIGNAL(disposeFrame()),
            &_sensorControl, SLOT(unrefFrame()));

    connect(&_sensorControl, SIGNAL(noMoreFrame()), this, SLOT(quit()));

    // Start our receiver thread
    _sensorControl.start();
}

Viewer::~Viewer()
{
    delete _viewRight;
    delete _sceneRight;
    delete _viewLeft;
    delete _sceneLeft;
    delete _viewDisparity;
    delete _sceneDisparity;
    delete _viewDisparityOverlay;
    delete _sceneDisparityOverlay;
    delete _viewRightRectified;
    delete _sceneRightRectified;
    delete _viewLeftRectified;
    delete _sceneLeftRectified;
}

void Viewer::leave()
{
    _sensorControl.stop();
    _sensorControl.wait(10000);
    this->quit();
}

void Viewer::insertFrames(QImage& displayImage, QImage& disparityImage)
{
    cv::Mat displayOut;
    cv::Mat disparityOut;
    
    cv::Mat displayView(544, 828, CV_8UC3, (void *)displayImage.constBits(), displayImage.bytesPerLine());
    cv::Mat disparityView(544, 828, CV_8UC3, (void *)disparityImage.constBits(), disparityImage.bytesPerLine());
    
    cv::cvtColor(displayView, displayOut, cv::COLOR_RGB2BGR);
    cv::cvtColor(disparityView, disparityOut, cv::COLOR_RGB2BGR);

    _displayVideoWriter.write(displayOut);
    _disparityVideoWriter.write(disparityOut);
}

void Viewer::displayFrames(const unsigned char* right, int rightWidth, int rightHeight, QImage::Format rightFormat,
                           const unsigned char* left, int leftWidth, int leftHeight, QImage::Format leftFormat,
                           const unsigned char* disp, int dispWidth, int dispHeight, QImage::Format dispFormat,
                           const unsigned char* rightRect, int rightRectWidth, int rightRectHeight, QImage::Format rightRectFormat,
                           const unsigned char* leftRect, int leftRectWidth, int leftRectHeight, QImage::Format leftRectFormat)
{
    QImage rightImage(right, rightWidth, rightHeight, rightFormat);

    // Rectified right as background
    QImage imgOverlay(right, rightWidth, rightHeight, rightFormat);
    
    // Overlay with disparity
    QPainter painter(&imgOverlay);
    QImage imgDisparity(disp, dispWidth, dispHeight, dispFormat);
    painter.setOpacity(0.5);
    painter.drawImage(imgOverlay.rect(), imgDisparity);
    painter.end();
    
    insertFrames(rightImage, imgOverlay);
    // Free the last received frame.
    emit disposeFrame();
}
