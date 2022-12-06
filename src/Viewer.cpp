#include <QDebug>
#include "Viewer.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

Viewer::Viewer(std::string hostname, int& argc, char** argv,
               bool _displayRightRaw, bool _displayLeftRaw,
               bool _displayDisp, bool _displayDispOverlay,
               bool _displayRightRect, bool _displayLeftRect,
               bool verbose, 
               std::string displayVideoName, cv::VideoWriter displayVideoWriter,
               std::string disparityVideoName, cv::VideoWriter disparityVideoWriter)
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
    , _displayRightRaw (_displayRightRaw)
    , _displayLeftRaw (_displayLeftRaw)
    , _displayDisp (_displayDisp)
    , _displayDispOverlay (_displayDispOverlay)
    , _displayRightRect (_displayRightRect)
    , _displayLeftRect (_displayLeftRect)
    , _displayVideoName (displayVideoName)
    , _displayVideoWriter (displayVideoWriter)
    , _disparityVideoName (disparityVideoName)
    , _disparityVideoWriter (disparityVideoWriter)
{
    _sceneRight = new QGraphicsScene();
    // _viewRight = new QGraphicsView(_sceneRight);
    // _viewRight->setWindowTitle(QString("ASHViewer - %1 - Raw Right View").arg(QString::fromStdString(hostname)));

    // _sceneLeft = new QGraphicsScene();
    // _viewLeft = new QGraphicsView(_sceneLeft);
    // _viewLeft->setWindowTitle(QString("ASHViewer - %1 - Raw Left View").arg(QString::fromStdString(hostname)));

    // _sceneDisparity = new QGraphicsScene();
    // _viewDisparity = new QGraphicsView(_sceneDisparity);
    // _viewDisparity->setWindowTitle(QString("ASHViewer - %1 - Disparity View").arg(QString::fromStdString(hostname)));

    _sceneDisparityOverlay = new QGraphicsScene();
    // _viewDisparityOverlay = new QGraphicsView(_sceneDisparityOverlay);
    // _viewDisparityOverlay->setWindowTitle(QString("ASHViewer - %1 - Disparity Overlay View").arg(QString::fromStdString(hostname)));

    // _sceneRightRectified = new QGraphicsScene();
    // _viewRightRectified = new QGraphicsView(_sceneRightRectified);
    // _viewRightRectified->setWindowTitle(QString("ASHViewer - %1 - Rectified Right View").arg(QString::fromStdString(hostname)));

    // _sceneLeftRectified = new QGraphicsScene();
    // _viewLeftRectified = new QGraphicsView(_sceneLeftRectified);
    // _viewLeftRectified->setWindowTitle(QString("ASHViewer - %1 - Rectified Left View").arg(QString::fromStdString(hostname)));

    /* Note that this signal will be called when all windows are closed */
    connect(_sceneRight, SIGNAL(destroyed()), this, SLOT(leave()));
    // connect(_sceneLeft, SIGNAL(destroyed()), this, SLOT(leave()));
    // connect(_sceneDisparity, SIGNAL(destroyed()), this, SLOT(leave()));
    connect(_sceneDisparityOverlay, SIGNAL(destroyed()), this, SLOT(leave()));
    // connect(_sceneRightRectified, SIGNAL(destroyed()), this, SLOT(leave()));
    // connect(_sceneLeftRectified, SIGNAL(destroyed()), this, SLOT(leave()));

    qRegisterMetaType< QImage::Format >("QImage::Format");
    connect(&_sensorControl, SIGNAL(newFrames(const unsigned char*,int,int,QImage::Format,
                                              const unsigned char*,int,int,QImage::Format)),
            this, SLOT(displayFrames(const unsigned char*,int,int,QImage::Format,
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

void Viewer::insertFrameToDisplayVideo(QImage& displayImage)
{
    
    cv::Mat out;
    cv::Mat view(544, 828, CV_8UC3, (void *)displayImage.constBits(), displayImage.bytesPerLine());
    cv::cvtColor(view, out, cv::COLOR_RGB2BGR);

    _displayVideoWriter.write(out);
}

void Viewer::insertFrameToDisparityVideo(QImage& disparityImage)
{
    
    cv::Mat out;
    cv::Mat view(544, 828, CV_8UC3, (void *)disparityImage.constBits(), disparityImage.bytesPerLine());
    cv::cvtColor(view, out, cv::COLOR_RGB2BGR);

    _disparityVideoWriter.write(out);
}

void Viewer::displayFrames(const unsigned char* right, int rightWidth, int rightHeight, QImage::Format rightFormat,
                           const unsigned char* disp, int dispWidth, int dispHeight, QImage::Format dispFormat)
{

    QImage displayImage(right, rightWidth, rightHeight, rightFormat);
    QImage disparityImage(disp, dispWidth, dispHeight, dispFormat);

    insertFrameToDisplayVideo(displayImage);
    insertFrameToDisparityVideo(disparityImage);
    
}
