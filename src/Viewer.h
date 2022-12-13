#ifndef VIEWER_H
#define VIEWER_H

#include <memory>
#include <string>
#include <QApplication>
#include <QString>
#include <QThread>
#include <QtWidgets/QGraphicsView> // QGraphicsView & QGraphicsScene
#include <QtWidgets/QGraphicsPixmapItem>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "SensorControlThread.h"

/*!
 * @class Viewer
 * @brief Simple example to display video and disparity from sensor heads in real time
 */
class Viewer : public QApplication
{
    Q_OBJECT
public:
    /**
     * @brief Viewer Construct a view of the specified head
     * @param argc The number of arguments passed to the program (passed by Qt)
     * @param argv The arguments passed to the program (passed by Qt)
     * @param head The head from which we want to view the stream
     */
    Viewer(std::string hostname, int& argc, char** argv,
           bool verbose, 
           std::string _displayVideoName, cv::VideoWriter _displayVideoWriter,
           std::string _disparityVideoName, cv::VideoWriter _disparityVideoWriter);

    ~Viewer();

public Q_SLOTS:
    /**
     * @brief displayFrames Called when a new frame is received from the SensorControlThread
     * @param right Pointer to right buffer data (or NULL if it should be not displayed)
     * @param rightWidth Width of right image
     * @param rightHeight Height of right image
     * @param rightFormat Format of right image
     * @param left Pointer to left buffer data (or NULL if it should be not displayed)
     * @param leftWidth Width of left image
     * @param leftHeight Height of left image
     * @param leftFormat Format of left image
     * @param disp Pointer to disp buffer data (or NULL if it should be not displayed)
     * @param dispWidth Width of disp image
     * @param dispHeight Height of disp image
     * @param dispFormat Format of disp image
     * @param rightRect Pointer to right rect buffer data (or NULL if it should be not displayed)
     * @param rightRectWidth Width of right rect image
     * @param rightRectHeight Height of right rect image
     * @param rightRectFormat Format of right rect image
     * @param leftRect Pointer to left rect buffer data (or NULL if it should be not displayed)
     * @param leftRectWidth Width of left rect image
     * @param leftRectHeight Height of left rect image
     * @param leftRectFormat Format of left rect image
     */
    void displayFrames(const unsigned char* right, int rightWidth, int rightHeight, QImage::Format rightFormat,
                       const unsigned char* left, int leftWidth, int leftHeight, QImage::Format leftFormat,
                       const unsigned char* disp, int dispWidth, int dispHeight, QImage::Format dispFormat,
                       const unsigned char* rightRect, int rightRectWidth, int rightRectHeight, QImage::Format rightRectFormat,
                       const unsigned char* leftRect, int leftRectWidth, int leftRectHeight, QImage::Format leftRectFormat);

    void leave();

Q_SIGNALS:
    void disposeFrame();

protected:
    /**
     * @brief updateView update the given view using the given QImage
     * @param view View to update (_viewRight, _viewDisparity or _viewRightRectified)
     * @param scene Scene to update (_sceneRight, _sceneDisparity or _sceneRightRectified)
     * @param pixmap [in/out] The pixmap to use (or to create if a pointer pointing to NULL is given)
     * @param image The image to display
     */
    void insertFrames(QImage& displayImage, QImage& disparityImage);

private:
    /**
     * @brief _sensorControl The thread used to receive frames from the head
     */
    SensorControlThread _sensorControl;

    /**
     * @brief _sceneRight This scene is used to render the right image of the head
     */
    QGraphicsScene *_sceneRight;

    /**
     * @brief _sceneLeft This scene is used to render the left image of the head
     */
    QGraphicsScene *_sceneLeft;

    /**
     * @brief _sceneDisparity This scene is used to render the disparity image of the head
     */
    QGraphicsScene *_sceneDisparity;

    /**
     * @brief _sceneDisparity This scene is used to render the disparity image of the head
     */
    QGraphicsScene *_sceneDisparityOverlay;

    /**
     * @brief _sceneRightRectified This scene is used to render the right image rectified of the head
     */
    QGraphicsScene *_sceneRightRectified;

    /**
     * @brief _sceneLeftRectified This scene is used to render the left image rectified of the head
     */
    QGraphicsScene *_sceneLeftRectified;

    /**
     * @brief _viewRight This view is used to render the right image of the head
     */
    QGraphicsView *_viewRight;

    /**
     * @brief _viewLeft This view is used to render the left image of the head
     */
    QGraphicsView *_viewLeft;

    /**
     * @brief _viewDisparity This view is used to render the disparity image of the head
     */
    QGraphicsView *_viewDisparity;

    /**
     * @brief _viewDisparity This view is used to render the disparity image of the head
     */
    QGraphicsView *_viewDisparityOverlay;

    /**
     * @brief _viewRightRectified This view is used to render the right image rectified of the head
     */
    QGraphicsView *_viewRightRectified;

    /**
     * @brief _viewLeftRectified This view is used to render the left image rectified of the head
     */
    QGraphicsView *_viewLeftRectified;

    /**
     * @brief _right This is used to render the right image
     */
    QGraphicsPixmapItem *_right;

    /**
     * @brief _left This is used to render the left image
     */
    QGraphicsPixmapItem *_left;

    /**
     * @brief _disparity This used to render the disparity image
     */
    QGraphicsPixmapItem *_disparity;

    /**
     * @brief _disparity This used to render the disparity overlay image
     */
    QGraphicsPixmapItem *_disparityOverlay;

    /**
     * @brief _rightRectified This is used to render the right rectified image
     */
    QGraphicsPixmapItem *_rightRectified;

    /**
     * @brief _leftRectified This is used to render the left rectified image
     */
    QGraphicsPixmapItem *_leftRectified;

    /**
     * @brief _displayVideoName This is used to save the display video locally
     */
    std::string _displayVideoName;

    /**
     * @brief _displayVideoWriter This is used to write the display video
     */
    cv::VideoWriter _displayVideoWriter;

    /**
     * @brief _disparityVideoName This is used to save the disparity video locally
     */
    std::string _disparityVideoName;

    /**
     * @brief _disparityVideoWriter This is used to write the disparity video
     */
    cv::VideoWriter _disparityVideoWriter;
};

#endif
