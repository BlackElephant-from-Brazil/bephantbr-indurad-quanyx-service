#include <QDebug>
#include <vector>

#include <iostream>
#include <iomanip>

#include "SensorControlThread.h"
#include "ImageProcessing.h"
#include "IOManager.h"

#include <JetsonGPIO.h>

/********************************************************************************
 *  INIT / EXIT
 ********************************************************************************/

SensorControlThread::SensorControlThread(std::string hostname, bool verbose)
    : _hostname(hostname), _head(Q_NULLPTR), _leaving(false), _rgbRight(NULL), _rightRectified(NULL), _leftRectified(NULL), _rgbDisp(NULL), _rightWidth(0), _rightHeight(0), _rightWidthRectified(0), _rightHeightRectified(0), _leftWidthRectified(0), _leftHeightRectified(0), _dispWidth(0), _dispHeight(0), _displayRightRaw(true), _displayLeftRaw(false), _displayDisp(true), _displayRightRect(true), _displayLeftRect(false), _verbose(verbose), _rightLut(NULL), _leftLut(NULL)
{
}

SensorControlThread::~SensorControlThread()
{
    /* Free allocated buffers */
    if (_rgbDisp)
    {
        free(_rgbDisp);
        _rgbDisp = NULL;
    }
    if (_rightRectified)
    {
        free(_rightRectified);
        _rightRectified = NULL;
    }
    if (_leftRectified)
    {
        free(_leftRectified);
        _leftRectified = NULL;
    }
    if (_rgbRight)
    {
        free(_rgbRight);
        _rgbRight = NULL;
    }
}

bool SensorControlThread::init()
{

    IDeviceManager manager;

    /* Detect sensor using manager class */
    if (!_hostname.empty())
    {
        shared_ptr<IDevice> head = manager.detectDevice(_hostname);
        if (!head)
        {
            vector<shared_ptr<IDevice>> heads = manager.detectDevices();
            for (shared_ptr<IDevice> head : heads)
            {
                string sensorName = head->getDeviceInformation()->getName();
                if (!sensorName.empty() && sensorName == _hostname)
                {
                    _head = head;
                    break;
                }
            }
            if (!_head)
            {
                qCritical() << "Can't find head " << _hostname.c_str() << endl;
                return false;
            }
        }
        else
        {
            _head = head;
        }
    }

    // Check if display options doesn't match head configuration.
    std::string streamMode = _head->getModeOfStreaming();
    cout << "streamMode: " << streamMode << endl;
    if (streamMode.compare("camera_raw_left_right") == 0)
    {
        if (_displayDisp)
        {
            cerr << "The head can not stream the disparity map. Please change the streaming mode." << endl;
            return false;
        }
    }

    if (streamMode.compare("camera_raw_right_disp") == 0)
    {
        if (_displayLeftRaw || _displayLeftRect)
        {
            cerr << "The head can not stream the left image. Please change the streaming mode." << endl;
            return false;
        }
    }

    /* For color sensors, convert I420 to rgb24
     * This buffer stores the converted rgb image */
    std::string streamType = _head->getDeviceInformation()->getStreamType();

    /* Retrieve size of frames */
    _rightWidth = _head->getCameraParameters()->getNumberOfColumns();
    _rightHeight = _head->getCameraParameters()->getNumberOfRows();

    _leftWidth = _head->getCameraParameters()->getNumberOfColumns();
    _leftHeight = _head->getCameraParameters()->getNumberOfRows();

    ILUT *iLut = _head->getCameraParameters()->getCalibrationLUT();
    if (iLut)
    {
        _rightWidthRectified = iLut->getRightRectifyImgWidth();
        _rightHeightRectified = iLut->getRightRectifyImgHeight();

        _leftWidthRectified = iLut->getLeftRectifyImgWidth();
        _leftHeightRectified = iLut->getLeftRectifyImgHeight();

        /* Retrieve LUT (for rectified right image computation) */
        _rightLut = iLut->getRightLut();

        /* Retrieve LUT (for rectified left image computation) */
        _leftLut = iLut->getLeftLut();
    }

    _dispWidth = _head->getWidthDisparity();
    _dispHeight = _head->getHeightDisparity();

    /* Retrieve RAW image format */
    int raw_buffer_size_factor = 1;
    _rightFormat = QImage::Format_Grayscale8;
    _leftFormat = QImage::Format_Grayscale8;
    if (streamType.compare("I420") == 0)
    {
        _rightFormat = QImage::Format_RGB888;
        _leftFormat = QImage::Format_RGB888;
        raw_buffer_size_factor = 3;
    }

    /* Pre-allocate computed buffers. Other buffers are allocated in the library
     * Note that buffers are allocated here even if they are not displayed because some
     * buffers are intermediate result in other computation. For example, RGB right image
     * is computed (debayering) also for right rectified image.
     */
    /* Alloc buffer for the debayering result made in ImageProcessing */
    _rgbRight = (unsigned char *)malloc(raw_buffer_size_factor * _rightWidth * _rightHeight);

    /* Alloc buffer for right rectified image */
    _rightRectified = (unsigned char *)malloc(raw_buffer_size_factor * _rightWidthRectified * _rightHeightRectified);

    /* Alloc buffer for left rectified image */
    _leftRectified = (unsigned char *)malloc(raw_buffer_size_factor * _leftWidthRectified * _leftHeightRectified);

    /* Disparity map is colored, let's allocated a buffer containing colored data */
    _rgbDisp = (unsigned char *)malloc(3 * _dispWidth * _dispHeight);

    /* Note that is the disparity map format provided by THIS CLASS,
     * not the one got from library which is a uint8 one-component format. */
    _dispFormat = QImage::Format_RGB888;

    return true;
}

void SensorControlThread::stop()
{
    _mutex.lock();
    _leaving = true;
    _mutex.unlock();
}

void SensorControlThread::unrefFrame()
{
    if (!_currentDisplayedFrames.isEmpty())
    {
        // qDebug() << "Dropping one frame from the queue of size: " << _currentDisplayedFrames.size();
        _currentDisplayedFrames.dequeue();
    }
}

/********************************************************************************
 *  THREAD MAIN FUNCTION
 ********************************************************************************/

void SensorControlThread::run()
{
    volatile bool leaving = false;

    /* Detect head and initialize class members */
    bool initOk = init();
    if (!initOk)
    {
        /* Quit application on error */
        Q_EMIT noMoreFrame();
        return;
    }

    /* If no frame is received within 2s, exit application */
    _head->setBlockingReadTimeout(2000);

    /* Configure */
    _head->configureImageAcquisition();

    /* Start */
    _head->startImageAcquisition();
    if (!_head->isImageAcquisitionStarted())
    {
        qWarning() << "Error while starting streaming ! " << endl;
    }

    std::time_t time_begin = std::time(0);
    int tick = 0;
    int encounter_frame = 0;
    _current_fps = 0;

    while (!leaving)
    {
        IOManager::clearNoFrameDetected();
        /* Get a frame and transmit it to Viewer */
        shared_ptr<IData> frame = _head->blockingRead();

        if (!frame)
        {
            std::cout << "hmmmm...." << std::endl;
            IOManager::alertNoFrameDetected();
            continue;
        }
        else if (!frame->isValid())
        {
            qDebug() << "No valid frame received. Stopping..." << endl;
            stop();
            Q_EMIT noMoreFrame();
        }
        else
        {

            encounter_frame++;
            std::time_t time_now = std::time(0) - time_begin;

            if (time_now - tick >= 1)
            {
                tick++;
                _current_fps = encounter_frame;
                encounter_frame = 0;
            }
            processFrame(frame);
        }

        /* Check if we are leaving */
        _mutex.lock();
        leaving = _leaving;
        _mutex.unlock();
    }

    /* Stop */
    _head->stopImageAcquisition();
}

/********************************************************************************
 *  PROCESS ONE RECEIVED FRAME
 ********************************************************************************/
void SensorControlThread::processFrame(shared_ptr<IData> frame)
{
    const unsigned char *right = NULL;
    const unsigned char *left = NULL;
    const unsigned char *disp = NULL;
    const unsigned char *rightRect = NULL;
    const unsigned char *leftRect = NULL;

    if (frame->isValid())
    {

        /* Handle right raw */
        if (_displayRightRaw || _displayRightRect)
        {

            /* Use directly buffer returned by library (Gray8 and I420)*/
            // NTV I420 has already been converted in RGB format by the library
            right = frame->getRightImageData();
        }

        /* Handle left raw */
        if (_displayLeftRaw || _displayLeftRect)
        {

            /* Use directly buffer returned by library (Gray8 and I420)*/
            // NTV I420 has already been converted in RGB format by the library
            left = frame->getLeftImageData();
        }

        /* Handle right rectified image */
        if (_displayRightRect)
        {

            /* Color sensor */
            if (_rightFormat == QImage::Format_RGB888)
            {

                /* Build a rectified rgb image from the calibration lut and the RGB image */
                ImageProcessing::getRectifyImgRgb(_rightRectified,
                                                  _rightWidthRectified,
                                                  _rightHeightRectified,
                                                  _rightLut,
                                                  right,
                                                  _rightWidth);
            }
            else
            {

                /* Build a rectified grey level image from the calibration lut and the grey level image*/
                ImageProcessing::getRectifyImg(_rightRectified,
                                               _rightWidthRectified,
                                               _rightHeightRectified,
                                               _rightLut,
                                               right,
                                               _rightWidth);
            }

            rightRect = _rightRectified;
        }

        /* Handle left rectified image */
        if (_displayLeftRect)
        {

            /* Color sensor */
            if (_leftFormat == QImage::Format_RGB888)
            {

                /* Build a rectified rgb image from the calibration lut and the RGB image */
                ImageProcessing::getRectifyImgRgb(_leftRectified,
                                                  _leftWidthRectified,
                                                  _leftHeightRectified,
                                                  _leftLut,
                                                  left,
                                                  _leftWidth);
            }
            else
            {

                /* Build a rectified grey level image from the calibration lut and the grey level image*/
                ImageProcessing::getRectifyImg(_leftRectified,
                                               _leftWidthRectified,
                                               _leftHeightRectified,
                                               _leftLut,
                                               left,
                                               _leftWidth);
            }

            leftRect = _leftRectified;
        }

        /* As right image could have been computed for right rectified image, drop the
         * pointer not to display if the user don't want to see it.
         */
        if (!_displayRightRaw)
        {
            right = NULL;
        }

        /* As left image could have been computed for left rectified image, drop the
         * pointer not to display if the user don't want to see it.
         */
        if (!_displayLeftRaw)
        {
            left = NULL;
        }

        /* Handle disparity map */
        if (_displayDisp)
        {
            if (strcmp(frame->getStreamingDispFormat(), "GRAY16") == 0)
            {

                /* Build a disparity rgb image from the color lut and the GRAY16 disparity data */
                ImageProcessing::convertGray162RGB(frame->getDisparityData(),
                                                   _rgbDisp,
                                                   frame->getWidthDisparityData(),
                                                   frame->getHeightDisparityData());
            }
            else
            {

                /* Build a disparity rgb image from the color lut and the GRAY8 disparity data */
                ImageProcessing::convertGray82RGB(frame->getDisparityData(),
                                                  _rgbDisp,
                                                  frame->getWidthDisparityData(),
                                                  frame->getHeightDisparityData());
            }

            disp = _rgbDisp;
        }
        _currentDisplayedFrames.append(frame);

        Q_EMIT newFrames(right, _rightWidth, _rightHeight, _rightFormat,
                         left, _leftWidth, _leftHeight, _leftFormat,
                         disp, _dispWidth, _dispHeight, _dispFormat,
                         rightRect, _rightWidthRectified, _rightHeightRectified, _rightFormat,
                         leftRect, _leftWidthRectified, _leftHeightRectified, _leftFormat);
    }

    if (_verbose)
    {
        cout << endl
             << "================ Begin Frame Metadata ================" << endl;
        cout << "Is valid: " << (frame->isValid() ? "yes" : "no") << endl;
        if (frame->isValid())
        {
            struct timespec triggerTimestamp, beforePushTimestamp;
            triggerTimestamp = frame->getSHAcquisitionTimestamp();
            beforePushTimestamp = frame->getSHDataOutTimestamp();

            cout << "Frame number:\t" << frame->getFrameNumber() << endl;
            cout << "Current FPS:\t" << _current_fps << endl;
            cout << "Width raw image:\t" << frame->getWidth2DImageData() << endl;
            cout << "Height raw image:\t" << frame->getHeight2DImageData() << endl;
            cout << "Width disparity image:\t" << frame->getWidthDisparityData() << endl;
            cout << "Height disparity image:\t" << frame->getHeightDisparityData() << endl;
            cout << "Streaming Disp Format:\t" << frame->getStreamingDispFormat() << endl;
            cout << "Timestamp trigger (sec):\t" << triggerTimestamp.tv_sec << "."
                 << setfill('0') << setw(9) << triggerTimestamp.tv_nsec << endl;
            cout << "Timestamp before pushed (sec):\t" << beforePushTimestamp.tv_sec << "."
                 << setfill('0') << setw(9) << beforePushTimestamp.tv_nsec << endl;
            cout << "Analog gain applied:\t" << frame->getAnalogGain() << endl;
            cout << "Exposure time (µs):\t" << frame->getExposureTimeUs() << endl;
        }
        cout << "================= End Frame Metadata =================" << endl;
    }
}
