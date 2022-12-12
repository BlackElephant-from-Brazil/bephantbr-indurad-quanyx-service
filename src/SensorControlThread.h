#ifndef SensorControlThread_H
#define SensorControlThread_H

#include <memory>
#include <QThread>
#include <QMutex>
#include <QImage>
#include <QQueue>

#include <IDevice.h>
#include <IData.h>

#include <IDeviceManager.h>

/*!
 * @class SensorControlThread
 * @brief Thread that is only used to get a buffer and give it to the Viewer using signal
 */
class SensorControlThread : public QThread
{
    Q_OBJECT

public:
    SensorControlThread(std::string hostname, bool verbose);
    ~SensorControlThread();

    /**
     * @brief stop Stop properly the thread
     *
     * This is a MT-safe function. This makes the run() function
     * to stop looping and return. The associated thread will exit normally.
     */
    void stop();

Q_SIGNALS:
    /**
     * @brief displayFrames Signal emitted when a new set of buffers is available
     *
     * For buffer which should not be displayed, the pointer is NULL.
     */
    void newFrames(const unsigned char* right, int rightWidth, int rightHeight, QImage::Format rightFormat,
                   const unsigned char* disp, int dispWidth, int dispHeight, QImage::Format dispFormat);

    /**
     * @brief noMoreFrame Signal sent when we did not get any new frame, before the blockingRead timeout expired
     */
    void noMoreFrame();

public Q_SLOTS:
    /**
     * @brief unrefFrame Release the reference on the last frame got.
     * In fact, this is even not mandatory as we are resetting when get a new frame in all cases.
     */
    void unrefFrame();

protected:

    /**
     * @brief processFrame Internal function to process one frame when a new one is returned by the library
     * @param frame the new frame
     */
    void processFrame(shared_ptr<IData> frame);

    /**
     * @brief run Thread main loop
     *
     * The streaming is then started and each new frame is got and
     * processed (calling processFrame()).
     *
     * The loop is stopped if stop() is called.
     */
    void run();

    /**
     * @brief init Library initialization
     * @return true on success, false on error
     *
     * Initialize library, detect and configure sensor.
     * Called at the beginning of this thread.
     */
    bool init();

private:

    /**
     * @brief _hostname The name of the sensor
     */
    std::string _hostname;

    /**
     * @brief _head The head we are viewving the stream from.
     */
    shared_ptr<IDevice> _head;

    /**
     * @brief _leaving The boolean used to check if we have to continue the reception of frame
     */
    bool _leaving;

    /**
     * @brief _mutex Mutex used to be sure we are not accessing _leaving from different threads
     */
    QMutex _mutex;

    /**
     * @brief _rgbRight The data right image colored data
     *
     * RGB RAW right image is computed from Bayer right image provided by the library
     *
     * In case of converted bayer image, the memory is allocated in this class, otherwise
     * it points to data internally-allocated by the library.
     */
    unsigned char *_rgbRight;

    /**
     * @brief _rightRectified The right rectified image (can either be RGB or grey level, depending on the sensor-type)
     *
     * The memory is allocated in this class. The library only provides lookup table to compute this image.
     */
    unsigned char* _rightRectified;

    /**
     * @brief _leftRectified The left rectified image (can either be RGB or grey level, depending on the sensor-type)
     *
     * The memory is allocated in this class. The library only provides lookup table to compute this image.
     */
    unsigned char* _leftRectified;

    /**
     * @brief _rgbDisp The disparity map in RGB format
     *
     * The library only provides disparity map in custom format (coded in uint8). This class
     * colorize disparity and the result is stored in _rgbDisp.
     *
     * The memory is allocated in this class.
     */
    unsigned char* _rgbDisp;

    /**
     * @brief _rightWidth The width of the right image
     */
    int _rightWidth;

    /**
     * @brief _leftWidth The width of the left image
     */
    int _leftWidth;

    /**
     * @brief _rightHeight The height of the right image
     */
    int _rightHeight;

    /**
     * @brief _leftHeight The height of the left image
     */
    int _leftHeight;

    /**
     * @brief _rightWidthRectified The width of the right rectified image (in pixels)
     */
    int _rightWidthRectified;

    /**
     * @brief _rightHeightRectified The height of the right rectified image (in pixels)
     */
    int _rightHeightRectified;

    /**
     * @brief _leftWidthRectified The width of the left rectified image (in pixels)
     */
    int _leftWidthRectified;

    /**
     * @brief _leftHeightRectified The height of the left rectified image (in pixels)
     */
    int _leftHeightRectified;

    /**
     * @brief _dispWidth The width of the disparity map
     */
    int _dispWidth;

    /**
     * @brief _dispHeight The height of the disparity map
     */
    int _dispHeight;

    /**
     * @brief current_fps frame encounter
     */
    int _current_fps;

    /**
     * @brief _displayRightRaw True if user wants to display right RAW images
     */
    bool _displayRightRaw;

    /**
     * @brief _displayLeftRaw True if user wants to display left RAW images
     */
    bool _displayLeftRaw;

    /**
     * @brief _displayDisp True if user wants to display disparity map
     */
    bool _displayDisp;

    /**
     * @brief _displayRightRect True if user wants to display right rectified image
     */
    bool _displayRightRect;

    /**
     * @brief _displayLeftRect True if user wants to display left rectified image
     */
    bool _displayLeftRect;

    /**
     * @brief _verbose True if user wants to get more information in standard output, like frame metata
     */
    bool _verbose;

    /**
     * @brief _rightLut The Look Up Table to use to build a right rectified image.
     *
     * The right lut is retrieved from the head
     */
    pixelCoord* _rightLut;

    /**
     * @brief _leftLut The Look Up Table to use to build a left rectified image.
     *
     * The left lut is retrieved from the head
     */
    pixelCoord* _leftLut;

    /**
     * @brief _rightFormat The format (Qt enum) for right image provided by this class to the Viewer
     *
     * Note that is also the format of rectified right image
     * Could be QImage::Format_RGB888 or QImage::Format_Indexed8
     */
    QImage::Format _rightFormat;

    /**
     * @brief _leftFormat The format (Qt enum) for left image provided by this class to the Viewer
     *
     * Note that is also the format of rectified left image
     * Could be QImage::Format_RGB888 or QImage::Format_Indexed8
     */
    QImage::Format _leftFormat;

    /**
     * @brief _dispFormat The format (Qt enum) for disparity map provided by this class to the Viewer
     *
     * For now, only colored disparity is sent, so it is QImage::Format_RGB888
     */
    QImage::Format _dispFormat;

    /**
     * @brief _currentDisplayedFrames The last frames we got.
     *
     * At the start of streaming, frames are received quicker than they are displayed.
     * So we have to store a references on them so the buffers are valid until displayed.
     */
    QQueue<shared_ptr<IData>> _currentDisplayedFrames;
};

#endif
