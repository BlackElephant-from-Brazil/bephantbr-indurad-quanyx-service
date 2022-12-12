#include "imageViewer.h"
#include "Colormap.h"

ImageViewer::ImageViewer (bool playData, bool recordData,QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::ImageViewer),
  _imageGra(nullptr),
  isRecording(false),
  isPlayed(false),
  frameNumber(0)
{
    ui->setupUi (this);
    this->setWindowTitle ("Omega image viewer");

    scene = new QGraphicsScene();
    ui->graphicsView->setScene(scene);

    qRegisterMetaType<Image>("Image");
    connect(&receiver,SIGNAL(rosRectRightSignal(Image)),this,SLOT(receiveRectRightImage(Image)));
    connect(&receiver,SIGNAL(rosRawRightSignal(Image)),this,SLOT(receiveRawRightImage(Image)));
    connect(&receiver,SIGNAL(rosRectLeftSignal(Image)),this,SLOT(receiveRectLeftImage(Image)));
    connect(&receiver,SIGNAL(rosRawLeftSignal(Image)),this,SLOT(receiveRawLeftImage(Image)));
    connect(&receiver,SIGNAL(rosDisparitySignal(Image)),this,SLOT(receiveDispImage(Image)));

    receiverThread = new QThread;
    receiver.moveToThread(receiverThread);
    connect(receiverThread, SIGNAL (started()), &receiver, SLOT (startSpinning()));
    connect(receiverThread, SIGNAL (finished()), &receiver, SLOT (stopSpinning()));

    if(!playData)
    {
        receiverThread->start();
    }
    else
    {
        frameNumber=0;
        QStringList nameFilter("image*");
        QDir directory(qApp->applicationDirPath());
        QStringList filesAndDirectories = directory.entryList(nameFilter);
        _dataFolder = qApp->applicationDirPath()+"/"+filesAndDirectories.at(0);
        startReadingData();
    }

    if(recordData)
        ui->recordButton->click();
}

void ImageViewer::colorDispFrame(Image & frame)
{
    if(frame.channels() != 1)
    {
        return;
    }

    // Colorise frame for display
    cvtColor(frame, frame, CV_GRAY2RGB);
    for (int y = 0; y < frame.rows; y++) {
        for (int x = 0; x < frame.cols; x++) {
            const unsigned char greyValue = frame.at<cv::Vec3b>(y, x)[0];

            frame.at<cv::Vec3b>(y, x)[0] = _colorMapDisp[greyValue][2];
            frame.at<cv::Vec3b>(y, x)[1] = _colorMapDisp[greyValue][1];
            frame.at<cv::Vec3b>(y, x)[2] = _colorMapDisp[greyValue][0];
        }
    }
}

void ImageViewer::displayFrame(Image & frame)
{
    QImage image;
    if(frame.channels() == 3)
    {
        cvtColor(frame, frame, CV_BGR2RGB); // convert image from BGR To RGB
        image = QImage((uchar*)frame.data, frame.cols, frame.rows, QImage::Format_RGB888);
    }
    else
    {
        image = QImage((uchar*)frame.data, frame.cols, frame.rows, QImage::Format_Indexed8);
    }

    if(image.isNull())
        return;

    if (_imageGra == nullptr)
    {
        _imageGra = scene->addPixmap(QPixmap::fromImage(image));
    }
    else
    {
        _imageGra->setPixmap(QPixmap::fromImage(image));
        
    }
    ui->graphicsView->fitInView(_imageGra, Qt::KeepAspectRatio);
    /* Ask to draw again */
    scene->update();
}

void ImageViewer::displayRecordedFrame(QVector<cv::Mat> & vect)
{
    if (vect.empty())
        return;

    Image frame = vect.at(frameNumber);
    displayFrame(frame);
}

void ImageViewer::recordFrame(Image & frame, cv::VideoWriter & writer, const QString & filename)
{
    /*If recording button is clicked*/
    if (!isRecording)
        return;

    bool use_compression = false;
    bool isColor = (frame.channels() != 1);
    const QString filepath = recordeFolder + filename;
    int codec = (use_compression) ? cv::VideoWriter::fourcc('M', 'J', 'P', 'G') : 0;
    double fps = 14.0;
    
    if (!use_compression || isColor)
    {
        if (!writer.isOpened())
        {
            writer.open(filepath.toUtf8().constData(), codec, fps, frame.size(), isColor);
        }
        writer.write(frame);
        return;
    }

    // use_compression = true && !isColor
    cv::Mat color_frame = frame.clone();
    cvtColor(color_frame,color_frame, CV_GRAY2RGB);
    if (!writer.isOpened())
    {
        writer.open(filepath.toUtf8().constData(), codec, fps, color_frame.size(), true);
    }
    writer.write(color_frame);
}

void ImageViewer::receiveRectRightImage(Image frame)
{
    recordFrame(frame, rectRightVideoWriter, "/rectRightImage.avi");

    /*If rectified view is checked*/
    if(ui->rectRightRadio->isChecked())
    {
        displayFrame(frame);
    }
}

void ImageViewer::receiveRectLeftImage(Image frame)
{
    recordFrame(frame, rectLeftVideoWriter, "/rectLeftImage.avi");

    /*If rectified view is checked*/
    if(ui->rectLeftRadio->isChecked())
    {
        displayFrame(frame);
    }
}

void ImageViewer::receiveDispImage(Image frame)
{
    recordFrame(frame, dispGrayVideoWriter, "/dispGrayImage.avi");

    /*If disparity view is checked*/
    if(ui->dispRadioGray->isChecked())
    {
        displayFrame(frame);
    }

    cv::Mat color_frame; // = frame.clone();
    frame.copyTo(color_frame);
    colorDispFrame(color_frame);
    recordFrame(color_frame, dispColorVideoWriter, "/dispColorImage.avi");

    // If disparity view is checked
    if(ui->dispRadioColor->isChecked())
    {
        displayFrame(color_frame);
    }
}

void ImageViewer::receiveRawRightImage(Image frame)
{
    recordFrame(frame, rawRightVideoWriter, "/rawRightImage.avi");

    /*If raw view is checked*/
    if(ui->rawRightRadio->isChecked())
    {
        displayFrame(frame);
    }
}

void ImageViewer::receiveRawLeftImage(Image frame)
{

    recordFrame(frame, rawLeftVideoWriter, "/rawLeftImage.avi");

    /*If raw view is checked*/
    if(ui->rawLeftRadio->isChecked())
    {
        displayFrame(frame);
    }
}

void ImageViewer::readData()
{
    /*If playing from existiong local file*/
    if(isPlayed)
    {
        if(ui->rawRightRadio->isChecked())
        {
            displayRecordedFrame(rawRightFramesVector);
        }
        else if(ui->rectRightRadio->isChecked())
        {
            displayRecordedFrame(rectRightFramesVector);
        }
        else if(ui->rawLeftRadio->isChecked())
        {
            displayRecordedFrame(rawLeftFramesVector);
        }
        else if(ui->rectLeftRadio->isChecked())
        {
            displayRecordedFrame(rectLeftFramesVector);
        }
        else if(ui->dispRadioGray->isChecked())
        {
            displayRecordedFrame(dispGrayFramesVector);
        }
        else
        {
            displayRecordedFrame(dispColorFramesVector);
        }

        if(rawRightFramesVector.size() != 0 && frameNumber < rawRightFramesVector.size()-1)
        {
            frameNumber ++;
            QTimer::singleShot(100,this,SLOT(readData())); // 10 FPS
        }
    }
}

void ImageViewer::selectRecordedFrame(bool next)
{
    if (next)
    {
        if(frameNumber == rawRightFramesVector.size()-1)
            return;
        frameNumber++;
    }
    else
    {
        if (frameNumber == 0)
            return;
        frameNumber--;
    }

    cv::Mat frame;
    if(ui->rawRightRadio->isChecked())
    {
        frame = rawRightFramesVector.at(frameNumber).clone();
    }
    else if(ui->rectRightRadio->isChecked())
    {
        frame = rectRightFramesVector.at(frameNumber).clone();
    }
    else if(ui->rawLeftRadio->isChecked())
    {
        frame = rawLeftFramesVector.at(frameNumber).clone();
    }
    else if(ui->rectLeftRadio->isChecked())
    {
        frame = rectLeftFramesVector.at(frameNumber).clone();
    }
    else if(ui->dispRadioGray->isChecked())
    {
        frame = dispGrayFramesVector.at(frameNumber).clone();
    }
    else
    {
        frame = dispColorFramesVector.at(frameNumber).clone();
    }
    displayFrame(frame);
}

void ImageViewer::on_previousButton_clicked()
{
    selectRecordedFrame(false);
}

void ImageViewer::on_nextButton_clicked()
{
    selectRecordedFrame(true);
}

void ImageViewer::on_playButton_clicked()
{
    if(isPlayed)
    {
        isPlayed = false;
        ui->playButton->setText("Play");
        ui->previousButton->setEnabled(true);
        ui->nextButton->setEnabled(true);
    }
    else
    {
        isPlayed = true;
        QTimer::singleShot(50,this,SLOT(readData()));
        ui->previousButton->setEnabled(false);
        ui->nextButton->setEnabled(false);
        ui->playButton->setText("Pause");
    }
}

void ImageViewer::readingData(const QString & filename, QVector<cv::Mat> & vect)
{
    const QString filepath = _dataFolder + filename;
    cv::VideoCapture video(filepath.toUtf8().constData()); // open the raw video file
    if (!video.isOpened())  // check if we succeeded
        return;

    for(int frameNum = 0; frameNum < video.get(CV_CAP_PROP_FRAME_COUNT);frameNum++)
    {
        Image frame;
        video >> frame; // get the next frame from video
        vect.append(frame);
    }
}

void ImageViewer::startReadingData()
{
    if(!_dataFolder.isEmpty())
    {
        ui->playButton->setEnabled(true);
        ui->liveButton->setEnabled(true);
        receiver.stopSpinning(); //stop spinning

        readingData("/rawRightImage.avi", rawRightFramesVector);
        readingData("/rectRightImage.avi", rectRightFramesVector);
        readingData("/rawLeftImage.avi", rawLeftFramesVector);
        readingData("/rectLeftImage.avi", rectLeftFramesVector);
        readingData("/dispGrayImage.avi", dispGrayFramesVector);
        readingData("/dispColorImage.avi", dispColorFramesVector);

        isPlayed = true;
        QTimer::singleShot(50,this,SLOT(readData()));
    }
}

void ImageViewer::on_openButton_clicked()
{

    frameNumber = 0;
    rawRightFramesVector.clear();
    dispGrayFramesVector.clear();
    rectRightFramesVector.clear();
    _dataFolder = QFileDialog::getExistingDirectory(this,"Select Output Folder",QApplication::applicationDirPath(),QFileDialog::ShowDirsOnly| QFileDialog::DontResolveSymlinks);
    startReadingData();
}

void ImageViewer::on_recordButton_clicked()
{
    if(!isRecording)
    {
        QDateTime date;
        QString dir = QString(qgetenv("ROS_RECORDER_PATH")) == ""  ? qApp->applicationDirPath() : QString(qgetenv("ROS_RECORDER_PATH"));
        recordeFolder = dir+"/image"+date.currentDateTime().toString("yyyyMMddHHmm");
        ui->recordButton->setText("Stop recording");
        QDir().mkdir(recordeFolder);
        isRecording = true;
    }
    else
    {
        rawRightVideoWriter.release();
        rectRightVideoWriter.release();
        rawLeftVideoWriter.release();
        rectLeftVideoWriter.release();
        dispGrayVideoWriter.release();
        dispColorVideoWriter.release();
        ui->recordButton->setText("Start recording");
        isRecording = false;
    }
}

void ImageViewer::on_liveButton_clicked()
{
    ui->liveButton->setEnabled(false);
    ui->previousButton->setEnabled(false);
    ui->nextButton->setEnabled(false);
    ui->playButton->setEnabled(false);
    isPlayed = false;
    receiver.startSpinning();
}

ImageViewer::~ImageViewer ()
{
    if(isRecording)
    {
        rawRightVideoWriter.release();
        rectRightVideoWriter.release();
        rawLeftVideoWriter.release();
        rectLeftVideoWriter.release();
        dispGrayVideoWriter.release();
        dispColorVideoWriter.release();
    }
    delete ui;
}
