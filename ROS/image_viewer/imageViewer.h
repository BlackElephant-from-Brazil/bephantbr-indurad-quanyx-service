#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

// Qt
#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include "ui_imageViewer.h"
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "imageReceiver.h"
#include <QGraphicsView>
#include <QVector>


namespace Ui
{
    class ImageViewer;
}

class ImageViewer : public QMainWindow
{
Q_OBJECT

public:
    explicit ImageViewer (bool playData, bool recordData, QWidget *parent = 0);

    ~ImageViewer ();

signals:

protected:

public slots:
    void receiveDispImage(Image frame);
    void receiveRectRightImage(Image frame);
    void receiveRawRightImage(Image frame);
    void receiveRectLeftImage(Image frame);
    void receiveRawLeftImage(Image frame);

private slots:
    void on_recordButton_clicked();
    void on_openButton_clicked();
    void on_previousButton_clicked();
    void on_playButton_clicked();
    void on_nextButton_clicked();
    void on_liveButton_clicked();

    void readData();
private:
    void displayFrame(Image & frame);
    void selectRecordedFrame(bool next);
    void displayRecordedFrame(QVector<cv::Mat> & vect);
    void recordFrame(Image & frame, cv::VideoWriter & writer, const QString & filename);
    void colorDispFrame(Image & frame);
    void readingData(const QString & filename, QVector<cv::Mat> & vect);
    void startReadingData();
    Ui::ImageViewer *ui;

    ImageReceiver receiver;

    QThread* receiverThread;

    QGraphicsScene *scene;

    QGraphicsPixmapItem *_imageGra;

    int frameNumber;
    bool isRecording;
    bool isPlayed;
    QString _dataFolder;
    QString recordeFolder;
    cv::VideoWriter rawRightVideoWriter;
    cv::VideoWriter rectRightVideoWriter;
    cv::VideoWriter rawLeftVideoWriter;
    cv::VideoWriter rectLeftVideoWriter;
    cv::VideoWriter dispGrayVideoWriter;
    cv::VideoWriter dispColorVideoWriter;

    QVector<cv::Mat> rawRightFramesVector;
    QVector<cv::Mat> rectRightFramesVector;
    QVector<cv::Mat> rawLeftFramesVector;
    QVector<cv::Mat> rectLeftFramesVector;
    QVector<cv::Mat> dispGrayFramesVector;
    QVector<cv::Mat> dispColorFramesVector;


};
#endif
