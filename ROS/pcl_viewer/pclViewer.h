#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// Qt
#include <QMainWindow>
#include <QDebug>
#include <QFileDialog>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>

#include "ui_pclViewer.h"

#include "pclReceiver.h"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>




namespace Ui
{
    class PclViewer;
}

class PclViewer : public QMainWindow
{
Q_OBJECT

public:
    explicit PclViewer (bool playData,bool recordData,QWidget *parent = 0);

    ~PclViewer ();

signals:

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;

    PointCloudT cloud;

public slots:
    void receivePcl(PointCloudT cloud);

private slots:
    void on_centerButton_clicked();
    void on_recordButton_clicked();
    void on_openButton_clicked();
    void on_previousButton_clicked();
    void on_playButton_clicked();
    void on_nextButton_clicked();
    void on_liveButton_clicked();

    void readpcd();
    void startReadingData();
private:
    Ui::PclViewer *ui;

    PclReceiver receiver;

    int frameNumber;

    bool isRecording;
    bool isPlayed;

    QString _dataFolder;
    QThread* receiverThread;
    QString recordeFolder;

};
#endif
