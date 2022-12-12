#include "pclViewer.h"


/**
 * @brief PclViewer::PclViewer Constructor
 * @param parent
 */
PclViewer::PclViewer (bool playData, bool recordData, QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PclViewer),
  isRecording(false)
{
    ui->setupUi (this);
    this->setWindowTitle ("Omega PCL viewer");

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<PointT>);
    cloud->points.resize (20000);

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("3D viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    ui->qvtkWidget->update ();

    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();

    qRegisterMetaType<PointCloudT>("PointCloudT");
    connect(&receiver,SIGNAL(rosPclSignal(PointCloudT)),this,SLOT(receivePcl(PointCloudT)));

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
        QStringList nameFilter("pcl*");
        QDir directory(qApp->applicationDirPath());
        QStringList filesAndDirectories = directory.entryList(nameFilter);
        _dataFolder = qApp->applicationDirPath()+"/"+filesAndDirectories.at(0);
        startReadingData();
        receiver.stopSpinning();
    }

    if(recordData)
        ui->recordButton->click();

    on_centerButton_clicked();
}

/**
 * @brief PclViewer::readpcd Read pcd file and update the viewer
 */
void PclViewer::readpcd()
{
    if (pcl::io::loadPCDFile<PointT>(QString(_dataFolder+"/data_pcd"+QString::number(frameNumber)+".pcd").toUtf8().constData(), *cloud) == -1)
    {
        receiver.startSpinning();
        return ;
    }
    else
    {
        frameNumber++;
        viewer->updatePointCloud(cloud,"sample cloud");
        ui->qvtkWidget->update ();
        if(isPlayed)
            QTimer::singleShot(500,this,SLOT(readpcd()));
    }
}

/**
 * @brief PclViewer::receivePcl receive frame from the PclReceiver and dispaly it
 * @param cloud
 */
void PclViewer::receivePcl(PointCloudT cloud)
{
    viewer->updatePointCloud(cloud,"sample cloud");
    ui->qvtkWidget->update ();

    if(isRecording)
    {
        pcl::io::savePCDFile(QString(recordeFolder+"/data_pcd"+QString::number(frameNumber)+".pcd").toUtf8().constData(), *cloud,true);
        frameNumber++;
    }
}

/**
 * @brief PclViewer::on_centerButton_clicked
 */
void PclViewer::on_centerButton_clicked()
{
    viewer->setCameraPosition(0, 0, -500,0, -1, 0);
    viewer->setCameraClipDistances (0, 25000);
}

/**
 * @brief PclViewer::on_recordButton_clicked
 */
void PclViewer::on_recordButton_clicked()
{
    if(!isRecording)
    {
        QDateTime date;
        QString dir = QString(qgetenv("ROS_RECORDER_PATH")) == ""  ? qApp->applicationDirPath() : QString(qgetenv("ROS_RECORDER_PATH"));
        recordeFolder = dir+"/pcl"+date.currentDateTime().toString("yyyyMMddHHmm");
        frameNumber=0;
        ui->recordButton->setText("Stop recording");
        QDir().mkdir(recordeFolder);
        isRecording = true;
    }
    else
    {
        ui->recordButton->setText("Start recording");
        isRecording = false;
    }
}

/**
 * @brief PclViewer::startReadingData launch the reading of the existance folder
 */
void PclViewer::startReadingData()
{

    if(!_dataFolder.isEmpty())
    {
        QStringList nameFilter("*.pcd");
        QDir directory(_dataFolder);
        QStringList txtFilesAndDirectories = directory.entryList(nameFilter);
        if(!txtFilesAndDirectories.isEmpty())
        {
            receiver.stopSpinning();
            QTimer::singleShot(50,this,SLOT(readpcd()));
            isPlayed = true;
            ui->playButton->setEnabled(true);
            ui->liveButton->setEnabled(true);
            return;
        }
    }
    QMessageBox::critical(this,"Error","no comatible files in selected folder",QMessageBox::Ok);
}

/**
 * @brief PclViewer::on_openButton_clicked
 */
void PclViewer::on_openButton_clicked()
{

    frameNumber=0;
    _dataFolder = QFileDialog::getExistingDirectory(this,"Select Output Folder",QApplication::applicationDirPath(),
                                                    QFileDialog::ShowDirsOnly| QFileDialog::DontResolveSymlinks);
    startReadingData();
}

/**
 * @brief PclViewer::on_previousButton_clicked
 */
void PclViewer::on_previousButton_clicked()
{
    frameNumber--;
    if (pcl::io::loadPCDFile<PointT>(QString(_dataFolder+"/data_pcd"+QString::number(frameNumber)+".pcd").toUtf8().constData(), *cloud) == -1) //* load the file
    {
        return ;
    }
    else
    {
        viewer->updatePointCloud(cloud,"sample cloud");
        ui->qvtkWidget->update ();
    }
}

/**
 * @brief PclViewer::on_playButton_clicked
 */
void PclViewer::on_playButton_clicked()
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
        QTimer::singleShot(50,this,SLOT(readpcd()));
        ui->previousButton->setEnabled(false);
        ui->nextButton->setEnabled(false);
        ui->playButton->setText("Pause");
    }
}

/**
 * @brief PclViewer::on_nextButton_clicked
 */
void PclViewer::on_nextButton_clicked()
{
    frameNumber++;
    if (pcl::io::loadPCDFile<PointT>(QString(_dataFolder+"/data_pcd"+QString::number(frameNumber)+".pcd").toUtf8().constData(), *cloud) == -1) //* load the file
    {
        return ;
    }
    else
    {
        viewer->updatePointCloud(cloud,"sample cloud");
        ui->qvtkWidget->update ();
    }
}

/**
 * @brief PclViewer::on_liveButton_clicked
 */
void PclViewer::on_liveButton_clicked()
{
    ui->liveButton->setEnabled(false);
    ui->previousButton->setEnabled(false);
    ui->nextButton->setEnabled(false);
    ui->playButton->setEnabled(false);
    isPlayed = false;
    receiver.startSpinning();
}

/**
 * @brief PclViewer::~PclViewer
 */
PclViewer::~PclViewer ()
{
  delete ui;
}
