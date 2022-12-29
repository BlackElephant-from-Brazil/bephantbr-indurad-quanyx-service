#include <iostream>
#include <sys/stat.h>

// ROS 
#include "ros/ros.h"

// Point Cloud Messages
#include <sensor_msgs/PointCloud2.h>
// Point Cloud Library
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <JetsonGPIO.h>

#include <IDevice.h>
#include <IDeviceManager.h>

using namespace std;
using namespace GPIO;
typedef pcl::PointXYZRGB PointT;

int counterPCD = 0;
std::string folderName = "";

void analizeFrame()
{
}

void turnServiceOn()
{
    cout << "changing to SERVICE" << endl;
    system("systemctl start change-to-service-process.service");
}

void configureGPIO()
{
    const int button_change_to_service = 22;

    cout << "STARTING GPIO" << endl;

    cleanup();
    setmode(BCM);

    setup(IOManager::button_change_to_service, IN);

    add_event_detect(IOManager::button_change_to_service, GPIO::Edge::RISING, IOManager::turnServiceOn, 10);
}

void createFolder()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d:%H-%M-%S");

    folderName = "/usr/bephantbr-indurad-quanyx-service/ROS/find_objects/pcds/" + oss.str();

    mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void deleteAll(string folderName)
{
    
    DIR *dr;
    struct dirent *en;
    dr = opendir(folderName.c_str()); 
    if (dr) {
        while ((en = readdir(dr)) != NULL) {
            string fileName = en->d_name;
            string fileToDelete = folderName + "/" + fileName;
            cout << fileToDelete << endl;
            remove(fileToDelete.c_str());
        }

        closedir(dr);
    }
}


bool compareString (std::string a, std::string b) 
{
    return a > b;
}

void deleteOlderFolder()
{
    string path = "/usr/bephantbr-indurad-quanyx-service/ROS/find_objects/pcds/";

    DIR *dr;
    struct dirent *en;
    dr = opendir(path.c_str()); 
    vector<string> folderNames;
    if (dr) {
        while ((en = readdir(dr)) != NULL) {
            string folderName = en->d_name;
            folderNames.push_back(folderName);
        }

        closedir(dr);
    }

    std::sort(folderNames.begin(),folderNames.end(),compareString);
    if (folderNames.size() >= 3) {
        folderNames.erase(folderNames.begin(), folderNames.begin() + 3);
        for (string folder: folderNames) {
            char folderToDelete[folder.length() + 1];
            strcpy(folderToDelete, folder.c_str());
            string folderInPath =  path + folderToDelete;
            deleteAll(folderInPath);
            rmdir(folderInPath.c_str());
        }
    }
}

void savePCLToPCD(pcl::PointCloud<PointT> cloud)
{
    auto fileName = folderName + "/" + std::to_string(counterPCD) + ".pcd";

    pcl::io::savePCDFileASCII(fileName, cloud);
    counterPCD++;
}

void subCallback(const sensor_msgs::PointCloud2& ros_msg)
{
    pcl::PointCloud<PointT> pcl_cloud;
    pcl::fromROSMsg(ros_msg, pcl_cloud);
    analizeFrame();
    savePCLToPCD(pcl_cloud);
}

void receiveDataFromROS(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n_;
    ros::Subscriber sub = n_.subscribe("/omega_pcl", 10, &subCallback);

    ros::spin();

}

/*****************************************************************
 * PALEATIVE SOLUTION
*****************************************************************/

string findFirstASHCamera()
{
    string sensorName = "";
    IDeviceManager manager;
    unsigned int microsecond = 1000000;
    vector<shared_ptr<IDevice>> heads;

    do
    {
        heads = manager.detectDevices();
        if (heads.size() > 0)
        {
            sensorName = heads[0]->getDeviceInformation()->getName();
        }
        else
        {
            cout << "no sensors found" << endl;
            system("pkill avahi");
            system("avahi-autoipd -D --force-bind --no-chroot eth0");
            usleep(5 * microsecond);
        }
    } while (heads.size() == 0);
    return sensorName;
}

void cameraConfigurations()
{
    string hostname = findFirstASHCamera();
    string cmd = "ASHConfig " + hostname + " -f 5 -b sgm_256x128";
    system(cmd.c_str());
}

/*****************************************************************
 * END OF PALEATIVE SOLUTION
*****************************************************************/

/*****************************************************************
 * MAIN FUNCTION
*****************************************************************/

int main(int argc, char *argv[])
{

    // Resolving problems with camera with paleative solution
    cameraConfigurations();

    // It runs correctly
    deleteOlderFolder();
    createFolder();
    receiveDataFromROS(argc, argv);
    return 0;
}
