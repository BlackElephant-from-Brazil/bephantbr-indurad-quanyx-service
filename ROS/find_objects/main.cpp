#include <iostream>
#include <direct.h>

// ROS 
#include "ros/ros.h"

// Point Cloud Messages
#include <sensor_msgs/PointCloud2.h>
// Point Cloud Library
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
typedef pcl::PointXYZRGB PointT;

int counterPCD = 0;
std::string folderName = "";

void analizeFrame()
{
    cout << "Analizing frame" << endl;
}

void createFolder()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d:%H-%M-%S");

    folderName = "/usr/bephantbr-indurad-quanyx-service/ROS/find_objects/pcds/" + oss.str();

    mkdir(folderName);
}


bool compareString (std::string a, std::string b) 
{
    return a > b;
}

void deleteOlderFolder()
{
    struct dirent *en;
    dr = opendir("/usr/bephantbr-indurad-quanyx-service/ROS/find_objects/pcds/"); 
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
            remove(folderToDelete);
        }
    }
}

void savePCLToPCD(pcl::PointCloud<PointT> cloud)
{
    auto fileName = folderName + std::to_string(counterPCD) + ".pcd";

    pcl::io::savePCDFileASCII(fileName, cloud);
    counterPCD++;
    cout << "Saving frame" << endl;
}

void subCallback(const sensor_msgs::PointCloud2& ros_msg)
{
    pcl::PointCloud<PointT> pcl_cloud;
    pcl::fromROSMsg(ros_msg, pcl_cloud);
    
    cout << "*************************** NEW FRAME HERE ***************************" << endl;
    // for (auto point : pcl_cloud.points) {
    //     cout << point << endl;
    // }
    // cout << pcl_cloud.points.size() << endl;
    analizeFrame();
    savePCLToPCD(pcl_cloud);
    cout << "*************************** END OF FRAME HERE ***************************" << endl;

}

void receiveDataFromROS(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n_;
    ros::Subscriber sub = n_.subscribe("/omega_pcl", 10, &subCallback);

    ros::spin();

}

/*****************************************************************
 * MAIN FUNCTION
*****************************************************************/

int main(int argc, char *argv[])
{
    deleteOlderFolder();
    createFolder();
    receiveDataFromROS(argc, argv);
    return 0;
}
