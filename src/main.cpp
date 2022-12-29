#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <sstream>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <regex>
#ifdef _WIN32
#include "wingetopt.h"
#else
#include <getopt.h>
#endif
#include "Viewer.h"
#include "IOManager.h"
#include <unistd.h>

#include <dirent.h>

#include <IDevice.h>
#include <IDeviceManager.h>


using namespace std;

/**
 * @brief _verbose True if user wants to get more information in standard output, like frame metata
 */
static bool _verbose = false;

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
            IOManager::clearNoSensorsFound();
        }
        else
        {
            cout << "no sensors found" << endl;
            system("pkill avahi");
            system("avahi-autoipd -D --force-bind --no-chroot eth0");
            IOManager::alertNoSensorsFound();
            usleep(5 * microsecond);
        }
    } while (heads.size() == 0);
    return sensorName;
}

bool compareString(std::string a, std::string b)
{
    return a > b;
}

void deleteLastVideos()
{
    string path = "/usr/bephantbr-indurad-quanyx-service/src/build/";
    DIR *dr;
    struct dirent *en;
    dr = opendir(path.c_str());
    vector<string> fileNames;
    if (dr)
    {
        while ((en = readdir(dr)) != NULL)
        {
            string fileName = en->d_name;
            if (fileName.find(".avi") != std::string::npos)
            {
                fileNames.push_back(fileName);
            }
        }

        closedir(dr);
    }
    std::sort(fileNames.begin(), fileNames.end(), compareString);
    if (fileNames.size() >= 4)
    {
        fileNames.erase(fileNames.begin(), fileNames.begin() + 4);
        for (string file : fileNames)
        {
            char fileToDelete[file.length() + 1];
            strcpy(fileToDelete, file.c_str());
            string fileInPath = path + fileToDelete;
            remove(fileInPath.c_str());
        }
    }
}

/********************************************************************************
 *  MAIN PROGRAM
 ********************************************************************************/

int main(int argc, char *argv[])
{
    IOManager::config();
    deleteLastVideos();
    string hostname = findFirstASHCamera();
    string cmd = "ASHConfig " + hostname + " -f 5 -b sgm_256x128";
    system(cmd.c_str());

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d:%H-%M-%S");
    auto displayVideoName = "/usr/bephantbr-indurad-quanyx-service/src/build/" + oss.str().append("_display_video.avi");
    auto disparityVideoName = "/usr/bephantbr-indurad-quanyx-service/src/build/" + oss.str().append("_disparity_video.avi");

    int frameWidth = 828;
    int frameHeigth = 544;

    cv::VideoWriter displayVideoWriter(displayVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5, cv::Size(frameWidth, frameHeigth));
    cv::VideoWriter disparityVideoWriter(disparityVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5, cv::Size(frameWidth, frameHeigth), 1);

    /* Create and launch Viewer */
    Viewer viewer(hostname, argc, argv, _verbose, displayVideoName, displayVideoWriter, disparityVideoName, disparityVideoWriter);

    return viewer.exec();
}