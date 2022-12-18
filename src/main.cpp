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
#include<unistd.h>

#include <dirent.h>

#include <IDevice.h>
#include <IDeviceManager.h>

#include <JetsonGPIO.h>

using namespace std;

/**
 * @brief _verbose True if user wants to get more information in standard output, like frame metata
 */
static bool _verbose = false;

void alertNoSensorsFound() 
{
    int output_no_sensor_found = 18;

    GPIO::setmode(GPIO::BCM);
    GPIO::setup(output_no_sensor_found, GPIO::OUT, 1);

    GPIO::output(output_no_sensor_found, 1);

    GPIO::cleanup();
}

void clearNoSensorsFound()
{
    int output_no_sensor_found = 18;

    GPIO::setmode(GPIO::BCM);
    GPIO::setup(output_no_sensor_found, GPIO::OUT, 0);

    GPIO::output(output_no_sensor_found, 0);

    GPIO::cleanup();
}

string findFirstASHCamera() 
{
    string sensorName = "";
    IDeviceManager manager;
    unsigned int microsecond = 1000000;
    vector<shared_ptr<IDevice>> heads;

    do {
        heads = manager.detectDevices();
        if (heads.size() > 0) {
            sensorName = heads[0]->getDeviceInformation()->getName();
            clearNoSensorsFound();
        } else {
            cout << "no sensors found" << endl;
            alertNoSensorsFound();
            usleep(1 * microsecond);
        }
    } while (heads.size() == 0);
    return sensorName;
}

bool compareString (std::string a, std::string b) 
{
    return a > b;
}

void deleteLastVideos()
{
    
    DIR *dr;
    struct dirent *en;
    dr = opendir("/usr/bephantbr-indurad-quanyx-service/src/build/"); 
    vector<string> fileNames;
    if (dr) {
        while ((en = readdir(dr)) != NULL) {
            string fileName = en->d_name;
            if (fileName.find(".avi") != std::string::npos) {
                fileNames.push_back(fileName);
            }
        }

        closedir(dr);
    }
    std::sort(fileNames.begin(),fileNames.end(),compareString);
    if (fileNames.size() >= 4) {
        fileNames.erase(fileNames.begin(), fileNames.begin()+4);
        for (string file: fileNames) {
            char fileToDelete[file.length() + 1];
            strcpy(fileToDelete, file.c_str());
            remove(fileToDelete);
        }
    }
}

/********************************************************************************
 *  MAIN PROGRAM
 ********************************************************************************/

int main(int argc, char *argv[])
{
    deleteLastVideos();
    string hostname = findFirstASHCamera();
    
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d:%H-%M-%S");
    auto displayVideoName = "/usr/bephantbr-indurad-quanyx-service/src/build/" + oss.str().append("_display_video.avi");
    auto disparityVideoName = "/usr/bephantbr-indurad-quanyx-service/src/build/" + oss.str().append("_disparity_video.avi");

    int frameWidth = 828;
	int frameHeigth = 544;

    cv::VideoWriter displayVideoWriter(displayVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(frameWidth, frameHeigth));
    cv::VideoWriter disparityVideoWriter(disparityVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(frameWidth, frameHeigth), 1);
    
    /* Create and launch Viewer */
    cout << "launching view" << endl;
    Viewer viewer(hostname, argc, argv, _verbose, displayVideoName, displayVideoWriter, disparityVideoName, disparityVideoWriter);

    return viewer.exec();
}