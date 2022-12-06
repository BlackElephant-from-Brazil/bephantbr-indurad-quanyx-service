#include <iostream>
#include <cstdlib>
#include <string>
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

using namespace std;


/********************************************************************************
 *  ARGUMENT PARSING
 ********************************************************************************/

/* Parameter read from command line */

/**
 * @brief _hostname The hostname of the head we want to use
 */
static string _hostname;

/**
 * @brief _displayRightRaw True if user wants to display Right RAW images
 */
static bool _displayRightRaw = false;

/**
 * @brief _displayLeftRaw True if user wants to display Left RAW images
 */
static bool _displayLeftRaw = false;

/**
 * @brief _displayDisp True if user wants to display disparity map
 */
static bool _displayDisp = false;

/**
 * @brief _displayDispOverlay True if user wants to display disparity map
 */
static bool _displayDispOverlay = false;

/**
 * @brief _displayRightRect True if user wants to display right rectified image
 */
static bool _displayRightRect = false;

/**
 * @brief _displayLeftRect True if user wants to display left rectified image
 */
static bool _displayLeftRect = false;

/**
 * @brief _verbose True if user wants to get more information in standard output, like frame metata
 */
static bool _verbose = false;

/**
 * @brief print_help Print help about program arguments
 */
static void print_help()
{
    cout << "Arguments : [hostname|ip] [-vdDrlRL] " << endl << endl;
    cout << "\tThe head can not stream both the disparity map and the left image." << endl << endl;
    cout << "\thostname:\tName of sensor. You can use detect program to find the name of your sensor." << endl;
    cout << "\tip:\t\tIP adress of the head if you use a static ip adress." << endl;
    cout << "\t-r --right\tDisplay right raw image" << endl;
    cout << "\t-l --left\tDisplay left raw image" << endl;
    cout << "\t-d --disp\tDisplay disparity map colorized. closer = red, further = green" << endl;
    cout << "\t-D --disp_rect\tDisplay same disparity with rectified right image as background" << endl;
    cout << "\t-R --rect_right\tDisplay right rectified image using calibration data" << endl;
    cout << "\t-L --rect_left\tDisplay left rectified image using calibration data" << endl << endl;
    cout << "\t-v --verbose\tVerbose mode: print metadata got for each frames" << endl << endl;
    cout << "\t-h --help\tDisplay this help" << endl << endl;
    cout << "Note that, if no display option is given, raw display is selected by default." << endl;
    cout << "Multiple display options can be used at the same time." << endl;
    exit(1);
}

/**
 * @brief parse_cmdline Parse program cmdline to retrieve arguments
 * @param argc the number of arguments to parse
 * @param argv the arguments to parse
 */
void parse_cmdline(int argc, char *argv[])
{
    struct option long_options[] = {
        {"disp", no_argument, 0, 'd'},
        {"disp_rect", no_argument, 0, 'D'},
        {"right", no_argument, 0, 'r'},
        {"left", no_argument, 0, 'l'},
        {"rect_right", no_argument, 0, 'R'},
        {"rect_left", no_argument, 0, 'L'},
        {"help", no_argument, 0, 'h'},
        {"verbose", no_argument, 0, 'v'},
        {0, 0, 0, 0}
    };

    while (1) {
        /* Use getopt to parse commandline */
        int option_index = 0, c;
        c = getopt_long(argc, argv, "vdDrlRLh",long_options, &option_index);
        if (c == -1) {
            break;
        }

        switch (c)
        {
        case 'v':
            _verbose = true;
            break;
        case 'r':
            _displayRightRaw = true;
            break;
        case 'l':
            _displayLeftRaw = true;
            break;
        case 'd':
            _displayDisp = true;
            break;
        case 'D':
            _displayDispOverlay = true;
            break;
        case 'R':
            _displayRightRect = true;
            break;
        case 'L':
            _displayLeftRect = true;
            break;
        case 'h':
        case '?':
            print_help();
            break;
        default:
            break;
        }
    }

    //if we have one more element at the end, it is our head hostname
    if (optind < argc) {
        _hostname = argv[optind++];
    }

    // Check we have a non empty head hostname to look for
    if (_hostname.empty()) {
        print_help();
    }

    // Check if we have display options that have both disparity and left image
    if ((_displayDisp || _displayDispOverlay) && (_displayLeftRaw || _displayLeftRect) ) {
        print_help();
    }

    // Check we have at least one display option
    if (!_displayRightRaw && !_displayLeftRaw && !_displayDisp && !_displayDispOverlay && !_displayRightRect && !_displayLeftRect) {
        cout << "Enabling RAW display by default" << endl;
        _displayRightRaw = true;
    }
}

/********************************************************************************
 *  MAIN PROGRAM
 ********************************************************************************/

int main(int argc, char *argv[])
{
    /* Get arguments from cmdline */
    parse_cmdline(argc, argv);
    
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d:%H-%M-%S");
    auto displayVideoName = oss.str().append("_display_video.avi");
    auto disparityVideoName = oss.str().append("_disparity_video.avi");

    int frameWidth = 828;
	int frameHeigth = 544;

    cv::VideoWriter displayVideoWriter(displayVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(frameWidth, frameHeigth));
    cv::VideoWriter disparityVideoWriter(disparityVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(frameWidth, frameHeigth));
    
    /* Create and launch Viewer */
    Viewer viewer(_hostname, argc, argv, _displayRightRaw, _displayLeftRaw, _displayDisp, _displayDispOverlay, _displayRightRect, _displayLeftRect, _verbose, displayVideoName, displayVideoWriter, disparityVideoName, disparityVideoWriter);

    return viewer.exec();
}

