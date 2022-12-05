#include "pclViewer.h"
#include <QApplication>
#include <QMainWindow>
#include <getopt.h>
#include <QDebug>


/**
 * @brief _recordData start recording pcl_viewer
 */
bool _recordData = false;

/**
 * @brief _playData strt playing a folder in the application folder
 */
bool _playData = false;


/**
 * @brief parse_cmdline parse the command line arguments
 * @param argc
 * @param argv
 */
void parse_cmdline(int argc, char *argv[])
{

    while (1) {
        /* Use getopt to parse commandline */

        int c = getopt(argc, argv, "rp");
        if (c == -1) {
            break;
        }

        switch (c)
        {
        case 'r':
            if(!_playData)
            {
                _recordData = true;
            }
            break;
        case 'p':
            if(!_recordData)
            {
                _playData = true;
            }
            break;
        default:
            break;
        }
    }
}

int main (int argc, char *argv[])
{
    parse_cmdline(argc,argv);
    ros::init(argc, argv, "pcl_viewer");
    QApplication a (argc, argv);
    PclViewer w(_playData,_recordData);
    w.show ();

    return a.exec ();
}
