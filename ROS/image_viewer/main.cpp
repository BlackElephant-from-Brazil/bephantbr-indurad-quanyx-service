#include "imageViewer.h"
#include <QMainWindow>
#include <signal.h>
#include <initializer_list>

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

void catchUnixSignals(std::initializer_list<int> quitSignals) {
    auto handler = [](int sig) -> void {
        // blocking and not aysnc-signal-safe func are valid
        printf("\nquit the application by signal(%d).\n", sig);
        QApplication::quit();
    };

    sigset_t blocking_mask;
    sigemptyset(&blocking_mask);
    for (auto sig : quitSignals)
        sigaddset(&blocking_mask, sig);

    struct sigaction sa;
    sa.sa_handler = handler;
    sa.sa_mask    = blocking_mask;
    sa.sa_flags   = 0;

    for (auto sig : quitSignals)
        sigaction(sig, &sa, nullptr);
}

int main (int argc, char *argv[])
{
    parse_cmdline(argc,argv);
    ros::init(argc, argv, "image_viewer");
    QApplication a (argc, argv);
    catchUnixSignals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});
    ImageViewer w(_playData,_recordData);
    w.show ();

    return a.exec ();
}
