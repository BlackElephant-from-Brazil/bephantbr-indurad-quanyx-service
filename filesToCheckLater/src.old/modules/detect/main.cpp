/**
 * @ingroup examples
 */
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <IDevice.h>
#include <IDeviceInformation.h>
#include <IDeviceManager.h>


using namespace std;

int main()
{
    
    /**
     * @brief manager The SensorHeadManager is used to detect heads available.
     */
    IDeviceManager manager;

    /* ---------------------
     *  Detect all sensors
     * ---------------------
     */
    cout << "Detecting sensors..." << endl;
    vector<shared_ptr<IDevice>> heads = manager.detectDevices();

    /* -------------------------------------------
     *  If no sensor detected, leave
     * -------------------------------------------
     */
    if (heads.size() == 0) {
        cout << "No sensor detected" << endl;
        return 1;
    }

    /* -------------------------------------------
     *  For each detected sensor, print its name
     * -------------------------------------------
     */
    for (const shared_ptr<IDevice> head: heads) {
        string sensorName = head->getDeviceInformation()->getName();
        if (sensorName != "") {
            cout << "Detected sensor head : " << sensorName << endl;
        } else {
            cerr << "Error while getting head name : " << endl;
        }
    }

    return 0;
}

