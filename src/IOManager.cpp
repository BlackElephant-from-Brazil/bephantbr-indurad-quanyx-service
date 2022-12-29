#include <iostream>
#include "IOManager.h"
#include <JetsonGPIO.h>

using namespace std;
using namespace GPIO;

void IOManager::config()
{
    cout << "STARTING GPIO" << endl;
    cleanup();
    setmode(BCM);

    setup(IOManager::led_no_sensors_found, OUT, LOW);
    setup(IOManager::led_no_frames_detected, OUT, LOW);

    setup(IOManager::button_reconfigure_network, IN);
    setup(IOManager::button_reload_service, IN);
    setup(IOManager::button_change_to_ros, IN);

    add_event_detect(IOManager::button_reconfigure_network, GPIO::Edge::RISING, IOManager::configureNetwork, 10);
    add_event_detect(IOManager::button_reload_service, GPIO::Edge::RISING, IOManager::reloadService, 10);
    add_event_detect(IOManager::button_change_to_ros, GPIO::Edge::RISING, IOManager::changeToRos, 10);
}

void IOManager::configureNetwork()
{
    cout << "configuring network" << endl;
    system("pkill avahi");
    system("avahi-autoipd -D --force-bind --no-chroot eth0");
}

void IOManager::reloadService()
{
    cout << "restarting service" << endl;
    system("systemctl reload-or-restart startup-boot.service");
}

void IOManager::changeToRos()
{
    cout << "changing to ROS" << endl;
    system("systemctl start change-to-ros-process.service");
}

void IOManager::alertNoSensorsFound()
{
    output(IOManager::led_no_sensors_found, HIGH);
}

void IOManager::clearNoSensorsFound()
{
    output(IOManager::led_no_sensors_found, LOW);
}

void IOManager::alertNoFrameDetected()
{
    output(IOManager::led_no_frames_detected, HIGH);
}

void IOManager::clearNoFrameDetected()
{
    output(IOManager::led_no_frames_detected, LOW);
}