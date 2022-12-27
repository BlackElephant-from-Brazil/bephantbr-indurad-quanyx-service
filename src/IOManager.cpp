#include <iostream>
#include "IOManager.h"
#include <JetsonGPIO.h>

using namespace std;
using namespace GPIO;

void IOManager::config()
{
    cout << "STARTING GPIO" << endl;
    setmode(BCM);

    setup(led_no_sensors_found, OUT, LOW);
    setup(led_no_frames_detected, OUT, LOW);

    setup(button_reconfigure_network, IN);
    setup(button_reload_service, IN);

    add_event_detect(button_reconfigure_network, GPIO::Edge::RISING, configureNetwork, 10);
    add_event_detect(button_reload_service, GPIO::Edge::RISING, reloadService, 10);
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

void IOManager::alertNoSensorsFound()
{
    output(led_no_sensors_found, HIGH);
}

void IOManager::clearNoSensorsFound()
{
    output(led_no_sensors_found, LOW);
}

void IOManager::alertNoFrameDetected()
{
    output(led_no_frames_detected, HIGH);
}

void IOManager::clearNoFrameDetected()
{
    output(led_no_frames_detected, LOW);
}