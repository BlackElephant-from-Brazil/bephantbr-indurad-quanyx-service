#ifndef IOMANAGER_H
#define IOMANAGER_H

class IOManager {

    public:
        static void config();
        static void alertNoSensorsFound();
        static void clearNoSensorsFound();
        static void alertNoFrameDetected();
        static void clearNoFrameDetected();
        static void configureNetwork();
        static void reloadService();
        static void changeToRos();

    private:
        static const int led_no_sensors_found = 18;
        static const int led_no_frames_detected = 17;
        static const int button_reconfigure_network = 4;
        static const int button_reload_service = 27;
        static const int button_change_to_ros = 22;

};

#endif