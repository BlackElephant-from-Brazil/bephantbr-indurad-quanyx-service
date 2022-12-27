class IOManager : public 
{
    public:

        IOManager();

        void alertNoSensorsFound();
        void clearNoSensorsFound();
        void alertNoFrameDetected();
        void clearNoFrameDetected();
        void configureNetwork();
        void reloadService();


    private:
        int led_no_sensors_found = 18;
        int led_no_frames_detected = 17;
        int button_reconfigure_network = 4;
        int button_reload_service = 27;
}