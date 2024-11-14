class Servo {
    public:
        Servo(int max_angle);
        void attach(int pin);
        void write(int angle);
        void write_microseconds(int angle);
        int read();
        int read_microseconds();
        bool attached();
        void detach();
        int calculate_pulse_width(int angle);
    private:
        int pin;
        int angle;
        int max_angle;
        bool is_attached;
        const static int MIN_PULSE_WIDTH_US = 500;
        const static int MAX_PULSE_WIDTH_US = 2500;
};