extern "C" {
    #include "driver/ledc.h"
}

class Servo {
    public:
        static const int DUTY_RESOLUTION = 14;
        static const int PWM_FREQUENCY = 50;
        Servo(int max_angle, ledc_channel_t channel);
        bool attach(int pin);
        bool write(int angle);
        bool write_us(int angle);
        int read();
        int read_us();
        bool attached();
        void detach();
        int angle_to_us(int angle);
        int us_to_angle(int us);
        int get_max_angle();
    private:
        int pin;
        int angle;
        int max_angle;
        bool is_attached;
        ledc_channel_t channel;
        const static int MIN_PULSE_WIDTH_US = 500;
        const static int MAX_PULSE_WIDTH_US = 2500;
};