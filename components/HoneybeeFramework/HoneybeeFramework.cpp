#include "HoneybeeFramework.h"
#include <esp_log.h>

namespace HoneybeeFramework {
    const int MAX_SUBSYSTEM_COUNT = 10;
    int subsystem_count = 0;

    Honeybee_ErrType process_crsf_frame(uint8_t* data, int buf_size, int si, CRSF_RC_ChannelData &channels) {

        // print the data
        // for (int i = 0; i < 16; i++) {
        //     ESP_LOGI("CRSF", "data[%d]: %02X", i, data[i]);
        // }

        // print relevant data
        // ESP_LOGI("CRSF", "si: %d", si);
        // ESP_LOGI("CRSF", "buf_size: %d", buf_size);
        // ESP_LOGI("CRSF", "data[si]: %02X", data[si]);

        // if there is not enough data to process the frame, return false
        if (si + 16 >= buf_size) return HONEYBEE_ERR;

        // first byte is sync byte, should be 0xC8
        uint8_t sync = data[si];
        
        // if the sync byte is not correct, return false
        if (sync != 0xC8) return HONEYBEE_ERR;

        // second byte is the length of the frame from [type] to [crc8] inclusive
        uint8_t frame_length = data[si + 1];
        
        // third byte is the type of the frame
        CRSF_FrameType frame_type = (CRSF_FrameType) data[si + 2];

        // calculate the crc8 of the frame
        // uint8_t crc8 = data[si + frame_length + 1];
        // uint8_t calculated_crc8 = calculate_crc8(data, si, frame_length + 2);

        // ESP_LOGI("CRSF", "crc8: %02X", crc8);
        // ESP_LOGI("CRSF", "calculated_crc8: %02X", calculated_crc8);

        // // reject the frame if the crc8 is incorrect
        // if (crc8 != calculated_crc8) return HONEYBEE_INVALID_CRC;

        // process the frame based on the type
        switch (frame_type) 
        {
            case CRSF_FrameType::rc_channels_packed:
                channels.chan0 = (data[si + 4] << 8) | data[si + 3];
                
                break;
            default:
                return HONEYBEE_ERR;
        }

        return HONEYBEE_OK;
    }

    uint8_t calculate_crc8(uint8_t* data, int si, int len) {
        uint8_t crc = 0x00;
        for (int i = si; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    void Drone::init() {

    }

    void Drone::update() 
    {

    }

    void Drone::add_actuator(ThrustActuator actuator) 
    {
        this->actuator = actuator;
    }

    void Drone::add_elrs(ELRSReceiver receiver) 
    {
        this->receiver = receiver;
    }

    DroneState Drone::get_state() 
    {
        return state;
    }

    void Drone::set_state(DroneState s) 
    {
        state = s;
    }

    int Servo::servo_count = 0;
    // given a pulse width in us, calculate the angle in degrees
    int Servo::us_to_angle(int us) {
        // Calculate the angle in degrees
        int angle = (us - MIN_PULSE_WIDTH_US) * max_angle / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);

        return angle;
    }

    // given an angle, calculate the pulse width in us to send to the servo
    int Servo::angle_to_us(int angle) {
        // Calculate the pulse width in us
        int pulse_width = (angle * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) / max_angle) + MIN_PULSE_WIDTH_US;

        return pulse_width;
    }

    // get max angle
    int Servo::get_max_angle() {
        return this->max_angle;
    }

    void Servo::init(int _max_angle) {
        max_angle = _max_angle;
        channel = (ledc_channel_t) servo_count++;

        // Configure the LEDC timer
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE,                      // Use low-speed mode
            .duty_resolution = resolution,                          // Set duty resolution. ESP32-S3 is capped to 14 bits.
            .timer_num = LEDC_TIMER_0,                              // Use timer 0
            .freq_hz = frequency,                                   // Frequency at 50Hz for servos
            .clk_cfg = LEDC_AUTO_CLK,                               // Use auto clock
            .deconfigure = false,                                   // Do not deconfigure timer
        };
        ledc_timer_config(&timer_config);
    }

    // establish pwm line to the servo
    bool Servo::attach(int _pin) {
        is_attached = true;

        // Configure the LEDC channel with the new PWM pin
        ledc_channel_config_t ledc_channel = {
            .gpio_num   = pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
            .flags      = {0} // doing this to avoid a warning
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        return true;
    }

    // write the angle in degrees to the servo
    bool Servo::write(int angle) {
        if (!this->is_attached) return false;

        // clamp the angle to be written to within the max angle
        angle = clamp(angle, 0, max_angle);

        // update class field
        this->angle = angle;

        // calculate the pulse width in us and normalize it
        int pulse_width = angle_to_us(angle);

        // period of the pwm signal in microseconds
        int period = 1.0 / frequency * 1000000;
        int duty = (float)pulse_width / (float)period * (1 << (int)resolution);

        // set the new duty cycle and update the LEDC channel
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

        // return successful write
        return true;
    }

    // write the pulse width in microseconds to the servo
    bool Servo::write_us(int us) {
        this->angle = us_to_angle(us);

        float pulse_width_normalized = normalize(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        // calculate the duty cycle using normalized pulse width
        int duty = pulse_width_normalized * (1 << (int)resolution);

        // set the new duty cycle and update the LEDC channel
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

        // return successful write
        return true;
    }

    // returns the angle in degrees of the servo in its current state
    int Servo::read() {
        return this->angle;
    }

    // returns the angle in microseconds with regards to the pwm width for the current angle
    int Servo::read_us() {
        return angle_to_us(this->angle);
    }

    // returns whether the servo is attached to a pin
    bool Servo::attached() {
        return this->is_attached;
    }

    void Servo::detach() {
        this->is_attached = false;
        this->pin = -1;

        // stop the LEDC channel
        ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
    }

    float clamp(float value, float min, float max) 
    {
        if (value < min) value = min;
        if (value > max) value = max;

        return value;
    }

    float normalize(float value, float min, float max) 
    {
        return (value - min) / (max - min);
    }

    void Propeller::init() {
    }

    void Propeller::attach(int data_pin) {
        this->data_pin = data_pin;
    }

    void Propeller::set_thrust(int thrust) {
        this->thrust = thrust;
    }

    int Propeller::get_thrust() {
        return this->thrust;
    }

    void Propeller::send_data() {
        // send the data to the propeller
    }

}