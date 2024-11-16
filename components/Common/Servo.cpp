extern "C" {
    #include "driver/ledc.h"
    #include "esp_log.h"
}

#include "Servo.h"
#include "math.h"

Servo::Servo(int max_angle, ledc_channel_t channel) {
    // Set fields
    this->pin = -1;
    this->angle = 0;
    this->is_attached = false;
    this->max_angle = max_angle;
    this->channel = channel;

    // Configure the LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,                      // Use low-speed mode
        .duty_resolution = (ledc_timer_bit_t) DUTY_RESOLUTION,  // Set duty resolution. ESP32-S3 is capped to 14 bits.
        .timer_num = LEDC_TIMER_0,                              // Use timer 0
        .freq_hz = PWM_FREQUENCY,                               // Frequency at 50Hz for servos
        .clk_cfg = LEDC_AUTO_CLK,                               // Use auto clock
        .deconfigure = false,                                   // Do not deconfigure timer
    };
    ledc_timer_config(&timer_config);
}

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

// establish pwm line to the servo
bool Servo::attach(int pin) {
    this->pin = pin;
    this->is_attached = true;

    // Configure the LEDC channel with the new PWM pin
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);

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
    float pulse_width_normalized = normalize(pulse_width, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

    // calculate the duty cycle using normalized pulse width
    int duty = pulse_width_normalized * (float)(1 << DUTY_RESOLUTION);

    // set the new duty cycle and update the LEDC channel
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

    // return successful write
    return true;
}

// write the pulse width in microseconds to the servo
bool Servo::write_us(int us) {
    this->angle = us_to_angle(us);

    int pulse_width_normalized = normalize(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

    // calculate the duty cycle using normalized pulse width
    int duty = pulse_width_normalized * (1 << DUTY_RESOLUTION);

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

    // set the channel config to null
    ledc_channel_config(nullptr);
}