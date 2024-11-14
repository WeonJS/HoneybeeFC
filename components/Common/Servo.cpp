#include "Servo.h"
#include "driver/ledc.h"
#include "esp_err.h"

Servo::Servo(int max_angle) {
    // Set fields
    this->pin = -1;
    this->angle = 0;
    this->is_attached = false;
    this->max_angle = max_angle;

    // Configure the LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,     // Use low-speed mode
        .duty_resolution = LEDC_TIMER_16_BIT,  // Set duty resolution
        .timer_num = LEDC_TIMER_0,             // Use timer 0
        .freq_hz = 50,                         // Frequency at 50Hz for servos
    };
    ledc_timer_config(&timer_config);
}

// given an angle, calculate the pulse width in microseconds to send to the servo
int Servo::calculate_pulse_width(int angle) {
    // Calculate the pulse width in microseconds
    // Calculate the pulse width
    int pulse_width = MIN_PULSE_WIDTH_US + (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) * angle / max_angle;

    return pulse_width;
}

// establish pwm line to the servo
void Servo::attach(int pin) {
    this->pin = pin;
    this->is_attached = true;

    // Configure the LEDC channel with the new PWM pin
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
}

void Servo::write(int angle) {
    this->angle = angle;

    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int pulse_width = calculate_pulse_width(angle);
    int duty = pulse_width * (1 << 16) / 20000;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void Servo::write_microseconds(int angle) {
    this->angle = angle;
}

int Servo::read() {
    return this->angle;
}

int Servo::read_microseconds() {
    return this->angle;
}

bool Servo::attached() {
    return this->is_attached;
}

void Servo::detach() {
    this->is_attached = false;
    this->pin = -1;
}