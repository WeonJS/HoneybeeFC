#include "honeybee_servo.h"
#include "honeybee_math.h"
#include "esp_log.h"

#define servo_TAG "hb_servo"

namespace honeybee_servo {

    int servo_count = 0;

    honeybee_utils::hb_err_t servo_t::set_can_rotate(bool can_rotate)
    {
        this->can_rotate = can_rotate;
        return honeybee_utils::HONEYBEE_OK;
    }

    // given a pulse width in us, calculate the angle in degrees
    int servo_t::us_to_angle(int us)
    {
        // Calculate the angle in degrees
        int angle = (us - MIN_PULSE_WIDTH_US) * max_angle / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);

        return angle;
    }

    // given an angle, calculate the pulse width in us to send to the servo
    int servo_t::angle_to_us(int angle)
    {
        // Calculate the pulse width in us
        int pulse_width = (angle * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) / max_angle) + MIN_PULSE_WIDTH_US;

        return pulse_width;
    }

    // get max angle
    int servo_t::get_max_angle()
    {
        return this->max_angle;
    }

    void servo_t::init(int _max_angle)
    {
        max_angle = _max_angle;
        channel = (ledc_channel_t) servo_count++;
        frequency = 50;
        resolution = LEDC_TIMER_14_BIT;

        // Configure the LEDC timer
        ledc_timer_config_t timer_config = {
            .speed_mode = LEDC_LOW_SPEED_MODE, // Use low-speed mode
            .duty_resolution = resolution,     // Set duty resolution. ESP32-S3 is capped to 14 bits.
            .timer_num = LEDC_TIMER_0,         // Use timer 0
            .freq_hz = frequency,              // Frequency at 50Hz for servos
            .clk_cfg = LEDC_AUTO_CLK,          // Use auto clock
            .deconfigure = false,              // Do not deconfigure timer
        };
        ledc_timer_config(&timer_config);
    }

    // establish pwm line to the servo
    bool servo_t::attach(int _pin)
    {
        pin = _pin;
        is_attached = true;

        // Configure the LEDC channel with the new PWM pin
        ledc_channel_config_t ledc_channel = {
            .gpio_num = _pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags = {0} // doing this to avoid a warning
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

        return true;
    };

    // write the angle in degrees to the servo
    bool servo_t::write(int angle)
    {
        if (!this->is_attached || !this->can_rotate)
            return false;

        // clamp the angle to be written to within the max angle
        angle = honeybee_math::clamp(angle, 0, max_angle);

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
    };

    // write the pulse width in microseconds to the servo
    bool servo_t::write_us(int us)
    {
        this->angle = us_to_angle(us);

        float pulse_width_normalized = honeybee_math::normalize(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        // calculate the duty cycle using normalized pulse width
        int duty = pulse_width_normalized * (1 << (int)resolution);

        // set the new duty cycle and update the LEDC channel
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

        // return successful write
        return true;
    }

    // returns the angle in degrees of the servo in its current state
    int servo_t::read()
    {
        return this->angle;
    }

    // returns the angle in microseconds with regards to the pwm width for the current angle
    int servo_t::read_us()
    {
        return angle_to_us(this->angle);
    }

    // returns whether the servo is attached to a pin
    bool servo_t::attached()
    {
        return this->is_attached;
    }

    void servo_t::detach()
    {
        this->is_attached = false;
        this->pin = -1;

        // stop the LEDC channel
        ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
    }
}
