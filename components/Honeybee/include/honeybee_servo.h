#pragma once

#include "driver/ledc.h"
#include "honeybee_utils.h"

namespace honeybee_servo {
    extern int servo_count;

    class servo_t
    {
        public:
            ledc_timer_bit_t resolution;
            uint32_t frequency;
            void init(int max_angle);
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
            honeybee_utils::hb_err_t set_can_rotate(bool can_rotate);
        private:
            int pin = -1;
            int angle = 0;
            int max_angle = 0;
            bool is_attached = false;
            ledc_channel_t channel;
            const static int MIN_PULSE_WIDTH_US = 500;
            const static int MAX_PULSE_WIDTH_US = 2500;
            bool can_rotate = true;
    };
}