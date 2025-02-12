extern "C" {
    #include <cmath>
    #include "esp_err.h"
    #include "esp_log.h"
}

#include "honeybee_esp32s3.h"

namespace honeybee_i2c {
    int num_i2c_connections = 0;
    hb_i2c_cnctn_t i2c_connections[MAX_I2C_CONNECTIONS];

    // Initialize the master bus and add it to our list of i2c connections.
    honeybee::hb_err_t hb_i2c_init_master(hb_i2c_master_cnctn_t *master)
    {
        // Create ESP32 config for the master bus.
        i2c_master_bus_config_t master_config = {
            .i2c_port = (i2c_port_num_t)num_i2c_connections,
            .sda_io_num = (gpio_num_t)master->sda_pin,
            .scl_io_num = (gpio_num_t)master->scl_pin,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
        };

        // Install the master bus.
        ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &master->master_handle));

        // Add this master to the list of i2c connections.
        i2c_connections[num_i2c_connections].type = I2C_MASTER;
        i2c_connections[num_i2c_connections].master = *master;
        num_i2c_connections++;

        return honeybee::HONEYBEE_OK;
    }

    // Write data to a device on our master bus.
    void hb_i2c_master_write(hb_i2c_bus_dev_t *device, unsigned char *data, long unsigned int data_size, int wait_ms)
    {
        esp_err_t ret = i2c_master_transmit(device->device, data, data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    void hb_i2c_master_read(hb_i2c_bus_dev_t *device, unsigned char *data, long unsigned int data_size, int wait_ms)
    {
        esp_err_t ret = i2c_master_receive(device->device, data, data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    // Write data and immediately read back the response.
    void hb_i2c_master_write_read(hb_i2c_bus_dev_t *device, unsigned char *write_data, long unsigned int write_data_size, unsigned char *read_data, long unsigned int read_data_size, int wait_ms)
    {
        esp_err_t ret = i2c_master_transmit_receive(device->device, write_data, write_data_size, read_data, read_data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    // void hb_i2c_master_read_dev_reg(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms)
    // {
        
    // }

    bool hb_i2c_check_dev_exists(hb_i2c_master_cnctn_t *master, unsigned short device_address, int wait_ms)
    {
        esp_err_t ret = i2c_master_probe(master->master_handle, device_address, wait_ms);
        ESP_ERROR_CHECK(ret);
        return ret == ESP_OK ? true : false;
    }


    honeybee::hb_err_t hb_i2c_init_dev(hb_i2c_master_cnctn_t *master, hb_i2c_bus_dev_t *device, long unsigned int wait_us)
    {
        i2c_device_config_t i2c_device_config = {
            .dev_addr_length = device->addr_len == I2C_7_BIT_ADDR ? I2C_ADDR_BIT_LEN_7 : I2C_ADDR_BIT_LEN_10,
            .device_address = device->addr,
            .scl_speed_hz = device->scl_speed_hz,
            .scl_wait_us = wait_us,
            .flags = {
                .disable_ack_check = false
            }
        };
        
        ESP_ERROR_CHECK(i2c_master_bus_add_device(master->master_handle, &i2c_device_config, &device->device));

        return honeybee::HONEYBEE_OK;
    }
}

namespace honeybee_uart {
    int num_uart_connections = 0;
    honeybee_uart::hb_uart_cnctn_t uart_connections[MAX_UART_CONNECTIONS];

    void uart_install_cnctn(hb_uart_cnctn_t cnctn) //uart_port_t port, int rx_pin, int tx_pin, int tx_buf_size, int rx_buf_size, int baud_rate
    {
        // Initialize the UART peripheral config
        uart_config_t uart_config = {
            .baud_rate = cnctn.baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = static_cast<uint8_t>(256), // this is ignored since we disable control flow. setting to avoid warning.
            .source_clk = UART_SCLK_APB,
            .flags = 0,
        };

        // Install the UART driver
        ESP_ERROR_CHECK(uart_driver_install(cnctn.port, cnctn.rx_buf_size, cnctn.tx_buf_size, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(cnctn.port, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(cnctn.port, cnctn.tx_pin, cnctn.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    int read_bytes(hb_uart_cnctn_t config, uint8_t* buf, int len)
    {
        return uart_read_bytes(config.port, buf, len, 0);
    }
}

namespace honeybee_math {
    
    void hb_vector2_t::set_x(double x)
    {
        this->x = x;
    }

    void hb_vector2_t::set_y(double y)
    {
        this->y = y;
    }

    double hb_vector2_t::get_x()
    {
        return this->x;
    }

    double hb_vector2_t::get_y()
    {
        return this->y;
    }

    hb_vector2_t operator+(hb_vector2_t a, hb_vector2_t b)
    {
        hb_vector2_t ret;
        ret.set_x(a.get_x() + b.get_x());
        ret.set_y(a.get_y() + b.get_y());
        return ret;
    }

    hb_vector2_t operator-(hb_vector2_t a, hb_vector2_t b)
    {
        hb_vector2_t ret;
        ret.set_x(a.get_x() - b.get_x());
        ret.set_y(a.get_y() - b.get_y());
        return ret;
    }

    hb_vector2_t hb_vector2_t::operator*(double b)
    {
        hb_vector2_t ret;
        ret.set_x(get_x() * b);
        ret.set_y(get_y() * b);
        return ret;
    }

    hb_vector2_t hb_vector2_t::operator/(double b)
    {
        hb_vector2_t ret;
        ret.set_x(get_x() / b);
        ret.set_y(get_y() / b);
        return ret;
    }

    // todo: map function for floats
    

    void hb_vector2_t::set(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    double hb_vector2_t::get_mag()
    {
        return sqrt(this->x * this->x + this->y * this->y);
    }


    double hb_vector2_t::dot(hb_vector2_t other)
    {
        return x * other.get_x() + y * other.get_y();
    }

    hb_vector2_t hb_vector2_t::clamp_mag(double max)
    {
        double magnitude = get_mag();
        if (magnitude > max) {
            hb_vector2_t ret;
            ret.set_x(this->x / magnitude * max);
            ret.set_y(this->y / magnitude * max);
            return ret;
        }
        return *this;
    }

    hb_vector2_t hb_vector2_t::normal()
    {
        double magnitude = sqrt(this->x * this->x + this->y * this->y);
        hb_vector2_t ret;
        ret.set_x(this->x / magnitude);
        ret.set_y(this->y / magnitude);
        return ret;
    }
}

namespace honeybee_dshot {
    void make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
    {
        frame->throttle = throttle;
        frame->telemetry = telemetry;
        uint16_t val = frame->val;
        uint8_t crc = ((val ^ (val >> 4) ^ (val >> 8)) & 0xF0) >> 4;;
        frame->crc = crc;
        val = frame->val;
        // change the endian
        frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
    }

    size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
    {
        rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
        rmt_encoder_handle_t bytes_encoder = dshot_encoder->bytes_encoder;
        rmt_encoder_handle_t copy_encoder = dshot_encoder->copy_encoder;
        rmt_encode_state_t session_state = RMT_ENCODING_RESET;
        rmt_encode_state_t state = RMT_ENCODING_RESET;
        size_t encoded_symbols = 0;

        // convert user data into dshot frame
        dshot_esc_throttle_t *throttle = (dshot_esc_throttle_t *)primary_data;
        dshot_esc_frame_t frame = {};
        make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);

        switch (dshot_encoder->state) {
            case 0: // send the dshot frame
                encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &frame, sizeof(frame), &session_state);
                if (session_state & RMT_ENCODING_COMPLETE) {
                    dshot_encoder->state = 1; // switch to next state when current encoding session finished
                }
                if (session_state & RMT_ENCODING_MEM_FULL) {
                    state = (rmt_encode_state_t)((int)state |(int) RMT_ENCODING_MEM_FULL);
                    goto out; // yield if there's no free space for encoding artifacts
                }
            // fall-through
            case 1:
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &dshot_encoder->dshot_delay_symbol,
                                                        sizeof(rmt_symbol_word_t), &session_state);
                if (session_state & RMT_ENCODING_COMPLETE) {
                    state = (rmt_encode_state_t)((int)state|(int)RMT_ENCODING_COMPLETE);
                    dshot_encoder->state = RMT_ENCODING_RESET; // switch to next state when current encoding session finished
                }
                if (session_state & RMT_ENCODING_MEM_FULL) {
                    state = (rmt_encode_state_t)((int)state | (int)RMT_ENCODING_MEM_FULL);
                    goto out; // yield if there's no free space for encoding artifacts
                }
        }
    out:
        *ret_state = state;
        return encoded_symbols;
    }

    esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder)
    {
        rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
        rmt_del_encoder(dshot_encoder->bytes_encoder);
        rmt_del_encoder(dshot_encoder->copy_encoder);
        free(dshot_encoder);
        return ESP_OK;
    }

    esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
    {
        rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
        rmt_encoder_reset(dshot_encoder->bytes_encoder);
        rmt_encoder_reset(dshot_encoder->copy_encoder);
        dshot_encoder->state = RMT_ENCODING_RESET;
        return ESP_OK;
    }

    esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
    {
        if (!config || !ret_encoder) {
            ESP_LOGE(dshot_TAG, "invalid argument");
            return ESP_ERR_INVALID_ARG;
        }

        rmt_dshot_esc_encoder_t *dshot_encoder = (rmt_dshot_esc_encoder_t *)rmt_alloc_encoder_mem(sizeof(rmt_dshot_esc_encoder_t));
        if (!dshot_encoder) {
            ESP_LOGE(dshot_TAG, "no mem for musical score encoder");
            return ESP_ERR_NO_MEM;
        }

        dshot_encoder->base.encode = rmt_encode_dshot_esc;
        dshot_encoder->base.del = rmt_del_dshot_encoder;
        dshot_encoder->base.reset = rmt_dshot_encoder_reset;

        uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
        rmt_symbol_word_t dshot_delay_symbol = {
            .duration0 = (uint16_t)(delay_ticks / 2),
            .level0 = 0,
            .duration1 = (uint16_t)(delay_ticks / 2),
            .level1 = 0
        };
        dshot_encoder->dshot_delay_symbol = dshot_delay_symbol;

        float period_ticks = (float)config->resolution / config->baud_rate;
        unsigned int t1h_ticks = (unsigned int)(period_ticks * 0.7485);
        unsigned int t1l_ticks = (unsigned int)(period_ticks - t1h_ticks);
        unsigned int t0h_ticks = (unsigned int)(period_ticks * 0.37425);
        unsigned int t0l_ticks = (unsigned int)(period_ticks - t0h_ticks);

        rmt_bytes_encoder_config_t bytes_encoder_config = {};
        bytes_encoder_config.bit0.duration0 = t0h_ticks;
        bytes_encoder_config.bit0.duration1 = t0l_ticks;
        bytes_encoder_config.bit0.level0 = 1;
        bytes_encoder_config.bit0.level1 = 0;

        bytes_encoder_config.bit1.duration0 = t1h_ticks;
        bytes_encoder_config.bit1.duration1 = t1l_ticks;
        bytes_encoder_config.bit1.level0 = 1;
        bytes_encoder_config.bit1.level1 = 0;
        bytes_encoder_config.flags.msb_first = 1;

        esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &dshot_encoder->bytes_encoder);
        if (ret != ESP_OK) {
            ESP_LOGE(dshot_TAG, "create bytes encoder failed");
            free(dshot_encoder);
            return ret;
        }

        rmt_copy_encoder_config_t copy_encoder_config = {};
        ret = rmt_new_copy_encoder(&copy_encoder_config, &dshot_encoder->copy_encoder);
        if (ret != ESP_OK) {
            ESP_LOGE(dshot_TAG, "create copy encoder failed");
            rmt_del_encoder(dshot_encoder->bytes_encoder);
            free(dshot_encoder);
            return ret;
        }
        
        *ret_encoder = &dshot_encoder->base;
        return ESP_OK;
    }

    bool dshot_cnctn_t::init(gpio_num_t gpio, dshot_mode_t dshot_mode, bool is_bidirectional, int startup_delay_ms)
    {
        ESP_LOGI(dshot_TAG, "Create RMT TX channel");
        esc_chan = NULL;
        tx_chan_config = {
            .gpio_num = gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
            .resolution_hz = DSHOT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
            .intr_priority = 0, // to get rid of warning
            .flags = {
                .invert_out = false,
                .with_dma = false,
                .io_loop_back = false,
                .io_od_mode = false,
            }
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

        ESP_LOGI(dshot_TAG, "Install Dshot ESC encoder");
        dshot_encoder = NULL;
        encoder_config = {
            .resolution = DSHOT_RESOLUTION_HZ,
            .baud_rate = 300000, // DSHOT300 protocol
            .post_delay_us = 50, // extra delay between each frame
        };
        ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

        ESP_LOGI(dshot_TAG, "Enable RMT TX channel");
        ESP_ERROR_CHECK(rmt_enable(esc_chan));

        tx_config = {
            .loop_count = -1, // infinite loop
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = false
            }
        };
        dshot_esc_throttle_t throttle = {
            .throttle = 0,
            .telemetry_req = false, // telemetry is not supported in this example
        };

        ESP_LOGI(dshot_TAG, "Start ESC by sending zero throttle for a while...");
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        vTaskDelay(pdMS_TO_TICKS(startup_delay_ms));

        return true;

    }

    void dshot_cnctn_t::send_throttle(int16_t throttle_value)
    {
        uint16_t final_throttle = honeybee_math::clamp<int16_t>(throttle_value, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        dshot_esc_throttle_t throttle = {
            .throttle = final_throttle,
            .telemetry_req = false, // telemetry is not supported in this example
        };

        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));

    }

    uint16_t dshot_cnctn_t::serialize_packet(const dshot_packet_t &dshot_packet)
    {
        uint16_t parsedRmtPaket = DSHOT_NULL_PACKET;
        uint16_t crc = calculateCRC(dshot_packet);

        // Complete the packet
        parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
        parsedRmtPaket = (parsedRmtPaket << 4) | crc;

        return parsedRmtPaket;
    }
}

namespace honeybee_crsf {
    honeybee::hb_err_t process_crsf_frame(uint8_t *data, int buf_size, int si, hb_crsf_rc_channel_data_t &channels)
    {

        // print the data
        // for (int i = 0; i < 16; i++) {
        //     ESP_LOGI("CRSF", "data[%d]: %02X", i, data[i]);
        // }

        // first byte is sync byte, should be 0xC8
        uint8_t sync = data[si];

        // if the sync byte is not correct, return false
        if (sync != 0xC8)
            return honeybee::HONEYBEE_ERR;

        // second byte is the length of the frame from [type] to [crc8] inclusive
        uint8_t frame_length = data[si + 1];

        // if there is not enough data to process the frame, return false
        if (si + frame_length >= buf_size)
            return honeybee::HONEYBEE_ERR;

        // third byte is the type of the frame
        hb_crsf_frame_type_t frame_type = (honeybee_crsf::hb_crsf_frame_type_t)data[si + 2];

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
            case honeybee_crsf::hb_crsf_frame_type_t::rc_channels_packed:
                channels.chan0 = (data[si + 4] << 8) | data[si + 3]; // roll
                channels.chan1 = (data[si + 5] << 5) | (data[si + 4] >> 3); // pitch
                channels.chan2 = (data[si + 7] << 10) | data[si + 6] << 2 | data[si + 5] >> 6; // throttle
                channels.chan3 = (data[si + 8] << 7) | (data[si + 7] >> 1); // yaw
                channels.chan4 = (data[si + 9] << 4) | (data[si + 8] >> 4); // button SA
                
                // for (int i = 0; i < frame_length + 2; i++)
                // {
                //     ESP_LOGI("crsf","%02X ", data[si + i]);
                // }
                // ESP_LOGI("crsf","\n");
                // printf("chan1: %d\n", channels.chan1);
                // printf("chan2: %d\n", channels.chan4);

                break;
            default:
                // ESP_LOGE("CRSF", "Unknown CRSF frame type: %d", data[si + 2]);
                return honeybee::HONEYBEE_ERR;
        }

        return honeybee::HONEYBEE_OK;
    }

    honeybee::hb_err_t update_rc_channels(honeybee_uart::hb_uart_cnctn_t uart_cnctn, honeybee_crsf::hb_crsf_rc_channel_data_t &channels)
    {
        uint8_t bytes_read = 64;
        uint8_t data[bytes_read];

        int byte_count = honeybee_uart::read_bytes(uart_cnctn, data, bytes_read);

        for (int i = 0; i < byte_count; i++)
        {
            // printf("%02X ", data[i]);
            // attempt to process the frame and update the channels
            honeybee::hb_err_t ret = process_crsf_frame(data, bytes_read, i, channels);

            // if the frame was processed successfully, skip to the next frame
            if (ret == honeybee::HONEYBEE_OK)
            {
                int frame_length = data[i + 1] + 2; // entire frame length including sync and length bytes
                i += frame_length;
            }
            else
            {
                // if the frame was not processed successfully, log an error
                // ESP_LOGE("crsf", "Error processing CRSF frame: %d", ret);
            }
        }
        // printf("\n");

        return honeybee::HONEYBEE_OK;
    }
}

namespace honeybee_utils {
    
}

namespace honeybee {
    honeybee::hb_drone_state_t drone_state = honeybee::DRONE_STATE_NONE;
    int hb_servo_t::servo_count = 0;

    void hb_pid_t::init(double kp, double ki, double kd, double min, double max)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->min = min;
        this->max = max;
        this->integral = 0;
        this->prev_err = 0;
    }

    double hb_pid_t::calculate(double setpoint, double process_variable, double dt)
    {
        double error = setpoint - process_variable;
        double p = kp * error;

        integral += error * dt;
        double i = ki * integral;

        double derivative = (error - prev_err) / dt;
        double d = kd * derivative;

        double output = honeybee_math::clamp<double>(p + i + d, min, max);

        prev_err = error;

        return output;
    }
    
    honeybee::hb_err_t hb_servo_t::set_can_rotate(bool can_rotate)
    {
        this->can_rotate = can_rotate;
        return honeybee::HONEYBEE_OK;
    }

    // given a pulse width in us, calculate the angle in degrees
    int hb_servo_t::us_to_angle(int us)
    {
        // Calculate the angle in degrees
        int angle = (us - MIN_PULSE_WIDTH_US) * max_angle / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);

        return angle;
    }

    // given an angle, calculate the pulse width in us to send to the servo
    int hb_servo_t::angle_to_us(int angle)
    {
        // Calculate the pulse width in us
        int pulse_width = (angle * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) / max_angle) + MIN_PULSE_WIDTH_US;

        return pulse_width;
    }

    // get max angle
    int hb_servo_t::get_max_angle()
    {
        return this->max_angle;
    }

    void hb_servo_t::init(int _max_angle)
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
    bool hb_servo_t::attach(int _pin)
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
    bool hb_servo_t::write(int angle)
    {
        if (!this->is_attached || !this->can_rotate)
            return false;

        // clamp the angle to be written to within the max angle
        angle = honeybee_math::clamp<int>(angle, 0, max_angle);

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
    bool hb_servo_t::write_us(int us)
    {
        this->angle = us_to_angle(us);

        float pulse_width_normalized = honeybee_math::map(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, 0, 1);

        // calculate the duty cycle using normalized pulse width
        int duty = pulse_width_normalized * (1 << (int)resolution);

        // set the new duty cycle and update the LEDC channel
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

        // return successful write
        return true;
    }

    // returns the angle in degrees of the servo in its current state
    int hb_servo_t::read()
    {
        return this->angle;
    }

    // returns the angle in microseconds with regards to the pwm width for the current angle
    int hb_servo_t::read_us()
    {
        return angle_to_us(this->angle);
    }

    // returns whether the servo is attached to a pin
    bool hb_servo_t::attached()
    {
        return this->is_attached;
    }

    void hb_servo_t::detach()
    {
        this->is_attached = false;
        this->pin = -1;

        // stop the LEDC channel
        ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
    }
}