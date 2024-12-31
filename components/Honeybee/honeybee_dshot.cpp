#include "honeybee_dshot.h"
#include "esp_log.h"
#include "honeybee_math.h"
#include "freertos/FreeRTOS.h"

#define dshot_TAG "dshot"

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
            .level1 = 0,
            
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

    bool dshot_connection_t::init(gpio_num_t gpio, dshot_mode_t dshot_mode, bool is_bidirectional, int startup_delay_ms)
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

    void dshot_connection_t::send_throttle(uint16_t throttle_value)
    {
        throttle_value = (uint16_t) honeybee_math::clamp(throttle_value, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        dshot_esc_throttle_t throttle = {
            .throttle = throttle_value,
            .telemetry_req = false, // telemetry is not supported in this example
        };

        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));

    }

    // Honeybee_ErrType dshot_connection_t::remove()
    // {
    //     // Uninstall the RMT driver
    //     ESP_ERROR_CHECK(del_chan);

    //     // Return HONEYBEE_OK to indicate that the motor has been uninstalled successfully
    //     return HONEYBEE_OK;
    // }

    // Calculates a CRC value for a DShot digital control signal packet
    // uint8_t dshot_connection_t::calculateCRC(const dshot_packet_t &dshot_packet)
    // {
    //     uint8_t crc;

    //     // Combine the throttle value and telemetric request flag into a 16-bit packet value
    //     const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;

    //     // Calculate the CRC value using different bitwise operations depending on the DShot configuration
    //     if (dshot_config.is_bidirectional)
    //     {
    //         // Bidirectional configuration: perform a bitwise negation of the result of XORing the packet with its right-shifted values by 4 and 8 bits,
    //         // and then bitwise AND the result with 0x0F
    //         const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
    //         crc = (~intermediate_result) & 0x0F;
    //     }
    //     else
    //     {
    //         // Unidirectional configuration: XOR the packet with its right-shifted values by 4 and 8 bits,
    //         // and then bitwise AND the result with 0x0F
    //         crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    //     }

    //     // Return the calculated CRC value as a 16-bit unsigned integer
    //     return crc;
    // }

    uint16_t dshot_connection_t::serialize_packet(const dshot_packet_t &dshot_packet)
    {
        uint16_t parsedRmtPaket = DSHOT_NULL_PACKET;
        uint16_t crc = calculateCRC(dshot_packet);

        // Complete the packet
        parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
        parsedRmtPaket = (parsedRmtPaket << 4) | crc;

        return parsedRmtPaket;
    }
}