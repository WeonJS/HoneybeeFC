#include "honeybee_crsf.h"

namespace honeybee_crsf {
    honeybee_utils::honeybee_err_t process_crsf_frame(uint8_t *data, int buf_size, int si, crsf_rc_channel_data_t &channels)
    {

        // print the data
        // for (int i = 0; i < 16; i++) {
        //     ESP_LOGI("CRSF", "data[%d]: %02X", i, data[i]);
        // }

        // first byte is sync byte, should be 0xC8
        uint8_t sync = data[si];

        // if the sync byte is not correct, return false
        if (sync != 0xC8)
            return honeybee_utils::HONEYBEE_ERR;

        // second byte is the length of the frame from [type] to [crc8] inclusive
        uint8_t frame_length = data[si + 1];

        // if there is not enough data to process the frame, return false
        if (si + frame_length >= buf_size)
            return honeybee_utils::HONEYBEE_ERR;

        // third byte is the type of the frame
        crsf_frame_type_t frame_type = (honeybee_crsf::crsf_frame_type_t)data[si + 2];

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
            case honeybee_crsf::crsf_frame_type_t::rc_channels_packed:
                channels.chan0 = (data[si + 4] << 8) | data[si + 3]; // roll
                channels.chan1 = (data[si + 5] << 5) | (data[si + 4] >> 3); // pitch
                channels.chan2 = (data[si + 7] << 10) | data[si + 6] << 2 | data[si + 5] >> 6; // throttle
                channels.chan3 = (data[si + 8] << 7) | (data[si + 7] >> 1); // yaw
                channels.chan4 = (data[si + 9] << 4) | (data[si + 8] >> 4); // button SA
                
                for (int i = 0; i < frame_length + 2; i++)
                {
                    // printf("%02X ", data[si + i]);
                }
                // printf("\n");
                // printf("chan1: %d\n", channels.chan1);
                // printf("chan2: %d\n", channels.chan4);

                break;
            default:
                return honeybee_utils::HONEYBEE_ERR;
        }

        return honeybee_utils::HONEYBEE_OK;
    }

    honeybee_utils::honeybee_err_t update_rc_channels(honeybee_uart::uart_connection_config_t uart_config, honeybee_crsf::crsf_rc_channel_data_t &channels)
    {
        unsigned int rx_buf_size = uart_config.rx_buf_size;
        uint8_t data[rx_buf_size];

        int byte_count = honeybee_uart::read_bytes(uart_config, data, rx_buf_size);

        for (int i = 0; i < byte_count; i++)
        {
            // attempt to process the frame and update the channels
            honeybee_utils::honeybee_err_t ret = process_crsf_frame(data, rx_buf_size, i, channels);

            // if the frame was processed successfully, skip to the next frame
            if (ret == honeybee_utils::HONEYBEE_OK)
            {
                int frame_length = data[i + 1] + 2; // entire frame length including sync and length bytes
                i += frame_length;
            }
            else
            {
                // if the frame was not processed successfully, log an error
                // ESP_LOGE(ELRSReceiver_TAG, "Error processing CRSF frame: %d", ret);
            }
        }

        return honeybee_utils::HONEYBEE_OK;
    }
}