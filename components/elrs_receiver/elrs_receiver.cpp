extern "C" {
    #include "esp_log.h"
}

#include "elrs_receiver.h"

#define elrs_receiver_TAG "elrs_receiver"

// constructor
elrs_receiver::elrs_receiver(uart_port_t port, int rx_pin, int tx_pin) : uart_peripheral(port, rx_pin, tx_pin, 1024, 1024, 420000) {
    // set the state to idle
    this->state = ELRS_IDLE;
}

// method to set the ELRS state
void elrs_receiver::set_state(elrs_receiver_state state) {
    this->state = state;
}

// method to get the rc channel data
crsf_rc_channel_data elrs_receiver::get_rc_channel_data() {
    return this->rc_channel_data;
}

// method to receive data from the buffer
void elrs_receiver::receive_from_buffer() {
    // buffer to store received bytes
    uint8_t buf[rx_buf_size];

    // number of bytes read from the buffer
    int byte_count = read_bytes(buf);

    // loop through the bytes received from the buffer
    for (int i = 0; i < byte_count; i++) {
        if (buf[i] == 0xC8) {
            // get length (from type to crc8 inclusive)
            uint8_t len = buf[i + 1];

            // get type of CRSF frame
            uint8_t type = buf[i + 2];

            // depending on the frame type, parse the payload
            switch (type) {
                case (int) crsf_frame_type::rc_channels_packed:
                    // create a struct to store the rc channel data
                    crsf_rc_channel_data rc_channel_data;

                    // parse payload into rc channel data
                    rc_channel_data.chan0 = (buf[i + 4] << 8) | buf[i + 3];

                    // ESP_LOGI(elrs_receiver_TAG, "Channel 0: %d", rc_channel_data.chan0);
                    // printf("Payload bytes:\n");
                    for (int j = 0; j < len; j++) {
                        printf("%02X ", buf[i + 3 + j]);
                    }
                    printf("\n");

                    break;
            }
        }
    }
}