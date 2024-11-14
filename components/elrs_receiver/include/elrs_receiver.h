#pragma once

#include "uart_peripheral.h"

struct crsf_rc_channel_data {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes
    unsigned chan0 : 11;
    unsigned chan1 : 11;
    unsigned chan2 : 11;
    unsigned chan3 : 11;
    unsigned chan4 : 11;
    unsigned chan5 : 11;
    unsigned chan6 : 11;
    unsigned chan7 : 11;
    unsigned chan8 : 11;
    unsigned chan9 : 11;
    unsigned chan10 : 11;
    unsigned chan11 : 11;
    unsigned chan12 : 11;
    unsigned chan13 : 11;
    unsigned chan14 : 11;
    unsigned chan15 : 11;
};

enum class crsf_frame_type : uint8_t {
    gps = 0x02,
    battery_sensor = 0x08,
    link_statistics = 0x14,
    rc_channels_packed = 0x16,
    attitude = 0x1E,
    flight_mode = 0x21,

    // Extended Header Frames, range: 0x28 to 0x96
    device_ping = 0x28,
    device_info = 0x29,
    parameter_settings_entry = 0x2B,
    parameter_read = 0x2C,
    parameter_write = 0x2D,
    command = 0x32
};

enum elrs_receiver_state {
    ELRS_IDLE,
    ELRS_ACTIVE
};

class elrs_receiver : public uart_peripheral {
    public:
        elrs_receiver(uart_port_t port, int rx_pin, int tx_pin);
        void set_state(elrs_receiver_state state);
        void receive_from_buffer();
        crsf_rc_channel_data get_rc_channel_data();
    private:
        elrs_receiver_state state;
        crsf_rc_channel_data rc_channel_data;
};