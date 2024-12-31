#include "honeybee_utils.h"
#include "honeybee_uart.h"

namespace honeybee_crsf {
    struct crsf_rc_channel_data_t
    {
        uint16_t chan0 : 11;
        uint16_t chan1 : 11;
        uint16_t chan2 : 11;
        uint16_t chan3 : 11;
        uint16_t chan4 : 11;
        uint16_t chan5 : 11;
        uint16_t chan6 : 11;
        uint16_t chan7 : 11;
        uint16_t chan8 : 11;
        uint16_t chan9 : 11;
        uint16_t chan10 : 11;
        uint16_t chan11 : 11;
        uint16_t chan12 : 11;
        uint16_t chan13 : 11;
        uint16_t chan14 : 11;
        uint16_t chan15 : 11;
    };

    enum class crsf_frame_type_t : uint8_t
    {
        gps = 0x02,
        battery_sensor = 0x08,
        link_statistics = 0x14,
        rc_channels_packed = 0x16,
        attitude = 0x1E,
        flight_mode = 0x21,

        // Extended header frames, range: 0x28 to 0x96
        device_ping = 0x28,
        device_info = 0x29,
        parameter_settings_entry = 0x2B,
        parameter_read = 0x2C,
        parameter_write = 0x2D,
        command = 0x32
    };
    
    honeybee_utils::honeybee_err_t process_crsf_frame(uint8_t *data, int buf_size, int si, crsf_rc_channel_data_t &channels);
    honeybee_utils::honeybee_err_t update_rc_channels(honeybee_uart::uart_connection_config_t uart_cnctn, crsf_rc_channel_data_t &channels);
}