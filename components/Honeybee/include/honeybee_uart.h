#pragma once

extern "C" {
    #include "driver/uart.h"
}

namespace honeybee_uart {
    struct hb_uart_config_t
    {
        int baud_rate;
        uart_port_t port;
        unsigned int rx_pin;
        unsigned int tx_pin;
        unsigned int rx_buf_size;
        unsigned int tx_buf_size;
    };

    void uart_install_connection(hb_uart_config_t config);

    int read_bytes(hb_uart_config_t config, uint8_t* buf, int len);
}