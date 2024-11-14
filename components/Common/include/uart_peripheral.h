#pragma once

extern "C" {
    #include "driver/uart.h"
}

#include "hardware_peripheral.h"


class uart_peripheral : public hardware_peripheral {
    public:
        uart_peripheral(uart_port_t port, int rx_pin, int tx_pin, int rx_buf_size, int tx_buf_size, int baud_rate);
        int read_bytes(uint8_t *buf);

    protected:
        int tx_pin;
        int rx_pin;
        int rx_buf_size;
        int tx_buf_size;
        int baud_rate;
        uart_port_t port;
};