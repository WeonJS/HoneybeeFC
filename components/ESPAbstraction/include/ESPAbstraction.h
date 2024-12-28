
#include "driver/uart.h"

namespace ESPAbstraction {
    void uart_install_connection(uart_port_t port, int rx_pin, int tx_pin, int tx_buf_size, int rx_buf_size, int baud_rate);
    int read_bytes(uart_port_t port, uint8_t* buf, int len);
}