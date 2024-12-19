
namespace ESPAbstraction {
    void uart_install_device(uart_port_t port, int rx_pin, int tx_pin, int tx_buf_size, int rx_buf_size, int baud_rate);
    void uart_read_bytes(uart_port_t port, uint8_t* buf, int len);
}