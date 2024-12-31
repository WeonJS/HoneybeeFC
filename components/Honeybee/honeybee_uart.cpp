#include "honeybee_uart.h"

namespace honeybee_uart {
    void uart_install_connection(uart_connection_config_t config) //uart_port_t port, int rx_pin, int tx_pin, int tx_buf_size, int rx_buf_size, int baud_rate
    {
        // Initialize the UART peripheral config
        uart_config_t uart_config = {
            .baud_rate = config.baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = static_cast<uint8_t>(256), // this is ignored since we disable control flow. setting to avoid warning.
            .source_clk = UART_SCLK_APB,
            .flags = 0,
        };

        // Install the UART driver
        ESP_ERROR_CHECK(uart_driver_install(config.port, config.rx_buf_size, config.tx_buf_size, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(config.port, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(config.port, config.tx_pin, config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

    int read_bytes(uart_connection_config_t config, uint8_t* buf, int len)
    {
        return uart_read_bytes(config.port, buf, len, pdMS_TO_TICKS(20));
    }
}