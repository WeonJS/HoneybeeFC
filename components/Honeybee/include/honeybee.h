#ifdef ESP32S3

#define MAX_I2C_CONNECTIONS 2
#define MAX_UART_CONNECTIONS 3

#endif

#ifdef ESP32C3

#define MAX_I2C_CONNECTIONS 1
#define MAX_UART_CONNECTIONS 2

#endif

#include "honeybee_i2c.h"
#include "honeybee_uart.h"
#include "honeybee_utils.h"

namespace honeybee {
    honeybee_utils::hb_drone_state_t drone_state = honeybee_utils::hb_drone_state_t::NONE;
    honeybee_i2c::hb_i2c_config_t i2c_connections[MAX_I2C_CONNECTIONS];
    honeybee_uart::hb_uart_config_t uart_connections[MAX_UART_CONNECTIONS];
}