extern "C" {
    #include "driver/i2c_master.h"
    #include "driver/i2c_types.h"
    #include "driver/uart.h"
    #include "esp_log.h"
    #include "esp_err.h"
}

namespace honeybee_i2c {
    struct i2c_connection_config_t
    {
        i2c_port_t port;
        unsigned int sda_pin;
        unsigned int scl_pin;
    };

    i2c_master_bus_handle_t i2c_init_master_bus(gpio_num_t sda, gpio_num_t scl);

    void i2c_master_write(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms = -1);

    void i2c_master_read(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms = -1);

    void i2c_master_write_read(i2c_master_dev_handle_t device, uint8_t *write_data, size_t write_data_size, uint8_t *read_data, size_t read_data_size, uint32_t wait_ms = -1);

    void i2c_master_read_device_register(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms = -1);

    void i2c_master_write_device_register(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms = -1);

    bool i2c_check_device_exists(i2c_master_bus_handle_t master_bus, uint8_t device_address, uint32_t wait_ms = -1);

    i2c_master_dev_handle_t i2c_init_device(i2c_master_bus_handle_t master_bus, uint8_t device_address, i2c_addr_bit_len_t addr_len = I2C_ADDR_BIT_LEN_7, uint32_t speed_hz = 100000, uint32_t wait_us = 0);
    
}
