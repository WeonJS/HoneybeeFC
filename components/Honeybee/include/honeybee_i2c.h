#ifdef ESP32S3 || ESP32C3
extern "C" {
    #include "driver/i2c_master.h"
    #include <driver/i2c_slave.h>
    #include "driver/i2c_types.h"
    #include "driver/uart.h"
    #include "esp_log.h"
    #include "esp_err.h"
}

#include "honeybee_utils.h"

#endif


namespace honeybee_i2c {

    struct hb_i2c_master_t
    {
        unsigned short address;
        unsigned int sda_pin;
        unsigned int scl_pin;

        #ifdef ESP32S3 || ESP32C3
        i2c_master_bus_config_t master_config;
        i2c_master_bus_handle_t master;
        i2c_master_dev_handle_t master_device;
        #endif
    };

    struct hb_i2c_slave_t {
        unsigned short address;
        unsigned int sda_pin;
        unsigned int scl_pin;

        #ifdef ESP32S3 || ESP32C3
        i2c_slave_config_t slave_config;
        i2c_slave_dev_handle_t slave_device;
        #endif
    };

    struct hb_i2c_bus_dev_t {

    };

    honeybee_utils::hb_err_t i2c_init_master(hb_i2c_master_t *config);

    void i2c_master_write(hb_i2c_master_t *master, unsigned char *data, long unsigned int data_size, unsigned int wait_ms = -1);

    void i2c_master_read(hb_i2c_master_t *master, unsigned char *data, long unsigned int data_size, unsigned int wait_ms = -1);

    void i2c_master_write_read(hb_i2c_master_t *master, unsigned char *write_data, long unsigned int write_data_size, unsigned char *read_data, long unsigned int read_data_size, unsigned int wait_ms = -1);

    bool i2c_check_dev_exists(hb_i2c_master_t *master, unsigned char device_address, unsigned int wait_ms = -1);

    honeybee_utils::hb_err_t i2c_init_dev(hb_i2c_master_t *master, hb_i2c_bus_dev_t *device, unsigned short device_address, unsigned char addr_bit_len = 7, unsigned int speed_hz = 400000, unsigned int wait_us = 0);
    
}
