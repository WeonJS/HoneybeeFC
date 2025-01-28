#include "honeybee_i2c.h"

namespace honeybee_i2c {
    honeybee_utils::hb_err_t i2c_init_master(hb_i2c_master_t *config)
    {
        ESP_ERROR_CHECK(i2c_new_master_bus(&config->master_config, &config->master_device));

        return honeybee_utils::HONEYBEE_OK;
        
    }

    void i2c_master_write(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms)
    {
        esp_err_t ret = i2c_master_transmit(device, data, data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    void i2c_master_read(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms)
    {
        esp_err_t ret = i2c_master_receive(device, data, data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    void i2c_master_write_read(i2c_master_dev_handle_t device, uint8_t *write_data, size_t write_data_size, uint8_t *read_data, size_t read_data_size, uint32_t wait_ms)
    {
        esp_err_t ret = i2c_master_transmit_receive(device, write_data, write_data_size, read_data, read_data_size, wait_ms);
        ESP_ERROR_CHECK(ret);
    }

    void i2c_master_read_dev_reg(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms)
    {
        
    }

    bool i2c_check_dev_exists(i2c_master_bus_handle_t master_bus, uint8_t device_address, uint32_t wait_ms)
    {
        esp_err_t ret = i2c_master_probe(master_bus, device_address, wait_ms);
        ESP_ERROR_CHECK(ret);
        return ret == ESP_OK ? true : false;
    }


    i2c_master_dev_handle_t i2c_init_dev(i2c_master_bus_handle_t master_bus, uint8_t device_address, i2c_addr_bit_len_t addr_len, uint32_t speed_hz, uint32_t wait_us)
    {
        i2c_device_config_t i2c_device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = device_address,
            .scl_speed_hz = speed_hz,
            .scl_wait_us = wait_us,
            .flags = {
                .disable_ack_check = false
            }
        };

        i2c_master_dev_handle_t i2c_device;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus, &i2c_device_config, &i2c_device));

        return i2c_device;
    }
}