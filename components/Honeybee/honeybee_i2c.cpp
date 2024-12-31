#include "honeybee_i2c.h"

namespace honeybee_i2c {
    i2c_master_bus_handle_t i2c_init_master(gpio_num_t sda, gpio_num_t scl)
    {
        i2c_master_bus_config_t esp_i2c_master_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .clk_source = i2c_clock_source_t::I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0, // i2c is synchronous, so no need for a queue
            .flags = {
                .enable_internal_pullup = false
            },
        };

        i2c_master_bus_handle_t esp_i2c_master;
        ESP_ERROR_CHECK(i2c_new_master_bus(&esp_i2c_master_config, &esp_i2c_master));

        return esp_i2c_master;
        
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