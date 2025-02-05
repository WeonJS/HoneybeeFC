idf_component_register(SRCS "honeybee_esp32s3.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_driver_rmt)
