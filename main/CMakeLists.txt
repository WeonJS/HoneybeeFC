idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES ESPAbstraction esp_driver_i2c driver esp_driver_rmt DShotRMT)