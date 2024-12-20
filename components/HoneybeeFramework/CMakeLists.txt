idf_component_register(SRCS "HoneybeeFramework.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c esp_driver_uart driver ELRSReceiver ThrustActuator)
