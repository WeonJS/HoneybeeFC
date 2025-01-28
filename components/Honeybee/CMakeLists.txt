idf_component_register(SRCS "honeybee_pid.cpp" "honeybee_math.cpp" "honeybee_dshot.cpp" "honeybee_crsf.cpp" "honeybee_servo.cpp" "honeybee_utils.cpp" "honeybee_uart.cpp" "honeybee_i2c.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_uart esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_driver_rmt)
