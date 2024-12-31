extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "driver/i2c_master.h"
    #include "driver/i2c_types.h"
    #include <cmath>
}

#include "honeybee_uart.h"
#include "honeybee_i2c.h"
#include "honeybee_crsf.h"
#include "honeybee_math.h"
#include "honeybee_servo.h"
#include "honeybee_dshot.h"


// The structure of a CRSF frame is as follows:
//[sync] [len] [type] [payload] [crc8]
// sync: marks the beginning of a frame, always 0xC8
// len: the number of bytes from [type] to [crc8] inclusive.
// type: the type of the frame, e.g. 0x16 for RC channels
// payload: the data of the frame
// crc8: a checksum of the frame

#define main_TAG "main"

#define ICM20948_I2C_ADDR 0x68
#define ICM20948_I2C_PWR_MGMT_REG_ADDR 0x06

enum flight_mode_t {
    VTOL,
    FORWARD_FLIGHT,
};


honeybee_crsf::crsf_rc_channel_data_t channels;


#define CRSF_RC_CHANNEL_MIN 174
#define CRSF_RC_CHANNEL_MAX 1811

float normalize_crsf_channel(uint16_t channel_value)
{
    return honeybee_math::map(channel_value, CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, 0, 1);
}

float get_yaw() {
    return normalize_crsf_channel(channels.chan3);
}

float get_throttle() {
    return normalize_crsf_channel(channels.chan2);
}

float get_pitch() {
    return normalize_crsf_channel(channels.chan1);
}

float get_roll() {
    return normalize_crsf_channel(channels.chan0);
}

int get_button_SA() {
    return (int) honeybee_math::map(channels.chan4, 191, 1792, 0, 1); // todo: ugly
}



class ICM20948 {
    public:
        void init(gpio_num_t sda_pin, gpio_num_t scl_pin);
        void update_axes();
    private:
        uint16_t gyro_x, gyro_y, gyro_z;
        uint16_t mag_x, mag_y, mag_z;
        i2c_master_bus_handle_t master;
        i2c_master_dev_handle_t device;
        const uint8_t I2C_ADDRESS = 0x68;
        const uint8_t I2C_PWR_MGMT_REG_ADDR = 0x06;
        const uint8_t I2C_GYRO_XOUT_H_REG_ADDR = 0x33;
        const uint8_t I2C_GYRO_XOUT_L_REG_ADDR = 0x34;
        const uint8_t I2C_GYRO_YOUT_H_REG_ADDR = 0x35;
        const uint8_t I2C_GYRO_YOUT_L_REG_ADDR = 0x36;
        const uint8_t I2C_GYRO_ZOUT_H_REG_ADDR = 0x37;
        const uint8_t I2C_GYRO_ZOUT_L_REG_ADDR = 0x38;

        const uint8_t I2C_MAG_XOUT_H_REG_ADDR = 0x12;
        const uint8_t I2C_MAG_XOUT_L_REG_ADDR = 0x11;
        const uint8_t I2C_MAG_YOUT_H_REG_ADDR = 0x14;
        const uint8_t I2C_MAG_YOUT_L_REG_ADDR = 0x13;
        const uint8_t I2C_MAG_ZOUT_H_REG_ADDR = 0x16;
        const uint8_t I2C_MAG_ZOUT_L_REG_ADDR = 0x15;


        void power_on();
        void power_off();
};

void ICM20948::init(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    master = honeybee_i2c::i2c_init_master_bus(sda_pin, scl_pin);
    device = honeybee_i2c::i2c_init_device(master, 0x68);

    power_on();
}

void ICM20948::update_axes() {
    // gyro
    uint8_t gyro_x_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_XOUT_H_REG_ADDR, &gyro_x_h, 1);

    uint8_t gyro_x_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_XOUT_L_REG_ADDR, &gyro_x_l, 1);

    gyro_x = (gyro_x_h << 8) | gyro_x_l;

    uint8_t gyro_y_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_YOUT_H_REG_ADDR, &gyro_y_h, 1);

    uint8_t gyro_y_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_YOUT_L_REG_ADDR, &gyro_y_l, 1);

    gyro_y = (gyro_y_h << 8) | gyro_y_l;

    uint8_t gyro_z_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_ZOUT_H_REG_ADDR, &gyro_z_h, 1);

    uint8_t gyro_z_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_GYRO_ZOUT_L_REG_ADDR, &gyro_z_l, 1);

    gyro_z = (gyro_z_h << 8) | gyro_z_l;

    
    // mag
    uint8_t mag_x_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_XOUT_H_REG_ADDR, &mag_x_h, 1);

    uint8_t mag_x_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_XOUT_L_REG_ADDR, &mag_x_l, 1);

    mag_x = (mag_x_h << 8) | mag_x_l;

    uint8_t mag_y_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_YOUT_H_REG_ADDR, &mag_y_h, 1);

    uint8_t mag_y_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_YOUT_L_REG_ADDR, &mag_y_l, 1);

    mag_y = (mag_y_h << 8) | mag_y_l;

    uint8_t mag_z_h;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_ZOUT_H_REG_ADDR, &mag_z_h, 1);

    uint8_t mag_z_l;
    honeybee_i2c::i2c_master_read_device_register(device, I2C_MAG_ZOUT_L_REG_ADDR, &mag_z_l, 1);

    mag_z = (mag_z_h << 8) | mag_z_l;

    // uint8_t data;
    // honeybee_i2c::i2c_master_read_device_register(device, 0x06, &data, 1);
    // ESP_LOGI(main_TAG, "Data: %d", data);

    ESP_LOGI(main_TAG, "Gyro X: %d\tGyro Y: %d\tGyro Z: %d\tMag X: %d\tMag Y: %d\tMag Z: %d", gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);
}

void ICM20948::power_on()
{
    uint8_t data = 0x01;
    honeybee_i2c::i2c_master_write_device_register(device, ICM20948_I2C_PWR_MGMT_REG_ADDR, &data, 1);
}

void ICM20948::power_off()
{
}

honeybee_math::Vector2 right_joystick_input;
honeybee_math::Vector2 left_joystick_input;

int max_roll_offset_angle = 45;

extern "C" void app_main(void)
{
    // ESP_LOGI(main_TAG, "Starting main application");

    // Create an ELRS receiver
    // honeybee_uart::uart_connection_config_t elrs_uart_connection;
    // elrs_uart_connection.port = UART_NUM_1; // uart0 is used by the console
    // elrs_uart_connection.rx_pin = 38;
    // elrs_uart_connection.tx_pin = 37;
    // elrs_uart_connection.rx_buf_size = 256;
    // elrs_uart_connection.tx_buf_size = 256;
    // honeybee_uart::uart_install_connection(elrs_uart_connection);

    // honeybee_servo::servo_t FL_servo;
    // FL_servo.init(180);
    // FL_servo.attach(8);

    // honeybee_servo::servo_t FR_servo;
    // FR_servo.init(180);
    // FR_servo.attach(18);

    // honeybee_servo::servo_t BL_servo;
    // BL_servo.init(180);
    // BL_servo.attach(17);

    // honeybee_servo::servo_t BR_servo;
    // BR_servo.init(180);
    // BR_servo.attach(7);

    // honeybee_servo::servo_t CamTiltServo;
    // CamTiltServo.init(180);
    // CamTiltServo.attach(16);

    // honeybee_servo::servo_t CamPanServo;
    // CamPanServo.init(180);
    // CamPanServo.attach(15);

    // honeybee_dshot::dshot_connection_t motor_BL; // bottom left, spin right
    // honeybee_dshot::dshot_connection_t motor_TL; // top left, spin left
    // honeybee_dshot::dshot_connection_t motor_BR; // bottom right, spin left
    // honeybee_dshot::dshot_connection_t motor_TR; // top right, spin right
    // motor_BL.init(GPIO_NUM_47, honeybee_dshot::DSHOT300, false);
    // motor_TL.init(GPIO_NUM_21, honeybee_dshot::DSHOT300, false);
    // motor_BR.init(GPIO_NUM_14, honeybee_dshot::DSHOT300, false);
    // motor_TR.init(GPIO_NUM_13, honeybee_dshot::DSHOT300, false);
    
    // ICM20948 icm;
    // icm.init(GPIO_NUM_6, GPIO_NUM_5);

    i2c_master_bus_config_t master_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_5,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags = {
            .enable_internal_pullup = false
        }
    };

    i2c_master_bus_handle_t master;
    ESP_ERROR_CHECK(i2c_new_master_bus(&master_config, &master));

    uint32_t wait = -1;
    i2c_device_config_t i2c_device_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x68,
            .scl_speed_hz = 100000,
            .scl_wait_us = wait,
        };

    i2c_master_dev_handle_t i2c_device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master, &i2c_device_config, &i2c_device));

    const uint8_t data = 0x06;
    i2c_master_transmit(i2c_device, &data, 1, -1);
    const uint8_t data2 = 0x01;
    i2c_master_transmit(i2c_device, &data2, 1, -1);

    // read power management register
    uint8_t data_read;
    i2c_master_transmit_receive(i2c_device, &data, 1, &data_read, 1, -1);
    ESP_LOGI(main_TAG, "Data: %d", data_read);


    while (true)
    {
        // Update the rc channels
        // honeybee_crsf::update_rc_channels(elrs_uart_connection, channels);
        // int pitch = get_pitch() * 180;
        // int roll = (get_roll() - 0.5f) * 2 * max_roll_offset_angle;
        // int dshot_throttle = honeybee_math::map(get_throttle(), 0, 1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        // right_joystick_input.set_x((get_roll() - 0.5f) * 2);
        // right_joystick_input.set_y((get_pitch() - 0.5f) * 2);
        // right_joystick_input = right_joystick_input.normal();

        // left_joystick_input.set_x((get_yaw() - 0.5f) * 2);
        // left_joystick_input.set_y((get_throttle() - 0.5f) * 2);
        // left_joystick_input = left_joystick_input.normal();

        // icm.update_axes();

        // // ESP_LOGI(main_TAG, "Pitch: %d\tRoll: %d", channels.chan1, channels.chan0);

        // // motor1.send_throttle(dshot_throttle);
        // // motor2.send_throttle(dshot_throttle);
        // // motor3.send_throttle(dshot_throttle);
        // // motor4.send_throttle(dshot_throttle);

        // flight_mode_t mode = (flight_mode_t) get_button_SA();
        // switch (mode) {
        //     case VTOL:
        //         FL_servo.write(0);
        //         FR_servo.write(180);
        //         BL_servo.write(180);
        //         BR_servo.write(0);

        //         FL_servo.set_can_rotate(false);
        //         FR_servo.set_can_rotate(false);
        //         BL_servo.set_can_rotate(false);
        //         BR_servo.set_can_rotate(false);

        //         break;
        //     case FORWARD_FLIGHT:
        //         FL_servo.set_can_rotate(true);
        //         FR_servo.set_can_rotate(true);
        //         BL_servo.set_can_rotate(true);
        //         BR_servo.set_can_rotate(true);

        //         FL_servo.write(pitch - roll);
        //         FR_servo.write(180 - pitch - roll);
        //         BL_servo.write(90);
        //         BR_servo.write(90);
        //         CamTiltServo.write(get_roll() * 180);
        //         CamPanServo.write(get_roll() * 180);
        //         break;
        //     default:
        //         break;
        // }
        // ESP_LOGI(main_TAG, "Value: %d", value);
    }
}
