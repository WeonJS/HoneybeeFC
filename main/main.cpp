extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "driver/i2c_master.h"
    #include "driver/i2c_types.h"
    #include "esp_timer.h"
    #include <cmath>
}

#include "honeybee_esp32s3.h"

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
#define ICM20948_SDA_PIN 6
#define ICM20948_SCL_PIN 5

#define SERVO_FL_PIN 8
#define SERVO_BL_PIN 18
#define SERVO_FR_PIN 7
#define SERVO_BR_PIN 17


#define CRSF_RC_CHANNEL_MIN 174
#define CRSF_RC_CHANNEL_MAX 1811

int previous_time_us = esp_timer_get_time();
double pitch, roll, yaw;

int get_delta_time_us()
{
    int current_time_us = esp_timer_get_time();
    int delta_time = current_time_us - previous_time_us;
    previous_time_us = current_time_us;
    return delta_time;
}

enum flight_mode_t {
    VTOL,
    FORWARD_FLIGHT,
};


honeybee_crsf::hb_crsf_rc_channel_data_t channels;



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

enum icm20948_user_bank_t {
    USER_BANK_0 = 0,
    USER_BANK_1 = 1,
    USER_BANK_2 = 2,
    USER_BANK_3 = 3,
};

enum icm20948_gyro_scale_t {
    GYRO_SCALE_250DPS = 0,
    GYRO_SCALE_500DPS = 1,
    GYRO_SCALE_1000DPS = 2,
    GYRO_SCALE_2000DPS = 3,
};

enum icm20948_accel_scale_t {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G = 1,
    ACCEL_SCALE_8G = 2,
    ACCEL_SCALE_16G = 3,
};

class ICM20948 {
    public:
        void init(gpio_num_t sda_pin, gpio_num_t scl_pin);
        void update_axes();

        double gyro_x_dps, gyro_y_dps, gyro_z_dps;
        int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
        double accel_x_g, accel_y_g, accel_z_g;
        int16_t accel_x_raw, accel_y_raw, accel_z_raw;
        int16_t mag_x_raw, mag_y_raw, mag_z_raw;
    private:
        

        honeybee_i2c::hb_i2c_master_cnctn_t master;
        honeybee_i2c::hb_i2c_bus_dev_t device;
        honeybee_i2c::hb_i2c_bus_dev_t device_mag;
        static constexpr uint8_t I2C_ADDRESS = 0x68;
        static constexpr uint8_t PWR_MGMT_1 = 0x06;
        static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
        static constexpr uint8_t ACCEL_XOUT_L = 0x2E;
        static constexpr uint8_t ACCEL_YOUT_H = 0x2F;
        static constexpr uint8_t ACCEL_YOUT_L = 0x30;
        static constexpr uint8_t ACCEL_ZOUT_H = 0x31;
        static constexpr uint8_t ACCEL_ZOUT_L = 0x32;
        static constexpr uint8_t GYRO_XOUT_H = 0x33;
        static constexpr uint8_t GYRO_XOUT_L = 0x34;
        static constexpr uint8_t GYRO_YOUT_H = 0x35;
        static constexpr uint8_t GYRO_YOUT_L = 0x36;
        static constexpr uint8_t GYRO_ZOUT_H = 0x37;
        static constexpr uint8_t GYRO_ZOUT_L = 0x38;
        static constexpr uint8_t REG_BANK_SEL = 0x7F;
        static constexpr uint8_t ACCEL_CONFIG = 0x14;
        static constexpr uint8_t GYRO_CONFIG_1 = 0x01;

        static constexpr double GYRO_SCALE_FACTOR_2000DPS = 16.4;
        static constexpr double ACCEL_SCALE_FACTOR_16G = 2048.0;

        void power_on();
        void power_off();
        void write_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data);
        void read_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data);
        void set_user_bank(icm20948_user_bank_t bank);
        void set_bypass_mode(bool bypass);
};

void ICM20948::init(gpio_num_t sda_pin, gpio_num_t scl_pin)
{
    master.scl_pin = ICM20948_SCL_PIN;
    master.sda_pin = ICM20948_SDA_PIN;
    honeybee_i2c::hb_i2c_init_master(&master);

    device.addr = ICM20948_I2C_ADDR;
    device.scl_speed_hz = 100000; // can't use high speed mode because 10kohm resistors
    device.addr_len = honeybee_i2c::I2C_7_BIT_ADDR;
    honeybee_i2c::hb_i2c_init_dev(&master, &device, I2C_ADDR_BIT_LEN_7);
    
    power_on();

    // enable 2000 DPS gyro and filtering.
    uint8_t data = (GYRO_SCALE_2000DPS << 1) | 0b00111001;
    write_reg(USER_BANK_2, GYRO_CONFIG_1, &data);
    read_reg(USER_BANK_2, GYRO_CONFIG_1, &data);
    // ESP_LOGI(main_TAG, "Gyro config: %x", data);

    // enable 16g accel and filtering.
    data = (ACCEL_SCALE_16G << 1) | 0x01;
    write_reg(USER_BANK_2, ACCEL_CONFIG, &data);
    read_reg(USER_BANK_2, ACCEL_CONFIG, &data);
    // ESP_LOGI(main_TAG, "Accel config: %x", data);

    // bypass mode and set up magnetometer
    set_bypass_mode(true);
    device_mag = {
        .addr_len = honeybee_i2c::I2C_7_BIT_ADDR,
        .addr = 0x0C,
        .scl_speed_hz = 100000,
    };
    honeybee_i2c::hb_i2c_init_dev(&master, &device_mag, I2C_ADDR_BIT_LEN_7);

    // enable magnetometer
    uint8_t mag_data[2];
    mag_data[0] = 0x31;
    mag_data[1] = 0x08;
    honeybee_i2c::hb_i2c_master_write(&device_mag, mag_data, 2);

    set_bypass_mode(false);
}


void ICM20948::write_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data)
{
    set_user_bank(user_bank);

    uint8_t buffer[2];
    buffer[0] = reg_addr;
    buffer[1] = *data;

    honeybee_i2c::hb_i2c_master_write(&device, buffer, 2);
}

void ICM20948::set_bypass_mode(bool bypass)
{
    uint8_t data = bypass ? 0x02 : 0x00;
    write_reg(USER_BANK_0, 0x0F, &data);
}

void ICM20948::read_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data)
{
    set_user_bank(user_bank);

    honeybee_i2c::hb_i2c_master_write_read(&device, &reg_addr, 1, data, 1);
}

void ICM20948::set_user_bank(icm20948_user_bank_t user_bank)
{
    uint8_t buffer[2];

    buffer[0] = REG_BANK_SEL;
    buffer[1] = (uint8_t) (user_bank << 4);
    
    honeybee_i2c::hb_i2c_master_write(&device, buffer, 2);
}

void ICM20948::update_axes() {

    set_user_bank(USER_BANK_0);

    uint8_t reg_start = GYRO_XOUT_H; // 0x33
    uint8_t data_out[6];

    honeybee_i2c::hb_i2c_master_write_read(&device, &reg_start, 1, data_out, 6);

    gyro_x_raw = (int16_t)((data_out[0] << 8) | data_out[1]);
    gyro_y_raw = (int16_t)((data_out[2] << 8) | data_out[3]);
    gyro_z_raw = (int16_t)((data_out[4] << 8) | data_out[5]);

    gyro_x_dps = (double) gyro_x_raw / GYRO_SCALE_FACTOR_2000DPS - 0.6;
    gyro_y_dps = (double) gyro_y_raw / GYRO_SCALE_FACTOR_2000DPS - 0.25;
    gyro_z_dps = (double) gyro_z_raw / GYRO_SCALE_FACTOR_2000DPS - 0.3;


    // accel
    reg_start = ACCEL_XOUT_H;
    honeybee_i2c::hb_i2c_master_write_read(&device, &reg_start, 1, data_out, 6);

    accel_x_raw = (int16_t) ((data_out[0] << 8) | data_out[1]);
    accel_y_raw = (int16_t) ((data_out[2] << 8) | data_out[3]);
    accel_z_raw = (int16_t) ((data_out[4] << 8) | data_out[5]);

    accel_x_g = (double) accel_x_raw / ACCEL_SCALE_FACTOR_16G - 0.005; // assuming ACCEL_SCALE_16G
    accel_y_g = (double) accel_y_raw / ACCEL_SCALE_FACTOR_16G + 0.027;
    accel_z_g = (double) accel_z_raw / ACCEL_SCALE_FACTOR_16G - 0.01;

    // mag
    set_bypass_mode(true);

    uint8_t status_bit;
    uint8_t st1 = 0x10;
    honeybee_i2c::hb_i2c_master_write_read(&device_mag, &st1, 1, &status_bit, 1);
    if (status_bit & 0x01) {

        reg_start = 0x11;
        honeybee_i2c::hb_i2c_master_write_read(&device_mag, &reg_start, 1, data_out, 6);

        int16_t mag_x_signed = (int16_t) ((data_out[1] << 8) | data_out[0]);
        int16_t mag_y_signed = (int16_t) ((data_out[3] << 8) | data_out[2]);
        int16_t mag_z_signed = (int16_t) ((data_out[5] << 8) | data_out[4]);

        mag_x_raw = (double) mag_x_signed;
        mag_y_raw = (double) mag_y_signed;
        mag_z_raw = (double) mag_z_signed;

        // printf("%f,%f,%f\n", mag_x, mag_y, mag_z);

        // read from st2 to clear the data ready bit
        uint8_t st2 = 0x18;
        honeybee_i2c::hb_i2c_master_write_read(&device_mag, &st2, 1, &status_bit, 1);
    }
    
    set_bypass_mode(false);
}

void ICM20948::power_on()
{
    // enable power management 1 register
    uint8_t temp_data = 0x01;
    write_reg(USER_BANK_0, PWR_MGMT_1, &temp_data);
    read_reg(USER_BANK_0, PWR_MGMT_1, &temp_data);
}

void ICM20948::power_off()
{
    uint8_t data = 0x00;
    write_reg(USER_BANK_0, PWR_MGMT_1, &data);
}

honeybee_math::hb_vector2_t right_joystick_input;
honeybee_math::hb_vector2_t left_joystick_input;

int max_roll_offset_angle = 45;

extern "C" void app_main(void)
{
    // ESP_LOGI(main_TAG, "Starting main application");

    // Create an ELRS receiver
    honeybee_uart::hb_uart_config_t elrs_uart_connection = {
        .baud_rate = 420000,
        .rx_pin = 38,
        .tx_pin = 37,
        .rx_buf_size = 256,
        .tx_buf_size = 256,
        .port = UART_NUM_1,
    };
    honeybee_uart::uart_install_connection(elrs_uart_connection);

    honeybee::hb_servo_t FL_servo;
    FL_servo.init(180);
    FL_servo.attach(SERVO_FL_PIN);

    honeybee::hb_servo_t FR_servo;
    FR_servo.init(180);
    FR_servo.attach(SERVO_FR_PIN);

    honeybee::hb_servo_t BL_servo;
    BL_servo.init(180);
    BL_servo.attach(SERVO_BL_PIN);

    honeybee::hb_servo_t BR_servo;
    BR_servo.init(180);
    BR_servo.attach(SERVO_BR_PIN);

    // honeybee::hb_servo_t CamTiltServo;
    // CamTiltServo.init(180);
    // CamTiltServo.attach(16);

    // honeybee::hb_servo_t CamPanServo;
    // CamPanServo.init(180);
    // CamPanServo.attach(15);

    honeybee_dshot::dshot_cnctn_t motor_BL; // bottom left, spin right
    honeybee_dshot::dshot_cnctn_t motor_FL; // top left, spin left
    honeybee_dshot::dshot_cnctn_t motor_BR; // bottom right, spin left
    honeybee_dshot::dshot_cnctn_t motor_FR; // top right, spin right
    motor_BL.init(GPIO_NUM_47, honeybee_dshot::DSHOT300, false);
    motor_FL.init(GPIO_NUM_21, honeybee_dshot::DSHOT300, false);
    motor_BR.init(GPIO_NUM_14, honeybee_dshot::DSHOT300, false);
    motor_FR.init(GPIO_NUM_13, honeybee_dshot::DSHOT300, false);
    
    ICM20948 icm;
    icm.init(GPIO_NUM_6, GPIO_NUM_5);

    while (true)
    {

        // Update the rc channels
        honeybee_crsf::update_rc_channels(elrs_uart_connection, channels);
        int pitch_input = get_pitch() * 180;
        int roll_input = (get_roll() - 0.5f) * 2 * max_roll_offset_angle;
        int dshot_throttle = honeybee_math::map(get_throttle(), 0, 1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        icm.update_axes();
        // todo: print pitch roll yaw
        // ESP_LOGI(main_TAG, "Pitch: %f\tRoll: %f\tYaw: %f", pitch, roll, yaw);
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            icm.accel_x_raw, icm.accel_y_raw, icm.accel_z_raw,  // next three: accelerometer
            icm.gyro_x_raw, icm.gyro_y_raw, icm.gyro_z_raw, // last three: gyro
            icm.mag_x_raw, icm.mag_y_raw, icm.mag_z_raw
        );
        // ESP_LOGI(main_TAG, "accel x: %f\taccel y: %f\taccel z: %f", icm.accel_x_g, icm.accel_y_g, icm.accel_z_g);

        // ESP_LOGI(main_TAG, "Pitch: %d\tRoll: %d", channels.chan1, channels.chan0);
        
        float dshot_throttle_FL = dshot_throttle;
        float dshot_throttle_FR = dshot_throttle;
        float dshot_throttle_BL = dshot_throttle;
        float dshot_throttle_BR = dshot_throttle;

        flight_mode_t mode = (flight_mode_t) get_button_SA();
        switch (mode) {
            case VTOL:
                FL_servo.write(180);
                FR_servo.write(180);
                BL_servo.write(180);
                BR_servo.write(180);

                FL_servo.set_can_rotate(false);
                FR_servo.set_can_rotate(false);
                BL_servo.set_can_rotate(false);
                BR_servo.set_can_rotate(false);

                motor_BL.send_throttle(dshot_throttle_BL);
                motor_FL.send_throttle(dshot_throttle_FL);
                motor_BR.send_throttle(dshot_throttle_BR);
                motor_FR.send_throttle(dshot_throttle_FR);

                break;
            case FORWARD_FLIGHT:
                FL_servo.set_can_rotate(true);
                FR_servo.set_can_rotate(true);
                BL_servo.set_can_rotate(true);
                BR_servo.set_can_rotate(true);

                FL_servo.write(pitch_input - roll_input);
                FR_servo.write(180 - pitch_input - roll_input);
                BL_servo.write(90);
                BR_servo.write(90);
                // CamTiltServo.write(get_roll() * 180);
                // CamPanServo.write(get_roll() * 180);

                motor_BL.send_throttle(0);
                motor_FL.send_throttle(dshot_throttle_FL);
                motor_BR.send_throttle(0);
                motor_FR.send_throttle(dshot_throttle_FR);
                break;
            default:
                break;
        }
        // ESP_LOGI(main_TAG, "Value: %d", value);
    }
}
