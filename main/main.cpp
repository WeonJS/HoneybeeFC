extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "driver/i2c_master.h"
    #include "driver/i2c_types.h"
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

enum flight_mode_t {
    VTOL,
    FORWARD_FLIGHT,
};


honeybee_crsf::hb_crsf_rc_channel_data_t channels;


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
        double accel_x_g, accel_y_g, accel_z_g;
    private:
        

        honeybee_i2c::hb_i2c_master_cnctn_t master;
        honeybee_i2c::hb_i2c_bus_dev_t device;
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

        static constexpr uint8_t GYRO_CONFIG_1 = 0x01;
        static constexpr uint8_t ACCEL_CONFIG = 0x14;
        static constexpr double GYRO_SCALE_FACTOR_2000DPS = 16.4;
        static constexpr double ACCEL_SCALE_FACTOR_16G = 2048.0;

        void power_on();
        void power_off();
        void write_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data);
        void read_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data);
        void set_user_bank(icm20948_user_bank_t bank);
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
    

    // enable 2000 DPS gyro.
    uint8_t data = GYRO_SCALE_2000DPS;
    write_reg(USER_BANK_2, GYRO_CONFIG_1, &data);

    // enable 16g accel
    data = ACCEL_SCALE_16G;
    write_reg(USER_BANK_2, ACCEL_CONFIG, &data);

    power_on();
}


void ICM20948::write_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data)
{
    set_user_bank(user_bank);

    honeybee_i2c::hb_i2c_master_write(&device, &reg_addr, 1);
    honeybee_i2c::hb_i2c_master_write(&device, data, 1);
}

void ICM20948::read_reg(icm20948_user_bank_t user_bank, uint8_t reg_addr, uint8_t* data)
{
    set_user_bank(user_bank);

    honeybee_i2c::hb_i2c_master_write_read(&device, &reg_addr, 1, data, 1);
}

void ICM20948::set_user_bank(icm20948_user_bank_t user_bank)
{
    uint8_t user_bank_reg = REG_BANK_SEL;
    honeybee_i2c::hb_i2c_master_write(&device, &user_bank_reg, 1);
    
    uint8_t data = user_bank;
    data <<= 4; // data goes into bits 5:4 (pg 76 icm datasheet)
    honeybee_i2c::hb_i2c_master_write(&device, &data, 1);
}

void ICM20948::update_axes() {
    unsigned char power = 5;
    honeybee_i2c::hb_i2c_master_read(&device, &power, 1);
    ESP_LOGI(main_TAG, "power: %d", power);

    // gyro
    uint8_t gyro_x_h, gyro_x_l;
    read_reg(USER_BANK_0, GYRO_XOUT_H, &gyro_x_h);
    read_reg(USER_BANK_0, GYRO_XOUT_L, &gyro_x_l);
    uint16_t gyro_x = (gyro_x_h << 8) | gyro_x_l;

    uint8_t gyro_y_h, gyro_y_l;
    read_reg(USER_BANK_0, GYRO_YOUT_H, &gyro_y_h);
    read_reg(USER_BANK_0, GYRO_YOUT_L, &gyro_y_l);
    uint16_t gyro_y = (gyro_y_h << 8) | gyro_y_l;

    uint8_t gyro_z_h, gyro_z_l;
    read_reg(USER_BANK_0, GYRO_ZOUT_H, &gyro_z_h);
    read_reg(USER_BANK_0, GYRO_ZOUT_L, &gyro_z_l);
    uint16_t gyro_z = (gyro_z_h << 8) | gyro_z_l;

    ESP_LOGI(main_TAG, "gyro_x: %d", gyro_x);
    int16_t gyro_x_signed = (int16_t) gyro_x;
    int16_t gyro_y_signed = (int16_t) gyro_y;
    int16_t gyro_z_signed = (int16_t) gyro_z;

    gyro_x_dps = (double) gyro_x_signed / GYRO_SCALE_FACTOR_2000DPS;
    gyro_y_dps = (double) gyro_y_signed / GYRO_SCALE_FACTOR_2000DPS;
    gyro_z_dps = (double) gyro_z_signed / GYRO_SCALE_FACTOR_2000DPS;

    // accel
    uint8_t accel_x_h, accel_x_l;
    read_reg(USER_BANK_0, ACCEL_XOUT_H, &accel_x_h);
    read_reg(USER_BANK_0, ACCEL_XOUT_L, &accel_x_l);
    uint16_t accel_x = (accel_x_h << 8) | accel_x_l;

    uint8_t accel_y_h, accel_y_l;
    read_reg(USER_BANK_0, ACCEL_YOUT_H, &accel_y_h);
    read_reg(USER_BANK_0, ACCEL_YOUT_L, &accel_y_l);
    uint16_t accel_y = (accel_y_h << 8) | accel_y_l;

    uint8_t accel_z_h, accel_z_l;
    read_reg(USER_BANK_0, ACCEL_ZOUT_H, &accel_z_h);
    read_reg(USER_BANK_0, ACCEL_ZOUT_L, &accel_z_l);
    uint16_t accel_z = (accel_z_h << 8) | accel_z_l;

    int16_t accel_x_signed = (int16_t) accel_x;
    int16_t accel_y_signed = (int16_t) accel_y;
    int16_t accel_z_signed = (int16_t) accel_z;

    accel_x_g = (double) accel_x_signed / ACCEL_SCALE_FACTOR_16G; // assuming ACCEL_SCALE_16G
    accel_y_g = (double) accel_y_signed / ACCEL_SCALE_FACTOR_16G;
    accel_z_g = (double) accel_z_signed / ACCEL_SCALE_FACTOR_16G;

    // ESP_LOGI(main_TAG, "---------------------------------------------");
    // ESP_LOGI(main_TAG, "|   Axis   |   Gyro (DPS)   |   Accel (G)   |");
    // ESP_LOGI(main_TAG, "---------------------------------------------");
    // ESP_LOGI(main_TAG, "|    X     |    %.2f        |    %.2f       |", gyro_x_dps, accel_x_g);
    // ESP_LOGI(main_TAG, "---------------------------------------------");
    // ESP_LOGI(main_TAG, "|    Y     |    %.2f        |    %.2f       |", gyro_y_dps, accel_y_g);
    // ESP_LOGI(main_TAG, "---------------------------------------------");
    // ESP_LOGI(main_TAG, "|    Z     |    %.2f        |    %.2f       |", gyro_z_dps, accel_z_g);
    // ESP_LOGI(main_TAG, "---------------------------------------------");
}

void ICM20948::power_on()
{
    // enable power management 1 register
    uint8_t temp_data = 0x01;
    write_reg(USER_BANK_0, PWR_MGMT_1, &temp_data);
    read_reg(USER_BANK_0, PWR_MGMT_1, &temp_data);
    ESP_LOGI(main_TAG, "PWR_MGMT_1: %02X", temp_data);
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

    honeybee_utils::servo_t FL_servo;
    FL_servo.init(180);
    FL_servo.attach(SERVO_FL_PIN);

    honeybee_utils::servo_t FR_servo;
    FR_servo.init(180);
    FR_servo.attach(SERVO_FR_PIN);

    honeybee_utils::servo_t BL_servo;
    BL_servo.init(180);
    BL_servo.attach(SERVO_BL_PIN);

    honeybee_utils::servo_t BR_servo;
    BR_servo.init(180);
    BR_servo.attach(SERVO_BR_PIN);

    // honeybee_utils::servo_t CamTiltServo;
    // CamTiltServo.init(180);
    // CamTiltServo.attach(16);

    // honeybee_utils::servo_t CamPanServo;
    // CamPanServo.init(180);
    // CamPanServo.attach(15);

    honeybee_dshot::dshot_connection_t motor_BL; // bottom left, spin right
    honeybee_dshot::dshot_connection_t motor_TL; // top left, spin left
    honeybee_dshot::dshot_connection_t motor_BR; // bottom right, spin left
    honeybee_dshot::dshot_connection_t motor_TR; // top right, spin right
    motor_BL.init(GPIO_NUM_47, honeybee_dshot::DSHOT300, false);
    motor_TL.init(GPIO_NUM_21, honeybee_dshot::DSHOT300, false);
    motor_BR.init(GPIO_NUM_14, honeybee_dshot::DSHOT300, false);
    motor_TR.init(GPIO_NUM_13, honeybee_dshot::DSHOT300, false);
    
    ICM20948 icm;
    icm.init(GPIO_NUM_6, GPIO_NUM_5);

    while (true)
    {

        // Update the rc channels
        honeybee_crsf::update_rc_channels(elrs_uart_connection, channels);
        int pitch = get_pitch() * 180;
        int roll = (get_roll() - 0.5f) * 2 * max_roll_offset_angle;
        int dshot_throttle = honeybee_math::map(get_throttle(), 0, 1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        icm.update_axes();
        ESP_LOGI(main_TAG, "Pitch: %f\tRoll: %f", icm.gyro_x_dps, icm.gyro_y_dps);

        // ESP_LOGI(main_TAG, "Pitch: %d\tRoll: %d", channels.chan1, channels.chan0);
        
        float dshot_throttle_FL = 0;
        float dshot_throttle_FR = 0;
        float dshot_throttle_BL = 0;
        float dshot_throttle_BR = 0;

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

                motor_BL.send_throttle(dshot_throttle);
                motor_TL.send_throttle(dshot_throttle);
                motor_BR.send_throttle(dshot_throttle);
                motor_TR.send_throttle(dshot_throttle);

                break;
            case FORWARD_FLIGHT:
                FL_servo.set_can_rotate(true);
                FR_servo.set_can_rotate(true);
                BL_servo.set_can_rotate(true);
                BR_servo.set_can_rotate(true);

                FL_servo.write(pitch - roll);
                FR_servo.write(180 - pitch - roll);
                BL_servo.write(90);
                BR_servo.write(90);
                // CamTiltServo.write(get_roll() * 180);
                // CamPanServo.write(get_roll() * 180);

                motor_BL.send_throttle(0);
                motor_TL.send_throttle(dshot_throttle);
                motor_BR.send_throttle(0);
                motor_TR.send_throttle(dshot_throttle);
                break;
            default:
                break;
        }
        // ESP_LOGI(main_TAG, "Value: %d", value);
    }
}
