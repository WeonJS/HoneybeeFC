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

int previous_time_us = 0;
int current_time_us = 0;
double pitch, roll, yaw;

double get_delta_time_s()
{
    int delta_time = current_time_us - previous_time_us;
    return (double) delta_time / 1000000.0;
}

enum flight_mode_t {
    VTOL,
    FORWARD_FLIGHT,
};

honeybee_crsf::hb_crsf_rc_channel_data_t channels;



double normalize_crsf_channel(uint16_t channel_value)
{
    return honeybee_math::map<double>(channel_value, CRSF_RC_CHANNEL_MIN, CRSF_RC_CHANNEL_MAX, 0, 1);
}

double get_yaw(double min = 0, double max = 1) {
    double normalized_value = normalize_crsf_channel(channels.chan3);
    return honeybee_math::map<double>(normalized_value, 0, 1, min, max);
}

double get_throttle(double min = 0, double max = 1) {
    double normalized_value = normalize_crsf_channel(channels.chan2);
    return honeybee_math::map<double>(normalized_value, 0, 1, min, max);
}

double get_pitch(double min = 0, double max = 1) {
    double normalized_value = normalize_crsf_channel(channels.chan1);
    return honeybee_math::map<double>(normalized_value, 0, 1, min, max);
}

double get_roll(double min = 0, double max = 1) {
    double normalized_value = normalize_crsf_channel(channels.chan0);
    return honeybee_math::map<double>(normalized_value, 0, 1, min, max);
}

int get_button_SA() {
    return (int) honeybee_math::map<int>(channels.chan4, 191, 1792, 0, 1); // todo: ugly
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

typedef struct imu_data_s {
    double pitch_cf, roll_cf, yaw_cf;
} imu_rot_data_t;

class ICM20948 {
    public:
        void init(gpio_num_t sda_pin, gpio_num_t scl_pin);
        void update_axes();

        StaticQueue_t imu_rot_data_static_queue;
        uint8_t queue_buffer[10 * sizeof(imu_rot_data_t)];
        QueueHandle_t imu_rot_data_queue;
    private:
        int16_t mag_x_raw, mag_y_raw, mag_z_raw;
        double pitch_cf, roll_cf, yaw_cf;

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
    uint8_t data = (GYRO_SCALE_2000DPS << 1) | 0x01;
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
        .scl_speed_hz = 400000,
    };
    honeybee_i2c::hb_i2c_init_dev(&master, &device_mag, I2C_ADDR_BIT_LEN_7);

    // enable magnetometer
    uint8_t mag_data[2];
    mag_data[0] = 0x31;
    mag_data[1] = 0x08;
    honeybee_i2c::hb_i2c_master_write(&device_mag, mag_data, 2);

    set_bypass_mode(false);

    imu_rot_data_queue = xQueueCreateStatic(10, sizeof(imu_rot_data_t), queue_buffer, &imu_rot_data_static_queue);
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

    // get raw values from imu
    double gyro_x_raw = (int16_t)((data_out[0] << 8) | data_out[1]);
    double gyro_y_raw = (int16_t)((data_out[2] << 8) | data_out[3]);
    double gyro_z_raw = (int16_t)((data_out[4] << 8) | data_out[5]);

    // convert to degrees per second
    double gyro_x_dps = (double) gyro_x_raw / GYRO_SCALE_FACTOR_2000DPS;
    double gyro_y_dps = (double) gyro_y_raw / GYRO_SCALE_FACTOR_2000DPS;
    double gyro_z_dps = (double) gyro_z_raw / GYRO_SCALE_FACTOR_2000DPS;

    // apply offsets for calibration
    gyro_x_dps -= 0.6;
    gyro_y_dps -= 0.25;
    gyro_z_dps -= 0.3;

    // accel
    reg_start = ACCEL_XOUT_H;
    honeybee_i2c::hb_i2c_master_write_read(&device, &reg_start, 1, data_out, 6);

    double accel_x_raw = (int16_t) ((data_out[0] << 8) | data_out[1]);
    double accel_y_raw = (int16_t) ((data_out[2] << 8) | data_out[3]);
    double accel_z_raw = (int16_t) ((data_out[4] << 8) | data_out[5]);

    // convert to g
    double accel_x_g = (double) accel_x_raw / ACCEL_SCALE_FACTOR_16G; // assuming ACCEL_SCALE_16G
    double accel_y_g = (double) accel_y_raw / ACCEL_SCALE_FACTOR_16G;
    double accel_z_g = (double) accel_z_raw / ACCEL_SCALE_FACTOR_16G;

    // apply offsets for calibration
    accel_x_g -= 0.05;
    accel_y_g += 0.027;
    accel_z_g -= 0.01;

    // NOTE FOR ACCEL: the +x axis is indicating that when a force is applied in the +x direction, the acceleration is positive.
    // flipping these raw axes because my imu is facing downwards and i want to treat it as facing upwards.
    accel_x_g = -accel_x_g;
    accel_z_g = -accel_z_g;

    // flipping x and y axes to match conventional drone axes
    double temp = accel_x_g;
    accel_x_g = accel_y_g;
    accel_y_g = temp;

    // mag
    set_bypass_mode(true);

    
    uint8_t status_bit;
    uint8_t st1 = 0x10;
    honeybee_i2c::hb_i2c_master_write_read(&device_mag, &st1, 1, &status_bit, 1);
    if (status_bit & 0x01) {

        reg_start = 0x11;
        honeybee_i2c::hb_i2c_master_write_read(&device_mag, &reg_start, 1, data_out, 6);

        mag_x_raw = (int16_t) ((data_out[1] << 8) | data_out[0]);
        mag_y_raw = (int16_t) ((data_out[3] << 8) | data_out[2]);
        mag_z_raw = (int16_t) ((data_out[5] << 8) | data_out[4]);

        
    }
    
    

    // convert to uT
    double mag_x_cal = mag_x_raw * 0.15;
    double mag_y_cal = mag_y_raw * 0.15;
    double mag_z_cal = mag_z_raw * 0.15;

    // printf("Raw:0,0,0,0,0,0,%d,%d,%d\n", (int)(mag_x_cal * 10), (int)(mag_y_cal * 10), (int)(mag_z_cal * 10));

    // apply calibrated offsets
    mag_x_cal -= 68.91;
    mag_y_cal -= (-42.63);
    mag_z_cal -= (-181.81);

    // apply magnetic mapping
    mag_x_cal = 1.016 * mag_x_cal + 0.016 * mag_y_cal + (-0.013) * mag_z_cal;
    mag_y_cal = 0.016 * mag_x_cal + 0.966 * mag_y_cal + 0.004 * mag_z_cal;
    mag_z_cal = (-0.013) * mag_x_cal + 0.004 * mag_y_cal + 1.020 * mag_z_cal;

    // imu faces downwards, so flip x (we want z to face up anyway so we won't flip that)
    mag_x_cal = -mag_x_cal;
    mag_y_cal = -mag_y_cal;

    // printf("Cal:%f,%f,%f\n", mag_x_cal, mag_y_cal, mag_z_cal);

    // flipping x and y axes to match conventional drone axes
    temp = mag_x_cal;
    mag_x_cal = mag_y_cal;
    mag_y_cal = temp;

    double Ax = accel_x_g;
    double Ay = accel_y_g;
    double Az = accel_z_g;

    double Mx = mag_x_cal;
    double My = mag_y_cal;
    double Mz = mag_z_cal;

    double pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az));
    double roll  = atan2(Ay, Az);

    double pitch_acc = honeybee_math::map<double>(pitch, -M_PI/2, M_PI/2, -90, 90);
    double roll_acc = honeybee_math::map<double>(roll, -M_PI/2, M_PI/2, -90, 90);

    // 2) Tilt-compensate the mag
    double heading_x = Mx*cos(pitch) + Mz*sin(pitch);
    double heading_y = Mx*sin(roll)*sin(pitch)
                    + My*cos(roll)
                    - Mz*sin(roll)*cos(pitch);

    // 3) Convert to degrees, and wrap 0..360
    double heading = atan2(heading_y, heading_x) * 180 / M_PI;
    if (heading < 0) heading += 360;
    

    // read from st2 to clear the data ready bit
    uint8_t st2 = 0x18;
    honeybee_i2c::hb_i2c_master_write_read(&device_mag, &st2, 1, &status_bit, 1);

    double dt = get_delta_time_s();
    double alpha = 0.5;

    pitch_cf = alpha * (pitch_cf + gyro_y_dps * dt) + (1 - alpha) * pitch_acc;
    roll_cf = alpha * (roll_cf + gyro_x_dps * dt) + (1 - alpha) * roll_acc;
    yaw_cf = alpha * (yaw_cf + gyro_z_dps * dt) + (1 - alpha) * heading;

    imu_rot_data_t imu_rot_data;
    imu_rot_data = {
        .pitch_cf = pitch_cf,
        .roll_cf = roll_cf,
        .yaw_cf = yaw_cf,
    };
    xQueueSend(imu_rot_data_queue, &imu_rot_data, 0);

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

honeybee_math::hb_vector2_t RJ_input;
honeybee_math::hb_vector2_t LJ_input;

int max_roll_offset_angle = 45;


typedef struct {
    honeybee_uart::hb_uart_cnctn_t elrs_uart_connection;
    honeybee_crsf::hb_crsf_rc_channel_data_t* channels;
    ICM20948* icm_ptr;
} task_params_t;

extern "C" void update_task(void* param) {
    task_params_t* params = (task_params_t*) param;

    while (true) {
        // Update RC channels (this function likely blocks until data is received)
        honeybee_crsf::update_rc_channels(params->elrs_uart_connection, *(params->channels));
        
        // Update sensor axes
        params->icm_ptr->update_axes();

        // Delay a bit or yield so that this task does not hog the CPU.
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void app_main(void)
{
    // ESP_LOGI(main_TAG, "Starting main application");

    // Create an ELRS receiver
    honeybee_uart::hb_uart_cnctn_t elrs_uart_connection = {
        .baud_rate = 420000,
        .rx_pin = 38,
        .tx_pin = 37,
        .rx_buf_size = 256,
        .tx_buf_size = 256,
        .port = UART_NUM_1,
    };
    honeybee_uart::uart_install_cnctn(elrs_uart_connection);

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

    int loops = 0;
    uint64_t last_time = esp_timer_get_time();

    task_params_t task_params = {
        .elrs_uart_connection = elrs_uart_connection,
        .channels = &channels,
        .icm_ptr = &icm,
    };

    xTaskCreatePinnedToCore(
        update_task,
        "update_rc_channels",
        4096,
        &task_params,
        1,
        NULL,
        0
    );

    honeybee::hb_pid_t pitch_pid;
    pitch_pid.init(4, 0.05, 2, -1024, 1024);

    honeybee::hb_pid_t roll_pid;
    roll_pid.init(4, 0.05, 2, -1024, 1024);

    honeybee::hb_pid_t yaw_pid;
    yaw_pid.init(3, 0.01, 1, -1024, 1024);

    while (true)
    {
        current_time_us = esp_timer_get_time();
        
        double pitch_input = get_pitch(-1, 1);
        double roll_input = get_roll(-1, 1);
        double throttle_input = get_throttle();
        double yaw_input = get_yaw(-1, 1);

        honeybee_math::hb_vector2_t RJ_input;
        RJ_input.set(roll_input, pitch_input);
        RJ_input = RJ_input.clamp_mag(1);

        // honeybee_math::hb_vector2_t LJ_input;
        // LJ_input.set(yaw_input, throttle_input);
        // LJ_input = LJ_input.clamp_mag(1);
        // printf("roll: %f, pitch: %f\n", roll_input, pitch_input);

        // printf("lj: %f, %f\n", LJ_input.get_x(), LJ_input.get_y());
        // printf("rj: %f, %f\n", RJ_input.get_x(), RJ_input.get_y());

        // printf("Pitch: %f\tRoll: %f\tThrottle: %d\n", get_pitch(), get_roll(), dshot_throttle);
        // printf("Pitch: %d\tRoll: %d\tThrottle: %d\n", pitch_input, roll_input, dshot_throttle);

        honeybee_math::hb_vector2_t input_pos_FL;
        input_pos_FL.set(1, -1);
        input_pos_FL = input_pos_FL.normal();

        honeybee_math::hb_vector2_t input_pos_FR;
        input_pos_FR.set(-1, -1);
        input_pos_FR = input_pos_FR.normal();

        honeybee_math::hb_vector2_t input_pos_BL;
        input_pos_BL.set(1, 1);
        input_pos_BL = input_pos_BL.normal();

        honeybee_math::hb_vector2_t input_pos_BR;
        input_pos_BR.set(-1, 1);
        input_pos_BR = input_pos_BR.normal();

        int dshot_throttle = honeybee_math::map<double>(throttle_input, 0, 1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        double FL_control = input_pos_FL.dot(RJ_input);
        double FR_control = input_pos_FR.dot(RJ_input);
        double BL_control = input_pos_BL.dot(RJ_input);
        double BR_control = input_pos_BR.dot(RJ_input);

        // printf("FL: %f\tFR: %f\tBL: %f\tBR: %f\n", FL_control, FR_control, BL_control, BR_control);
        
        int16_t max_control_thrust = 300;
        int16_t dshot_throttle_FL = (int16_t) honeybee_math::map<double>(FL_control, -1, 1, -max_control_thrust, max_control_thrust) + dshot_throttle;
        int16_t dshot_throttle_FR = (int16_t) honeybee_math::map<double>(FR_control, -1, 1, -max_control_thrust, max_control_thrust) + dshot_throttle;
        int16_t dshot_throttle_BL = (int16_t) honeybee_math::map<double>(BL_control, -1, 1, -max_control_thrust, max_control_thrust) + dshot_throttle;
        int16_t dshot_throttle_BR = (int16_t) honeybee_math::map<double>(BR_control, -1, 1, -max_control_thrust, max_control_thrust) + dshot_throttle;

        printf("FL: %d\tFR: %d\tBL: %d\tBR: %d\n", dshot_throttle_FL, dshot_throttle_FR, dshot_throttle_BL, dshot_throttle_BR);

        imu_rot_data_t imu_rot_data;
        if(xQueueReceive(icm.imu_rot_data_queue, &imu_rot_data, 0) == pdPASS) {
            // printf("Cal: %f, %f, %f\n", imu_rot_data.pitch_cf, imu_rot_data.roll_cf, imu_rot_data.yaw_cf);
        }

        double pitch_val = pitch_pid.calculate(0, imu_rot_data.pitch_cf, get_delta_time_s());
        double roll_val = roll_pid.calculate(0, imu_rot_data.roll_cf, get_delta_time_s());
        double yaw_val = yaw_pid.calculate(45, imu_rot_data.yaw_cf, get_delta_time_s());
        // printf("Pitch: %f\t Roll: %f\t Yaw: %f\n", pitch_val, roll_val, yaw_val);

        // printf("bl thrust: %d\tfl thrust: %d\tbr thrust: %d\tfr thrust: %d\n", dshot_throttle_BL, dshot_throttle_FL, dshot_throttle_BR, dshot_throttle_FR);

        vTaskDelay(pdMS_TO_TICKS(1));

        // todo: print pitch roll yaw
        // ESP_LOGI(main_TAG, "Pitch: %f\tRoll: %f\tYaw: %f", pitch, roll, yaw);
        // printf("Cal:%f,%f,%f,%f,%f,%f,%f\n",
        //     icm.accel_x_g, icm.accel_y_g, icm.accel_z_g,  // next three: accelerometer
        //     icm.gyro_x_dps, icm.gyro_y_dps, icm.gyro_z_dps, // last three: gyro
        //     icm.heading
        // );
        // printf("cf:%f,%f,%f,%d\n", icm.pitch_cf, icm.roll_cf, icm.yaw_cf, a++); 

        // printf("Cal:%d,%d,%d\n", icm.mag_x_cal, icm.mag_y_cal, icm.mag_z_cal);
        // ESP_LOGI(main_TAG, "accel x: %f\taccel y: %f\taccel z: %f", icm.accel_x_g, icm.accel_y_g, icm.accel_z_g);

        // ESP_LOGI(main_TAG, "Pitch: %d\tRoll: %d", channels.chan1, channels.chan0);
        
        

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

                FL_servo.write(pitch_input + roll_input);
                FR_servo.write(pitch_input + roll_input);
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
        
        loops++;
        uint64_t now = esp_timer_get_time();
        if (now - last_time >= 1000000) { // 1 second elapsed
            printf("Loop count: %d loops per second\n", loops);
            loops = 0; // Reset counter
            last_time = now;
        }
        previous_time_us = current_time_us;
    }
}
