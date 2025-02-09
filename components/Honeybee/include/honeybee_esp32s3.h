#pragma once

#define MAX_I2C_CONNECTIONS 2
#define MAX_UART_CONNECTIONS 3

extern "C" {
    #include "driver/ledc.h"
    #include "driver/rmt_tx.h"
    #include "driver/rmt_types.h"
    #include "driver/i2c_master.h"
    #include "driver/i2c_slave.h"
    #include "driver/uart.h"
}

#include "honeybee_common.h"

// DShot
#define DSHOT_PACKET_LENGTH 17 // last bit is for pause
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_PAUSE_LENGTH 21 // 21 bit pause is recommended
#define DSHOT_NULL_PACKET 0b0000000000000000
#define DSHOT_CLK_DIVIDER 8 // i guess this reduces the frequency to something that dshot can handle. need to look into this more.
#define DSHOT_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#define dshot_TAG "DShot"


namespace honeybee {
    enum hb_drone_state_t
    {
        DRONE_STATE_RUNNING,
        DRONE_STATE_STOPPED,
        DRONE_STATE_NONE
    };

    enum hb_err_t
    {
        HONEYBEE_OK,
        HONEYBEE_ERR,
        HONEYBEE_INVALID_CRC
    };

    extern hb_drone_state_t drone_state;

    class hb_pid {
        public:
            hb_pid(double Kp, double Ki, double Kd);

            void setTunings(double Kp, double Ki, double Kd);
            void setOutputLimits(double min, double max);
            void setMode(bool mode);
            void setSampleTime(int sampleTime);

            double compute(double input, double setpoint);
        
        private:
            double _Kp;
            double _Ki;
            double _Kd;

            double _min;
            double _max;

            bool _mode; // true = manual, false = automatic

            int _sampleTime;

            double _lastInput;
            double _output;
            double _integral;
            unsigned long _lastTime;
    };

    class hb_servo_t
    {
        public:
            ledc_timer_bit_t resolution;
            uint32_t frequency;
            void init(int max_angle);
            bool attach(int pin);
            bool write(int angle);
            bool write_us(int angle);
            int read();
            int read_us();
            bool attached();
            void detach();
            int angle_to_us(int angle);
            int us_to_angle(int us);
            int get_max_angle();
            honeybee::hb_err_t set_can_rotate(bool can_rotate);
        private:
            int pin = -1;
            int angle = 0;
            int max_angle = 0;
            bool is_attached = false;
            ledc_channel_t channel;
            const static int MIN_PULSE_WIDTH_US = 500;
            const static int MAX_PULSE_WIDTH_US = 2500;
            bool can_rotate = true;
            static int servo_count;
    };

    class hb_complementary_fltr_t
    {
        public:
        private:
    };
}

namespace honeybee_utils {
    
}

namespace honeybee_i2c {

    

    // The address length of a device.
    typedef enum hb_i2c_address_len_e {
        I2C_7_BIT_ADDR,
        I2C_10_BIT_ADDR
    } hb_i2c_address_len_t;

    // The i2c type of a device.
    typedef enum hb_i2c_device_type_e {
        I2C_MASTER,
        I2C_SLAVE
    } hb_i2c_device_type_t;

    // The master connection that this microcontroller is managing.
    typedef struct hb_i2c_master_cnctn_s
    {
        unsigned int sda_pin;
        unsigned int scl_pin;

        // ESP32S3 specific
        i2c_master_bus_handle_t master_handle;
    } hb_i2c_master_cnctn_t;

    // The slave connection that this microcontroller is managing.
    typedef struct hb_i2c_slave_cnctn_s {
        unsigned short address;
        unsigned int sda_pin;
        unsigned int scl_pin;

        // ESP32S3 specific
        i2c_slave_dev_handle_t slave_handle;
    } hb_i2c_slave_cnctn_t;

    // A bus connection which this microcontroller is managing, either as a slave or master.
    typedef struct hb_i2c_cnctn_s {
        hb_i2c_device_type_t type;
        union {
            hb_i2c_master_cnctn_t master;
            hb_i2c_slave_cnctn_t slave;
        };
    } hb_i2c_cnctn_t;

    // A slave device on our master bus.
    typedef struct hb_i2c_master_bus_dev_s {
        hb_i2c_address_len_t addr_len;
        unsigned short addr;
        unsigned int scl_speed_hz;

        // ESP32S3 specific
        i2c_master_dev_handle_t device;
    } hb_i2c_bus_dev_t;

    extern int num_i2c_connections;
    extern hb_i2c_cnctn_t i2c_connections[MAX_I2C_CONNECTIONS];

    honeybee::hb_err_t hb_i2c_init_master(hb_i2c_master_cnctn_t *config);

    void hb_i2c_master_write(hb_i2c_bus_dev_t *device, unsigned char *data, long unsigned int data_size, int wait_ms = -1);

    void hb_i2c_master_read(hb_i2c_bus_dev_t *device, unsigned char *data, long unsigned int data_size, int wait_ms = -1);

    void hb_i2c_master_write_read(hb_i2c_bus_dev_t *device, unsigned char *write_data, long unsigned int write_data_size, unsigned char *read_data, long unsigned int read_data_size, int wait_ms = -1);

    bool hb_i2c_check_dev_exists(hb_i2c_master_cnctn_t *master, unsigned short device_address, int wait_ms = -1);

    honeybee::hb_err_t hb_i2c_init_dev(hb_i2c_master_cnctn_t *master, hb_i2c_bus_dev_t *device, long unsigned int wait_us = 0);
    
}

namespace honeybee_uart {
    

    struct hb_uart_config_t
    {
        int baud_rate;
        unsigned int rx_pin;
        unsigned int tx_pin;
        unsigned int rx_buf_size;
        unsigned int tx_buf_size;

        uart_port_t port;
    };

    extern int num_uart_connections;
    extern hb_uart_config_t uart_connections[MAX_UART_CONNECTIONS];

    void uart_install_connection(hb_uart_config_t config);

    int read_bytes(hb_uart_config_t config, uint8_t* buf, int len);
}

namespace honeybee_math {
    class hb_vector2_t {
        public:
            void set_x(float x);
            void set_y(float y);
            float get_x();
            float get_y();
            float dot(hb_vector2_t other);
            hb_vector2_t normal();
        private:
            float x, y;
    };

    float map(float value, float in_min, float in_max, float out_min, float out_max);
    float clamp(float value, float min, float max);
    float normalize(float value, float min, float max);
}


namespace honeybee_dshot {
    

    enum dshot_mode_t
    {
        DSHOT_OFF,
        DSHOT150,
        DSHOT300,
        DSHOT600,
        DSHOT1200
    };

    struct dshot_packet_t
    {
        uint16_t throttle_value : 11;
        uint16_t telemetric_request : 1;
        uint16_t checksum : 4;
    };

    struct dshot_config_t
    {
        dshot_mode_t mode;
        rmt_channel_handle_t rmt_channel;
        bool is_bidirectional;
        gpio_num_t gpio_num;
        uint8_t mem_block_symbols;
        float time_high_0_us; // time low in microseconds
        float time_high_1_us; // time high in microseconds
    };

    struct rmt_dshot_encoder_config_t {
        uint32_t resolution;
    };

    typedef struct {
        rmt_encoder_t base;
        rmt_encoder_t *bytes_encoder;
        rmt_encoder_t *copy_encoder;
        rmt_symbol_word_t dshot_delay_symbol;
        int state;
    } rmt_dshot_esc_encoder_t;

    typedef struct {
        uint16_t throttle;  /*!< Throttle value */
        bool telemetry_req; /*!< Telemetry request */
    } dshot_esc_throttle_t;

    /**
     * @brief Type of Dshot ESC encoder configuration
     */
    struct dshot_esc_encoder_config_t {
        uint32_t resolution;    /*!< Encoder resolution, in Hz */
        uint32_t baud_rate;     /*!< Dshot protocol runs at several different baud rates, e.g. DSHOT300 = 300k baud rate */
        uint32_t post_delay_us; /*!< Delay time after one Dshot frame, in microseconds */
    };

    typedef union {
        struct {
            uint16_t crc: 4;       /*!< CRC checksum */
            uint16_t telemetry: 1; /*!< Telemetry request */
            uint16_t throttle: 11; /*!< Throttle value */
        };
        uint16_t val;
    } dshot_esc_frame_t;

    // Represents a single DShot channel.
    class dshot_cnctn_t
    {
        public:

            bool init(gpio_num_t gpio_num, dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false, int startup_delay_ms = 3000);

            void send_throttle(uint16_t throttle_value);

            void remove();

        private:
            int dshot_cnctn_ts = 0;
            rmt_channel_handle_t esc_chan;
            rmt_tx_channel_config_t tx_chan_config;
            rmt_encoder_handle_t rmt_encoder;
            rmt_encoder_handle_t dshot_encoder;
            dshot_esc_encoder_config_t encoder_config;
            rmt_transmit_config_t tx_config;


            uint8_t calculateCRC(const dshot_packet_t &dshot_packet);  // Calculates the CRC checksum for a DShot packet.
            uint16_t serialize_packet(const dshot_packet_t &dshot_packet); // Parses an RMT packet to obtain a DShot packet.
            void send_packet(const dshot_packet_t &dshot_packet); // Sends a DShot packet via RMT.
    };

    void make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry);
    size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
    esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder);
    esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder);
    esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);
}


namespace honeybee_crsf {
    struct hb_crsf_rc_channel_data_t
    {
        uint16_t chan0 : 11;
        uint16_t chan1 : 11;
        uint16_t chan2 : 11;
        uint16_t chan3 : 11;
        uint16_t chan4 : 11;
        uint16_t chan5 : 11;
        uint16_t chan6 : 11;
        uint16_t chan7 : 11;
        uint16_t chan8 : 11;
        uint16_t chan9 : 11;
        uint16_t chan10 : 11;
        uint16_t chan11 : 11;
        uint16_t chan12 : 11;
        uint16_t chan13 : 11;
        uint16_t chan14 : 11;
        uint16_t chan15 : 11;
    };

    enum class hb_crsf_frame_type_t : uint8_t
    {
        gps = 0x02,
        battery_sensor = 0x08,
        link_statistics = 0x14,
        rc_channels_packed = 0x16,
        attitude = 0x1E,
        flight_mode = 0x21,

        // Extended header frames, range: 0x28 to 0x96
        device_ping = 0x28,
        device_info = 0x29,
        parameter_settings_entry = 0x2B,
        parameter_read = 0x2C,
        parameter_write = 0x2D,
        command = 0x32
    };
    
    honeybee::hb_err_t process_crsf_frame(uint8_t *data, int buf_size, int si, hb_crsf_rc_channel_data_t &channels);
    honeybee::hb_err_t update_rc_channels(honeybee_uart::hb_uart_config_t uart_cnctn, hb_crsf_rc_channel_data_t &channels);
}
