#pragma once

extern "C" {
    #include "driver/i2c_master.h"
    #include <driver/uart.h>
    #include <driver/ledc.h>
}

#include "ELRSReceiver.h"
#include "ThrustActuator.h"

namespace HoneybeeFramework {

    enum DroneState
    {
        RUNNING,
        STOPPED,
        NONE
    };

    enum SerialConnectionType
    {
        UART,
        I2C,
        SPI
    };

    enum Honeybee_ErrType
    {
        HONEYBEE_OK,
        HONEYBEE_ERR,
        HONEYBEE_INVALID_CRC
    };

    struct CRSF_RC_ChannelData
    {
        unsigned chan0 : 11;
        unsigned chan1 : 11;
        unsigned chan2 : 11;
        unsigned chan3 : 11;
        unsigned chan4 : 11;
        unsigned chan5 : 11;
        unsigned chan6 : 11;
        unsigned chan7 : 11;
        unsigned chan8 : 11;
        unsigned chan9 : 11;
        unsigned chan10 : 11;
        unsigned chan11 : 11;
        unsigned chan12 : 11;
        unsigned chan13 : 11;
        unsigned chan14 : 11;
        unsigned chan15 : 11;
    };

    enum class CRSF_FrameType : uint8_t 
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

    struct SerialConnectionConfig 
    {
        SerialConnectionType type;
    };

    struct UART_ConnectionConfig : public SerialConnectionConfig
    {
        uart_port_t port;
        unsigned int rx_pin;
        unsigned int tx_pin;
        unsigned int rx_buf_size;
        unsigned int tx_buf_size;
    };

    struct I2C_ConnectionConfig : SerialConnectionConfig
    {
        i2c_port_t port;
        unsigned int sda_pin;
        unsigned int scl_pin;
    };

    class SerialConnection
    {
        public:
            SerialConnectionConfig config;
            unsigned int data_rate;
    };

    /// @brief Represents any subsystem in the framework. Could be a hardware subsystem, or a code subsystem.
    class SubSystem 
    {
        public:
            virtual void update() = 0;
    };

    class Drone
    {
        public:
            void init();
            void update();
            void set_state(DroneState state);
            DroneState get_state();
            void add_elrs(ELRSReceiver receiver);
            void add_actuator(ThrustActuator actuator);
        private:
            DroneState state;
            ELRSReceiver receiver;
            ThrustActuator actuator;
    };

    class Propeller
    {
        public:
            void init();
            void attach(int data_pin);
            void set_thrust(int thrust);
            int get_thrust();
            void send_data();
        private:
            int thrust;
            int data_pin;
            int max_thrust;
    };

    class Servo 
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
        private:
            int pin = -1;
            int angle = 0;
            int max_angle = 0;
            bool is_attached = false;
            ledc_channel_t channel;
            const static int MIN_PULSE_WIDTH_US = 500;
            const static int MAX_PULSE_WIDTH_US = 2500;
            static int servo_count;
    };

    Honeybee_ErrType process_crsf_frame(uint8_t* frame, int buf_size, int si, CRSF_RC_ChannelData &channels);
    uint8_t calculate_crc8(uint8_t* data, int si, int len);
    float clamp(float value, float min, float max);
    float normalize(float value, float min, float max);
}