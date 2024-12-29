extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "driver/i2c_master.h"
    #include "driver/ledc.h"
    #include "driver/rmt_tx.h"
    #include "driver/rmt_types.h"
    #include "driver/i2c_master.h"
    #include "driver/i2c_slave.h"
    #include <cmath>
}

#include "ESPAbstraction.h"


#include <bitset>
#include <esp_err.h>
#include <esp_check.h>

// The structure of a CRSF frame is as follows:
//[sync] [len] [type] [payload] [crc8]
// sync: marks the beginning of a frame, always 0xC8
// len: the number of bytes from [type] to [crc8] inclusive.
// type: the type of the frame, e.g. 0x16 for RC channels
// payload: the data of the frame
// crc8: a checksum of the frame

#define main_TAG "main"

#define DSHOT_PACKET_LENGTH 17 // last bit is for pause
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_PAUSE_LENGTH 21 // 21 bit pause is recommended
#define DSHOT_NULL_PACKET 0b0000000000000000
#define DSHOT_CLK_DIVIDER 8 // i guess this reduces the frequency to something that dshot can handle. need to look into this more.
#define DSHOT_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution

#define ICM20948_I2C_ADDR 0x68
#define ICM20948_I2C_PWR_MGMT_REG_ADDR 0x06

enum DroneState
{
    RUNNING,
    STOPPED,
    NONE
};

enum FlightMode {
    VTOL,
    FORWARD_FLIGHT,
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

CRSF_RC_ChannelData channels;

float clamp(float value, float min, float max)
{
    if (value < min)
        value = min;
    if (value > max)
        value = max;

    return value;
}

float normalize(float value, float min, float max)
{
    return (value - min) / (max - min);
}

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
    Honeybee_ErrType set_can_rotate(bool can_rotate);
private:
    int pin = -1;
    int angle = 0;
    int max_angle = 0;
    bool is_attached = false;
    ledc_channel_t channel;
    const static int MIN_PULSE_WIDTH_US = 500;
    const static int MAX_PULSE_WIDTH_US = 2500;
    static int servo_count;
    bool can_rotate = true;
};

Honeybee_ErrType Servo::set_can_rotate(bool can_rotate)
{
    this->can_rotate = can_rotate;
    return HONEYBEE_OK;
}

int Servo::servo_count = 0;
// given a pulse width in us, calculate the angle in degrees
int Servo::us_to_angle(int us)
{
    // Calculate the angle in degrees
    int angle = (us - MIN_PULSE_WIDTH_US) * max_angle / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);

    return angle;
}

// given an angle, calculate the pulse width in us to send to the servo
int Servo::angle_to_us(int angle)
{
    // Calculate the pulse width in us
    int pulse_width = (angle * (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US) / max_angle) + MIN_PULSE_WIDTH_US;

    return pulse_width;
}

// get max angle
int Servo::get_max_angle()
{
    return this->max_angle;
}

void Servo::init(int _max_angle)
{
    max_angle = _max_angle;
    channel = (ledc_channel_t)servo_count++;
    frequency = 50;
    resolution = LEDC_TIMER_14_BIT;

    // Configure the LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // Use low-speed mode
        .duty_resolution = resolution,     // Set duty resolution. ESP32-S3 is capped to 14 bits.
        .timer_num = LEDC_TIMER_0,         // Use timer 0
        .freq_hz = frequency,              // Frequency at 50Hz for servos
        .clk_cfg = LEDC_AUTO_CLK,          // Use auto clock
        .deconfigure = false,              // Do not deconfigure timer
    };
    ledc_timer_config(&timer_config);
}

// establish pwm line to the servo
bool Servo::attach(int _pin)
{
    pin = _pin;
    is_attached = true;

    // Configure the LEDC channel with the new PWM pin
    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {0} // doing this to avoid a warning
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    return true;
}

// write the angle in degrees to the servo
bool Servo::write(int angle)
{
    if (!this->is_attached || !this->can_rotate)
        return false;

    // clamp the angle to be written to within the max angle
    angle = clamp(angle, 0, max_angle);

    // update class field
    this->angle = angle;

    // calculate the pulse width in us and normalize it
    int pulse_width = angle_to_us(angle);

    // period of the pwm signal in microseconds
    int period = 1.0 / frequency * 1000000;
    int duty = (float)pulse_width / (float)period * (1 << (int)resolution);

    // set the new duty cycle and update the LEDC channel
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));

    // return successful write
    return true;
}

// write the pulse width in microseconds to the servo
bool Servo::write_us(int us)
{
    this->angle = us_to_angle(us);

    float pulse_width_normalized = normalize(us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

    // calculate the duty cycle using normalized pulse width
    int duty = pulse_width_normalized * (1 << (int)resolution);

    // set the new duty cycle and update the LEDC channel
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

    // return successful write
    return true;
}

// returns the angle in degrees of the servo in its current state
int Servo::read()
{
    return this->angle;
}

// returns the angle in microseconds with regards to the pwm width for the current angle
int Servo::read_us()
{
    return angle_to_us(this->angle);
}

// returns whether the servo is attached to a pin
bool Servo::attached()
{
    return this->is_attached;
}

void Servo::detach()
{
    this->is_attached = false;
    this->pin = -1;

    // stop the LEDC channel
    ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
}

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

enum dshot_mode_t
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
};
// rmt_tx_channel_config_t dshot_tx_rmt_config;

struct dshot_packet_t
{
    uint16_t throttle_value : 11;
    uint16_t telemetric_request : 1;
    uint16_t checksum : 4;
};

class SerialConnection
{
public:
    SerialConnectionConfig config;
    unsigned int data_rate;
};

Honeybee_ErrType process_crsf_frame(uint8_t *data, int buf_size, int si, CRSF_RC_ChannelData &channels)
{

    // print the data
    // for (int i = 0; i < 16; i++) {
    //     ESP_LOGI("CRSF", "data[%d]: %02X", i, data[i]);
    // }

    // first byte is sync byte, should be 0xC8
    uint8_t sync = data[si];

    // if the sync byte is not correct, return false
    if (sync != 0xC8)
        return HONEYBEE_ERR;

    // second byte is the length of the frame from [type] to [crc8] inclusive
    uint8_t frame_length = data[si + 1];

    // if there is not enough data to process the frame, return false
    if (si + frame_length >= buf_size)
        return HONEYBEE_ERR;

    // third byte is the type of the frame
    CRSF_FrameType frame_type = (CRSF_FrameType)data[si + 2];

    // calculate the crc8 of the frame
    // uint8_t crc8 = data[si + frame_length + 1];
    // uint8_t calculated_crc8 = calculate_crc8(data, si, frame_length + 2);

    // ESP_LOGI("CRSF", "crc8: %02X", crc8);
    // ESP_LOGI("CRSF", "calculated_crc8: %02X", calculated_crc8);

    // // reject the frame if the crc8 is incorrect
    // if (crc8 != calculated_crc8) return HONEYBEE_INVALID_CRC;

    // process the frame based on the type
    switch (frame_type)
    {
        case CRSF_FrameType::rc_channels_packed:
            channels.chan0 = (data[si + 4] << 8) | data[si + 3]; // roll
            channels.chan1 = (data[si + 5] << 5) | (data[si + 4] >> 3); // pitch
            channels.chan2 = (data[si + 7] << 10) | data[si + 6] << 2 | data[si + 5] >> 6; // throttle
            channels.chan3 = (data[si + 8] << 7) | (data[si + 7] >> 1); // yaw
            channels.chan4 = (data[si + 9] << 4) | (data[si + 8] >> 4); // button SA
            
            for (int i = 0; i < frame_length + 2; i++)
            {
                // printf("%02X ", data[si + i]);
            }
            // printf("\n");
            // printf("chan1: %d\n", channels.chan1);
            // printf("chan2: %d\n", channels.chan4);

            break;
        default:
            return HONEYBEE_ERR;
    }

    return HONEYBEE_OK;
}
// todo: map function for floats
float map(float value, float in_min, float in_max, float out_min, float out_max)
{
    float val = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return clamp(val, out_min, out_max);
}

#define CRSF_RC_CHANNE_MIN 174
#define CRSF_RC_CHANNE_MAX 1811

float get_yaw() {
    return map(channels.chan3, CRSF_RC_CHANNE_MIN, CRSF_RC_CHANNE_MAX, 0, 1);
}

float get_throttle() {
    return map(channels.chan2, CRSF_RC_CHANNE_MIN, CRSF_RC_CHANNE_MAX, 0, 1);
}

float get_pitch() {
    return map(channels.chan1, CRSF_RC_CHANNE_MIN, CRSF_RC_CHANNE_MAX, 0, 1);
}

float get_roll() {
    return map(channels.chan0, CRSF_RC_CHANNE_MIN, CRSF_RC_CHANNE_MAX, 0, 1);
}

int get_button_SA() {
    return (int) map(channels.chan4, 191, 1792, 0, 1);
}

void update_rc_channels(UART_ConnectionConfig data_config)
{
    unsigned int rx_buf_size = data_config.rx_buf_size;
    uint8_t data[rx_buf_size];

    int byte_count = ESPAbstraction::read_bytes(data_config.port, data, rx_buf_size);
    ESP_LOGI(main_TAG,"byte_count: %d", byte_count);

    for (int i = 0; i < byte_count; i++)
    {
        // attempt to process the frame and update the channels
        Honeybee_ErrType ret = process_crsf_frame(data, rx_buf_size, i, channels);

        // if the frame was processed successfully, skip to the next frame
        if (ret == HONEYBEE_OK)
        {
            int frame_length = data[i + 1] + 2; // entire frame length including sync and length bytes
            i += frame_length;
        }
        else
        {
            // if the frame was not processed successfully, log an error
            // ESP_LOGE(ELRSReceiver_TAG, "Error processing CRSF frame: %d", ret);
        }
    }
}

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

struct dshot_pause_packet_t
{
    uint16_t pause : DSHOT_PAUSE_LENGTH;
};


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

static void make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = frame->val;
    uint8_t crc = ((val ^ (val >> 4) ^ (val >> 8)) & 0xF0) >> 4;;
    frame->crc = crc;
    val = frame->val;
    // change the endian
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}


static size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                   const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = dshot_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = dshot_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    // convert user data into dshot frame
    dshot_esc_throttle_t *throttle = (dshot_esc_throttle_t *)primary_data;
    dshot_esc_frame_t frame = {};
    make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);

    switch (dshot_encoder->state) {
    case 0: // send the dshot frame
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &frame, sizeof(frame), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            dshot_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = (rmt_encode_state_t)((int)state |(int) RMT_ENCODING_MEM_FULL);
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &dshot_encoder->dshot_delay_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            state = (rmt_encode_state_t)((int)state|(int)RMT_ENCODING_COMPLETE);
            dshot_encoder->state = RMT_ENCODING_RESET; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state = (rmt_encode_state_t)((int)state | (int)RMT_ENCODING_MEM_FULL);
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_del_encoder(dshot_encoder->bytes_encoder);
    rmt_del_encoder(dshot_encoder->copy_encoder);
    free(dshot_encoder);
    return ESP_OK;
}

static esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
    rmt_encoder_reset(dshot_encoder->bytes_encoder);
    rmt_encoder_reset(dshot_encoder->copy_encoder);
    dshot_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    if (!config || !ret_encoder) {
        ESP_LOGE(main_TAG, "invalid argument");
        return ESP_ERR_INVALID_ARG;
    }

    rmt_dshot_esc_encoder_t *dshot_encoder = (rmt_dshot_esc_encoder_t *)rmt_alloc_encoder_mem(sizeof(rmt_dshot_esc_encoder_t));
    if (!dshot_encoder) {
        ESP_LOGE(main_TAG, "no mem for musical score encoder");
        return ESP_ERR_NO_MEM;
    }

    dshot_encoder->base.encode = rmt_encode_dshot_esc;
    dshot_encoder->base.del = rmt_del_dshot_encoder;
    dshot_encoder->base.reset = rmt_dshot_encoder_reset;

    uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
    rmt_symbol_word_t dshot_delay_symbol = {
        .duration0 = (uint16_t)(delay_ticks / 2),
        .level0 = 0,
        .duration1 = (uint16_t)(delay_ticks / 2),
        .level1 = 0,
        
    };
    dshot_encoder->dshot_delay_symbol = dshot_delay_symbol;

    float period_ticks = (float)config->resolution / config->baud_rate;
    unsigned int t1h_ticks = (unsigned int)(period_ticks * 0.7485);
    unsigned int t1l_ticks = (unsigned int)(period_ticks - t1h_ticks);
    unsigned int t0h_ticks = (unsigned int)(period_ticks * 0.37425);
    unsigned int t0l_ticks = (unsigned int)(period_ticks - t0h_ticks);

    rmt_bytes_encoder_config_t bytes_encoder_config = {};
    bytes_encoder_config.bit0.duration0 = t0h_ticks;
    bytes_encoder_config.bit0.duration1 = t0l_ticks;
    bytes_encoder_config.bit0.level0 = 1;
    bytes_encoder_config.bit0.level1 = 0;

    bytes_encoder_config.bit1.duration0 = t1h_ticks;
    bytes_encoder_config.bit1.duration1 = t1l_ticks;
    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.level1 = 0;
    bytes_encoder_config.flags.msb_first = 1;

    esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &dshot_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(main_TAG, "create bytes encoder failed");
        free(dshot_encoder);
        return ret;
    }

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &dshot_encoder->copy_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(main_TAG, "create copy encoder failed");
        rmt_del_encoder(dshot_encoder->bytes_encoder);
        free(dshot_encoder);
        return ret;
    }
    
    *ret_encoder = &dshot_encoder->base;
    return ESP_OK;
}


// Represents a single DShot channel.
class DShotChannel
{
    public:

        bool init(gpio_num_t gpio_num, dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);

        void send_throttle(uint16_t throttle_value);

        Honeybee_ErrType remove();

    private:
        int DShotChannels = 0;
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

bool DShotChannel::init(gpio_num_t gpio, dshot_mode_t dshot_mode, bool is_bidirectional)
{
    ESP_LOGI(main_TAG, "Create RMT TX channel");
    esc_chan = NULL;
    tx_chan_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .resolution_hz = DSHOT_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

    ESP_LOGI(main_TAG, "Install Dshot ESC encoder");
    dshot_encoder = NULL;
    encoder_config = {
        .resolution = DSHOT_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(main_TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    tx_config = {
        .loop_count = -1, // infinite loop
    };
    dshot_esc_throttle_t throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_LOGI(main_TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(3000));

    return true;
}

void DShotChannel::send_throttle(uint16_t throttle_value)
{
    throttle_value = (uint16_t) clamp(throttle_value, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    dshot_esc_throttle_t throttle = {
        .throttle = throttle_value,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    // the previous loop transfer is till undergoing, we need to stop it and restart,
    // so that the new throttle can be updated on the output
    ESP_ERROR_CHECK(rmt_disable(esc_chan));
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

}


// Honeybee_ErrType DShotChannel::remove()
// {
//     // Uninstall the RMT driver
//     ESP_ERROR_CHECK(del_chan);

//     // Return HONEYBEE_OK to indicate that the motor has been uninstalled successfully
//     return HONEYBEE_OK;
// }

// Calculates a CRC value for a DShot digital control signal packet
// uint8_t DShotChannel::calculateCRC(const dshot_packet_t &dshot_packet)
// {
//     uint8_t crc;

//     // Combine the throttle value and telemetric request flag into a 16-bit packet value
//     const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;

//     // Calculate the CRC value using different bitwise operations depending on the DShot configuration
//     if (dshot_config.is_bidirectional)
//     {
//         // Bidirectional configuration: perform a bitwise negation of the result of XORing the packet with its right-shifted values by 4 and 8 bits,
//         // and then bitwise AND the result with 0x0F
//         const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
//         crc = (~intermediate_result) & 0x0F;
//     }
//     else
//     {
//         // Unidirectional configuration: XOR the packet with its right-shifted values by 4 and 8 bits,
//         // and then bitwise AND the result with 0x0F
//         crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
//     }

//     // Return the calculated CRC value as a 16-bit unsigned integer
//     return crc;
// }

uint16_t DShotChannel::serialize_packet(const dshot_packet_t &dshot_packet)
{
    uint16_t parsedRmtPaket = DSHOT_NULL_PACKET;
    uint16_t crc = calculateCRC(dshot_packet);

    // Complete the packet
    parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
    parsedRmtPaket = (parsedRmtPaket << 4) | crc;

    return parsedRmtPaket;
}


i2c_master_bus_handle_t i2c_init_master_bus(gpio_num_t sda, gpio_num_t scl)
{
    i2c_master_bus_config_t esp_i2c_master_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = i2c_clock_source_t::I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags = {
            .enable_internal_pullup = false
        }
    };

    i2c_master_bus_handle_t esp_i2c_master;
    ESP_ERROR_CHECK(i2c_new_master_bus(&esp_i2c_master_config, &esp_i2c_master));

    return esp_i2c_master;
    
}

void i2c_master_write(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms = -1)
{
    esp_err_t ret = i2c_master_transmit(device, data, data_size, wait_ms);
    ESP_ERROR_CHECK(ret);
}

void i2c_master_read(i2c_master_dev_handle_t device, uint8_t *data, size_t data_size, uint32_t wait_ms = -1)
{
    esp_err_t ret = i2c_master_receive(device, data, data_size, wait_ms);
    ESP_ERROR_CHECK(ret);
}

void i2c_master_write_read(i2c_master_dev_handle_t device, uint8_t *write_data, size_t write_data_size, uint8_t *read_data, size_t read_data_size, uint32_t wait_ms = -1)
{
    esp_err_t ret = i2c_master_transmit_receive(device, write_data, write_data_size, read_data, read_data_size, wait_ms);
    ESP_ERROR_CHECK(ret);
}

void i2c_master_read_device_register(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms = -1)
{
    i2c_master_write_read(device, &reg_addr, 1, data, data_size, wait_ms);
}

void i2c_master_write_device_register(i2c_master_dev_handle_t device, uint8_t reg_addr, uint8_t *data, size_t data_size, uint32_t wait_ms = -1)
{
    i2c_master_write(device, &reg_addr, 1, wait_ms); // todo: reg_addr might be 2 bytes because 10 bit addresses
    i2c_master_write(device, data, data_size, wait_ms);
}

bool i2c_check_device_exists(i2c_master_bus_handle_t master_bus, uint8_t device_address, uint32_t wait_ms = -1)
{
    esp_err_t ret = i2c_master_probe(master_bus, device_address, wait_ms);
    ESP_ERROR_CHECK(ret);
    return ret == ESP_OK ? true : false;
}


i2c_master_dev_handle_t i2c_init_device(i2c_master_bus_handle_t master_bus, uint8_t device_address, i2c_addr_bit_len_t addr_len = I2C_ADDR_BIT_LEN_7, uint32_t speed_hz = 100000, uint32_t wait_us = 0)
{
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = speed_hz,
        .scl_wait_us = wait_us,
    };

    i2c_master_dev_handle_t i2c_device;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus, &i2c_device_config, &i2c_device));

    return i2c_device;
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
    master = i2c_init_master_bus(sda_pin, scl_pin);
    device = i2c_init_device(master, 0x68);

    power_on();
}

void ICM20948::update_axes() {
    // gyro
    uint8_t gyro_x_h;
    i2c_master_read_device_register(device, I2C_GYRO_XOUT_H_REG_ADDR, &gyro_x_h, 1);

    uint8_t gyro_x_l;
    i2c_master_read_device_register(device, I2C_GYRO_XOUT_L_REG_ADDR, &gyro_x_l, 1);

    gyro_x = (gyro_x_h << 8) | gyro_x_l;

    uint8_t gyro_y_h;
    i2c_master_read_device_register(device, I2C_GYRO_YOUT_H_REG_ADDR, &gyro_y_h, 1);

    uint8_t gyro_y_l;
    i2c_master_read_device_register(device, I2C_GYRO_YOUT_L_REG_ADDR, &gyro_y_l, 1);

    gyro_y = (gyro_y_h << 8) | gyro_y_l;

    uint8_t gyro_z_h;
    i2c_master_read_device_register(device, I2C_GYRO_ZOUT_H_REG_ADDR, &gyro_z_h, 1);

    uint8_t gyro_z_l;
    i2c_master_read_device_register(device, I2C_GYRO_ZOUT_L_REG_ADDR, &gyro_z_l, 1);

    gyro_z = (gyro_z_h << 8) | gyro_z_l;

    
    // mag
    uint8_t mag_x_h;
    i2c_master_read_device_register(device, I2C_MAG_XOUT_H_REG_ADDR, &mag_x_h, 1);

    uint8_t mag_x_l;
    i2c_master_read_device_register(device, I2C_MAG_XOUT_L_REG_ADDR, &mag_x_l, 1);

    mag_x = (mag_x_h << 8) | mag_x_l;

    uint8_t mag_y_h;
    i2c_master_read_device_register(device, I2C_MAG_YOUT_H_REG_ADDR, &mag_y_h, 1);

    uint8_t mag_y_l;
    i2c_master_read_device_register(device, I2C_MAG_YOUT_L_REG_ADDR, &mag_y_l, 1);

    mag_y = (mag_y_h << 8) | mag_y_l;

    uint8_t mag_z_h;
    i2c_master_read_device_register(device, I2C_MAG_ZOUT_H_REG_ADDR, &mag_z_h, 1);

    uint8_t mag_z_l;
    i2c_master_read_device_register(device, I2C_MAG_ZOUT_L_REG_ADDR, &mag_z_l, 1);

    mag_z = (mag_z_h << 8) | mag_z_l;


    ESP_LOGI(main_TAG, "Gyro X: %d\tGyro Y: %d\tGyro Z: %d\tMag X: %d\tMag Y: %d\tMag Z: %d", gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z);


}

void ICM20948::power_on()
{
// turn on the ICM
    uint8_t data = 0x01;
    i2c_master_write_device_register(device, I2C_PWR_MGMT_REG_ADDR, &data, 1);

    // turn on mag
    // uint8_t data_mag = 0x02;
    // i2c_master_write_device_register(device, 0x31, &data_mag, 1);
}

void ICM20948::power_off()
{
    uint8_t data = 0x00;
    i2c_master_write_device_register(device, I2C_PWR_MGMT_REG_ADDR, &data, 0);
}

class Vector2 {
    public:
        void set_x(float x);
        void set_y(float y);
        float get_x();
        float get_y();
        float dot(Vector2 other);
        Vector2 normal();
    private:
        float x, y;
};

float Vector2::dot(Vector2 other)
{
    return this->x * other.get_x() + this->y * other.get_y();
}

Vector2 Vector2::normal()
{
    float magnitude = sqrt(this->x * this->x + this->y * this->y);
    Vector2 ret;
    ret.set_x(this->x / magnitude);
    ret.set_y(this->y / magnitude);
    return ret;
}

void Vector2::set_x(float x)
{
    this->x = x;
}

void Vector2::set_y(float y)
{
    this->y = y;
}

Vector2::Vector2()
{
    this->x = 0;
    this->y = 0;
}

float Vector2::get_x()
{
    return this->x;
}

float Vector2::get_y()
{
    return this->y;
}

Vector2 operator+(Vector2 a, Vector2 b)
{
    Vector2 ret;
    ret.set_x(a.get_x() + b.get_x());
    ret.set_y(a.get_y() + b.get_y());
    return ret;
}

Vector2 operator-(Vector2 a, Vector2 b)
{
    Vector2 ret;
    ret.set_x(a.get_x() - b.get_x());
    ret.set_y(a.get_y() - b.get_y());
    return ret;
}

Vector2 operator*(Vector2 a, float b)
{
    Vector2 ret;
    ret.set_x(a.get_x() * b);
    ret.set_y(a.get_y() * b);
    return ret;
}

Vector2 operator*(float a, Vector2 b)
{
    Vector2 ret;
    ret.set_x(a * b.get_x());
    ret.set_y(a * b.get_y());
    return ret;
}

Vector2 operator/(Vector2 a, float b)
{
    Vector2 ret;
    ret.set_x(a.get_x() / b);
    ret.set_y(a.get_y() / b);
    return ret;
}

int max_roll_offset_angle = 45;

extern "C" void app_main(void)
{
    ESP_LOGI(main_TAG, "Starting main application");

    // Create an ELRS receiver
    UART_ConnectionConfig elrs_uart_connection;
    elrs_uart_connection.port = UART_NUM_0;
    elrs_uart_connection.rx_pin = 38;
    elrs_uart_connection.tx_pin = 37;
    elrs_uart_connection.rx_buf_size = 256;
    elrs_uart_connection.tx_buf_size = 256;
    elrs_uart_connection.type = SerialConnectionType::UART;
    SerialConnection elrs_connection;
    elrs_connection.config = elrs_uart_connection;
    elrs_connection.data_rate = 420000;
    ESPAbstraction::uart_install_connection(elrs_uart_connection.port, elrs_uart_connection.rx_pin, elrs_uart_connection.tx_pin, elrs_uart_connection.rx_buf_size, elrs_uart_connection.tx_buf_size, elrs_connection.data_rate);

    Servo FL_servo;
    FL_servo.init(180);
    FL_servo.attach(8);

    Servo FR_servo;
    FR_servo.init(180);
    FR_servo.attach(18);

    Servo BL_servo;
    BL_servo.init(180);
    BL_servo.attach(17);

    Servo BR_servo;
    BR_servo.init(180);
    BR_servo.attach(7);

    Servo CamTiltServo;
    CamTiltServo.init(180);
    CamTiltServo.attach(16);

    Servo CamPanServo;
    CamPanServo.init(180);
    CamPanServo.attach(15);

    DShotChannel motor1; // bottom left, spin right
    DShotChannel motor2; // top left, spin left
    DShotChannel motor3; // bottom right, spin left
    DShotChannel motor4; // top right, spin right
    motor1.init(GPIO_NUM_47, DSHOT300, false);
    motor2.init(GPIO_NUM_21, DSHOT300, false);
    motor3.init(GPIO_NUM_14, DSHOT300, false);
    motor4.init(GPIO_NUM_13, DSHOT300, false);
    
    ICM20948 icm;
    icm.init(GPIO_NUM_6, GPIO_NUM_5);


    while (true)
    {

        // Update the rc channels
        update_rc_channels(elrs_uart_connection);
        int pitch = get_pitch() * 180;
        int roll = (get_roll() - 0.5f) * 2 * max_roll_offset_angle;
        int dshot_throttle = map(get_throttle(), 0, 1, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        icm.update_axes();

        // ESP_LOGI(main_TAG, "Pitch: %d\tRoll: %d", channels.chan1, channels.chan0);

        motor1.send_throttle(dshot_throttle);
        motor2.send_throttle(dshot_throttle);
        motor3.send_throttle(dshot_throttle);
        motor4.send_throttle(dshot_throttle);



        FlightMode mode = (FlightMode) get_button_SA();
        switch (mode) {
            case VTOL:
                FL_servo.write(0);
                FR_servo.write(180);
                BL_servo.write(180);
                BR_servo.write(0);

                FL_servo.set_can_rotate(false);
                FR_servo.set_can_rotate(false);
                BL_servo.set_can_rotate(false);
                BR_servo.set_can_rotate(false);
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
                CamTiltServo.write(get_roll() * 180);
                CamPanServo.write(get_roll() * 180);
                break;
            default:
                break;
        }
        // ESP_LOGI(main_TAG, "Value: %d", value);
    }
}
