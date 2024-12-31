#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"

#define DSHOT_PACKET_LENGTH 17 // last bit is for pause
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_PAUSE_LENGTH 21 // 21 bit pause is recommended
#define DSHOT_NULL_PACKET 0b0000000000000000
#define DSHOT_CLK_DIVIDER 8 // i guess this reduces the frequency to something that dshot can handle. need to look into this more.
#define DSHOT_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution

namespace honeybee_dshot {
    

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
    class dshot_connection_t
    {
        public:

            bool init(gpio_num_t gpio_num, dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);

            void send_throttle(uint16_t throttle_value);

            void remove();

        private:
            int dshot_connection_ts = 0;
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