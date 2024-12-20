extern "C" {
    #include "esp_log.h"
}
#include "ELRSReceiver.h"
#include "ESPAbstraction.h"

#define ELRSReceiver_TAG "ELRSReceiver"

/// @brief Should be called to initialize the ELRS receiver, after appropriate fields have been set.
void ELRSReceiver::init() 
{
    // Set the serial connection's config to our data config
    data_connection.config = data_config;
    data_connection.data_rate = 420000;

    // Install the UART device
    ESPAbstraction::uart_install_connection(data_config.port, data_config.rx_pin, data_config.tx_pin, 1024, 1024, data_connection.data_rate);
}

void ELRSReceiver::update() 
{
    // Update the rc channels
    update_rc_channels();
    ESP_LOGI(ELRSReceiver_TAG, "Channel 0: %d", rc_channels.chan0);
}

CRSF_RC_ChannelData ELRSReceiver::get_rc_channels()
{
    return rc_channels;
}

void ELRSReceiver::update_rc_channels()
{
    unsigned int rx_buf_size = data_config.rx_buf_size;
    uint8_t data[rx_buf_size];
    
    int byte_count = ESPAbstraction::read_bytes(data_config.port, data, rx_buf_size);
    CRSF_RC_ChannelData channels;

    for (int i = 0; i < byte_count; i++)
    {
        // attempt to process the frame and update the channels
        Honeybee_ErrType ret = process_crsf_frame(data, rx_buf_size, i, channels);

        // if the frame was processed successfully, skip to the next frame
        if (ret == HONEYBEE_OK)
        {
            int frame_length = data[i + 1] + 2;
            i += frame_length;
        } else {
            // if the frame was not processed successfully, log an error
            //ESP_LOGE(ELRSReceiver_TAG, "Error processing CRSF frame: %d", ret);
        }
    }

    // update the class field
    rc_channels = channels;
}