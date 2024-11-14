extern "C" {
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "driver/uart.h"
}

#include "honeybee_common_types.h"
#include "uart_peripheral.h"
#include "elrs_receiver.h"
#include "Servo.h"

// The structure of a CRSF frame is as follows:
//[sync] [len] [type] [payload] [crc8]
// sync: marks the beginning of a frame, always 0xC8
// len: the number of bytes from [type] to [crc8] inclusive.
// type: the type of the frame, e.g. 0x16 for RC channels
// payload: the data of the frame
// crc8: a checksum of the frame

#define main_TAG "main"

extern "C" void app_main(void)
{
    elrs_receiver receiver = elrs_receiver(UART_NUM_1, 41, 42);
    Servo servo = Servo(180);

    while (1) {
        receiver.receive_from_buffer();
        int angle = (receiver.get_rc_channel_data().chan0 - 900) / 5;
        servo.write(angle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
