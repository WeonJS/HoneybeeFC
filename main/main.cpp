extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
}

#include "honeybee_common_types.h"
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
    Servo servo = Servo(180, LEDC_CHANNEL_0);
    servo.attach(5);

}
