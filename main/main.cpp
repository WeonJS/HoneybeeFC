extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "esp_system.h"
    #include "esp_log.h"
}

#include "ELRSReceiver.h"
#include "ThrustActuator.h"

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
    ESP_LOGI(main_TAG, "Starting main application");
    Drone drone;

    // Initialize the ELRS receiver
    ELRSReceiver receiver;
    UART_ConnectionConfig config 
    {
        .port = UART_NUM_1,
        .rx_pin = 41,
        .tx_pin = 42,
        .rx_buf_size = 1024,
        .tx_buf_size = 1024,
    };
    config.type = SerialConnectionType::UART;
    receiver.data_config = config;
    receiver.init();
    drone.add_subsystem(&receiver);


    // Thrust actuating subsystem
    ThrustActuator actuator;
    actuator.init();
    actuator.attach_servo(ActuatorPosition::FORWARD_LEFT, 5);
    actuator.attach_servo(ActuatorPosition::FORWARD_RIGHT, 6);
    actuator.attach_servo(ActuatorPosition::BACK_LEFT, 7);
    actuator.attach_servo(ActuatorPosition::BACK_RIGHT, 8);
    drone.add_subsystem(&actuator);

    drone.set_state(RUNNING);

    while (drone.get_state() == RUNNING)
    {
        drone.update();
    }
    

    
}
