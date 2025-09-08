#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

#include "can_driver.hpp"

// -----------------------------------------------------------------------------
// Pin configuration (ensure these match your wiring)
#define CAN_TX_GPIO GPIO_NUM_4
#define CAN_RX_GPIO GPIO_NUM_5
#define LED_GPIO    GPIO_NUM_47
// -----------------------------------------------------------------------------

#define OBD_PID_COOLANT_TEMP 0x05
static const char *TAG = "CAN_OBD";

// // ----- OBD helpers -----------------------------------------------------------
// static twai_message_t requestCoolantTempMag(void) {
//     twai_message_t msgTx{};
//     msgTx.identifier = 0x7DF;             // OBD-II functional request ID
//     msgTx.data_length_code = 8;
//     msgTx.flags = 0;

//     msgTx.data[0] = 0x02;                 // Number of additional data bytes
//     msgTx.data[1] = 0x01;                 // Mode 01 - current data
//     msgTx.data[2] = OBD_PID_COOLANT_TEMP; // PID 05 - coolant temp
//     // remaining bytes are already zeroed by aggregate initialization
//     return msgTx;
// }

// static float parseCoolantTempResponse(const twai_message_t* message) {
//     // Response format: [Len, 0x41, PID, A, ...]; Temp(C) = A - 40
//     if (message->data_length_code >= 4 &&
//         message->data[1] == 0x41 &&
//         message->data[2] == OBD_PID_COOLANT_TEMP) {
//         return (float)message->data[3] - 40.0f;
//     }
//     return NAN;
// }

// static void parseTwaiResponse(const twai_message_t* message) {
//     // Typical ECU response IDs are 0x7E8–0x7EF
//     if (message->identifier >= 0x7E8 && message->identifier <= 0x7EF) {
//         float t = parseCoolantTempResponse(message);
//         if (!isnan(t)) {
//             ESP_LOGI(TAG, "Coolant Temperature: %.1f °C", t);
//         }
//     }
// }



// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void) {
    CanDriver canDriver(CanDriver::Bitrate::BITRATE_500K, CAN_TX_GPIO, CAN_RX_GPIO);
    ESP_ERROR_CHECK(canDriver.init());
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for CAN controller to stabilize
    ESP_LOGI(TAG, "CAN driver initialized");

    //Set LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_GPIO, 1);    

    twai_node_status_t status = canDriver.getStatus();
    for(;;){
        ESP_LOGI(TAG, "CAN Status - State: %d, RX Error Count: %d, TX Error Count: %d",
                status.state,
                status.rx_error_count,
                status.tx_error_count);
        gpio_set_level(LED_GPIO, gpio_get_level(LED_GPIO) ^ 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
