// main.cpp
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
#include "utilities.h"
#include "led_status.hpp"

#define OBD_PID_COOLANT_TEMP 0x05
static const char *TAG = "CAN_OBD";
static LedError led(LED_GPIO);

// ----- OBD helpers -----------------------------------------------------------
static twai_frame_t requestCoolantTempFrame(void) {
    // Static buffer to hold the message data
    static uint8_t tx_data[8] = {0};
    
    // Prepare the data
    tx_data[0] = 0x02;                 // Number of additional data bytes
    tx_data[1] = 0x01;                 // Mode 01 - current data
    tx_data[2] = OBD_PID_COOLANT_TEMP; // PID 05 - coolant temp
    // Remaining bytes are already zeroed
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    twai_frame_t frame = {};
    
    // Set up the header
    frame.header.id = 0x7DF;                // OBD-II functional request ID
    frame.header.dlc = twaifd_len2dlc(sizeof(tx_data));   // Data length code
    frame.header.ide = false;                   // Standard Frame Format (11-bit ID)
    frame.header.rtr = 0;                   // Data frame (not remote frame)
    frame.header.fdf = 0;                   // Classic CAN format
    frame.header.brs = 0;                   // No bit rate switching
    frame.header.esi = 0;                   // No error state indicator
    frame.header.timestamp = 0;             // Not used for TX
    frame.header.trigger_time = 0;          // Not used for immediate transmission
    
    // Set up the buffer
    frame.buffer = tx_data;
    frame.buffer_len = sizeof(tx_data);
    
    return frame;
}

static float parseCoolantTempResponse(const CanDriver::can_frame* message) {
    // Check if the frame is valid and has enough data to process
    if (message->length >= 4 &&
        message->data[1] == 0x41 &&
        message->data[2] == OBD_PID_COOLANT_TEMP) {
        // Return temperature in Celsius based on the formula: Temp(C) = A - 40
        return (float)message->data[3] - 40.0f;
    }
    return NAN;  // Return Not-a-Number (NAN) if conditions are not met
}

static void parseTwaiResponse(const CanDriver::can_frame* message) {
    // Typical ECU response IDs are 0x7E8–0x7EF
    if (message->id >= 0x7E8 && message->id <= 0x7EF) {
        // Parse coolant temperature response
        float t = parseCoolantTempResponse(message);
        if (!isnan(t)) {
            ESP_LOGI(TAG, "Coolant Temperature: %.1f °C", t);
        }
    }
}

void sendTask(void* param) {
    CanDriver* canDriver = static_cast<CanDriver*>(param);
    twai_frame_t msgTx = requestCoolantTempFrame();
    for (;;) {
        esp_err_t ret = canDriver->transmit(&msgTx, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "OBD request sent");
            // led.blink();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void recvTask(void* param) {
    CanDriver* canDriver = static_cast<CanDriver*>(param);
    CanDriver::can_frame rx_frame;

    for (;;) {
        
        if (canDriver->receive(rx_frame, 1000) == ESP_OK) {
            ESP_LOGI("CAN", "Received frame ID: 0x%lx, DLC: %d", 
                     rx_frame.id, rx_frame.length);
            parseTwaiResponse(&rx_frame);
            // led.blink(3);
        } else {
            ESP_LOGI("CAN", "No message received (timeout)");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void) {
    led.init();

    CanDriver canDriver(CanDriver::Bitrate::BITRATE_500K, CAN_TX_GPIO, CAN_RX_GPIO, CAN_LBK_GPIO);
    canDriver.debug_mode(false);
    // Initialize the CAN driver
    esp_err_t ret = canDriver.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %s", esp_err_to_name(ret));
        led.error();
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 

    ESP_LOGI(TAG, "CAN driver initialized");

    led.blink(2);

    // xTaskCreate(sendTask, "sendTask", 4096, &canDriver, 5, nullptr);
    // xTaskCreate(recvTask, "recvTask", 4096, &canDriver, 5, nullptr);

    ESP_LOGI(TAG, "CAN-Sniffer Running");

    twai_frame_t msgTx = requestCoolantTempFrame();
    for (;;) {
        esp_err_t ret = canDriver.transmit(&msgTx, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "OBD request sent");
            led.blink();
        }

        CanDriver::can_frame rx_frame;
        
        if (canDriver.receive(rx_frame, 1000) == ESP_OK) {
            ESP_LOGI("CAN", "RX id=0x%03lX dlc=%d", rx_frame.id, rx_frame.length);
            for (int i = 0; i < rx_frame.length; ++i) {
                ESP_LOGI("CAN", "  d[%d]=0x%02X", i, rx_frame.data[i]);
            }
            parseTwaiResponse(&rx_frame);
            led.blink(3);
        } else {
            ESP_LOGI("CAN", "No message received (timeout)");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

