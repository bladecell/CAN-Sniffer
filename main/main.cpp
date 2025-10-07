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

// PID definitions
#define OBD_PID_COOLANT_TEMP    0x05
#define OBD_MODE_CURRENT_DATA   0x01
#define OBD_MODE_DTC_REQUEST    0x03
#define OBD_MODE_CLEAR_DTC      0x04
#define OBD_FUNCTIONAL_ID       0x7DF

// OBD-II response mode definitions
#define OBD_RESPONSE_CURRENT_DATA   0x41  // Response to mode 01
#define OBD_RESPONSE_FREEZE_FRAME   0x42  // Response to mode 02
#define OBD_RESPONSE_STORED_DTC     0x43  // Response to mode 03
#define OBD_RESPONSE_CLEAR_DTC      0x44  // Response to mode 04
#define OBD_RESPONSE_OXYGEN_TEST    0x45  // Response to mode 05
#define OBD_RESPONSE_PENDING_DTC    0x47  // Response to mode 07

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
    frame.header.id = OBD_FUNCTIONAL_ID;                // OBD-II functional request ID
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

static void decodeDTC(uint8_t byte1, uint8_t byte2, char* dtc_string) {
    // Extract the first character based on the first 2 bits of byte1
    char first_char;
    switch ((byte1 >> 6) & 0x03) {
        case 0: first_char = 'P'; break;  // Powertrain
        case 1: first_char = 'C'; break;  // Chassis
        case 2: first_char = 'B'; break;  // Body
        case 3: first_char = 'U'; break;  // Network/User
        default: first_char = '?'; break;
    }
    
    // Extract the remaining 14 bits to form the 4-digit hex code
    uint16_t code = ((byte1 & 0x3F) << 8) | byte2;
    snprintf(dtc_string, 6, "%c%04X", first_char, code);
}

static void parseDTCResponse(const CanDriver::can_frame* message) {
    if (message->length < 3) return;
    
    uint8_t length = message->data[0];
    uint8_t mode = message->data[1];
    
    if (mode == OBD_RESPONSE_STORED_DTC) {
        if (length < 2) {
            ESP_LOGI(TAG, "No stored DTCs");
            return;
        }
        
        uint8_t dtc_count = message->data[2];
        ESP_LOGI(TAG, "Number of stored DTCs: %d", dtc_count);
        
        // Each DTC is 2 bytes, parse available DTCs in this frame
        int available_dtcs = (message->length - 3) / 2;
        int dtcs_to_parse = (dtc_count < available_dtcs) ? dtc_count : available_dtcs;
        
        for (int i = 0; i < dtcs_to_parse; i++) {
            if (3 + (i * 2) + 1 < message->length) {
                char dtc_string[6];
                decodeDTC(message->data[3 + (i * 2)], message->data[4 + (i * 2)], dtc_string);
                ESP_LOGI(TAG, "DTC %d: %s", i + 1, dtc_string);
            }
        }
        
        // If there are more DTCs than fit in one frame, they'll come in subsequent frames
        if (dtc_count > available_dtcs) {
            ESP_LOGI(TAG, "More DTCs expected in following frames...");
        }
    } else if (mode == OBD_RESPONSE_PENDING_DTC) {
        uint8_t dtc_count = message->data[2];
        ESP_LOGI(TAG, "Number of pending DTCs: %d", dtc_count);
        
        // Parse pending DTCs similar to stored DTCs
        int available_dtcs = (message->length - 3) / 2;
        int dtcs_to_parse = (dtc_count < available_dtcs) ? dtc_count : available_dtcs;
        
        for (int i = 0; i < dtcs_to_parse; i++) {
            if (3 + (i * 2) + 1 < message->length) {
                char dtc_string[6];
                decodeDTC(message->data[3 + (i * 2)], message->data[4 + (i * 2)], dtc_string);
                ESP_LOGI(TAG, "Pending DTC %d: %s", i + 1, dtc_string);
            }
        }
    }
}

static void parseTwaiResponse(const CanDriver::can_frame* message) {
    // Typical ECU response IDs are 0x7E8–0x7EF
    if (message->id >= 0x7E8 && message->id <= 0x7EF) {
        
        if (message->length < 2) return;
        
        uint8_t mode = message->data[1];
        
        switch (mode) {
            case OBD_RESPONSE_STORED_DTC:
            case OBD_RESPONSE_PENDING_DTC:
                parseDTCResponse(message);
                break;
                
            case OBD_RESPONSE_CLEAR_DTC:
                ESP_LOGI(TAG, "DTCs cleared successfully");
                break;
        }
    }
}


// Function to request stored DTCs
static twai_frame_t requestDTCFrame(void) {
    // Static buffer to hold the message data
    static uint8_t tx_data[8] = {0};
    
    // Prepare the data for DTC request
    tx_data[0] = 0x01;                 // Number of additional data bytes
    tx_data[1] = OBD_MODE_DTC_REQUEST; // Mode 03 - request stored DTCs
    // Remaining bytes are already zeroed
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    twai_frame_t frame = {};
    
    // Set up the header
    frame.header.id = OBD_FUNCTIONAL_ID;        // OBD-II functional request ID
    frame.header.dlc = twaifd_len2dlc(sizeof(tx_data));
    frame.header.ide = false;                   // Standard Frame Format (11-bit ID)
    frame.header.rtr = 0;                       // Data frame (not remote frame)
    frame.header.fdf = 0;                       // Classic CAN format
    frame.header.brs = 0;                       // No bit rate switching
    frame.header.esi = 0;                       // No error state indicator
    frame.header.timestamp = 0;                 // Not used for TX
    frame.header.trigger_time = 0;              // Not used for immediate transmission
    
    // Set up the buffer
    frame.buffer = tx_data;
    frame.buffer_len = sizeof(tx_data);
    
    return frame;
}

// Function to request clear stored DTCs
static twai_frame_t requestClearStoredDTC(void) {
    // Static buffer to hold the message data
    static uint8_t tx_data[8] = {0};
    
    // Prepare the data for DTC request
    tx_data[0] = 0x01;                 // Number of additional data bytes
    tx_data[1] = OBD_MODE_CLEAR_DTC; // Mode 03 - request stored DTCs
    // Remaining bytes are already zeroed
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
    
    twai_frame_t frame = {};
    
    // Set up the header
    frame.header.id = OBD_FUNCTIONAL_ID;        // OBD-II functional request ID
    frame.header.dlc = twaifd_len2dlc(sizeof(tx_data));
    frame.header.ide = false;                   // Standard Frame Format (11-bit ID)
    frame.header.rtr = 0;                       // Data frame (not remote frame)
    frame.header.fdf = 0;                       // Classic CAN format
    frame.header.brs = 0;                       // No bit rate switching
    frame.header.esi = 0;                       // No error state indicator
    frame.header.timestamp = 0;                 // Not used for TX
    frame.header.trigger_time = 0;              // Not used for immediate transmission
    
    // Set up the buffer
    frame.buffer = tx_data;
    frame.buffer_len = sizeof(tx_data);
    
    return frame;
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
        
        rx_frame = {};
        
        if (canDriver->receive(rx_frame, 100) == ESP_OK) {
            // ESP_LOGI("CAN", "RX id=0x%03lX dlc=%d", rx_frame.id, rx_frame.length);
            // for (int i = 0; i < rx_frame.length; ++i) {
            //     ESP_LOGI("CAN", "  d[%d]=0x%02X", i, rx_frame.data[i]);
            // }
            parseTwaiResponse(&rx_frame);
            led.blink(2);
        } /*else {
            ESP_LOGI("CAN", "No message received (timeout)");
        }*/
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
    xTaskCreate(recvTask, "recvTask", 4096, &canDriver, 5, nullptr);

    ESP_LOGI(TAG, "CAN-Sniffer Running");
    twai_frame_t msgTx = requestClearStoredDTC();
    ret = canDriver.transmit(&msgTx, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "OBD request sent");
        led.blink();
    }
    vTaskDelay(pdMS_TO_TICKS(10000));

    msgTx = requestDTCFrame();
    for (;;) {
        esp_err_t ret = canDriver.transmit(&msgTx, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "OBD request sent");
            led.blink();
        }

        // CanDriver::can_frame rx_frame;
        
        // if (canDriver.receive(rx_frame, 1000) == ESP_OK) {
        //     ESP_LOGI("CAN", "RX id=0x%03lX dlc=%d", rx_frame.id, rx_frame.length);
        //     for (int i = 0; i < rx_frame.length; ++i) {
        //         ESP_LOGI("CAN", "  d[%d]=0x%02X", i, rx_frame.data[i]);
        //     }
        //     parseTwaiResponse(&rx_frame);
        //     led.blink(3);
        // } else {
        //     ESP_LOGI("CAN", "No message received (timeout)");
        // }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

