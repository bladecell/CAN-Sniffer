#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#define CAN_TX_GPIO GPIO_NUM_4
#define CAN_RX_GPIO GPIO_NUM_5
#define OBD_PID_COOLANT_TEMP 0x05

static const char *TAG = "CAN_OBD";

extern "C" {

    void checkTwaiRequest(esp_err_t result) {
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Request sent");
        } else {
            ESP_LOGE(TAG, "Failed to send request. Error: ");
            switch (result) {
                case ESP_ERR_INVALID_ARG:
                    ESP_LOGE(TAG, "ESP_ERR_INVALID_ARG - Invalid argument");
                    break;
                case ESP_ERR_TIMEOUT:
                    ESP_LOGE(TAG, "ESP_ERR_TIMEOUT - Timed out waiting for space on TX queue");
                    break;
                case ESP_ERR_INVALID_STATE:
                    ESP_LOGE(TAG, "ESP_ERR_INVALID_STATE - TWAI driver not in running state or TX queue disabled");
                    break;
                case ESP_ERR_NOT_SUPPORTED:
                    ESP_LOGE(TAG, "ESP_ERR_NOT_SUPPORTED - Listen only mode");
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown error code: 0x%X", result);
                    break;
            }
            
            // Check TWAI driver status for additional debugging
            twai_status_info_t status_info;
            if (twai_get_status_info(&status_info) == ESP_OK) {
                ESP_LOGI(TAG, "TWAI Status - State: %d, TX Queue msgs: %d, RX Queue msgs: %d", 
                            status_info.state, 
                            status_info.msgs_to_tx, status_info.msgs_to_rx);
                ESP_LOGI(TAG, "TX Error Count: %d, RX Error Count: %d", 
                            status_info.tx_error_counter, status_info.rx_error_counter);
                ESP_LOGI(TAG, "Bus Error Count: %d, Arbitration Lost: %d",
                            status_info.bus_error_count, status_info.arb_lost_count);
            }
        }
    }

    void requestCoolantTemp(void) {
        twai_message_t msgTx;

        msgTx.identifier = 0x7DF;
        msgTx.data_length_code = 8;
        msgTx.flags = 0;

        msgTx.data[0] = 0x02;            // Number of additional bytes
        msgTx.data[1] = 0x01;            // Mode 01 - Show current data
        msgTx.data[2] = OBD_PID_COOLANT_TEMP; // PID 05 - Engine Coolant Temperature
        msgTx.data[3] = 0x00;            // Padding
        msgTx.data[4] = 0x00;            // Padding
        msgTx.data[5] = 0x00;            // Padding
        msgTx.data[6] = 0x00;            // Padding
        msgTx.data[7] = 0x00;            // Padding

        esp_err_t result = twai_transmit(&msgTx, pdMS_TO_TICKS(1000));
        checkTwaiRequest(result);

    }

    float parseCoolantTempResponse(twai_message_t* message) {
        // OBD-II response format for coolant temp:
        // [Length, Mode+0x40, PID, Data_A, ...]
        // Temperature = Data_A - 40 (in Celsius)
        
        if (message->data_length_code >= 4 && 
            message->data[1] == 0x41 &&  // Response mode (0x01 + 0x40)
            message->data[2] == OBD_PID_COOLANT_TEMP) {
            
            float tempCelsius = message->data[3] - 40;
            
            ESP_LOGI(TAG, "Coolant Temperature: %.1f°C", tempCelsius);
            return tempCelsius;
        }
        
        return -999; // Invalid reading
    }

    void parseTwaiResponse(twai_message_t* message) {
        if (message->identifier >= 0x7E8 && message->identifier <= 0x7EF) {
            // This is likely an OBD-II response (ECU response IDs)
            float temp = parseCoolantTempResponse(message);
            if (temp != -999) {
                ESP_LOGI(TAG, "✓ Coolant temperature parsed successfully");
            }
        }
    }

    void can_task(void *pvParameters) {
        TickType_t lastRequestTime = 0;
        TickType_t lastStatusCheck = 0;
        twai_message_t message;
        
        while (1) {
            TickType_t currentTime = xTaskGetTickCount();
            
            // Check and recover from BUS-OFF state every 2 seconds
            if ((currentTime - lastStatusCheck) > pdMS_TO_TICKS(2000)) {
                twai_status_info_t status_info;
                if (twai_get_status_info(&status_info) == ESP_OK) {
                    if (status_info.state == TWAI_STATE_BUS_OFF) {
                        ESP_LOGW(TAG, "TWAI in BUS-OFF state. Attempting recovery...");
                        twai_initiate_recovery();
                        vTaskDelay(pdMS_TO_TICKS(100));
                        ESP_LOGI(TAG, "Recovery attempt completed");
                    } else if (status_info.state != TWAI_STATE_RUNNING) {
                        ESP_LOGW(TAG, "TWAI state: %d (not running)", status_info.state);
                    }
                }
                lastStatusCheck = currentTime;
            }
            
            // Only send requests when driver is in running state
            twai_status_info_t current_status;
            if (twai_get_status_info(&current_status) == ESP_OK && current_status.state == TWAI_STATE_RUNNING) {
                // Send coolant temperature request every 5 seconds
                if ((currentTime - lastRequestTime) > pdMS_TO_TICKS(5000)) {
                    requestCoolantTemp();
                    lastRequestTime = currentTime;
                }
            }
            
            // Listen for responses
            if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
                ESP_LOGI(TAG, "Received CAN message: ID=0x%03X DLC=%d Data=", 
                        message.identifier, message.data_length_code);
                
                // Print data bytes
                char data_str[64] = {0};
                for (int i = 0; i < message.data_length_code; i++) {
                    sprintf(data_str + strlen(data_str), "%02X ", message.data[i]);
                }
                ESP_LOGI(TAG, "Data: %s", data_str);
                
                parseTwaiResponse(&message);
            }
            
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent overwhelming the CAN bus
        }
    }

    void init_twai() {
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        // Install and start TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            ESP_LOGI(TAG, "TWAI driver installed");
        } else {
            ESP_LOGE(TAG, "Failed to install TWAI driver");
            return;
        }
        
        if (twai_start() == ESP_OK) {
            ESP_LOGI(TAG, "TWAI driver started. Ready for OBD-II communication...");
        } else {
            ESP_LOGE(TAG, "Failed to start TWAI driver");
            return;
        }
    }

    void app_main(void)
    {
        init_twai();
        // Create CAN task
        xTaskCreate(can_task, "can_task", 4096, NULL, 5, NULL);
        
        ESP_LOGI(TAG, "CAN OBD-II application started");
    }
}