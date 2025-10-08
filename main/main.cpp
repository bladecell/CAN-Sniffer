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
#include "obd2.hpp"


static const char *TAG = "APP_MAIN";
static LedError led(LED_GPIO);

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_DEBUG);
    led.init();

    CanDriver canDriver(CanDriver::Bitrate::BITRATE_500K, CAN_TX_GPIO, CAN_RX_GPIO, CAN_LBK_GPIO);
    // Initialize the CAN driver
    canDriver.debug_mode(false); // Enable loopback for testing
    esp_err_t ret = canDriver.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %s", esp_err_to_name(ret));
        led.error();
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 

    ESP_LOGI(TAG, "CAN driver initialized");

    led.blink(2);

    OBD2 obd2(canDriver);
    obd2.init();

    for(;;) {
        obd2.query_supported_pids(PID_PIDS_SUPPORTED_1_20);
        if (obd2.is_supported(PID_ENGINE_RPM)) {
            ESP_LOGI(TAG, "Engine RPM PID is supported");
        } else {
            ESP_LOGI(TAG, "Engine RPM PID is NOT supported");
        }
        if (obd2.is_supported(PID_ENGINE_LOAD)) {
            ESP_LOGI(TAG, "Engine Load PID is supported");
        } else {
            ESP_LOGI(TAG, "Engine Load PID is NOT supported");
        }
        if (obd2.is_supported(PID_COOLANT_TEMP)) {
            ESP_LOGI(TAG, "Coolant Temp PID is supported");
        } else {
            ESP_LOGI(TAG, "Coolant Temp PID is NOT supported");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    


}

