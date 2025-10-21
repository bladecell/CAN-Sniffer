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

    for(int delay = 10; delay > 0; delay--) {
        ESP_LOGI(TAG, "Starting OBD-II in %d...", delay);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    led.blink(2);

    OBD2 obd2(canDriver);
    obd2.init();

    for(;;) {
        
        obd2.req(PID_ENGINE_RPM);
        obd2.req(PID_ENGINE_LOAD);
        obd2.req(PID_COOLANT_TEMP);

        if (obd2.isValid(PID_ENGINE_RPM)) {
            float rpm = obd2.getValue(PID_ENGINE_RPM);
            ESP_LOGI(TAG, "Engine RPM: %.2f %s", rpm, obd2.getUnit(PID_ENGINE_RPM));
        } else {
            ESP_LOGW(TAG, "Engine RPM data is not valid");
        }

        if (obd2.isValid(PID_ENGINE_LOAD)) {
            float load = obd2.getValue(PID_ENGINE_LOAD);
            ESP_LOGI(TAG, "Engine Load: %.2f %s", load, obd2.getUnit(PID_ENGINE_LOAD));
        } else {
            ESP_LOGW(TAG, "Engine Load data is not valid");
        }

        if (obd2.isValid(PID_COOLANT_TEMP)) {
            float temp = obd2.getValue(PID_COOLANT_TEMP);
            ESP_LOGI(TAG, "Coolant Temperature: %.2f %s", temp, obd2.getUnit(PID_COOLANT_TEMP));
        } else {
            ESP_LOGW(TAG, "Coolant Temperature data is not valid");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

