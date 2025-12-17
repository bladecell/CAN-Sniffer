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

#define DEBUG_MODE 1

static const char *TAG = "APP_MAIN";
static LedError led(LED_GPIO);

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_DEBUG);
    led.init();

    CanDriver::Config config = {
        .bitrate = CanDriver::Bitrate::BITRATE_500K,
        .tx_pin = CAN_TX_GPIO,
        .rx_pin = CAN_RX_GPIO,
        .lbk_pin = CAN_LBK_GPIO,
        .debug = DEBUG_MODE,
    };

    CanDriver::getInstance().init(config);

    if (!CanDriver::getInstance().isInitialized())
    {
        led.error();
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "CAN driver initialized");

    for (int delay = 5; delay > 0; delay--)
    {
        ESP_LOGI(TAG, "Starting OBD-II in %d...", delay);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    led.blink(2);

    OBD2 &obd2 = OBD2::getInstance();

    vTaskDelay(pdMS_TO_TICKS(2000));
    for (;;)
    {
        if (!CanDriver::getInstance().isBusConnected() || !obd2.isPidInit())
        {
            ESP_LOGW(TAG, "CAN bus is not connected or PID not initialized");
            led.blink(1);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        break;
    }
    ESP_LOGI(TAG, "PID OK");

    obd2.requestConfirmedDTCs();
    vTaskDelay(pdMS_TO_TICKS(3000));
    std::vector<std::string> dtc = obd2.getDTC(MODE_DTCS);

    if (dtc.empty())
    {
        ESP_LOGI("DTC", "No DTC active");
    }
    else
    {
        std::string all_dtc;
        for (const auto &d : dtc)
        {
            if (!all_dtc.empty())
                all_dtc += ", ";
            all_dtc += d;
        }
        ESP_LOGI("DTC", "%s", all_dtc.c_str());
    }

    // std::string vin;

    // bool vinflag = false;

    // obd2.startContinuousMode();
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // for (;;)
    // {
    //     if (!canDriver.isBusConnected() || !obd2.isPidInit())
    //     {
    //         ESP_LOGW(TAG, "CAN bus is not connected or PID not initialized");
    //         led.blink(1);
    //         vTaskDelay(pdMS_TO_TICKS(5000));
    //         continue;
    //     }
    //     else
    //     {
    //         led.off();
    //     }

    //     if (!vinflag)
    //     {
    //         obd2.requestVIN();
    //         ESP_LOGI(TAG, "Requesting VIN");
    //         vinflag = true;
    //     }

    //     if (!vin.empty())
    //     {
    //         ESP_LOGI("VIN", "%s", vin.c_str());
    //     }
    //     else
    //     {
    //         vin = obd2.getVIN();
    //     }

    //     float rpm = obd2.getValue(PID_ENGINE_RPM);
    //     float load = obd2.getValue(PID_ENGINE_LOAD);
    //     float temp = obd2.getValue(PID_COOLANT_TEMP);

    //     // Get the units for a more informative log
    //     const char *rpm_unit = obd2.getUnit(PID_ENGINE_RPM);
    //     const char *load_unit = obd2.getUnit(PID_ENGINE_LOAD);
    //     const char *temp_unit = obd2.getUnit(PID_COOLANT_TEMP);

    //     ESP_LOGI("Live Data", "RPM: %5.0f %s | Load: %6.2f %s | Temp: %3.0f %s",
    //              rpm, rpm_unit,
    //              load, load_unit,
    //              temp, temp_unit);

    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}