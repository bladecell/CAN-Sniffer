// main.cpp
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

#include "can_driver.hpp"
#include "utilities.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "settings.hpp"
#include "wifi.hpp"
#include "async_web_server.hpp"
#include "secrets.h"
#include "webserver.hpp"

#define DEBUG_MODE 1

static const char *TAG = "APP_MAIN";
static LedError led(LED_GPIO);

esp_err_t setup_wifi()
{
    // Configure
    WIFI::Config config;
    config.ssid = "CAN-SNIFFER-AP";
    config.password = "";
    config.channel = 6;
    config.max_connections = 4;
    config.auth_mode = WIFI_AUTH_OPEN;
    config.mode = WIFI_MODE_STA;
    config.sta_ssid = WIFI_SSID;
    config.sta_password = WIFI_PASSWORD;
    config.sta_auth_mode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = WIFI::getInstance().init(config);
    ret |= WIFI::getInstance().start();
    return ret;
}

esp_err_t setup_can()
{
    CanDriver::Config config;
    config.bitrate = CanDriver::Bitrate::BITRATE_500K;
    config.rx_pin = CAN_RX_GPIO;
    config.tx_pin = CAN_TX_GPIO;
    config.lbk_pin = CAN_LBK_GPIO;
    config.rs_pin = CAN_RS_GPIO;
    config.debug = DEBUG_MODE;
    config.filter = false;
    config.mfilter_cfg.id = 0b011100000000;   // 0b111 11100000
    config.mfilter_cfg.mask = 0b011100000000; // 0b111 11100000
    config.mfilter_cfg.is_ext = false;        // Standard 11-bit IDs

    // Settings::getInstance().getCanConfig(config);

    // config.debug = DEBUG_MODE;

    esp_err_t ret = CanDriver::getInstance().init(config);

    vTaskDelay(pdMS_TO_TICKS(1000));
    if (ret != ESP_OK || !CanDriver::getInstance().isInitialized())
    {
        ESP_LOGE(TAG, "Failed to initialize CAN driver");
        return ret;
    }

    ESP_LOGI(TAG, "CAN driver initialized");

    return ESP_OK;
}

esp_err_t setup_obd()
{
    auto &obd = OBD2::getInstance();

    // 1. Engine Load: A * 100 / 255
    obd.addPID(
        MODE_CURRENT_DATA, PID_ENGINE_LOAD, "Engine Load", PERCENTAGE,
        "Calculated engine load", "A * 100 / 255", 0.0f, 100.0f,
        2, UPDATE_FAST, 0xf59e0b, "gauge");

    // 2. Coolant Temp: A - 40
    obd.addPID(
        MODE_CURRENT_DATA, PID_COOLANT_TEMP, "Coolant Temp", DEGREES_CELCIUS,
        "Engine coolant temperature", "A - 40", -40.0f, 215.0f,
        3, UPDATE_SLOW, 0xef4444, "thermometer");

    // 3. Engine RPM: ((A * 256) + B) / 4
    obd.addPID(
        MODE_CURRENT_DATA, PID_ENGINE_RPM, "Engine RPM", RPM,
        "Engine speed", "((A * 256) + B) / 4", 0.0f, 16383.75f,
        1, UPDATE_FAST, 0x3b82f6, "droplet");

    esp_err_t ret = obd.init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize OBD2 interface");
        return ret;
    }

    ESP_LOGI(TAG, "OBD2 interface initialized");

    return ESP_OK;
}

void t_request_sample(void *pvParameters)
{
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

    std::string vin;

    bool vinflag = false;

    obd2.startContinuousMode();
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
        else
        {
            led.off();
        }

        if (!vinflag)
        {
            obd2.requestVIN();
            ESP_LOGI(TAG, "Requesting VIN");
            vinflag = true;
        }

        if (!vin.empty())
        {
            ESP_LOGI("VIN", "%s", vin.c_str());
        }
        else
        {
            vin = obd2.getVIN();
        }

        float rpm = obd2.getValue(PID_ENGINE_RPM);
        float load = obd2.getValue(PID_ENGINE_LOAD);
        float temp = obd2.getValue(PID_COOLANT_TEMP);

        // Get the units for a more informative log
        const char *rpm_unit = obd2.getUnit(PID_ENGINE_RPM);
        const char *load_unit = obd2.getUnit(PID_ENGINE_LOAD);
        const char *temp_unit = obd2.getUnit(PID_COOLANT_TEMP);

        ESP_LOGI("Live Data", "RPM: %5.0f %s | Load: %6.2f %s | Temp: %3.0f %s",
                 rpm, rpm_unit,
                 load, load_unit,
                 temp, temp_unit);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void)
{

    // esp_log_level_set("CAN_DRIVER", ESP_LOG_DEBUG);
    // esp_log_level_set("ASYNC_WEB_SERVER", ESP_LOG_DEBUG);

    // Component setup
    led.init();

    esp_err_t (*setup_functions[])() = {
        setup_wifi,
        setup_can,
        setup_obd,
        setup_web_server,
    };

    for (size_t i = 0; i < sizeof(setup_functions) / sizeof(setup_functions[0]); i++)
    {
        if (setup_functions[i]() != ESP_OK)
        {
            led.error();
            return;
        }
    }

    // xTaskCreate(t_request_sample, "request_sample", 4096, NULL, 5, NULL);
}