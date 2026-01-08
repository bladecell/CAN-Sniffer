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

esp_err_t setup_obd()
{
    CanDriver::Config config;
    config.bitrate = CanDriver::Bitrate::BITRATE_500K;
    config.rx_pin = CAN_RX_GPIO;
    config.tx_pin = CAN_TX_GPIO;
    config.lbk_pin = CAN_LBK_GPIO;
    config.rs_pin = CAN_RS_GPIO;
    config.debug = DEBUG_MODE;

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

    ret |= OBD2::getInstance().init();

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

    esp_log_level_set("CAN_DRIVER", ESP_LOG_DEBUG);

    // Component setup
    led.init();

    // if (setup_wifi() != ESP_OK)
    // {
    //     led.error();
    //     return;
    // }

    if (setup_obd() != ESP_OK)
    {
        led.error();
        return;
    }

    // if (setup_web_server() != ESP_OK)
    // {
    //     led.error();
    //     return;
    // }

    ESP_LOGI(TAG, "Started OBD-II Reader...");

    led.blink(2);

    for (int i = 5; i >= 1; i--)
    {
        ESP_LOGI(TAG, "Starting in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Requesting Odometer value");

    uint16_t did = 0xF4A6;
    uint16_t id = 0x7E0;
    uint8_t mode = 0x22;
    uint8_t len = 0x03;

    uint8_t txData[8] = {len, mode, (uint8_t)((did >> 8) & 0x0F), (uint8_t)(did & 0xFF), 0x00, 0x00, 0x00, 0x00};
    twai_frame_t tx = {};

    tx.header.id = id; // OBD-II  request ID
    tx.header.dlc = twaifd_len2dlc(sizeof(txData));
    tx.header.ide = false;      // Standard Frame Format (11-bit ID)
    tx.header.rtr = 0;          // Data frame (not remote frame)
    tx.header.fdf = 0;          // Classic CAN format
    tx.header.brs = 0;          // No bit rate switching
    tx.header.esi = 0;          // No error state indicator
    tx.header.timestamp = 0;    // Not used for TX
    tx.header.trigger_time = 0; // Not used for immediate transmission

    tx.buffer = txData;
    tx.buffer_len = sizeof(txData);

    CanDriver::getInstance().transmit(&tx, 100);

    // xTaskCreate(t_request_sample, "request_sample", 4096, NULL, 5, NULL);
}