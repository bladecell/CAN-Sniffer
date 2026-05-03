#pragma once

#include "esp_err.h"
#include "nvs.h"
#include "wifi.hpp"
#include "can_driver.hpp"

class Settings
{
public:
    static Settings &getInstance()
    {
        static Settings instance;
        return instance;
    }

    esp_err_t init();

    // Wifi Settings
    esp_err_t setWifiConfig(const WIFI::Config &config);
    esp_err_t getWifiConfig(WIFI::Config &config);

    // CAN Settings
    esp_err_t setCanConfig(const CanDriver::Config &config);
    esp_err_t getCanConfig(CanDriver::Config &config);

    // Completely wipes NVS
    esp_err_t factoryReset();

private:
    Settings(const Settings &) = delete;
    Settings &operator=(const Settings &) = delete;

    Settings() {}

    // Helper to open NVS handle
    esp_err_t openHandle(nvs_handle_t *handle, nvs_open_mode_t mode);
};