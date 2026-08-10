#pragma once

#include "can_driver.hpp"
#include "esp_err.h"
#include "nvs.h"
#include "supervisor.hpp"
#include "wifi.hpp"

class Settings
{
public:
    static Settings& getInstance()
    {
        static Settings instance;
        return instance;
    }

    esp_err_t init();

    // Wifi Settings
    esp_err_t setWifiConfig(const WIFI::Config& config);
    esp_err_t getWifiConfig(WIFI::Config& config);
    esp_err_t setDefaultWifiConfig();

    // CAN Settings
    esp_err_t setCanConfig(const CanDriver::Config& config);
    esp_err_t getCanConfig(CanDriver::Config& config);
    esp_err_t setDefaultCanConfig();

    // Supervisor Settings
    esp_err_t setSupervisorConfig(const SUPERVISOR::Config& config);
    esp_err_t getSupervisorConfig(SUPERVISOR::Config& config);
    esp_err_t setDefaultSupervisorConfig();

    // Completely wipes NVS
    esp_err_t factoryReset();

private:
    Settings(const Settings&)            = delete;
    Settings& operator=(const Settings&) = delete;

    Settings()
    {
    }

    // Helper to open NVS handle
    esp_err_t openHandle(nvs_handle_t* handle, nvs_open_mode_t mode);
};