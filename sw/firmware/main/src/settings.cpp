#include "settings.hpp"
#include "esp_log.h"
#include <cstring>

static const char *TAG = "SETTINGS";
static const char *NVS_NAMESPACE = "storage";

esp_err_t Settings::init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t Settings::openHandle(nvs_handle_t *handle, nvs_open_mode_t mode)
{
    return nvs_open(NVS_NAMESPACE, mode, handle);
}

esp_err_t Settings::setWifiConfig(const WIFI::Config &config)
{
    nvs_handle_t handle;
    esp_err_t err = openHandle(&handle, NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    // Save Strings
    // Note: nvs_set_str keys must be < 15 chars
    err |= nvs_set_str(handle, "w_ssid", config.ssid.c_str());
    err |= nvs_set_str(handle, "w_pass", config.password.c_str());
    err |= nvs_set_str(handle, "w_sta_ssid", config.sta_ssid.c_str());
    err |= nvs_set_str(handle, "w_sta_pass", config.sta_password.c_str());

    // Save POD (Plain Old Data)
    // Saving individually is safer for future upgrades
    err |= nvs_set_u8(handle, "w_chan", config.channel);
    err |= nvs_set_u8(handle, "w_mode", (uint8_t)config.mode);
    err |= nvs_set_u8(handle, "w_auth", (uint8_t)config.auth_mode);

    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
        ESP_LOGI(TAG, "WiFi Config Saved");
    }

    nvs_close(handle);
    return err;
}

esp_err_t Settings::getWifiConfig(WIFI::Config &config)
{
    nvs_handle_t handle;
    esp_err_t err = openHandle(&handle, NVS_READONLY);
    if (err != ESP_OK)
        return err; // Likely first boot, return error so defaults are used

    size_t len;
    char buf[64]; // Max SSID/Pass length

    // Load SSID
    len = sizeof(buf);
    if (nvs_get_str(handle, "w_ssid", buf, &len) == ESP_OK)
        config.ssid = std::string(buf);

    // Load Pass
    len = sizeof(buf);
    if (nvs_get_str(handle, "w_pass", buf, &len) == ESP_OK)
        config.password = std::string(buf);

    // Load STA SSID
    len = sizeof(buf);
    if (nvs_get_str(handle, "w_sta_ssid", buf, &len) == ESP_OK)
        config.sta_ssid = std::string(buf);

    // Load STA Pass
    len = sizeof(buf);
    if (nvs_get_str(handle, "w_sta_pass", buf, &len) == ESP_OK)
        config.sta_password = std::string(buf);

    uint8_t val8;
    if (nvs_get_u8(handle, "w_chan", &val8) == ESP_OK)
        config.channel = val8;
    if (nvs_get_u8(handle, "w_mode", &val8) == ESP_OK)
        config.mode = (wifi_mode_t)val8;
    if (nvs_get_u8(handle, "w_auth", &val8) == ESP_OK)
        config.auth_mode = (wifi_auth_mode_t)val8;

    nvs_close(handle);
    return ESP_OK;
}

// --- CAN CONFIG (Mostly numbers, use Blob for speed) ---

esp_err_t Settings::setCanConfig(const CanDriver::Config &config)
{
    nvs_handle_t handle;
    esp_err_t err = openHandle(&handle, NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    // WARNING: If you change the struct layout in code later, you must wipe NVS or handle versions.
    err = nvs_set_blob(handle, "can_cfg", &config, sizeof(CanDriver::Config));

    if (err == ESP_OK)
    {
        nvs_commit(handle);
        ESP_LOGI(TAG, "CAN Config Saved");
    }
    nvs_close(handle);
    return err;
}

esp_err_t Settings::getCanConfig(CanDriver::Config &config)
{
    nvs_handle_t handle;
    esp_err_t err = openHandle(&handle, NVS_READONLY);
    if (err != ESP_OK)
        return err;

    size_t required_size = sizeof(CanDriver::Config);

    // Check if blob exists and is correct size
    err = nvs_get_blob(handle, "can_cfg", &config, &required_size);

    nvs_close(handle);
    return err;
}

esp_err_t Settings::factoryReset()
{
    ESP_LOGW(TAG, "PERFORMING FACTORY RESET: Wiping NVS...");

    // 1. De-initialize NVS first
    esp_err_t err = nvs_flash_deinit();

    // 2. Erase the Partition
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_INITIALIZED)
    {
        err = nvs_flash_erase();
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Re-initialize immediately
    err = nvs_flash_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to re-init NVS after wipe: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Factory Reset Complete. NVS is clean.");
    return ESP_OK;
}