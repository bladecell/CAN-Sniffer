#include "settings.hpp"

#include <cstring>

#include "esp_log.h"
#include "nvs_flash.h"
#include "secrets.h"
#include "utilities.h"
#include "wifi.hpp"

static const char* TAG           = "SETTINGS";
static const char* NVS_NAMESPACE = "storage";

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

esp_err_t Settings::openHandle(nvs_handle_t* handle, nvs_open_mode_t mode)
{
    return nvs_open(NVS_NAMESPACE, mode, handle);
}

esp_err_t Settings::setWifiConfig(const WIFI::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READWRITE);
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

esp_err_t Settings::getWifiConfig(WIFI::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READONLY);
    if (err != ESP_OK)
        return err;

    size_t    len;
    char      buf[64];
    bool      has_missing = false;
    esp_err_t fatal_err   = ESP_OK;

    auto check_err = [&](esp_err_t res) {
        if (res == ESP_ERR_NVS_NOT_FOUND) {
            has_missing = true;
            return false;
        } else if (res != ESP_OK) {
            fatal_err = res;
            return false;
        }
        return true;
    };

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "w_ssid", buf, &len)))
        config.ssid = std::string(buf);

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "w_pass", buf, &len)))
        config.password = std::string(buf);

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "w_sta_ssid", buf, &len)))
        config.sta_ssid = std::string(buf);

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "w_sta_pass", buf, &len)))
        config.sta_password = std::string(buf);

    uint8_t val8;
    if (check_err(nvs_get_u8(handle, "w_chan", &val8)))
        config.channel = val8;
    if (check_err(nvs_get_u8(handle, "w_mode", &val8)))
        config.mode = (wifi_mode_t)val8;
    if (check_err(nvs_get_u8(handle, "w_auth", &val8)))
        config.auth_mode = (wifi_auth_mode_t)val8;

    nvs_close(handle);

    if (fatal_err != ESP_OK)
        return fatal_err;
    if (has_missing)
        return ESP_ERR_NVS_NOT_FOUND;

    return ESP_OK;
}

esp_err_t Settings::setDefaultWifiConfig()
{
    WIFI::Config config;
    config.ssid            = "Can Sniffer";
    config.password        = "";
    config.channel         = 6;
    config.max_connections = 4;
    config.auth_mode       = WIFI_AUTH_OPEN;
    config.mode            = WIFI_MODE_STA;
    config.sta_ssid        = WIFI_SSID;
    config.sta_password    = WIFI_PASSWORD;
    config.sta_auth_mode   = WIFI_AUTH_WPA2_PSK;
    return setWifiConfig(config);
}

// --- CAN CONFIG (Mostly numbers, use Blob for speed) ---

esp_err_t Settings::setCanConfig(const CanDriver::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    err |= nvs_set_u32(handle, "c_bitrate", (uint32_t)config.bitrate);
    err |= nvs_set_i32(handle, "c_tx_pin", config.tx_pin);
    err |= nvs_set_i32(handle, "c_rx_pin", config.rx_pin);
    err |= nvs_set_i32(handle, "c_lbk_pin", config.lbk_pin);
    err |= nvs_set_i32(handle, "c_rs_pin", config.rs_pin);
    err |= nvs_set_u8(handle, "c_debug", config.debug ? 1 : 0);
    err |= nvs_set_u8(handle, "c_rs_mode", (uint8_t)config.rs_mode);
    err |= nvs_set_u32(handle, "c_tx_q", config.tx_queue_depth);
    err |= nvs_set_u32(handle, "c_rx_q", (uint32_t)config.rx_queue_size);
    err |= nvs_set_u8(handle, "c_filter", config.filter ? 1 : 0);
    err |= nvs_set_blob(handle, "c_mfilter", &config.mfilter_cfg, sizeof(twai_mask_filter_config_t));

    if (err == ESP_OK)
    {
        nvs_commit(handle);
        ESP_LOGI(TAG, "CAN Config Saved");
    }
    nvs_close(handle);
    return err;
}

esp_err_t Settings::getCanConfig(CanDriver::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READONLY);
    if (err != ESP_OK)
        return err;

    uint32_t  val32;
    int32_t   vali32;
    uint8_t   val8;
    bool      has_missing = false;
    esp_err_t fatal_err   = ESP_OK;

    auto check_err = [&](esp_err_t res) {
        if (res == ESP_ERR_NVS_NOT_FOUND) {
            has_missing = true;
            return false;
        } else if (res != ESP_OK) {
            fatal_err = res;
            return false;
        }
        return true;
    };

    if (check_err(nvs_get_u32(handle, "c_bitrate", &val32)))
        config.bitrate = (CanDriver::Bitrate)val32;
    if (check_err(nvs_get_i32(handle, "c_tx_pin", &vali32)))
        config.tx_pin = (gpio_num_t)vali32;
    if (check_err(nvs_get_i32(handle, "c_rx_pin", &vali32)))
        config.rx_pin = (gpio_num_t)vali32;
    if (check_err(nvs_get_i32(handle, "c_lbk_pin", &vali32)))
        config.lbk_pin = (gpio_num_t)vali32;
    if (check_err(nvs_get_i32(handle, "c_rs_pin", &vali32)))
        config.rs_pin = (gpio_num_t)vali32;
    if (check_err(nvs_get_u8(handle, "c_debug", &val8)))
        config.debug = (val8 != 0);
    if (check_err(nvs_get_u8(handle, "c_rs_mode", &val8)))
        config.rs_mode = (CanDriver::RS_MODE)val8;
    if (check_err(nvs_get_u32(handle, "c_tx_q", &val32)))
        config.tx_queue_depth = val32;
    if (check_err(nvs_get_u32(handle, "c_rx_q", &val32)))
        config.rx_queue_size = val32;
    if (check_err(nvs_get_u8(handle, "c_filter", &val8)))
        config.filter = (val8 != 0);

    size_t req_size = sizeof(twai_mask_filter_config_t);
    check_err(nvs_get_blob(handle, "c_mfilter", &config.mfilter_cfg, &req_size));

    nvs_close(handle);

    if (fatal_err != ESP_OK)
        return fatal_err;
    if (has_missing)
        return ESP_ERR_NVS_NOT_FOUND;

    return ESP_OK;
}

esp_err_t Settings::setDefaultCanConfig()
{
    CanDriver::Config config;
    config.bitrate            = CanDriver::Bitrate::BITRATE_500K;
    config.rx_pin             = CAN_RX_GPIO;
    config.tx_pin             = CAN_TX_GPIO;
    config.lbk_pin            = CAN_LBK_GPIO;
    config.rs_pin             = CAN_RS_GPIO;
    config.debug              = DEBUG_MODE;
    config.filter             = false;
    config.mfilter_cfg.id     = 0b011100000000;
    config.mfilter_cfg.mask   = 0b011100000000;
    config.mfilter_cfg.is_ext = false;  // Standard 11-bit IDs
    return setCanConfig(config);
}

esp_err_t Settings::setSupervisorConfig(const SUPERVISOR::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READWRITE);
    if (err != ESP_OK)
        return err;

    err |= nvs_set_str(handle, "pid_def_path", config.pid_def_path.c_str());
    err |= nvs_set_str(handle, "dtc_desc_path", config.dtc_desc_path.c_str());

    if (err == ESP_OK)
    {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Supervisor Config Saved");
    }
    nvs_close(handle);
    return err;
}

esp_err_t Settings::getSupervisorConfig(SUPERVISOR::Config& config)
{
    nvs_handle_t handle;
    esp_err_t    err = openHandle(&handle, NVS_READONLY);
    if (err != ESP_OK)
        return err;

    char      buf[256];
    size_t    len;
    bool      has_missing = false;
    esp_err_t fatal_err   = ESP_OK;

    auto check_err = [&](esp_err_t res) {
        if (res == ESP_ERR_NVS_NOT_FOUND) {
            has_missing = true;
            return false;
        } else if (res != ESP_OK) {
            fatal_err = res;
            return false;
        }
        return true;
    };

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "pid_def_path", buf, &len)))
        config.pid_def_path = std::string(buf);

    len = sizeof(buf);
    if (check_err(nvs_get_str(handle, "dtc_desc_path", buf, &len)))
        config.dtc_desc_path = std::string(buf);

    nvs_close(handle);

    if (fatal_err != ESP_OK)
        return fatal_err;
    if (has_missing)
        return ESP_ERR_NVS_NOT_FOUND;

    return ESP_OK;
}

esp_err_t Settings::setDefaultSupervisorConfig()
{
    SUPERVISOR::Config config;
    config.pid_def_path  = PID_DEF_DB_PATH;
    config.dtc_desc_path = DTC_DESC_DB_PATH;
    return setSupervisorConfig(config);
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
