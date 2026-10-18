#include "settings.hpp"

#include <cstring>

#include <iterator>
#include <string_view>

#include "can_config_limits.hpp"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "settings_validation.hpp"
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
        err = nvs_flash_erase();
        if (err != ESP_OK)
        {
            return err;
        }
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t Settings::openHandle(nvs_handle_t* handle, nvs_open_mode_t mode)
{
    return nvs_open(NVS_NAMESPACE, mode, handle);
}

namespace sv = settings_validation;

namespace
{
// SD, LED and card-detect pins from utilities.h: the CAN pins must not collide with them.
constexpr int kBoardPins[] = {MISO, MOSI, SCLK, CS, SD_DETECT, LED_GPIO};

// One pass over the keys of a section. A key that is absent marks the section "missing" (first boot). A stored value that
// cannot be used (too long for its buffer, wrong type, out of range) is logged and replaced by the caller's fallback for
// that key only. Any other NVS error is returned to the caller; none of this triggers a factory reset.
struct NvsRead
{
    nvs_handle_t h;
    bool         missing = false;
    esp_err_t    fatal   = ESP_OK;
    bool         bad     = false;

    bool accept(esp_err_t e, const char* key, bool optional)
    {
        bad = false;
        if (e == ESP_OK)
        {
            return true;
        }
        if (e == ESP_ERR_NVS_NOT_FOUND)
        {
            missing = missing || !optional;
        }
        else if (e == ESP_ERR_NVS_INVALID_LENGTH || e == ESP_ERR_NVS_TYPE_MISMATCH)
        {
            ESP_LOGW(TAG, "Stored '%s' unusable (%s); using default for this key", key, esp_err_to_name(e));
            bad = true;
        }
        else
        {
            fatal = e;
        }
        return false;
    }
};

esp_err_t nvsGet(nvs_handle_t h, const char* k, uint8_t* v)
{
    return nvs_get_u8(h, k, v);
}
esp_err_t nvsGet(nvs_handle_t h, const char* k, uint32_t* v)
{
    return nvs_get_u32(h, k, v);
}
esp_err_t nvsGet(nvs_handle_t h, const char* k, int32_t* v)
{
    return nvs_get_i32(h, k, v);
}

template <class Raw, class T>
void readNum(NvsRead& r, const char* key, T& dst, T fallback, bool (*valid)(int64_t), bool optional = false)
{
    Raw       v{};
    esp_err_t e = nvsGet(r.h, key, &v);
    if (!r.accept(e, key, optional))
    {
        if (r.bad)
        {
            dst = fallback;
        }
        return;
    }
    if (valid(static_cast<int64_t>(v)))
    {
        dst = static_cast<T>(v);
    }
    else
    {
        ESP_LOGW(TAG, "Stored '%s' out of range; using default for this key", key);
        dst = fallback;
    }
}

bool anyValue(int64_t)
{
    return true;
}

// N is the read buffer size; the validators cap the length below it, so a string we wrote always fits.
template <size_t N>
void readStr(NvsRead& r, const char* key, std::string& dst, const std::string& fallback, bool (*valid)(std::string_view))
{
    char      buf[N];
    size_t    len = N;
    esp_err_t e   = nvs_get_str(r.h, key, buf, &len);
    if (!r.accept(e, key, false))
    {
        if (r.bad)
        {
            dst = fallback;
        }
        return;
    }
    buf[N - 1] = '\0';
    if (valid(std::string_view(buf)))
    {
        dst = buf;
    }
    else
    {
        ESP_LOGW(TAG, "Stored '%s' invalid; using default for this key", key);
        dst = fallback;
    }
}

esp_err_t finish(const NvsRead& r)
{
    if (r.fatal != ESP_OK)
    {
        return r.fatal;
    }
    return r.missing ? ESP_ERR_NVS_NOT_FOUND : ESP_OK;
}
}  // namespace

const char* Settings::checkWifiConfig(const WIFI::Config& c)
{
    return sv::wifiCheck(c.ssid, c.password, c.auth_mode, c.mode, c.channel, c.max_connections, c.sta_ssid, c.sta_password,
                         c.sta_auth_mode);
}

const char* Settings::checkCanConfig(const CanDriver::Config& c)
{
    if (!sv::bitrateValid(static_cast<int64_t>(c.bitrate)))
    {
        return "bitrate must be 125000, 250000, 500000 or 1000000";
    }
    if (!sv::rsModeValid(static_cast<int64_t>(c.rs_mode)))
    {
        return "rs_mode must be 0 or 1";
    }
    if (!can_limits::canConfigValid(static_cast<double>(c.rx_queue_size), static_cast<double>(c.tx_queue_depth)))
    {
        return "rx_queue_size must be 1..256 and tx_queue_depth 1..64";
    }
    if (!sv::filterValid(c.mfilter_cfg.id, c.mfilter_cfg.mask, c.mfilter_cfg.is_ext))
    {
        return "mfilter_cfg id and mask must fit 11 bits (29 bits when is_ext)";
    }
    return sv::canPinsCheck(c.tx_pin, c.rx_pin, c.lbk_pin, c.rs_pin, kBoardPins, std::size(kBoardPins));
}

const char* Settings::checkSupervisorConfig(const SUPERVISOR::Config& c)
{
    if (!sv::sdPathValid(c.pid_def_path))
    {
        return "pid_def_path must be an /sdcard/ path of at most 255 bytes";
    }
    if (!sv::sdPathValid(c.dtc_desc_path))
    {
        return "dtc_desc_path must be an /sdcard/ path of at most 255 bytes";
    }
    return nullptr;
}

WIFI::Config Settings::defaultWifiConfig()
{
    WIFI::Config config;
    config.ssid            = "Can Sniffer";
    config.password        = "";
    config.channel         = 6;
    config.max_connections = 4;
    config.auth_mode       = WIFI_AUTH_OPEN;
    config.mode            = WIFI_MODE_STA;
    config.sta_ssid        = "";
    config.sta_password    = "";
    config.sta_auth_mode   = WIFI_AUTH_WPA2_PSK;
    return config;
}

CanDriver::Config Settings::defaultCanConfig()
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
    return config;
}

SUPERVISOR::Config Settings::defaultSupervisorConfig()
{
    SUPERVISOR::Config config;
    config.pid_def_path  = PID_DEF_DB_PATH;
    config.dtc_desc_path = DTC_DESC_DB_PATH;
    return config;
}

esp_err_t Settings::setWifiConfig(const WIFI::Config& config)
{
    if (const char* why = checkWifiConfig(config))
    {
        ESP_LOGW(TAG, "Wi-Fi config rejected: %s", why);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t    ret = openHandle(&handle, NVS_READWRITE);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Note: nvs_set_str keys must be < 15 chars. Fields are saved individually so future upgrades can add keys.
    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "w_ssid", config.ssid.c_str()), out, TAG, "w_ssid");
    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "w_pass", config.password.c_str()), out, TAG, "w_pass");
    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "w_sta_ssid", config.sta_ssid.c_str()), out, TAG, "w_sta_ssid");
    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "w_sta_pass", config.sta_password.c_str()), out, TAG, "w_sta_pass");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_chan", config.channel), out, TAG, "w_chan");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_mode", (uint8_t)config.mode), out, TAG, "w_mode");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_auth", (uint8_t)config.auth_mode), out, TAG, "w_auth");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_maxconn", config.max_connections), out, TAG, "w_maxconn");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_ssidhid", config.ssid_hidden ? 1 : 0), out, TAG, "w_ssidhid");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_pmf", config.pmf_required ? 1 : 0), out, TAG, "w_pmf");
    ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "w_gtkrekey", config.gtk_rekey_interval), out, TAG, "w_gtkrekey");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_sta_auth", (uint8_t)config.sta_auth_mode), out, TAG, "w_sta_auth");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "w_sta_retry", config.sta_max_retry), out, TAG, "w_sta_retry");
    ESP_GOTO_ON_ERROR(nvs_commit(handle), out, TAG, "commit");
    ESP_LOGI(TAG, "WiFi Config Saved");

out:
    nvs_close(handle);
    return ret;
}

esp_err_t Settings::getWifiConfig(WIFI::Config& config)
{
    NvsRead   r;
    esp_err_t err = openHandle(&r.h, NVS_READONLY);
    if (err != ESP_OK)
    {
        return err;
    }

    const WIFI::Config def = defaultWifiConfig();
    readStr<sv::kWifiStrBuf>(r, "w_ssid", config.ssid, def.ssid, [](std::string_view s) { return sv::ssidValid(s, false); });
    readStr<sv::kWifiStrBuf>(r, "w_pass", config.password, def.password, sv::passwordValid);
    readStr<sv::kWifiStrBuf>(r, "w_sta_ssid", config.sta_ssid, def.sta_ssid, [](std::string_view s) { return sv::ssidValid(s, true); });
    readStr<sv::kWifiStrBuf>(r, "w_sta_pass", config.sta_password, def.sta_password, sv::passwordValid);
    readNum<uint8_t>(r, "w_chan", config.channel, def.channel, sv::channelValid);
    readNum<uint8_t>(r, "w_mode", config.mode, def.mode, sv::wifiModeValid);
    readNum<uint8_t>(r, "w_auth", config.auth_mode, def.auth_mode, sv::wifiAuthValid);

    // Optional fields: default if absent, so existing configs upgrade cleanly
    readNum<uint8_t>(r, "w_maxconn", config.max_connections, def.max_connections, sv::maxConnValid, true);
    readNum<uint8_t>(r, "w_ssidhid", config.ssid_hidden, def.ssid_hidden, anyValue, true);
    readNum<uint8_t>(r, "w_pmf", config.pmf_required, def.pmf_required, anyValue, true);
    readNum<uint32_t>(r, "w_gtkrekey", config.gtk_rekey_interval, def.gtk_rekey_interval, anyValue, true);
    readNum<uint8_t>(r, "w_sta_auth", config.sta_auth_mode, def.sta_auth_mode, sv::wifiAuthValid, true);
    readNum<uint8_t>(r, "w_sta_retry", config.sta_max_retry, def.sta_max_retry, anyValue, true);

    nvs_close(r.h);

    err = finish(r);
    if (err == ESP_OK)
    {
        if (const char* why = checkWifiConfig(config))
        {
            ESP_LOGW(TAG, "Stored Wi-Fi config inconsistent (%s); using defaults for this boot", why);
            config = def;
        }
    }
    return err;
}

esp_err_t Settings::setDefaultWifiConfig()
{
    return setWifiConfig(defaultWifiConfig());
}

// --- CAN CONFIG (Mostly numbers, use Blob for speed) ---

esp_err_t Settings::setCanConfig(const CanDriver::Config& config)
{
    if (const char* why = checkCanConfig(config))
    {
        ESP_LOGW(TAG, "CAN config rejected: %s", why);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t    ret = openHandle(&handle, NVS_READWRITE);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "c_bitrate", (uint32_t)config.bitrate), out, TAG, "c_bitrate");
    ESP_GOTO_ON_ERROR(nvs_set_i32(handle, "c_tx_pin", config.tx_pin), out, TAG, "c_tx_pin");
    ESP_GOTO_ON_ERROR(nvs_set_i32(handle, "c_rx_pin", config.rx_pin), out, TAG, "c_rx_pin");
    ESP_GOTO_ON_ERROR(nvs_set_i32(handle, "c_lbk_pin", config.lbk_pin), out, TAG, "c_lbk_pin");
    ESP_GOTO_ON_ERROR(nvs_set_i32(handle, "c_rs_pin", config.rs_pin), out, TAG, "c_rs_pin");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "c_debug", config.debug ? 1 : 0), out, TAG, "c_debug");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "c_rs_mode", (uint8_t)config.rs_mode), out, TAG, "c_rs_mode");
    ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "c_tx_q", config.tx_queue_depth), out, TAG, "c_tx_q");
    ESP_GOTO_ON_ERROR(nvs_set_u32(handle, "c_rx_q", (uint32_t)config.rx_queue_size), out, TAG, "c_rx_q");
    ESP_GOTO_ON_ERROR(nvs_set_u8(handle, "c_filter", config.filter ? 1 : 0), out, TAG, "c_filter");
    ESP_GOTO_ON_ERROR(nvs_set_blob(handle, "c_mfilter", &config.mfilter_cfg, sizeof(twai_mask_filter_config_t)), out, TAG,
                      "c_mfilter");
    ESP_GOTO_ON_ERROR(nvs_commit(handle), out, TAG, "commit");
    ESP_LOGI(TAG, "CAN Config Saved");

out:
    nvs_close(handle);
    return ret;
}

esp_err_t Settings::getCanConfig(CanDriver::Config& config)
{
    NvsRead   r;
    esp_err_t err = openHandle(&r.h, NVS_READONLY);
    if (err != ESP_OK)
    {
        return err;
    }

    const CanDriver::Config def = defaultCanConfig();
    readNum<uint32_t>(r, "c_bitrate", config.bitrate, def.bitrate, sv::bitrateValid);
    readNum<int32_t>(r, "c_tx_pin", config.tx_pin, def.tx_pin, sv::gpioUsable);
    readNum<int32_t>(r, "c_rx_pin", config.rx_pin, def.rx_pin, sv::gpioUsable);
    readNum<int32_t>(r, "c_lbk_pin", config.lbk_pin, def.lbk_pin, sv::gpioUsable);
    readNum<int32_t>(r, "c_rs_pin", config.rs_pin, def.rs_pin, sv::gpioUsable);
    readNum<uint8_t>(r, "c_debug", config.debug, def.debug, anyValue);
    readNum<uint8_t>(r, "c_rs_mode", config.rs_mode, def.rs_mode, sv::rsModeValid);
    readNum<uint32_t>(r, "c_tx_q", config.tx_queue_depth, def.tx_queue_depth,
                      [](int64_t v) { return can_limits::txQueueDepthValid(static_cast<double>(v)); });
    readNum<uint32_t>(r, "c_rx_q", config.rx_queue_size, def.rx_queue_size,
                      [](int64_t v) { return can_limits::rxQueueSizeValid(static_cast<double>(v)); });
    readNum<uint8_t>(r, "c_filter", config.filter, def.filter, anyValue);

    twai_mask_filter_config_t filter{};
    size_t                    req_size = sizeof(filter);
    if (r.accept(nvs_get_blob(r.h, "c_mfilter", &filter, &req_size), "c_mfilter", false))
    {
        if (req_size == sizeof(filter) && sv::filterValid(filter.id, filter.mask, filter.is_ext))
        {
            config.mfilter_cfg = filter;
        }
        else
        {
            ESP_LOGW(TAG, "Stored 'c_mfilter' invalid; using default for this key");
            config.mfilter_cfg = def.mfilter_cfg;
        }
    }
    else if (r.bad)
    {
        config.mfilter_cfg = def.mfilter_cfg;
    }

    nvs_close(r.h);

    err = finish(r);
    if (err == ESP_OK)
    {
        if (const char* why = checkCanConfig(config))
        {
            ESP_LOGW(TAG, "Stored CAN config inconsistent (%s); using defaults for this boot", why);
            config = def;
        }
    }
    return err;
}

esp_err_t Settings::setDefaultCanConfig()
{
    return setCanConfig(defaultCanConfig());
}

esp_err_t Settings::setSupervisorConfig(const SUPERVISOR::Config& config)
{
    if (const char* why = checkSupervisorConfig(config))
    {
        ESP_LOGW(TAG, "System config rejected: %s", why);
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t    ret = openHandle(&handle, NVS_READWRITE);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "pid_def_path", config.pid_def_path.c_str()), out, TAG, "pid_def_path");
    ESP_GOTO_ON_ERROR(nvs_set_str(handle, "dtc_desc_path", config.dtc_desc_path.c_str()), out, TAG, "dtc_desc_path");
    ESP_GOTO_ON_ERROR(nvs_commit(handle), out, TAG, "commit");
    ESP_LOGI(TAG, "Supervisor Config Saved");

out:
    nvs_close(handle);
    return ret;
}

esp_err_t Settings::getSupervisorConfig(SUPERVISOR::Config& config)
{
    NvsRead   r;
    esp_err_t err = openHandle(&r.h, NVS_READONLY);
    if (err != ESP_OK)
    {
        return err;
    }

    const SUPERVISOR::Config def = defaultSupervisorConfig();
    readStr<sv::kPathBuf>(r, "pid_def_path", config.pid_def_path, def.pid_def_path, sv::sdPathValid);
    readStr<sv::kPathBuf>(r, "dtc_desc_path", config.dtc_desc_path, def.dtc_desc_path, sv::sdPathValid);

    nvs_close(r.h);
    return finish(r);
}

esp_err_t Settings::setDefaultSupervisorConfig()
{
    return setSupervisorConfig(defaultSupervisorConfig());
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
