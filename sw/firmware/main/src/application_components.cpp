#include "application_components.hpp"

#include "battery.hpp"
#include "can_driver.hpp"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "obd2_simulator.hpp"
#include "sd_card.hpp"
#include "settings.hpp"
#include "settings_merge.hpp"
#include "supervisor.hpp"
#include "utilities.h"
#include "webserver.hpp"
#include "wifi.hpp"

namespace
{
// Preserve the existing log tag while moving ownership of the wiring.
const char* const TAG = "SUPERVISOR";
esp_pm_lock_handle_t powerLock = nullptr;
constexpr const char* MOUNT_POINT = "/sdcard";
}

namespace application_components
{
esp_err_t stopSdCard(void*)
{
    return SDCard::getInstance().deinit();
}

esp_err_t stopBattery(void*)
{
    battery_deinit();
    return ESP_OK;
}

esp_err_t stopCan(void*)
{
    return CanDriver::getInstance().deinit();
}

esp_err_t stopObd(void*)
{
    return OBD2::getInstance().deinit();
}

esp_err_t stopWifi(void*)
{
    return WIFI::getInstance().deinit();
}

esp_err_t stopWebserver(void*)
{
    return stop_web_server();
}

esp_err_t setupPowerManagement(void*)
{
    esp_pm_config_t pm_config = {.max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};

    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure power management: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_sleep", &powerLock);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create power lock: %s", esp_err_to_name(err));
        powerLock = nullptr;
        return err;
    }

    err = esp_pm_lock_acquire(powerLock);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to acquire power lock: %s", esp_err_to_name(err));
        esp_pm_lock_delete(powerLock);
        powerLock = nullptr;
        return err;
    }

    return ESP_OK;
}

esp_err_t releasePowerLock()
{
    return powerLock ? esp_pm_lock_release(powerLock) : ESP_OK;
}

esp_err_t acquirePowerLock()
{
    return powerLock ? esp_pm_lock_acquire(powerLock) : ESP_OK;
}

esp_err_t bootstrapSettings(void*)
{
    esp_err_t err = Settings::getInstance().init();
    if (err != ESP_OK)
    {
        return err;
    }
    return SUPERVISOR::getInstance().load_config_from_nvs();
}

esp_err_t setupFlashFilesystem(void*)
{
    // Value-init then assign: IDF 6 littlefs has a conditional blockdev member.
    esp_vfs_littlefs_conf_t www_conf{};
    www_conf.base_path       = "/www";
    www_conf.partition_label = "www";
    www_conf.read_only       = true;

    esp_err_t err = esp_vfs_littlefs_register(&www_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE("FS", "Failed to mount WWW partition. Did you flash it?");
        return err;
    }
    ESP_LOGI("FS", "WWW partition mounted successfully at /www");

    return ESP_OK;
}

esp_err_t setupWebserver(void*)
{
    return setup_web_server();
}

// A missing key preserves other saved keys. Operational errors use defaults;
// only partition corruption invokes the supervisor's bounded recovery policy.
template <class Cfg, class Get, class Store, class SetDefault>
static esp_err_t load_section(const char* name, Cfg& cfg, const Cfg& defaults, Get get, Store store,
                              SetDefault setDefault)
{
    esp_err_t err = get(cfg);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "%s config incomplete or first boot. Keeping valid stored keys, defaulting the rest.", name);
        err = settings_merge::recoverMissing(cfg, defaults, ESP_ERR_NVS_NOT_FOUND, get, store, setDefault);
    }
    else if (err == ESP_ERR_NVS_CORRUPT_KEY_PART)
    {
        return SUPERVISOR::getInstance().recover_corrupt_settings(name);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Reading %s config failed (%s); using defaults for this boot", name, esp_err_to_name(err));
        cfg = defaults;
    }
    return ESP_OK;
}

esp_err_t setupWifi(void*)
{
    Settings&    settings = Settings::getInstance();
    WIFI::Config config;
    esp_err_t    err = load_section(
        "WIFI", config, Settings::defaultWifiConfig(), [&](WIFI::Config& c) { return settings.getWifiConfig(c); },
        [&](const WIFI::Config& c) { return settings.setWifiConfig(c); }, [&] { settings.setDefaultWifiConfig(); });
    if (err != ESP_OK)
    {
        return err;
    }

    err = WIFI::getInstance().init(config);
    if (err != ESP_OK)
    {
        return err;
    }
    return WIFI::getInstance().start();
}

esp_err_t setupCan(void*)
{
    Settings&         settings = Settings::getInstance();
    CanDriver::Config config;
    esp_err_t         err = load_section(
        "CAN", config, Settings::defaultCanConfig(), [&](CanDriver::Config& c) { return settings.getCanConfig(c); },
        [&](const CanDriver::Config& c) { return settings.setCanConfig(c); }, [&] { settings.setDefaultCanConfig(); });
    if (err != ESP_OK)
    {
        return err;
    }

    if (config.debug)
    {
        CanDriver::getInstance().setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    }
    else
    {
        CanDriver::getInstance().setSimHooks(nullptr, nullptr, nullptr);
    }

    esp_err_t ret = CanDriver::getInstance().init(config);
    CanDriver::getInstance().setRxCallback(LedStatus::staticBlink, nullptr);

    if (ret != ESP_OK || !CanDriver::getInstance().isInitialized())
    {
        ESP_LOGE(TAG, "Failed to initialize CAN driver");
        return ret;
    }

    ESP_LOGI(TAG, "CAN driver initialized");

    return ESP_OK;
}

esp_err_t setupObd(void*)
{
    auto& obd = OBD2::getInstance();

    // VIN and DTC subscriptions are reinstalled after each successful teardown.
    obd.connected_subscribe(
        [](bool connected)
        {
            if (!connected)
            {
                return;
            }

            auto& obd2 = OBD2::getInstance();

            esp_err_t err = obd2.requestVIN();
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "VIN request failed: %s", esp_err_to_name(err));
            }

            const uint8_t dtc_modes[] = {MODE_DTCS, MODE_PENDING_DTCS, MODE_PERMANENT_DTCS};
            for (const uint8_t mode : dtc_modes)
            {
                err = obd2.requestDTC(mode);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "DTC mode 0x%02X request failed: %s", mode, esp_err_to_name(err));
                }
            }

            ESP_LOGI(TAG, "DTC request sent, waiting for response...");
        });

    obd.connected_subscribe(
        [](bool connected)
        {
            if (!connected)
            {
                ESP_LOGI(TAG, "OBD disconnected.");
                SUPERVISOR::getInstance().notify_disconnected();
            }
        });

    esp_err_t ret = obd.init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize OBD2 interface: %s", esp_err_to_name(ret));
        return ret;
    }

    if (!SDCard::getInstance().is_mounted())
    {
        ESP_LOGW(TAG, "No SD card mounted, OBD2 runs without PID definitions");
        return ESP_OK;
    }

    ret = SUPERVISOR::getInstance().load_pid_def_from_json(SUPERVISOR::getInstance().get_pid_def_path().c_str());

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load PID definitions: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "OBD2 interface initialized");

    return ESP_OK;
}

esp_err_t setupBattery(void*)
{
    return battery_init();
}

esp_err_t setupSdCard(void*)
{
    SDCard::Config config;
    config.miso_pin               = MISO;
    config.mosi_pin               = MOSI;
    config.sclk_pin               = SCLK;
    config.cd_pin                 = SD_DETECT;
    config.base_path              = MOUNT_POINT;
    config.slot                   = SDMMC_HOST_SLOT_1;
    config.max_files              = 5;
    config.format_if_mount_failed = false;

    SDCard::getInstance().on_mount(
        []()
        {
            auto& sd = SDCard::getInstance();
            if (sd.create_directory("/config") != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to create /config directory");
            }

            if (sd.create_directory("/logs") != ESP_OK)
            {
                ESP_LOGE("SD_CALLBACK", "Failed to create /logs directory");
            }
        });

    esp_err_t ret = SDCard::getInstance().init(config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    if (SDCard::getInstance().is_mounted())
    {
        ESP_LOGI(TAG, "SD card initialized and mounted at %s", config.base_path);
    }
    else
    {
        ESP_LOGW(TAG, "SD card driver ready, no card mounted (insert a card; SD features disabled)");
    }

    return ESP_OK;
}
}

// The existing member still owns its configuration; only the settings-loading
// implementation is colocated with the application's other section loaders.
esp_err_t SUPERVISOR::load_config_from_nvs()
{
    Settings& settings = Settings::getInstance();
    return application_components::load_section(
        "System", _config, Settings::defaultSupervisorConfig(), [&](Config& c)
        { return settings.getSupervisorConfig(c); }, [&](const Config& c) { return settings.setSupervisorConfig(c); },
        [&] { settings.setDefaultSupervisorConfig(); });
}
