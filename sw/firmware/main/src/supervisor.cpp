// supervisor.cpp

#include "supervisor.hpp"

#include <cctype>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <limits>
#include <sys/stat.h>

#include "async_web_server.hpp"
#include "atomic_file.hpp"
#include "battery.hpp"
#include "can_driver.hpp"
#include "error_policy.hpp"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "input_limits.hpp"
#include "led_status.hpp"
#include "middleware.hpp"
#include "obd2.hpp"
#include "pid_json.hpp"
#include "obd2_simulator.hpp"
#include "sd_card.hpp"
#include "settings.hpp"
#include "utilities.h"
#include "webserver.hpp"

// Consecutive restarts caused by the ERROR state. RTC_NOINIT survives a software restart; the magic guards power-on garbage.
static RTC_NOINIT_ATTR uint32_t s_errorMagic;
static RTC_NOINIT_ATTR uint32_t s_errorRestarts;
#include "wifi.hpp"

static const char* TAG = "SUPERVISOR";

#define MOUNT_POINT "/sdcard"

namespace
{
constexpr TickType_t SD_OPERATION_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);

// SDCard accepts either leading-slash form for a relative path and one trailing
// slash.  Normalize those accepted spellings before comparing copy endpoints.
bool canonical_sd_relative_path(const char* path, std::string& canonical_path)
{
    if (!SDCard::is_path_under(path, MOUNT_POINT))
    {
        return false;
    }

    const char* relative_path = path + strlen(MOUNT_POINT);
    if (SDCard::validate_relative_path(relative_path, true) != ESP_OK)
    {
        return false;
    }

    if (*relative_path == '/')
    {
        ++relative_path;
    }

    canonical_path = relative_path;
    if (!canonical_path.empty() && canonical_path.back() == '/')
    {
        canonical_path.pop_back();
    }

    // FAT filename matching is case-insensitive for ASCII characters.  Do the
    // fold locally instead of using locale-sensitive ctype helpers or probing
    // the filesystem (which could race with the subsequent copy).
    for (char& character : canonical_path)
    {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (byte >= static_cast<unsigned char>('a') && byte <= static_cast<unsigned char>('z'))
        {
            character = static_cast<char>(byte - ('a' - 'A'));
        }
    }

    return true;
}

bool is_confirmed_missing_errno(int error_number)
{
    return error_number == ENOENT || error_number == ENOTDIR;
}
}

SUPERVISOR::SUPERVISOR()
{
}
SUPERVISOR::~SUPERVISOR()
{
    stop();
}

void SUPERVISOR::start()
{
    State expected = State::UNINITIALIZED;
    if (!eState.compare_exchange_strong(expected, State::STARTING))
    {
        ESP_LOGW(TAG, "Cannot start supervisor from state %d", static_cast<int>(expected));
        return;
    }

    BaseType_t result = xTaskCreatePinnedToCore(taskWrapper, "SupervisorTask", SUPERVISOR_TASK_STACK_SIZE, this,
                                                tskIDLE_PRIORITY + 1, &xTaskHandle, SUPERVISOR_TASK_CORE_ID);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create SupervisorTask!");
        eState = State::ERROR;
    }
}

void SUPERVISOR::stop()
{
    State expected = State::RUNNING;
    if (!eState.compare_exchange_strong(expected, State::STOPPING))
    {
        ESP_LOGW(TAG, "Cannot stop supervisor from state %d", static_cast<int>(expected));
        return;
    }

    if (xTaskHandle != nullptr)
    {
        vTaskDelete(xTaskHandle);
        xTaskHandle = nullptr;
    }
}

void SUPERVISOR::taskWrapper(void* param)
{
    SUPERVISOR* supervisor = static_cast<SUPERVISOR*>(param);
    supervisor->task();
}

void SUPERVISOR::task()
{
    uint32_t inactive_timer = 0;
    uint32_t error_timer    = 0;
    while (1)
    {
        switch (eState)
        {
            case State::UNINITIALIZED:
                eState = State::STARTING;
                break;
            case State::STARTING:
            {
                eState = State::STARTING;

                bool critical_failed = false;

                for (auto& step : _setup_steps)
                {
                    ESP_LOGI(TAG, "Initializing module: %s", step.name);

                    step.result = step.init_fn();

                    if (step.result != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Setup failed for [%s] with error: %s", step.name, esp_err_to_name(step.result));

                        if (step.is_critical)
                        {
                            eState          = State::ERROR;
                            critical_failed = true;
                            break;
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Module %s: OK", step.name);
                    }
                }

                if (critical_failed)
                {
                    break;
                }

                s_errorRestarts = 0;  // a clean start ends the consecutive-ERROR count
                eState = State::NOT_CONNECTED;
                ESP_LOGI(TAG, "System INITIALIZED");

                break;
            }
            case State::NOT_CONNECTED:
                if (CanDriver::getInstance().isBusConnected())
                {
                    ESP_LOGI(TAG, "CAN bus connected, system RUNNING");
                    inactive_timer = 0;
                    eState         = State::RUNNING;
                }
                else
                {
                    ESP_LOGW(TAG, "Waiting for CAN bus connection...");
                    float voltage = get_battery_voltage();

                    if (std::isnan(voltage))
                    {
                        ESP_LOGW(TAG, "Battery voltage unknown (ADC unavailable); using the bus state only");
                    }
                    if (voltage > 13.2f)
                    {
                        ESP_LOGI(TAG, "Alternator is running (%.2fV). Staying awake.", voltage);
                        inactive_timer = 0;
                    }
                    else
                    {
                        inactive_timer++;
                        if (inactive_timer >= SLEEP_TRANSITION_TIMER_S)
                        {
                            ESP_LOGI(TAG, "CAN disconnected for %ds (Voltage %.2fV), transitioning to SLEEP",
                                     SLEEP_TRANSITION_TIMER_S, voltage);
                            inactive_timer = 0;
                            eState         = State::STOPPING;
                        }
                    }
                }
                break;
            case State::RUNNING:
                if (!CanDriver::getInstance().isBusConnected())
                {
                    ESP_LOGW(TAG, "CAN bus disconnected, system NOT_CONNECTED");
                    eState = State::NOT_CONNECTED;
                }
                // monitor voltage, bus connection, etc, separate function probably
                break;
            case State::STOPPING:
            {
                ESP_LOGI(TAG, "Shutting down modules for sleep...");
                // Shutdown retries every second; a component that never stops must not drain the battery forever.
                static uint8_t stopAttempts = 0;
                constexpr uint8_t kMaxStopAttempts = 10;
                if (++stopAttempts > kMaxStopAttempts)
                {
                    ESP_LOGE(TAG, "Shutdown failed %u times, restarting", kMaxStopAttempts);
                    esp_restart();
                }

                // Deinit components
                esp_err_t obdStop = stop_web_server();
                if (obdStop != ESP_OK)
                {
                    ESP_LOGE(TAG, "Web server shutdown incomplete (%s); retrying", esp_err_to_name(obdStop));
                    break;
                }
                obdStop = OBD2::getInstance().deinit();
                if (obdStop != ESP_OK)
                {
                    ESP_LOGE(TAG, "OBD2 shutdown incomplete (%s); retrying before CAN shutdown", esp_err_to_name(obdStop));
                    break;
                }
                WIFI::getInstance().deinit();
                obdStop = CanDriver::getInstance().deinit();
                if (obdStop != ESP_OK)
                {
                    ESP_LOGE(TAG, "CAN shutdown failed: %s", esp_err_to_name(obdStop));
                    break;
                }

                // 4. Put CAN transceiver into Standby Mode
                gpio_set_level(CAN_RS_GPIO, 1);

                // LED off and its task released for the sleep; the wake path calls init() again.
                LedStatus::getInstance().deinit();

                stopAttempts = 0;
                eState = State::SLEEPING;
                break;
            }
            case State::SLEEPING:
            {
                float voltage = get_battery_voltage();

                uint64_t sleep_time_us = 10000000;  // 10 seconds normal sleep

                if (std::isnan(voltage))
                {
                    ESP_LOGW(TAG, "Battery voltage unknown (ADC unavailable). Normal sleep...");
                }
                else if (voltage < LONG_SLEEP_VOLTAGE_LIMIT)
                {
                    sleep_time_us = 60000000;  // 60 seconds battery low sleep
                    ESP_LOGW(TAG, "Battery low (%.2fV)! Sleeping aggressively...", voltage);
                }
                else
                {
                    ESP_LOGI(TAG, "Battery OK (%.2fV). Normal sleep...", voltage);
                }
                ESP_LOGI(TAG, "Entering Light Sleep...");

                // Release the lock to allow Automatic Light Sleep
                if (pm_lock_)
                {
                    esp_pm_lock_release(pm_lock_);
                }

                // 1. Configure wakeup timer
                esp_sleep_enable_timer_wakeup(sleep_time_us);

                // 2. Just delay the task! FreeRTOS will seamlessly enter Light Sleep
                vTaskDelay(pdMS_TO_TICKS(sleep_time_us / 1000));

#ifdef CONFIG_PM_ENABLE
                // Re-acquire the lock to prevent sleep during operation
                if (pm_lock_)
                {
                    esp_pm_lock_acquire(pm_lock_);
                }
#endif

                ESP_LOGI(TAG, "Woke up. Target sleep duration achieved.");
                ESP_LOGI(TAG, "Checking bus activity...");

                gpio_set_level(CAN_RS_GPIO, 0);

                if (LedStatus::getInstance().init(LED_GPIO) != ESP_OK)
                {
                    ESP_LOGW(TAG, "LED init after wake failed; running without status LED");
                }

                esp_err_t startupErr = setup_can();
                if (startupErr != ESP_OK)
                {
                    ESP_LOGE(TAG, "CAN startup after wake failed: %s; entering ERROR", esp_err_to_name(startupErr));
                    eState = State::ERROR;
                    break;
                }

                if (CanDriver::getInstance().quickCheckBus())
                {
                    ESP_LOGI(TAG, "Bus is ALIVE! Starting full system...");

                    startupErr = setup_obd();
                    if (startupErr != ESP_OK)
                    {
                        ESP_LOGE(TAG, "OBD startup after wake failed: %s; retrying ordered shutdown",
                                 esp_err_to_name(startupErr));
                        eState = State::STOPPING;
                        break;
                    }

                    startupErr = setup_wifi();
                    if (startupErr != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Wi-Fi startup after wake failed: %s; retrying ordered shutdown",
                                 esp_err_to_name(startupErr));
                        eState = State::STOPPING;
                        break;
                    }

                    startupErr = setup_webserver();
                    if (startupErr != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Web server startup after wake failed: %s; retrying ordered shutdown",
                                 esp_err_to_name(startupErr));
                        eState = State::STOPPING;
                        break;
                    }

                    eState = State::NOT_CONNECTED;
                }
                else
                {
                    ESP_LOGI(TAG, "Bus is dead. Going back to sleep.");

                    esp_err_t canStop = stop_web_server();
                    if (canStop != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Web server shutdown after dead-bus check incomplete (%s); retrying ordered shutdown",
                                 esp_err_to_name(canStop));
                        eState = State::STOPPING;
                        break;
                    }
                    canStop = CanDriver::getInstance().deinit();
                    if (canStop != ESP_OK)
                    {
                        ESP_LOGE(TAG, "CAN shutdown after dead-bus check failed: %s; retrying ordered shutdown",
                                 esp_err_to_name(canStop));
                        eState = State::STOPPING;
                        break;
                    }
                    gpio_set_level(CAN_RS_GPIO, 1);
                    LedStatus::getInstance().deinit();
                }
                break;
            }
            case State::ERROR:
            {
                // Blink for a while so the fault is visible, then shut every module down like STOPPING does and
                // deep-sleep on a timer. Deep sleep powers Wi-Fi, the web server and the CAN node off; the wake is a
                // reboot. Terminal state: nothing else sets eState while it is ERROR.
                if (error_timer < error_policy::kBlinkS)
                {
                    error_timer++;
                    LedStatus::getInstance().blink(0xFFFFFFFF, 100, 100);
                    break;
                }

                const uint32_t attempt = error_policy::validCount(s_errorMagic, s_errorRestarts);
                const uint32_t backoff = error_policy::backoffS(attempt);
                s_errorMagic           = error_policy::kMagic;
                s_errorRestarts        = attempt + 1;
                ESP_LOGE(TAG, "ERROR state: deep sleep for %us, then reboot (consecutive error %u)",
                         static_cast<unsigned>(backoff), static_cast<unsigned>(attempt + 1));

                // Best effort, one pass: deep sleep clears whatever a failed step leaves behind.
                esp_err_t stopErr = stop_web_server();
                if (stopErr == ESP_OK)
                {
                    stopErr = OBD2::getInstance().deinit();
                }
                if (stopErr != ESP_OK)
                {
                    ESP_LOGW(TAG, "Shutdown before error sleep incomplete: %s", esp_err_to_name(stopErr));
                }
                WIFI::getInstance().deinit();
                if (CanDriver::getInstance().deinit() == ESP_OK)
                {
                    gpio_set_level(CAN_RS_GPIO, 1);  // transceiver standby
                }
                LedStatus::getInstance().deinit();

                esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(backoff) * 1000000ULL);
                esp_deep_sleep_start();  // does not return
                restart_system();
                break;
            }
            default:
                ESP_LOGE(TAG, "Unknown state %d", static_cast<int>(eState.load()));
                break;
        }
        SDCard::getInstance().update_card_status();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----- component setup functions --------------------------------------------------------------

esp_err_t SUPERVISOR::setup_power_management()
{
    SUPERVISOR& instance = getInstance();

    esp_pm_config_t pm_config = {.max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};

    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure power management: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_sleep", &instance.pm_lock_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create power lock: %s", esp_err_to_name(err));
        instance.pm_lock_ = nullptr;
        return err;
    }

    err = esp_pm_lock_acquire(instance.pm_lock_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to acquire power lock: %s", esp_err_to_name(err));
        esp_pm_lock_delete(instance.pm_lock_);
        instance.pm_lock_ = nullptr;
        return err;
    }

    return ESP_OK;
}

esp_err_t SUPERVISOR::setup_settings()
{
    esp_err_t err = Settings::getInstance().init();
    if (err != ESP_OK)
    {
        return err;
    }
    return SUPERVISOR::getInstance().load_config_from_nvs();
}

esp_err_t SUPERVISOR::setup_flash_filesystem()
{
    // 1. Mount WWW Partition (Read-Only)
    // Value-init then assign: IDF 6 littlefs adds a conditional `blockdev` member that designated init would flag.
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

    // Storage partition has been removed.

    return ESP_OK;
}

esp_err_t SUPERVISOR::setup_webserver()
{
    return setup_web_server();
}

// Only a corrupt NVS key partition justifies wiping the settings. A missing section is first boot (save the defaults).
// Any other read error (out of memory, bad handle, ...) runs this boot on the in-memory defaults and keeps the stored values.
template <class Cfg, class Get, class SetDefault>
static esp_err_t load_section(const char* name, Cfg& cfg, const Cfg& defaults, Get get, SetDefault setDefault)
{
    esp_err_t err = get(cfg);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "First boot detected. Saving default %s config.", name);
        setDefault();
        cfg = defaults;
        err = get(cfg);
    }
    else if (err == ESP_ERR_NVS_CORRUPT_KEY_PART)
    {
        const uint32_t resets = error_policy::validCount(s_errorMagic, s_errorRestarts);
        if (!error_policy::nvsResetAllowed(resets))
        {
            ESP_LOGE(TAG, "NVS still corrupt after %u factory resets; failing %s setup (ERROR back-off)",
                     static_cast<unsigned>(resets), name);
            return ESP_FAIL;
        }
        s_errorMagic    = error_policy::kMagic;
        s_errorRestarts = resets + 1;
        ESP_LOGE(TAG, "NVS corruption detected! Factory resetting (%u of %u)...", static_cast<unsigned>(resets + 1),
                 static_cast<unsigned>(error_policy::kMaxNvsResets));
        Settings::getInstance().factoryReset();
        SUPERVISOR::getInstance().restart_system();
        return ESP_FAIL;
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Reading %s config failed (%s); using defaults for this boot", name, esp_err_to_name(err));
        cfg = defaults;
    }
    return ESP_OK;
}

esp_err_t SUPERVISOR::setup_wifi()
{
    Settings&    settings = Settings::getInstance();
    WIFI::Config config;
    esp_err_t    err = load_section("WIFI", config, Settings::defaultWifiConfig(),
                                    [&](WIFI::Config& c) { return settings.getWifiConfig(c); }, [&] { settings.setDefaultWifiConfig(); });
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

esp_err_t SUPERVISOR::setup_can()
{
    Settings&         settings = Settings::getInstance();
    CanDriver::Config config;
    esp_err_t         err = load_section("CAN", config, Settings::defaultCanConfig(),
                                         [&](CanDriver::Config& c) { return settings.getCanConfig(c); }, [&] { settings.setDefaultCanConfig(); });
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

esp_err_t SUPERVISOR::setup_obd()
{
    auto& obd = OBD2::getInstance();

    // VIN and DTC on connect
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
                ESP_LOGW("SUPERVISOR", "VIN request failed: %s", esp_err_to_name(err));
            }

            const uint8_t dtc_modes[] = {MODE_DTCS, MODE_PENDING_DTCS, MODE_PERMANENT_DTCS};
            for (const uint8_t mode : dtc_modes)
            {
                err = obd2.requestDTC(mode);
                if (err != ESP_OK)
                {
                    ESP_LOGW("SUPERVISOR", "DTC mode 0x%02X request failed: %s", mode, esp_err_to_name(err));
                }
            }

            ESP_LOGI("SUPERVISOR", "DTC request sent, waiting for response...");
        });

    // Supervisor state on disconnect
    obd.connected_subscribe(
        [](bool connected)
        {
            if (!connected)
            {
                ESP_LOGI("SUPERVISOR", "OBD disconnected.");
                // Only RUNNING falls back; this must not overwrite ERROR, STOPPING or SLEEPING.
                State expected = State::RUNNING;
                SUPERVISOR::getInstance().eState.compare_exchange_strong(expected, State::NOT_CONNECTED);
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

esp_err_t SUPERVISOR::setup_battery()
{
    return battery_init();
}

esp_err_t SUPERVISOR::setup_sd_card()
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

uint32_t SUPERVISOR::get_uptime_seconds() const
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

std::string SUPERVISOR::get_restart_reason() const
{
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason)
    {
        case ESP_RST_UNKNOWN:
            return std::string("Unknown");
        case ESP_RST_POWERON:
            return std::string("Power on");
        case ESP_RST_EXT:
            return std::string("Software reset");
        case ESP_RST_PANIC:
            return std::string("Panic reset");
        case ESP_RST_INT_WDT:
            return std::string("Interrupt watchdog");
        case ESP_RST_TASK_WDT:
            return std::string("Task watchdog");
        case ESP_RST_WDT:
            return std::string("Watchdog reset");
        case ESP_RST_DEEPSLEEP:
            return std::string("Deep sleep");
        case ESP_RST_BROWNOUT:
            return std::string("Brownout reset");
        case ESP_RST_SDIO:
            return std::string("SDIO reset");
        case ESP_RST_USB:
            return std::string("USB reset");
        case ESP_RST_JTAG:
            return std::string("JTAG reset");
        case ESP_RST_EFUSE:
            return std::string("Efuse reset");
        case ESP_RST_PWR_GLITCH:
            return std::string("Power glitch");
        case ESP_RST_CPU_LOCKUP:
            return std::string("CPU lockup");
        default:
            return std::string("invalid reason");
    }
}

std::string SUPERVISOR::get_MAC_address() const
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char buf[18];
    (void)snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return std::string(buf);
}

void SUPERVISOR::restart_system()
{
    esp_restart();
}

float SUPERVISOR::get_battery_voltage() const
{
    return battery_read();
}

SUPERVISOR::PidDefinitionSaveResult SUPERVISOR::save_pid_def_to_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to write PID definitions outside /sdcard");
        return {ESP_ERR_INVALID_ARG, false};
    }

    auto& obd2 = OBD2::getInstance();
    std::vector<PIDDefinitionData> definitions;
    esp_err_t snapshotErr = obd2.getDefinitionSnapshot(definitions);
    if (snapshotErr != ESP_OK)
    {
        return {snapshotErr, false};
    }

    cJSON* rootArray = pidjson::serialize(definitions);
    if (rootArray == nullptr)
    {
        return {ESP_ERR_NO_MEM, false};
    }

    char* jsonString = cJSON_PrintUnformatted(rootArray);
    cJSON_Delete(rootArray);

    if (jsonString == nullptr)
    {
        return {ESP_ERR_NO_MEM, false};
    }

    const size_t len = strlen(jsonString);
    esp_err_t    ret = ESP_OK;

    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            free(jsonString);
            return {operation.status(), operation.status() == ESP_ERR_TIMEOUT};
        }

        // Temp file, fsync, rename: a failed or interrupted save keeps the previous PID file (SUP-2).
        switch (atomic_file::write(path, jsonString, len))
        {
            case atomic_file::Result::Ok:
                break;
            case atomic_file::Result::PathTooLong:
                ret = ESP_ERR_INVALID_ARG;
                break;
            case atomic_file::Result::NoMem:
                ret = ESP_ERR_NO_MEM;
                break;
            default:
                ESP_LOGE(TAG, "Failed to save PID definitions to %s; previous file kept", path);
                ret = ESP_FAIL;
                break;
        }
    }

    free(jsonString);

    if (ret != ESP_OK)
    {
        return {ret, false};
    }

    ESP_LOGI(TAG, "Successfully saved PID definitions to %s", path);
    return {ESP_OK, false};
}

esp_err_t SUPERVISOR::load_pid_def_from_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to read PID definitions outside /sdcard");
        return ESP_ERR_INVALID_ARG;
    }

    char*  buffer     = nullptr;
    size_t bytes_read = 0;

    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            return operation.status();
        }

        atomic_file::recover(path);  // finish a save that power loss interrupted

        struct stat st;
        if (stat(path, &st) != 0)
        {
            const int stat_errno = errno;
            if (is_confirmed_missing_errno(stat_errno))
            {
                ESP_LOGW(TAG, "JSON file not found (expected on first boot or missing): %s", path);
                return ESP_ERR_NOT_FOUND;
            }

            ESP_LOGE(TAG, "Failed to inspect JSON file %s (errno %d)", path, stat_errno);
            return ESP_FAIL;
        }

        if (st.st_size <= 0)
        {
            ESP_LOGW(TAG, "JSON file is empty or invalid: %s", path);
            return ESP_ERR_INVALID_ARG;
        }

        if (static_cast<uintmax_t>(st.st_size) > input_limits::PID_FILE_MAX)
        {
            ESP_LOGE(TAG, "JSON file is too large to load (%jd > %zu bytes): %s", static_cast<intmax_t>(st.st_size),
                     input_limits::PID_FILE_MAX, path);
            return ESP_ERR_INVALID_SIZE;
        }

        const size_t file_size = static_cast<size_t>(st.st_size);

        buffer = (char*)heap_caps_malloc(file_size + 1, MALLOC_CAP_SPIRAM);
        if (buffer == nullptr)
        {
            buffer = (char*)malloc(file_size + 1);
            if (buffer == nullptr)
            {
                ESP_LOGE(TAG, "Failed to allocate %zu bytes for JSON file", file_size + 1);
                return ESP_ERR_NO_MEM;
            }
        }

        FILE* f = fopen(path, "r");
        if (f == nullptr)
        {
            const int open_errno = errno;
            ESP_LOGE(TAG, "Failed to open file for reading: %s (errno %d)", path, open_errno);
            free(buffer);
            return is_confirmed_missing_errno(open_errno) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
        }

        errno      = 0;
        bytes_read = fread(buffer, 1, file_size, f);
        bool read_complete = bytes_read == file_size && ferror(f) == 0;

        // A successful fixed-size fread alone does not prove that the source
        // did not grow after stat().  Require EOF and an unchanged file size.
        if (read_complete)
        {
            errno = 0;
            if (fgetc(f) != EOF)
            {
                read_complete = false;
            }
        }
        if (ferror(f) != 0)
        {
            read_complete = false;
        }

        struct stat after_read;
        errno             = 0;
        const int fstat_ret = fstat(fileno(f), &after_read);
        if (fstat_ret != 0 || after_read.st_size != st.st_size)
        {
            read_complete = false;
        }

        const int close = fclose(f);

        if (!read_complete || close != 0)
        {
            ESP_LOGE(TAG, "Failed to read a complete, consistent JSON file from %s", path);
            free(buffer);
            return ESP_FAIL;
        }
    }

    buffer[bytes_read] = '\0';

    const char* parse_end = nullptr;
    cJSON* rootArray = cJSON_ParseWithLengthOpts(buffer, bytes_read, &parse_end, 0);

    if (rootArray != nullptr && parse_end != nullptr && parse_end >= buffer && parse_end <= buffer + bytes_read)
    {
        while (parse_end < buffer + bytes_read && std::isspace(static_cast<unsigned char>(*parse_end)))
        {
            ++parse_end;
        }

        if (parse_end != buffer + bytes_read)
        {
            cJSON_Delete(rootArray);
            rootArray = nullptr;
        }
    }
    else
    {
        cJSON_Delete(rootArray);
        rootArray = nullptr;
    }

    free(buffer);

    if (rootArray == nullptr || !cJSON_IsArray(rootArray))
    {
        ESP_LOGE(TAG, "Failed to parse complete JSON, or root is not an array");
        cJSON_Delete(rootArray);
        return ESP_ERR_INVALID_ARG;
    }

    int http_status = 200;
    cJSON* error_response = m_pid_def_put(rootArray, &http_status);
    bool success = error_response == nullptr && http_status == 204;
    cJSON_Delete(error_response);
    cJSON_Delete(rootArray);

    if (!success)
    {
        ESP_LOGW(TAG, "Failed to atomically load PID definitions from %s", path);
        if (http_status == 500)
        {
            return ESP_ERR_NO_MEM;
        }
        if (http_status == 503)
        {
            return ESP_ERR_TIMEOUT;
        }
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Successfully loaded PID definitions from %s", path);
    return ESP_OK;
}

esp_err_t SUPERVISOR::copy_file(const char* src_path, const char* dest_path)
{
    if (src_path == nullptr || dest_path == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::string canonical_source;
    std::string canonical_destination;
    if (!canonical_sd_relative_path(src_path, canonical_source) ||
        !canonical_sd_relative_path(dest_path, canonical_destination))
    {
        ESP_LOGE("FileOps", "Refusing file copy outside /sdcard: %s -> %s", src_path, dest_path);
        return ESP_ERR_INVALID_ARG;
    }

    if (canonical_source == canonical_destination)
    {
        ESP_LOGE("FileOps", "Refusing to copy a file onto itself: %s -> %s", src_path, dest_path);
        return ESP_ERR_INVALID_ARG;
    }

    const size_t chunk_size = 2048;
    char*        buffer     = (char*)malloc(chunk_size);
    if (buffer == nullptr)
    {
        ESP_LOGE("FileOps", "Failed to allocate memory for file copy buffer");
        return ESP_ERR_NO_MEM;
    }

    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            free(buffer);
            return operation.status();
        }

        FILE* f_src = fopen(src_path, "rb");
        if (f_src == nullptr)
        {
            ESP_LOGE("FileOps", "Failed to open source file: %s", src_path);
            free(buffer);
            return ESP_FAIL;
        }

        FILE* f_dest = fopen(dest_path, "wb");
        if (f_dest == nullptr)
        {
            ESP_LOGE("FileOps", "Failed to open destination file: %s", dest_path);
            (void)fclose(f_src);
            free(buffer);
            return ESP_FAIL;
        }

        size_t    bytes_read = 0;
        esp_err_t ret        = ESP_OK;

        while ((bytes_read = fread(buffer, 1, chunk_size, f_src)) > 0)
        {
            const size_t bytes_written = fwrite(buffer, 1, bytes_read, f_dest);
            if (bytes_written != bytes_read)
            {
                ESP_LOGE("FileOps", "Write error to %s (Disk full?)", dest_path);
                ret = ESP_FAIL;
                break;
            }
        }

        if (ferror(f_src) != 0)
        {
            ret = ESP_FAIL;
        }
        if (fflush(f_dest) != 0)
        {
            ret = ESP_FAIL;
        }

        free(buffer);
        const int src_close  = fclose(f_src);
        const int dest_close = fclose(f_dest);
        if (src_close != 0 || dest_close != 0)
        {
            ret = ESP_FAIL;
        }

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    ESP_LOGI("FileOps", "Successfully copied %s to %s", src_path, dest_path);
    return ESP_OK;
}

esp_err_t SUPERVISOR::load_config_from_nvs()
{
    Settings& settings = Settings::getInstance();
    return load_section("System", _config, Settings::defaultSupervisorConfig(),
                        [&](Config& c) { return settings.getSupervisorConfig(c); }, [&] { settings.setDefaultSupervisorConfig(); });
}

const char* SUPERVISOR::setup_function_to_string(SetupFunction func) const
{
    size_t index = static_cast<size_t>(func);
    if (index < _setup_steps.size())
    {
        return _setup_steps[index].name;
    }
    return "UNKNOWN";
}

esp_err_t SUPERVISOR::get_setup_result(SetupFunction func) const
{
    size_t index = static_cast<size_t>(func);
    if (index < _setup_steps.size())
    {
        return _setup_steps[index].result;
    }
    return ESP_ERR_INVALID_ARG;
}
