// supervisor.cpp

#include "supervisor.hpp"

#include <cstring>

#include "async_web_server.hpp"
#include "battery.hpp"
#include "can_driver.hpp"
#include "esp_err.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "obd2_simulator.hpp"
#include "sd_card.hpp"
#include "settings.hpp"
#include "utilities.h"
#include "webserver.hpp"
#include "wifi.hpp"

static const char* TAG = "SUPERVISOR";

#define MOUNT_POINT "/sdcard"

SUPERVISOR::SUPERVISOR()
{
}
SUPERVISOR::~SUPERVISOR()
{
    stop();
}

void SUPERVISOR::start()
{
    if (eState != State::UNINITIALIZED)
    {
        ESP_LOGW(TAG, "Cannot start supervisor from state %d", eState);
        return;
    }

    eState = State::STARTING;

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
    if (eState != State::RUNNING)
    {
        ESP_LOGW(TAG, "Cannot stop supervisor from state %d", eState);
        return;
    }

    eState = State::STOPPING;

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
                }

                if (critical_failed)
                {
                    break;
                }

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

                // Deinit components
                stop_web_server();
                OBD2::getInstance().deinit();
                WIFI::getInstance().deinit();
                CanDriver::getInstance().deinit();

                // 4. Put CAN transceiver into Standby Mode
                gpio_set_level(CAN_RS_GPIO, 1);

                eState = State::SLEEPING;
                break;
            }
            case State::SLEEPING:
            {
                float voltage = get_battery_voltage();

                uint64_t sleep_time_us = 10000000;  // 10 seconds normal sleep

                if (voltage < LONG_SLEEP_VOLTAGE_LIMIT)
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
                    esp_pm_lock_release(pm_lock_);

                // 1. Configure wakeup timer
                esp_sleep_enable_timer_wakeup(sleep_time_us);

                // 2. Just delay the task! FreeRTOS will seamlessly enter Light Sleep
                vTaskDelay(pdMS_TO_TICKS(sleep_time_us / 1000));

#ifdef CONFIG_PM_ENABLE
                // Re-acquire the lock to prevent sleep during operation
                if (pm_lock_)
                    esp_pm_lock_acquire(pm_lock_);
#endif

                ESP_LOGI(TAG, "Woke up. Target sleep duration achieved.");
                ESP_LOGI(TAG, "Checking bus activity...");

                gpio_set_level(CAN_RS_GPIO, 0);

                setup_can();

                if (CanDriver::getInstance().quickCheckBus())
                {
                    ESP_LOGI(TAG, "Bus is ALIVE! Starting full system...");

                    setup_obd();
                    setup_wifi();
                    setup_webserver();

                    eState = State::NOT_CONNECTED;
                }
                else
                {
                    ESP_LOGI(TAG, "Bus is dead. Going back to sleep.");

                    stop_web_server();
                    CanDriver::getInstance().deinit();
                    gpio_set_level(CAN_RS_GPIO, 1);
                }
                break;
            }
            case State::ERROR:
                LedStatus::getInstance().blink(0xFFFFFFFF, 100, 100);
                break;
            default:
                ESP_LOGE(TAG, "Unknown state %d", eState);
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
    esp_vfs_littlefs_conf_t www_conf = {.base_path              = "/www",
                                        .partition_label        = "www",
                                        .partition              = nullptr,
                                        .format_if_mount_failed = false,
                                        .read_only              = true,
                                        .dont_mount             = false,
                                        .grow_on_mount          = false};

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

esp_err_t SUPERVISOR::setup_wifi()
{
    // Configure
    WIFI::Config config;
    esp_err_t    err = Settings::getInstance().getWifiConfig(config);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "First boot detected. Saving default WIFI config.");
        Settings::getInstance().setDefaultWifiConfig();
        err = Settings::getInstance().getWifiConfig(config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to retrieve default WIFI config: %s", esp_err_to_name(err));
            return err;
        }
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS corruption detected! Factory resetting...");
        Settings::getInstance().factoryReset();
        SUPERVISOR::getInstance().restart_system();
        return ESP_FAIL;
    }

    esp_err_t ret = WIFI::getInstance().init(config);
    ret |= WIFI::getInstance().start();
    return ret;
}

esp_err_t SUPERVISOR::setup_can()
{
    CanDriver::Config config;

    esp_err_t err = Settings::getInstance().getCanConfig(config);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "First boot detected. Saving default CAN config.");
        Settings::getInstance().setDefaultCanConfig();
        err = Settings::getInstance().getCanConfig(config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to retrieve default CAN config: %s", esp_err_to_name(err));
            return err;
        }
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS corruption detected! Factory resetting...");
        Settings::getInstance().factoryReset();
        SUPERVISOR::getInstance().restart_system();
        return ESP_FAIL;
    }

    if (config.debug)
        CanDriver::getInstance().setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    else
        CanDriver::getInstance().setSimHooks(nullptr, nullptr, nullptr);

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
                return;

            auto& obd2 = OBD2::getInstance();

            esp_err_t err = obd2.requestVIN();
            if (err != ESP_OK)
            {
                ESP_LOGW("SUPERVISOR", "VIN request failed: %s", esp_err_to_name(err));
            }

            err = obd2.requestDTC(MODE_DTCS);
            if (err == ESP_OK)
                err = obd2.requestDTC(MODE_PENDING_DTCS);
            // if (err == ESP_OK)
            //     err = obd2.requestDTC(MODE_PERMANENT_DTCS);

            ESP_LOGI("SUPERVISOR", "DTC request sent, waiting for response...");

            if (err != ESP_OK)
            {
                ESP_LOGW("SUPERVISOR", "DTC request failed: %s", esp_err_to_name(err));
            }
        });

    // Supervisor state on disconnect
    obd.connected_subscribe(
        [](bool connected)
        {
            if (!connected)
            {
                ESP_LOGI("SUPERVISOR", "OBD disconnected.");
                SUPERVISOR::getInstance().eState = State::NOT_CONNECTED;
            }
        });

    esp_err_t ret = obd.init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize OBD2 interface: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = SUPERVISOR::getInstance().load_pid_def_from_json(SUPERVISOR::getInstance().get_pid_def_path().c_str());

    if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Failed to initialize OBD2 interface: %s", esp_err_to_name(ret));
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
    config.format_if_mount_failed = true;

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

    ESP_LOGI(TAG, "SD card initialized and mounted at %s", config.base_path);

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
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

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

esp_err_t SUPERVISOR::save_pid_def_to_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to write PID definitions outside /sdcard");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON*                rootArray = cJSON_CreateArray();
    auto&                 obd2      = OBD2::getInstance();
    std::vector<uint16_t> pid_keys  = obd2.getPIDs();

    for (const auto& pid : pid_keys)
    {
        PIDDefinitionData def  = {};
        cJSON*            item = cJSON_CreateObject();

        obd2.getDef(pid, def);

        cJSON_AddNumberToObject(item, "id", def.id);
        cJSON_AddNumberToObject(item, "mode", def.mode);
        cJSON_AddNumberToObject(item, "pid", def.pid);
        cJSON_AddNumberToObject(item, "len", def.len);
        cJSON_AddStringToObject(item, "name", def.name.c_str());
        cJSON_AddStringToObject(item, "unit", def.unit.c_str());
        cJSON_AddStringToObject(item, "desc", def.description.c_str());
        cJSON_AddStringToObject(item, "formula", def.formula.c_str());
        cJSON_AddNumberToObject(item, "minV", def.minValue);
        cJSON_AddNumberToObject(item, "maxV", def.maxValue);
        cJSON_AddNumberToObject(item, "priority", def.priority);
        cJSON_AddNumberToObject(item, "interval", def.updateInterval_ms);
        cJSON_AddNumberToObject(item, "color", def.color);
        cJSON_AddStringToObject(item, "icon", def.icon.c_str());

        cJSON_AddItemToArray(rootArray, item);
    }

    char* jsonString = cJSON_PrintUnformatted(rootArray);
    cJSON_Delete(rootArray);

    if (jsonString == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    FILE* f = fopen(path, "w");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", path);
        free(jsonString);
        return ESP_FAIL;
    }

    size_t len     = strlen(jsonString);
    size_t written = fwrite(jsonString, 1, len, f);
    fclose(f);

    free(jsonString);

    if (written != len)
    {
        ESP_LOGE(TAG, "Failed to write complete JSON to %s", path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Successfully saved PID definitions to %s", path);
    return ESP_OK;
}

esp_err_t SUPERVISOR::load_pid_def_from_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to read PID definitions outside /sdcard");
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st;
    if (stat(path, &st) != 0)
    {
        ESP_LOGW(TAG, "JSON file not found (expected on first boot or missing): %s", path);
        return ESP_ERR_NOT_FOUND;
    }

    size_t file_size = st.st_size;
    if (file_size == 0)
    {
        ESP_LOGW(TAG, "JSON file is empty: %s", path);
        return ESP_OK;
    }

    char* buffer = (char*)heap_caps_malloc(file_size + 1, MALLOC_CAP_SPIRAM);
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
        ESP_LOGE(TAG, "Failed to open file for reading: %s", path);
        free(buffer);
        return ESP_FAIL;
    }

    size_t bytes_read = fread(buffer, 1, file_size, f);
    fclose(f);

    if (bytes_read == 0 && file_size > 0)
    {
        ESP_LOGE(TAG, "Failed to read data from %s", path);
        free(buffer);
        return ESP_FAIL;
    }

    buffer[bytes_read] = '\0';

    cJSON* rootArray = cJSON_Parse(buffer);
    free(buffer);

    if (rootArray == nullptr || !cJSON_IsArray(rootArray))
    {
        ESP_LOGE(TAG, "Failed to parse JSON, or root is not an array");
        cJSON_Delete(rootArray);
        return ESP_ERR_INVALID_ARG;
    }

    auto&     obd2        = OBD2::getInstance();
    cJSON*    item        = nullptr;
    esp_err_t add_pid_err = ESP_OK;

    std::vector<uint16_t> pid_keys = obd2.getPIDs();

    for (const auto& pid : pid_keys)
    {
        esp_err_t ret = obd2.removePID(pid);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to remove PID: %u - %s", pid, esp_err_to_name(ret));
            return ret;
        }
    }

    cJSON_ArrayForEach(item, rootArray)
    {
        if (!cJSON_IsObject(item))
            continue;

        cJSON* id       = cJSON_GetObjectItem(item, "id");
        cJSON* mode     = cJSON_GetObjectItem(item, "mode");
        cJSON* pid      = cJSON_GetObjectItem(item, "pid");
        cJSON* name     = cJSON_GetObjectItem(item, "name");
        cJSON* formula  = cJSON_GetObjectItem(item, "formula");
        cJSON* interval = cJSON_GetObjectItem(item, "interval");

        if (!cJSON_IsNumber(id) || !cJSON_IsNumber(mode) || !cJSON_IsNumber(pid) || !cJSON_IsString(name) ||
            !cJSON_IsString(formula) || !cJSON_IsNumber(interval))
        {
            ESP_LOGW(TAG, "Skipping malformed PID entry in JSON");
            continue;
        }

        uint16_t parsed_pid = static_cast<uint16_t>(pid->valueint);

        cJSON* len      = cJSON_GetObjectItem(item, "len");
        cJSON* unit     = cJSON_GetObjectItem(item, "unit");
        cJSON* desc     = cJSON_GetObjectItem(item, "desc");
        cJSON* minV     = cJSON_GetObjectItem(item, "minV");
        cJSON* maxV     = cJSON_GetObjectItem(item, "maxV");
        cJSON* priority = cJSON_GetObjectItem(item, "priority");
        cJSON* color    = cJSON_GetObjectItem(item, "color");
        cJSON* icon     = cJSON_GetObjectItem(item, "icon");

        uint8_t     parsed_len      = (parsed_pid > 0xFF) ? 3 : 2;
        std::string parsed_unit     = "";
        std::string parsed_desc     = "";
        float       parsed_minV     = 0.0f;
        float       parsed_maxV     = 0.0f;
        uint8_t     parsed_priority = 0;
        uint32_t    parsed_color    = 0x4EB31B;
        std::string parsed_icon     = "";

        if (cJSON_IsNumber(len))
        {
            parsed_len = static_cast<uint8_t>(len->valueint);
        }
        if (cJSON_IsString(unit))
        {
            parsed_unit = std::string(unit->valuestring);
        }
        if (cJSON_IsString(desc))
        {
            parsed_desc = std::string(desc->valuestring);
        }
        if (cJSON_IsNumber(minV))
        {
            parsed_minV = static_cast<float>(minV->valuedouble);
        }
        if (cJSON_IsNumber(maxV))
        {
            parsed_maxV = static_cast<float>(maxV->valuedouble);
        }
        if (cJSON_IsNumber(priority))
        {
            parsed_priority = static_cast<uint8_t>(priority->valueint);
        }
        if (cJSON_IsNumber(color))
        {
            parsed_color = static_cast<uint32_t>(color->valuedouble);
        }
        if (cJSON_IsString(icon))
        {
            parsed_icon = std::string(icon->valuestring);
        }

        esp_err_t ret = obd2.addPID(static_cast<uint32_t>(id->valuedouble), static_cast<uint8_t>(mode->valueint),
                                    parsed_pid, parsed_len, std::string(name->valuestring), parsed_unit, parsed_desc,
                                    std::string(formula->valuestring), parsed_minV, parsed_maxV, parsed_priority,
                                    static_cast<uint16_t>(interval->valueint), parsed_color, parsed_icon);
        if (ret != ESP_OK)
        {
            add_pid_err = ret;
        }
    }

    cJSON_Delete(rootArray);

    if (add_pid_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Loaded PID definitions from %s with errors: %s", path, esp_err_to_name(add_pid_err));
    }
    else
    {
        ESP_LOGI(TAG, "Successfully loaded PID definitions from %s", path);
    }
    return add_pid_err;
}

esp_err_t SUPERVISOR::copy_file(const char* src_path, const char* dest_path)
{
    if (src_path == nullptr || dest_path == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!SDCard::is_path_under(src_path, "/sdcard") || !SDCard::is_path_under(dest_path, "/sdcard"))
    {
        ESP_LOGE("FileOps", "Refusing file copy outside /sdcard: %s -> %s", src_path, dest_path);
        return ESP_ERR_INVALID_ARG;
    }

    FILE* f_src = fopen(src_path, "rb");
    if (f_src == nullptr)
    {
        ESP_LOGE("FileOps", "Failed to open source file: %s", src_path);
        return ESP_FAIL;
    }

    FILE* f_dest = fopen(dest_path, "wb");
    if (f_dest == nullptr)
    {
        ESP_LOGE("FileOps", "Failed to open destination file: %s", dest_path);
        fclose(f_src);
        return ESP_FAIL;
    }

    const size_t chunk_size = 2048;
    char*        buffer     = (char*)malloc(chunk_size);
    if (buffer == nullptr)
    {
        ESP_LOGE("FileOps", "Failed to allocate memory for file copy buffer");
        fclose(f_src);
        fclose(f_dest);
        return ESP_ERR_NO_MEM;
    }

    size_t    bytes_read = 0;
    esp_err_t ret        = ESP_OK;

    while ((bytes_read = fread(buffer, 1, chunk_size, f_src)) > 0)
    {
        size_t bytes_written = fwrite(buffer, 1, bytes_read, f_dest);
        if (bytes_written != bytes_read)
        {
            ESP_LOGE("FileOps", "Write error to %s (Disk full?)", dest_path);
            ret = ESP_FAIL;
            break;
        }
    }

    free(buffer);
    fclose(f_src);
    fclose(f_dest);

    if (ret == ESP_OK)
    {
        ESP_LOGI("FileOps", "Successfully copied %s to %s", src_path, dest_path);
    }

    return ret;
}

esp_err_t SUPERVISOR::load_config_from_nvs()
{
    esp_err_t err = Settings::getInstance().getSupervisorConfig(_config);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS config not found. Saving default System config.");
        Settings::getInstance().setDefaultSupervisorConfig();
        err = Settings::getInstance().getSupervisorConfig(_config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to retrieve default System config: %s", esp_err_to_name(err));
            return err;
        }
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS corruption detected! Factory resetting...");
        Settings::getInstance().factoryReset();
        restart_system();
        return ESP_FAIL;
    }

    return ESP_OK;
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