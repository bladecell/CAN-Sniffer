#pragma once
#include <array>
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SUPERVISOR_TASK_STACK_SIZE 4096
#define SUPERVISOR_TASK_CORE_ID 1
#define PID_DEF_DB_PATH "/sdcard/config/pid_def.json"
#define DTC_DESC_DB_PATH "/sdcard/config/dtcs.bin"
#define SLEEP_TRANSITION_TIMER_S 60
#define LONG_SLEEP_VOLTAGE_LIMIT 11.8f

class SUPERVISOR
{
public:
    struct Config
    {
        std::string pid_def_path  = PID_DEF_DB_PATH;
        std::string dtc_desc_path = DTC_DESC_DB_PATH;
    };

    enum class State
    {
        UNINITIALIZED,
        STARTING,
        NOT_CONNECTED,
        RUNNING,
        STOPPING,
        SLEEPING,
        ERROR,
    };

    enum class SetupFunction : size_t
    {
        SETTINGS = 0,
        POWER_MANAGEMENT,
        FLASH_FILESYSTEM,
        SD_CARD,
        BATTERY,
        CAN,
        OBD,
        WIFI,
        WEBSERVER,
        COUNT
    };

    struct SetupStep
    {
        SetupFunction id;
        const char*   name;
        esp_err_t (*init_fn)();
        esp_err_t result      = ESP_FAIL;
        bool      is_critical = false;
    };

    SUPERVISOR();
    ~SUPERVISOR();

    static SUPERVISOR& getInstance()
    {
        static SUPERVISOR instance;
        return instance;
    }

    const char* setup_function_to_string(SetupFunction func) const;
    esp_err_t   get_setup_result(SetupFunction func) const;
    const auto& get_setup_steps() const
    {
        return _setup_steps;
    }

    void start();
    void stop();

    uint32_t          get_uptime_seconds() const;
    std::string       get_restart_reason() const;
    std::string       get_MAC_address() const;
    float             get_battery_voltage() const;
    void              restart_system();
    SUPERVISOR::State get_state() const
    {
        return eState;
    }

    std::string get_pid_def_path() const
    {
        return _config.pid_def_path;
    }
    std::string get_dtc_desc_path() const
    {
        return _config.dtc_desc_path;
    }
    void set_pid_def_path(const std::string& path)
    {
        _config.pid_def_path = path;
    }
    void set_dtc_desc_path(const std::string& path)
    {
        _config.dtc_desc_path = path;
    }

    esp_err_t load_config_from_nvs();
    esp_err_t save_pid_def_to_json(const char* path);
    esp_err_t load_pid_def_from_json(const char* path);
    esp_err_t copy_file(const char* src_path, const char* dest_path);

private:
    SUPERVISOR(const SUPERVISOR&)            = delete;
    SUPERVISOR& operator=(const SUPERVISOR&) = delete;

    void        task();
    static void taskWrapper(void* param);

    TaskHandle_t         xTaskHandle = nullptr;
    SUPERVISOR::State    eState      = State::UNINITIALIZED;
    esp_pm_lock_handle_t pm_lock_    = nullptr;
    Config               _config     = {};

    static esp_err_t setup_settings();
    static esp_err_t setup_power_management();
    static esp_err_t setup_flash_filesystem();
    static esp_err_t setup_wifi();
    static esp_err_t setup_can();
    static esp_err_t setup_obd();
    static esp_err_t setup_battery();
    static esp_err_t setup_sd_card();
    static esp_err_t setup_webserver();

    std::array<SetupStep, static_cast<size_t>(SetupFunction::COUNT)> _setup_steps = {{
        {SetupFunction::SETTINGS, "SETTINGS", &SUPERVISOR::setup_settings, ESP_FAIL, true},
        {SetupFunction::POWER_MANAGEMENT, "POWER_MANAGEMENT", &SUPERVISOR::setup_power_management, ESP_FAIL, true},
        {SetupFunction::FLASH_FILESYSTEM, "FLASH_FILESYSTEM", &SUPERVISOR::setup_flash_filesystem, ESP_FAIL, true},
        {SetupFunction::SD_CARD, "SD_CARD", &SUPERVISOR::setup_sd_card, ESP_FAIL, false},
        {SetupFunction::BATTERY, "BATTERY", &SUPERVISOR::setup_battery, ESP_FAIL, false},
        {SetupFunction::CAN, "CAN", &SUPERVISOR::setup_can, ESP_FAIL, false},
        {SetupFunction::OBD, "OBD", &SUPERVISOR::setup_obd, ESP_FAIL, false},
        {SetupFunction::WIFI, "WIFI", &SUPERVISOR::setup_wifi, ESP_FAIL, true},
        {SetupFunction::WEBSERVER, "WEBSERVER", &SUPERVISOR::setup_webserver, ESP_FAIL, true},
    }};
};