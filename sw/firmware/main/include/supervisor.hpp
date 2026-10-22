#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "supervisor_core.hpp"

#define SUPERVISOR_TASK_STACK_SIZE 4096
#define SUPERVISOR_TASK_CORE_ID 1
#define PID_DEF_DB_PATH "/sdcard/config/pid_def.json"
#define DTC_DESC_DB_PATH "/sdcard/config/dtcs.bin"
#define SLEEP_TRANSITION_TIMER_S supervisor::kInactiveTicks
#define LONG_SLEEP_VOLTAGE_LIMIT supervisor::kLongSleepVoltage

class SUPERVISOR
{
public:
    struct PidDefinitionSaveResult
    {
        // Set only when the bounded SD operation lease could not be acquired.
        esp_err_t error;
        bool      sd_card_busy;
    };

    struct Config
    {
        std::string            pid_def_path  = PID_DEF_DB_PATH;
        std::string            dtc_desc_path = DTC_DESC_DB_PATH;
        power_settings::Config power{};
    };

    using State         = supervisor::State;
    using SetupFunction = supervisor::Id;
    using SetupStep     = supervisor::SetupStep;

    static SUPERVISOR& getInstance()
    {
        static SUPERVISOR instance;
        return instance;
    }

    const char* setup_function_to_string(SetupFunction func) const;
    esp_err_t   get_setup_result(SetupFunction func) const;
    const auto& get_setup_steps() const
    {
        return core_.legacySetupSteps();
    }

    esp_err_t registerComponent(const supervisor::Descriptor& descriptor)
    {
        return core_.registerComponent(descriptor);
    }
    esp_err_t init(const supervisor::SettingsBootstrap& bootstrap);
    // Request the usual ordered sleep cycle, without blocking a teardown dependency.
    void     stop();
    uint32_t reserve_sleep_request();
    void     finish_sleep_request(uint32_t token, bool responseSent);

    uint32_t          get_uptime_seconds() const;
    const char*       get_restart_reason() const;
    void              get_MAC_address(char (&out)[18]) const;
    float             get_battery_voltage() const;
    void              restart_system();
    SUPERVISOR::State get_state() const
    {
        return core_.state();
    }

    supervisor::Lifecycle get_lifecycle_status(SetupFunction id) const
    {
        return core_.status(id);
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

    esp_err_t               load_config_from_nvs();
    PidDefinitionSaveResult save_pid_def_to_json(const char* path);
    esp_err_t               load_pid_def_from_json(const char* path);
    esp_err_t               copy_file(const char* src_path, const char* dest_path);

    // Application event only: the core guards RUNNING -> NOT_CONNECTED.
    void notify_disconnected() { core_.disconnected(); }
    // Settings-loading task path only. Applies the existing bounded factory-reset
    // policy, then restarts; no caller can read or mutate raw RTC recovery state.
    esp_err_t recover_corrupt_settings(const char* section);

private:
    // Tasks and callbacks retain this address for the lifetime of the firmware.
    SUPERVISOR();
    ~SUPERVISOR()                            = default;
    SUPERVISOR(const SUPERVISOR&)            = delete;
    SUPERVISOR& operator=(const SUPERVISOR&) = delete;

    void                          task();
    static void                   taskWrapper(void* param);
    static supervisor::Operations operations(SUPERVISOR* self);

    TaskHandle_t         xTaskHandle   = nullptr;
    portMUX_TYPE         lifecycleMux_ = portMUX_INITIALIZER_UNLOCKED;
    Config               _config       = {};

    supervisor::Core core_;
};
