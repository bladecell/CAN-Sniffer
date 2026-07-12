// supervisor.hpp
#pragma once
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SUPERVISOR_TASK_STACK_SIZE 4096
#define SUPERVISOR_TASK_CORE_ID 1
#define PID_DEF_DB_PATH "/storage/pid_def.json"
#define DTC_DESC_DB_PATH "/storage/dtcs.bin"

class SUPERVISOR
{
public:
    enum class State
    {
        UNINITIALIZED,
        STARTING,
        NOT_CONNECTED,
        RUNNING,
        STOPPING,
        ERROR,
    };

    SUPERVISOR();
    ~SUPERVISOR();

    static SUPERVISOR& getInstance()
    {
        static SUPERVISOR instance;
        return instance;
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

    esp_err_t save_pid_def_to_json(const char* path);
    esp_err_t load_pid_def_from_json(const char* path);
    esp_err_t copy_file(const char* src_path, const char* dest_path);

private:
    SUPERVISOR(const SUPERVISOR&)            = delete;
    SUPERVISOR& operator=(const SUPERVISOR&) = delete;

    void              task();
    static void       taskWrapper(void* param);
    TaskHandle_t      xTaskHandle;
    SUPERVISOR::State eState = State::UNINITIALIZED;

    // component initializers
    static esp_err_t setup_flash_filesystem();
    static esp_err_t setup_wifi();
    static esp_err_t setup_can();
    static esp_err_t setup_obd();
    static esp_err_t setup_battery();
    static esp_err_t setup_sd_card();
    static esp_err_t setup_webserver();

    esp_err_t (*setup_functions[7])() = {
        SUPERVISOR::setup_flash_filesystem,
        SUPERVISOR::setup_sd_card,
        SUPERVISOR::setup_battery,
        SUPERVISOR::setup_can,
        SUPERVISOR::setup_obd,
        SUPERVISOR::setup_wifi,
        SUPERVISOR::setup_webserver,
    };
};