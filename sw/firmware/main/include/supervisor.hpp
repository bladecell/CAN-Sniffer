// supervisor.hpp
#pragma once
#include <cstdint>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SUPERVISOR_TASK_STACK_SIZE 4096
#define SUPERVISOR_TASK_CORE_ID 1

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

private:
    SUPERVISOR(const SUPERVISOR&)            = delete;
    SUPERVISOR& operator=(const SUPERVISOR&) = delete;

    void              task();
    static void       taskWrapper(void* param);
    TaskHandle_t      xTaskHandle;
    SUPERVISOR::State eState = State::UNINITIALIZED;

    // component initializers
    static esp_err_t setup_wifi();
    static esp_err_t setup_can();
    static esp_err_t setup_obd();
    static esp_err_t setup_battery();

    esp_err_t (*setup_functions[5])() = {
        SUPERVISOR::setup_battery, SUPERVISOR::setup_can, SUPERVISOR::setup_obd, SUPERVISOR::setup_wifi, nullptr,
    };
};