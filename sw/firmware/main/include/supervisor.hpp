// supervisor.hpp
#include <cstdint>
#include <string>

#include "async_web_server.hpp"
#include "can_driver.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "obd2_utils.hpp"
#include "secrets.h"
#include "settings.hpp"
#include "utilities.h"
#include "webserver.hpp"
#include "wifi.hpp"

#define SUPERVISOR_TASK_STACK_SIZE 4096
#define SUPERVISOR_TASK_CORE_ID 1

class SUPERVISOR
{
public:
    enum class State
    {
        UNINITIALIZED,
        STARTING,
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

    uint32_t    get_uptime_seconds() const;
    std::string get_restart_reason() const;
    std::string get_MAC_address() const;
    void        restart_system();

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

    esp_err_t (*setup_functions[4])() = {
        SUPERVISOR::setup_wifi,
        SUPERVISOR::setup_can,
        SUPERVISOR::setup_obd,
        setup_web_server,
    };
};