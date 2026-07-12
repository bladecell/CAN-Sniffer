// main.cpp
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "led_status.hpp"
#include "obd2.hpp"
#include "obd2_utils.hpp"
#include "supervisor.hpp"
#include "utilities.h"

#define PID_DERIVED_TEST_1 0xD001
#define PID_DERIVED_TEST_2 0xD002
#define PID_DERIVED_TEST_3 0xD003
#define PID_DERIVED_TEST_4 0xD004

static const char* TAG = "APP_MAIN";

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void)
{
    esp_log_level_set("CAN_DRIVER", ESP_LOG_DEBUG);
    LedStatus& led = LedStatus::getInstance();
    led.init(LED_GPIO);
    // esp_log_level_set("ASYNC_WEB_SERVER", ESP_LOG_DEBUG);

    SUPERVISOR::getInstance().start();

    while (SUPERVISOR::getInstance().get_state() != SUPERVISOR::State::RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGI(TAG, "Started");

    while (1)
    {
        vTaskDelay(1000);
    }
}