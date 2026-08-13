// main.cpp
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_status.hpp"
#include "supervisor.hpp"
#include "utilities.h"

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