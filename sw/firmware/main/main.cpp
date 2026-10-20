// main.cpp
#include "application_components.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "error_policy.hpp"
#include "esp_sleep.h"
#include "led_status.hpp"
#include "production_components.hpp"
#include "supervisor.hpp"
#include "utilities.h"

static const char* TAG = "APP_MAIN";

static esp_err_t registerComponents(SUPERVISOR& supervisor)
{
    for (const auto& component : production_components::descriptors())
    {
        const esp_err_t error = supervisor.registerComponent(component);
        if (error != ESP_OK)
        {
            return error;
        }
    }
    return ESP_OK;
}

static const char* resetReasonName(esp_reset_reason_t r)
{
    switch (r)
    {
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXT";
        case ESP_RST_SW:
            return "SW";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INT_WDT";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT";
        case ESP_RST_WDT:
            return "WDT";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        case ESP_RST_USB:
            return "USB";
        case ESP_RST_JTAG:
            return "JTAG";
        case ESP_RST_EFUSE:
            return "EFUSE";
        case ESP_RST_PWR_GLITCH:
            return "PWR_GLITCH";
        case ESP_RST_CPU_LOCKUP:
            return "CPU_LOCKUP";
        default:
            return "UNKNOWN";
    }
}

// ----- app_main --------------------------------------------------------------
extern "C" void app_main(void)
{
    const esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGI(TAG, "Reset reason: %s (%d), free heap %u, min free heap %u", resetReasonName(reason), (int)reason,
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    esp_log_level_set("CAN_DRIVER", ESP_LOG_DEBUG);
    LedStatus& led = LedStatus::getInstance();
    if (led.init(LED_GPIO) != ESP_OK)
    {
        ESP_LOGW(TAG, "LED init failed; running without status LED");
    }
    // esp_log_level_set("ASYNC_WEB_SERVER", ESP_LOG_DEBUG);

    auto&     supervisor = SUPERVISOR::getInstance();
    esp_err_t error      = registerComponents(supervisor);
    if (error == ESP_OK)
    {
        // Admission runs before this synchronous infrastructure stage; ordinary
        // component initialization remains on the supervisor task afterward.
        error = supervisor.init({nullptr, application_components::bootstrapSettings});
    }
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Startup rejected: %s", esp_err_to_name(error));
        // No supervisor task exists to manage power, so nothing else would ever sleep. Rejection is deterministic:
        // deep sleep at the longest backoff and retry on timer wake. Task-creation failure already ran recovery.
        (void)led.deinit();
        if (esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(error_policy::kMaxBackoffS) * 1000000ULL) == ESP_OK)
        {
            esp_deep_sleep_start();
        }
        ESP_LOGE(TAG, "Deep sleep unavailable; idling");
        while (1)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }

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
