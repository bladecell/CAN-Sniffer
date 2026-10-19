// main.cpp
#include "application_components.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_status.hpp"
#include "supervisor.hpp"
#include "utilities.h"

static const char* TAG = "APP_MAIN";

static esp_err_t registerComponents(SUPERVISOR& supervisor)
{
    using namespace supervisor;
    const Descriptor components[] = {
        {Id::POWER_MANAGEMENT, "POWER_MANAGEMENT", application_components::setupPowerManagement, nullptr,
         nullptr, true, Participation::RETAINED, Role::POWER_MANAGEMENT, 1},
        {Id::FLASH_FILESYSTEM, "FLASH_FILESYSTEM", application_components::setupFlashFilesystem, nullptr,
         nullptr, true, Participation::RETAINED, Role::FLASH_FILESYSTEM, 2},
        {Id::SD_CARD, "SD_CARD", application_components::setupSdCard,
         application_components::stopSdCard, nullptr, false, Participation::NORMAL, Role::SD_CARD, 3, 4},
        {Id::BATTERY, "BATTERY", application_components::setupBattery,
         application_components::stopBattery, nullptr, false, Participation::BATTERY, Role::BATTERY, 4},
        {Id::CAN, "CAN", application_components::setupCan, application_components::stopCan,
         nullptr, false, Participation::PROBE, Role::CAN, 5, 3},
        {Id::OBD,
         "OBD",
         application_components::setupObd,
         application_components::stopObd,
         nullptr,
         false,
         Participation::NORMAL,
         Role::OBD,
         6,
         1,
         {{{Id::CAN}, {Id::SD_CARD}}},
         2},
        {Id::WIFI, "WIFI", application_components::setupWifi,
         application_components::stopWifi, nullptr, true, Participation::NORMAL, Role::WIFI, 7, 2},
        {Id::WEBSERVER,
         "WEBSERVER",
         application_components::setupWebserver,
         application_components::stopWebserver,
         nullptr,
         true,
         Participation::NORMAL,
         Role::WEBSERVER,
         8,
         0,
         {{{Id::OBD}, {Id::WIFI}, {Id::FLASH_FILESYSTEM}, {Id::SD_CARD}}},
         4},
    };
    for (const auto& component : components)
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
        // No callbacks retain main's storage. Do not enter the RUNNING wait on
        // a configuration rejection; task-creation failure already ran recovery.
        return;
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
