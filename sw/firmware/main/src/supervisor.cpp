// supervisor.cpp

#include "supervisor.hpp"

#include "can_driver.hpp"
#include "esp_log_level.h"
#include "led_status.hpp"

static const char* TAG = "SUPERVISOR";

SUPERVISOR::SUPERVISOR()
{
    // initialization code
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

    BaseType_t result =
        xTaskCreatePinnedToCore(taskWrapper, "SupervisorTask", 4096, this, tskIDLE_PRIORITY + 1, &xTaskHandle, 0);

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
    while (1)
    {
        switch (eState)
        {
            case State::UNINITIALIZED:
                eState = State::STARTING;
                break;
            case State::STARTING:
            {
                bool success = true;
                for (size_t i = 0; i < sizeof(setup_functions) / sizeof(setup_functions[0]); i++)
                {
                    if (setup_functions[i]() != ESP_OK)
                    {
                        LedStatus::getInstance().blink(0xFFFFFFFF, 100, 100);
                        success = false;
                    }
                }

                if (success)
                {
                    eState = State::RUNNING;
                    ESP_LOGI(TAG, "System RUNNING");
                }
                else
                {
                    eState = State::ERROR;
                }
                break;
            }
            case State::RUNNING:
                // monitor voltage, bus connection, etc, separate function probably
                break;
            case State::STOPPING:
                // gracefull shutdown
                break;
            case State::ERROR:
                // handle error, maybe try to restart components or just log and wait for manual intervention
                break;
            default:
                ESP_LOGE(TAG, "Unknown state %d", eState);
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ----- component setup functions --------------------------------------------------------------
esp_err_t SUPERVISOR::setup_wifi()
{
    // Configure
    WIFI::Config config;
    config.ssid            = "CAN-SNIFFER-AP";
    config.password        = "";
    config.channel         = 6;
    config.max_connections = 4;
    config.auth_mode       = WIFI_AUTH_OPEN;
    config.mode            = WIFI_MODE_STA;
    config.sta_ssid        = WIFI_SSID;
    config.sta_password    = WIFI_PASSWORD;
    config.sta_auth_mode   = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = WIFI::getInstance().init(config);
    ret |= WIFI::getInstance().start();
    return ret;
}

esp_err_t SUPERVISOR::setup_can()
{
    CanDriver::Config config;
    config.bitrate            = CanDriver::Bitrate::BITRATE_500K;
    config.rx_pin             = CAN_RX_GPIO;
    config.tx_pin             = CAN_TX_GPIO;
    config.lbk_pin            = CAN_LBK_GPIO;
    config.rs_pin             = CAN_RS_GPIO;
    config.debug              = DEBUG_MODE;
    config.filter             = false;
    config.mfilter_cfg.id     = 0b011100000000;  // 0b111 11100000
    config.mfilter_cfg.mask   = 0b011100000000;  // 0b111 11100000
    config.mfilter_cfg.is_ext = false;           // Standard 11-bit IDs

    // Settings::getInstance().getCanConfig(config);

    esp_err_t ret = CanDriver::getInstance().init(config);
    CanDriver::getInstance().setRxCallback(LedStatus::staticBlink, nullptr);

    vTaskDelay(pdMS_TO_TICKS(1000));
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

    esp_err_t ret = obd.init();

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize OBD2 interface");
        return ret;
    }

    ESP_LOGI(TAG, "OBD2 interface initialized");

    return ESP_OK;
}