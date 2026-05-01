// supervisor.cpp

#include "supervisor.hpp"

#include "can_driver.hpp"
#include "esp_log_level.h"
#include "esp_system.h"
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

    BaseType_t result = xTaskCreatePinnedToCore(taskWrapper, "SupervisorTask", SUPERVISOR_TASK_STACK_SIZE, this,
                                                tskIDLE_PRIORITY + 1, &xTaskHandle, SUPERVISOR_TASK_CORE_ID);

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

uint32_t SUPERVISOR::get_uptime_seconds() const
{
    return xTaskGetTickCount() / configTICK_RATE_HZ;
}

std::string SUPERVISOR::get_restart_reason() const
{
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason)
    {
        case ESP_RST_UNKNOWN:
            return std::string("Unknown");
        case ESP_RST_POWERON:
            return std::string("Power on");
        case ESP_RST_EXT:
            return std::string("Software reset");
        case ESP_RST_PANIC:
            return std::string("Panic reset");
        case ESP_RST_INT_WDT:
            return std::string("Interrupt watchdog");
        case ESP_RST_TASK_WDT:
            return std::string("Task watchdog");
        case ESP_RST_WDT:
            return std::string("Watchdog reset");
        case ESP_RST_DEEPSLEEP:
            return std::string("Deep sleep");
        case ESP_RST_BROWNOUT:
            return std::string("Brownout reset");
        case ESP_RST_SDIO:
            return std::string("SDIO reset");
        case ESP_RST_USB:
            return std::string("USB reset");
        case ESP_RST_JTAG:
            return std::string("JTAG reset");
        case ESP_RST_EFUSE:
            return std::string("Efuse reset");
        case ESP_RST_PWR_GLITCH:
            return std::string("Power glitch");
        case ESP_RST_CPU_LOCKUP:
            return std::string("CPU lockup");
        default:
            return std::string("invalid reason");
    }
}

std::string SUPERVISOR::get_MAC_address() const
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char buf[18];  // "AA:BB:CC:DD:EE:FF\0"
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return std::string(buf);
}

void SUPERVISOR::restart_system()
{
    esp_restart();
}
