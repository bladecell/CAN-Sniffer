#pragma once

#include <atomic>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define HEALTH_CHECK_TASK_PRIO 3  // Periodic monitoring
#define TX_TASK_PRIO 4            // Periodic monitoring
#define TX_QUEUE_SIZE 10
#define MIN_TRANSMIT_PERIOD_MS 4

#define CORE_ID_CAN_TASKS 1
#define HEALTHCHECK_PING_PERIOD_MS 2000

class CanDriver
{
public:
    ~CanDriver();
    bool initialized = false;
    enum class Bitrate
    {
        BITRATE_125K = 125000,
        BITRATE_250K = 250000,
        BITRATE_500K = 500000,
        BITRATE_1M   = 1000000
    };
    struct CanFrame
    {
        twai_frame_header_t header;
        uint8_t             data[8];
        size_t              length;
    };

    enum class STATE
    {
        NOT_INITIALIZED = 0,
        BUS_OFF         = 1,
        NOT_CONNECTED   = 2,
        CONNECTED       = 3
    };

    enum class RS_MODE
    {
        HIGH_SPEED    = 0,
        SLOPE_CONTROL = 1,
    };

    bool debug_mode;

    struct Config
    {
        Bitrate                   bitrate;
        gpio_num_t                tx_pin;
        gpio_num_t                rx_pin;
        gpio_num_t                lbk_pin;
        gpio_num_t                rs_pin;
        bool                      debug          = false;
        RS_MODE                   rs_mode        = RS_MODE::HIGH_SPEED;
        uint32_t                  tx_queue_depth = 20U;
        size_t                    rx_queue_size  = 20;
        bool                      filter         = false;
        twai_mask_filter_config_t mfilter_cfg    = {};
    };

    static CanDriver& getInstance()
    {
        static CanDriver instance;
        return instance;
    }

    CanDriver();

    inline bool isInitialized() const
    {
        return canState.load() != STATE::NOT_INITIALIZED;
    }

    inline bool isBusConnected() const
    {
        return canState.load() == STATE::CONNECTED;
    }

    STATE getState() const
    {
        return canState.load();
    }

    void setDebugMode(bool enable);

    esp_err_t init(const Config& config);

    esp_err_t deinit();

    twai_node_status_t getStatus();

    esp_err_t transmit(CanDriver::CanFrame& tx_frame, int timeout_ms = 1000);

    esp_err_t receive(CanDriver::CanFrame& frame, int timeout_ms = 100);

    size_t availableMessages();

    esp_err_t flushRxQueue();

    twai_onchip_node_config_t getNodeConfig() const
    {
        return nodeConfig;
    }

    bool quickCheckBus();

    Config getConfig() const
    {
        return m_config;
    }

    // Connection Change Callback
    typedef void (*ConnectionChangeCallback_t)(void* arg, bool connected);
    void setConnectionChangeCallback(ConnectionChangeCallback_t callback, void* arg);

    // RX Callback
    typedef void (*RxCallback_t)(void* arg);
    void setRxCallback(RxCallback_t callback, void* arg);

    QueueHandle_t getRxQueueHandle()
    {
        return rxQueue;
    }

private:
    CanDriver(const CanDriver&)            = delete;
    CanDriver& operator=(const CanDriver&) = delete;
    // Callbacks
    static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx);

    static bool IRAM_ATTR twai_tx_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx);

    static bool IRAM_ATTR twai_state_change_cb(twai_node_handle_t handle, const twai_state_change_event_data_t* edata,
                                               void* user_ctx);

    static bool IRAM_ATTR twai_bus_err_cb(twai_node_handle_t handle, const twai_error_event_data_t* edata,
                                          void* user_ctx);

    // Connection Callback
    void                       connectionChangeCb(bool connected);
    ConnectionChangeCallback_t connectionChangeCallback    = nullptr;
    void*                      connectionChangeCallbackArg = nullptr;

    // RX Callback
    void         rxCb();
    RxCallback_t rxCallback    = nullptr;
    void*        rxCallbackArg = nullptr;

    // Twai Configuration
    twai_onchip_node_config_t nodeConfig{};
    twai_node_handle_t        nodeHdl;
    twai_node_record_t        nodeRecord{};
    gpio_num_t                LBK_PIN;
    Config                    m_config{};

    // RX Queue
    QueueHandle_t rxQueue;
    size_t        RX_QUEUE_SIZE;

    // Bus checks
    esp_err_t             pingBus();
    std::atomic<uint32_t> consecutiveStuffErrors = 1;
    std::atomic<uint32_t> consecutiveAckErrors   = 1;
    void                  healthCheckTask();
    std::atomic<STATE>    canState;
    static void           healthCheckTaskWrapper(void* param);
    TaskHandle_t          healthCheckTaskHandle = nullptr;

    // TX
    TickType_t    last_tx_ticks = 0;
    void          txTask();
    static void   txTaskWrapper(void* param);
    TaskHandle_t  txTaskHandle = nullptr;
    QueueHandle_t txQueue;
};
