#pragma once

#include <atomic>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

struct CanDriverLifecycleTestAccess;

#define HEALTH_CHECK_TASK_PRIO 3  // Periodic monitoring
#define TX_TASK_PRIO 4            // Periodic monitoring
#define TX_QUEUE_SIZE 10
#define MIN_TRANSMIT_PERIOD_MS 4

#define CORE_ID_CAN_TASKS 1
#define HEALTHCHECK_PING_PERIOD_MS 2000
#define NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS 5000
#define TASK_STOP_TIMEOUT_MS 3000  // deinit waits this long for the TX and health tasks to leave on their own

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

    bool debug_mode = false;

    struct Config
    {
        Bitrate                   bitrate = Bitrate::BITRATE_500K;
        gpio_num_t                tx_pin  = GPIO_NUM_NC;
        gpio_num_t                rx_pin  = GPIO_NUM_NC;
        gpio_num_t                lbk_pin = GPIO_NUM_NC;
        gpio_num_t                rs_pin  = GPIO_NUM_NC;
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

    // Safe to call repeatedly. A failed init() releases everything it acquired. ESP_ERR_INVALID_STATE: a previous deinit()
    // failed and left resources behind (call deinit() again first).
    esp_err_t init(const Config& config);

    // Idempotent. Stops the TX and health tasks cooperatively; ESP_ERR_TIMEOUT (nothing freed, retry later) if they do not
    // leave within TASK_STOP_TIMEOUT_MS or if the connection callback is still running after 5 s.
    esp_err_t deinit();

    twai_node_status_t getStatus();

    esp_err_t transmit(CanDriver::CanFrame& tx_frame, int timeout_ms = 1000);
    // Puts the frame at the front of the TX queue and never waits (ISO-TP flow control must beat poll traffic).
    esp_err_t transmitUrgent(CanDriver::CanFrame& tx_frame);

    // Called for every frame that really left the controller (TWAI "TX done", ISR context, so the function must be
    // IRAM-safe, short and non-blocking) and, from the TX task, for a frame twai_node_transmit refused (success false).
    // `data` holds the first `length` (at most 8) data bytes and is only valid during the call.
    typedef void (*TxDoneFn)(void* arg, uint32_t id, const uint8_t* data, uint8_t length, bool success);
    void setTxDoneHook(TxDoneFn fn, void* arg)
    {
        txDoneFn_.store(nullptr);  // never run a new function with the old argument
        txDoneArg_.store(arg);
        txDoneFn_.store(fn);
    }

    esp_err_t receive(CanDriver::CanFrame& frame, int timeout_ms = 100);

    size_t availableMessages();

    esp_err_t flushRxQueue();

    const twai_onchip_node_config_t& getNodeConfig() const
    {
        return nodeConfig;
    }

    bool quickCheckBus();

    const Config& getConfig() const
    {
        return m_config;
    }

    // Connection Change Callback
    typedef void (*ConnectionChangeCallback_t)(void* arg, bool connected);
    esp_err_t setConnectionChangeCallback(ConnectionChangeCallback_t callback, void* arg,
                                          TickType_t waitTicks = portMAX_DELAY);
    bool isConnectionChangeCallbackTask() const;

    // RX Callback
    typedef void (*RxCallback_t)(void* arg);
    void setRxCallback(RxCallback_t callback, void* arg);

    // Simulation hooks (injected by the OBD2 simulator component at runtime)
    typedef void (*SimStartFn)(CanDriver*);
    typedef void (*SimStopFn)();
    // ISR context; must be IRAM_ATTR. Gets the CAN ID and the data bytes of every frame seen on the loopback bus.
    typedef void (*SimNotifyFn)(uint32_t id, const uint8_t* data, uint8_t length, BaseType_t* woken);
    void setSimHooks(SimStartFn start, SimStopFn stop, SimNotifyFn notify)
    {
        simStart_  = start;
        simStop_   = stop;
        simNotify_ = notify;
    }

    QueueHandle_t getRxQueueHandle()
    {
        return rxQueue;
    }

private:
    friend struct CanDriverLifecycleTestAccess;

    CanDriver(const CanDriver&)            = delete;
    CanDriver& operator=(const CanDriver&) = delete;
    // Callbacks
    static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx);

    static bool IRAM_ATTR twai_tx_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx);

    static bool IRAM_ATTR twai_state_change_cb(twai_node_handle_t handle, const twai_state_change_event_data_t* edata,
                                               void* user_ctx);

    static bool IRAM_ATTR twai_bus_err_cb(twai_node_handle_t handle, const twai_error_event_data_t* edata,
                                          void* user_ctx);

    // Lifecycle (see can_driver.cpp). lifecycleMutex_ serializes init()/deinit(); stopRequested_ asks both tasks to leave;
    // the *Running_ flags are the tasks' completion handshake (cleared by the task as its last action before vTaskDelete).
    esp_err_t             bringUp(const Config& config);
    esp_err_t             teardown();
    bool                  hasResources() const;
    SemaphoreHandle_t     lifecycleMutex_ = nullptr;
    std::atomic<bool>     stopRequested_{false};
    std::atomic<bool>     txRunning_{false};
    std::atomic<bool>     healthRunning_{false};

    // Connection Callback
    void                       connectionChangeCb(bool connected);
    ConnectionChangeCallback_t connectionChangeCallback    = nullptr;
    void*                      connectionChangeCallbackArg = nullptr;
    SemaphoreHandle_t          connectionChangeCallbackFence = nullptr;
    std::atomic<TaskHandle_t>   connectionChangeCallbackTask{nullptr};

    // RX Callback
    void         rxCb();
    RxCallback_t rxCallback    = nullptr;
    void*        rxCallbackArg = nullptr;

    // The TWAI driver queues only a pointer to the twai_frame_t and reports that same pointer in TX done, so a frame and its
    // data must stay untouched until then. At most tx_queue_depth frames wait in the driver plus one on the wire: a ring of
    // depth + 1 slots, allocated once in init() and freed in deinit() after the node is deleted. Used by the TX task only.
    struct TxSlot
    {
        twai_frame_t msg;
        uint8_t      data[8];
    };
    TxSlot* txRing_     = nullptr;
    size_t  txRingSize_ = 0;
    std::atomic<TxDoneFn> txDoneFn_{nullptr};
    std::atomic<void*>    txDoneArg_{nullptr};
    void IRAM_ATTR        notifyTxDone(const twai_frame_t* frame, bool success);

    // Simulation hooks (nullptr until injected by the simulator component)
    SimStartFn  simStart_  = nullptr;
    SimStopFn   simStop_   = nullptr;
    SimNotifyFn simNotify_ = nullptr;

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
