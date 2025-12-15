#pragma once

#include <atomic>

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "obd2_simulator.hpp"

#define STATE_NOT_INITIALIZED 0
#define STATE_BUS_OFF 10
#define STATE_NOT_CONNECTED 20
#define STATE_CONNECTED 40

#define HEALTH_CHECK_TASK_PRIO 3 // Periodic monitoring

#define CORE_ID_CAN_TASKS 0
#define MIN_TRANSMIT_PERIOD_MS 8 // Minimum time between consecutive transmissions in milliseconds
#define HEALTHCHECK_PING_PERIOD_MS 2000

#define LOG_CAN_FRAME(LOG_TAG, DIR, CAN_ID, DATA_PTR)                    \
    ESP_LOGD(LOG_TAG, "%s %X %02X %02X %02X %02X %02X %02X %02X %02X",   \
             DIR,                                                        \
             CAN_ID,                                                     \
             (DATA_PTR)[0], (DATA_PTR)[1], (DATA_PTR)[2], (DATA_PTR)[3], \
             (DATA_PTR)[4], (DATA_PTR)[5], (DATA_PTR)[6], (DATA_PTR)[7])

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
        BITRATE_1M = 1000000
    };
    struct CanFrame
    {
        uint32_t id;
        uint8_t data[8];
        size_t length;
    };

    bool debug_mode;

    explicit CanDriver(Bitrate bitrate = Bitrate::BITRATE_500K, gpio_num_t tx_pin = GPIO_NUM_5, gpio_num_t rx_pin = GPIO_NUM_4, gpio_num_t lbk_pin = GPIO_NUM_6, bool debug = false, uint32_t tx_queue_depth = 20U, size_t rx_queue_size = 20);

    inline bool isInitialized() const
    {
        return canState.load() != STATE_NOT_INITIALIZED;
    }

    inline bool isBusConnected() const
    {
        return canState.load() == STATE_CONNECTED;
    }

    void setDebugMode(bool enable);

    esp_err_t init(bool debug = false);

    esp_err_t deinit();

    twai_node_status_t getStatus();

    esp_err_t transmit(twai_frame_t *tx_msg, int timeout_ms = 1000);

    esp_err_t receive(CanDriver::CanFrame &frame, int timeout_ms = 100);

    size_t availableMessages();

    esp_err_t flushRxQueue();

    typedef void (*ConnectionCallback)(void *arg, bool connected);
    void setConnectionCallback(ConnectionCallback callback, void *arg);

    QueueHandle_t getRxQueueHandle()
    {
        return rxQueue;
    }

protected:
    void notifyConnectionChange(bool connected);

private:
    // Callbacks
    static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx);

    static bool IRAM_ATTR twai_tx_cb(twai_node_handle_t handle,
                                     const twai_tx_done_event_data_t *edata,
                                     void *user_ctx);

    static bool IRAM_ATTR twai_state_change_cb(twai_node_handle_t handle,
                                               const twai_state_change_event_data_t *edata,
                                               void *user_ctx);

    static bool IRAM_ATTR twai_bus_err_cb(twai_node_handle_t handle,
                                          const twai_error_event_data_t *edata,
                                          void *user_ctx);

    // Connection Callback
    ConnectionCallback connectionCallback = nullptr;
    void *callbackArg;

    // Twai Configuration
    twai_onchip_node_config_t nodeConfig{};
    twai_node_handle_t nodeHdl;
    twai_node_record_t nodeRecord{};
    gpio_num_t LBK_PIN;

    // RX Queue
    QueueHandle_t rxQueue;
    size_t RX_QUEUE_SIZE;

    // Bus checks
    esp_err_t pingBus();
    std::atomic<uint32_t> consecutiveStuffErrors = 1;
    std::atomic<uint32_t> consecutiveAckErrors = 1;
    void healthCheckTask();
    std::atomic<uint8_t> canState = STATE_NOT_INITIALIZED;
    static void healthCheckTaskWrapper(void *param);
    TaskHandle_t healthCheckTaskHandle = nullptr;
};
