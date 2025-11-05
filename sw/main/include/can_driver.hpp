#pragma once

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

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
    explicit CanDriver(Bitrate bitrate = Bitrate::BITRATE_500K, gpio_num_t tx_pin = GPIO_NUM_5, gpio_num_t rx_pin = GPIO_NUM_4, gpio_num_t lbk_pin = GPIO_NUM_6, uint32_t tx_queue_depth = 5U, size_t rx_queue_size = 20)
    {
        nodeConfig.io_cfg.tx = tx_pin;
        nodeConfig.io_cfg.rx = rx_pin;
        nodeConfig.bit_timing.bitrate = static_cast<uint32_t>(bitrate);
        nodeConfig.tx_queue_depth = tx_queue_depth;
        nodeHdl = NULL;
        nodeRecord.bus_err_num = 0;
        RX_QUEUE_SIZE = rx_queue_size;
        LBK_PIN = lbk_pin;
        nodeConfig.bit_timing.sp_permill = 800;
        nodeConfig.bit_timing.ssp_permill = 0;
    }

    bool isInitialized() const
    {
        return initialized;
    }

    void debug_mode(bool enable)
    {
        gpio_reset_pin(LBK_PIN);
        gpio_set_direction(LBK_PIN, GPIO_MODE_OUTPUT);
        initialized = false;
        if (enable)
        {
            nodeConfig.flags.enable_self_test = 1;
            nodeConfig.flags.enable_loopback = 1;
            gpio_set_level(LBK_PIN, 1);
        }
        else
        {
            nodeConfig.flags.enable_self_test = 0;
            nodeConfig.flags.enable_loopback = 0;
            gpio_set_level(LBK_PIN, 0);
        }
    }

    esp_err_t init();

    esp_err_t deinit();

    twai_node_status_t getStatus();

    esp_err_t transmit(twai_frame_t *tx_msg, int timeout_ms = 1000);

    esp_err_t receive(CanDriver::CanFrame &frame, int timeout_ms = 100);

    size_t availableMessages();

    esp_err_t flushRxQueue();

private:
    // Callbacks
    static bool IRAM_ATTR twai_rx_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx);

    static bool IRAM_ATTR twai_state_change_cb(twai_node_handle_t handle,
                                               const twai_state_change_event_data_t *edata,
                                               void *user_ctx);

    static bool IRAM_ATTR twai_bus_err_cb(twai_node_handle_t handle,
                                          const twai_error_event_data_t *edata,
                                          void *user_ctx);

    // Twai Configuration
    twai_onchip_node_config_t nodeConfig{};
    twai_node_handle_t nodeHdl;
    twai_node_record_t nodeRecord{};
    gpio_num_t LBK_PIN;

    // RX Queue
    QueueHandle_t rxQueue;
    size_t RX_QUEUE_SIZE;
};
