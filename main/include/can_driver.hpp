#pragma once

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

class CanDriver {
    public:
        ~CanDriver();
        bool initialized = false;
        enum class Bitrate {
            BITRATE_125K = 125000,
            BITRATE_250K = 250000,
            BITRATE_500K = 500000,
            BITRATE_1M   = 1000000
        };
        struct can_frame {
            uint32_t id;
            uint8_t data[8];
            size_t length;
        };
        explicit CanDriver(Bitrate bitrate = Bitrate::BITRATE_500K, gpio_num_t tx_pin = GPIO_NUM_5 , gpio_num_t rx_pin = GPIO_NUM_4
                , gpio_num_t lbk_pin = GPIO_NUM_6 ,uint32_t tx_queue_depth = 5U, size_t rx_queue_size = 20) {
            node_config.io_cfg.tx = tx_pin;
            node_config.io_cfg.rx = rx_pin;
            node_config.bit_timing.bitrate = static_cast<uint32_t>(bitrate);
            node_config.tx_queue_depth = tx_queue_depth;
            node_hdl = NULL;
            node_record.bus_err_num = 0;
            RX_QUEUE_SIZE = rx_queue_size;
            LBK_PIN = lbk_pin;
            node_config.bit_timing.bitrate = 500000;
            node_config.bit_timing.sp_permill = 800;
            node_config.bit_timing.ssp_permill = 0;
        }

        bool isInitialized() const {
            return initialized;
        }

        void debug_mode(bool enable) {
            gpio_reset_pin(LBK_PIN);
            gpio_set_direction(LBK_PIN, GPIO_MODE_OUTPUT);
            initialized = false;
            if (enable) {
                node_config.flags.enable_self_test = 1;
                node_config.flags.enable_loopback = 1;
                gpio_set_level(LBK_PIN, 1);
            } else {
                node_config.flags.enable_self_test = 0;
                node_config.flags.enable_loopback = 0;
                gpio_set_level(LBK_PIN, 0);
            }
        }

        esp_err_t init();

        esp_err_t deinit();

        twai_node_status_t getStatus();

        esp_err_t transmit(twai_frame_t* tx_msg, int timeout_ms = 1000);

        esp_err_t receive(CanDriver::can_frame& frame, int timeout_ms = 100);



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

        //Twai Configuration
        twai_onchip_node_config_t node_config{};
        twai_node_handle_t node_hdl;
        twai_node_record_t node_record{};
        gpio_num_t LBK_PIN;

        //RX Queue
        QueueHandle_t rx_queue;
        size_t RX_QUEUE_SIZE;
};
