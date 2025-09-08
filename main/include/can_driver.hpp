#pragma once

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"

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
        explicit CanDriver(Bitrate bitrate = Bitrate::BITRATE_500K, gpio_num_t tx_pin = GPIO_NUM_4 , gpio_num_t rx_pin = GPIO_NUM_5
                ,uint32_t tx_queue_depth = 5U, twai_node_handle_t node_hdl = NULL) {
            node_config.io_cfg.tx = tx_pin;
            node_config.io_cfg.rx = rx_pin;
            node_config.bit_timing.bitrate = static_cast<uint32_t>(bitrate);
            node_config.tx_queue_depth = tx_queue_depth;
            node_hdl = node_hdl;
            node_record.bus_err_num = 0;
        }

        bool isInitialized() const {
            return initialized;
        }
        void setBitrate(Bitrate bitrate) {
            node_config.bit_timing.bitrate = static_cast<uint32_t>(bitrate);
        }
        void setTxPin(gpio_num_t tx_pin) {
            node_config.io_cfg.tx = tx_pin;
        }
        void setRxPin(gpio_num_t rx_pin) {
            node_config.io_cfg.rx = rx_pin;
        }
        void setTxQueueDepth(uint32_t depth) {
            node_config.tx_queue_depth = depth;
        }
        void setTwaiNodeHandle(twai_node_handle_t node_hdl) {
            node_hdl = node_hdl;
        }

        esp_err_t init();
        esp_err_t deinit();
        twai_node_status_t getStatus();
        

    private:
        twai_onchip_node_config_t node_config{};
        twai_node_handle_t node_hdl;
        twai_node_record_t node_record{};
};