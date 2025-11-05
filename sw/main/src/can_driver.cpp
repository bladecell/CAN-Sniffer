#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can_driver.hpp"

static const char *TAG = "CAN_DRIVER";

CanDriver::~CanDriver() {
    (void)deinit();
    
    if (rxQueue != nullptr) {
        vQueueDelete(rxQueue);
        rxQueue = nullptr;
    }
}

esp_err_t CanDriver::init() {
    esp_err_t ret;
    if(isInitialized()) {
        return ESP_OK;
    }

    //Create RX Queue
    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (rxQueue == nullptr) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    //Create TWAI Instance
    ret = twai_new_node_onchip(&nodeConfig, &nodeHdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Creating TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    //Register Callbacks
    twai_event_callbacks_t cbs = {
        .on_tx_done = NULL,
        .on_rx_done = twai_rx_cb,
        .on_state_change = twai_state_change_cb,
        .on_error = twai_bus_err_cb,
    };

    ret = twai_node_register_event_callbacks(nodeHdl, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Registering TWAI event callbacks failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // twai_timing_advanced_config_t timing_config = {
    //     .brp = 10,              // prescaler, 80MHz/10 = 8MHz -> 125 ns per tq
    //     .prop_seg = 1,          // propagation segment
    //     .tseg_1 = 11,           // phase segment 1
    //     .tseg_2 = 3,            // phase segment 2
    //     .sjw = 2,               // resynchronization jump width
    //     .triple_sampling = false
    // };
    // ret = twai_node_reconfig_timing(nodeHdl, &timing_config, NULL);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuring TWAI timing failed: %s", esp_err_to_name(ret));
        return ret;
    }

    //Start TWAI Instance
    ret = twai_node_enable(nodeHdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Starting TWAI Controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialization successful
    initialized = true;
    return ESP_OK;
}

esp_err_t CanDriver::deinit() {
    if(!isInitialized()) {
        return ESP_OK;
    }

    esp_err_t ret = twai_node_delete(nodeHdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Deleting TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }

    flushRxQueue();

    return ESP_OK;
}

twai_node_status_t CanDriver::getStatus() {
    twai_node_status_t status{};
    esp_err_t ret = twai_node_get_info(nodeHdl, &status, &nodeRecord);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Getting TWAI Controller status failed: %s", esp_err_to_name(ret));
    }

    return status;
}

esp_err_t CanDriver::transmit(twai_frame_t* tx_msg, int timeout_ms) {
    if (tx_msg == nullptr) {
        ESP_LOGE(TAG, "tx_msg is NULL!");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "Transmitting message with ID: 0x%08X", tx_msg->header.id);
    ESP_LOGD(TAG, "Data buffer: ");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, tx_msg->buffer, tx_msg->buffer_len, ESP_LOG_DEBUG);
    esp_err_t ret = twai_node_transmit(nodeHdl, tx_msg, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transmitting message failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}


esp_err_t CanDriver::receive(CanDriver::CanFrame& frame, int timeout_ms) {
    if (!isInitialized() || !rxQueue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    
    if (xQueueReceive(rxQueue, &frame, ticks) == pdTRUE) {
        ESP_LOGD(TAG, "Received message with ID: 0x%08X", frame.id);
        ESP_LOGD(TAG, "Data buffer: ");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, frame.data, frame.length, ESP_LOG_DEBUG);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}
    
bool IRAM_ATTR CanDriver::twai_rx_cb(twai_node_handle_t handle, 
                                const twai_rx_done_event_data_t *edata, 
                                void *user_ctx) {
    CanDriver* driver = static_cast<CanDriver*>(user_ctx);
    if (!driver || !driver->rxQueue) {
        return false;
    }
    
    CanDriver::CanFrame frame{};
    BaseType_t woken = pdFALSE;

    twai_frame_t rx_frame = {
        .header = {},
        .buffer = frame.data,
        .buffer_len = sizeof(frame.data) / sizeof(uint8_t),
    };
    
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        frame.id = rx_frame.header.id;
        frame.length = rx_frame.header.dlc;
        if (xQueueSendFromISR(driver->rxQueue, &frame, &woken) != pdTRUE) {
            ESP_EARLY_LOGW(TAG, "RX queue full, message dropped");
        }
    }
    
    return woken == pdTRUE;
}

bool IRAM_ATTR CanDriver::twai_state_change_cb(twai_node_handle_t handle, 
                                    const twai_state_change_event_data_t *edata, 
                                    void *user_ctx) {
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGW(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
    return false;
}

bool IRAM_ATTR CanDriver::twai_bus_err_cb(twai_node_handle_t handle, 
                                const twai_error_event_data_t *edata, 
                                void *user_ctx) {
    if (edata->err_flags.arb_lost) {
        ESP_EARLY_LOGW(TAG, "Arbitration lost - another node won bus access");
    }

    if (edata->err_flags.bit_err) {
        ESP_EARLY_LOGE(TAG, "Bit error - dominant/recessive mismatch");
    }

    if (edata->err_flags.form_err) {
        ESP_EARLY_LOGE(TAG, "Frame format error - protocol violation");
    }

    if (edata->err_flags.stuff_err) {
        ESP_EARLY_LOGE(TAG, "Bit stuffing error - protocol violation");
    }

    if (edata->err_flags.ack_err) {
        ESP_EARLY_LOGW(TAG, "No acknowledgment - no other nodes responded");
    }

    // Access the combined error flags using val
    if (edata->err_flags.val != 0) {
        ESP_EARLY_LOGW(TAG, "Bus error occurred: 0x%x", edata->err_flags.val);
    }

    // // Perform recovery or other actions as needed
    // esp_err_t ret = twai_node_recover(handle);
    // if (ret != ESP_OK) {
    //     ESP_EARLY_LOGE(TAG, "TWAI node recovery failed: %s", esp_err_to_name(ret));
    // }

    return false;
}


size_t CanDriver::availableMessages() {
    if (!rxQueue) {
        return 0;
    }
    return uxQueueMessagesWaiting(rxQueue);
}

esp_err_t CanDriver::flushRxQueue() {
    if (!rxQueue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xQueueReset(rxQueue);
    return ESP_OK;
}