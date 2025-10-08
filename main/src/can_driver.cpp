#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can_driver.hpp"

static const char *TAG = "CAN_DRIVER";

CanDriver::~CanDriver() {
    (void)deinit();
    
    if (rx_queue != nullptr) {
        vQueueDelete(rx_queue);
        rx_queue = nullptr;
    }
}

esp_err_t CanDriver::init() {
    esp_err_t ret;
    if(isInitialized()) {
        return ESP_OK;
    }

    //Create RX Queue
    rx_queue = xQueueCreate(RX_QUEUE_SIZE, sizeof(CanDriver::can_frame));
    if (rx_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    //Create TWAI Instance
    ret = twai_new_node_onchip(&node_config, &node_hdl);
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

    ret = twai_node_register_event_callbacks(node_hdl, &cbs, this);
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
    // ret = twai_node_reconfig_timing(node_hdl, &timing_config, NULL);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuring TWAI timing failed: %s", esp_err_to_name(ret));
        return ret;
    }

    //Start TWAI Instance
    ret = twai_node_enable(node_hdl);
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

    esp_err_t ret = twai_node_delete(node_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Deleting TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }

    flushRxQueue();

    return ESP_OK;
}

twai_node_status_t CanDriver::getStatus() {
    twai_node_status_t status{};
    esp_err_t ret = twai_node_get_info(node_hdl, &status, &node_record);
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
    for (size_t i = 0; i < tx_msg->buffer_len; i++) {
        ESP_LOGD(TAG, "0x%02X ", tx_msg->buffer[i]);
    }
    esp_err_t ret = twai_node_transmit(node_hdl, tx_msg, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transmitting message failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}


esp_err_t CanDriver::receive(CanDriver::can_frame& frame, int timeout_ms) {
    if (!isInitialized() || !rx_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    
    if (xQueueReceive(rx_queue, &frame, ticks) == pdTRUE) {
        ESP_LOGD(TAG, "Received message with ID: 0x%08X", frame.id);
        ESP_LOGD(TAG, "Data buffer: ");
        for (size_t i = 0; i < frame.length; i++) {
            ESP_LOGD(TAG, "0x%02X ", frame.data[i]);
        }
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}
    
bool IRAM_ATTR CanDriver::twai_rx_cb(twai_node_handle_t handle, 
                                const twai_rx_done_event_data_t *edata, 
                                void *user_ctx) {
    CanDriver* driver = static_cast<CanDriver*>(user_ctx);
    if (!driver || !driver->rx_queue) {
        return false;
    }
    
    CanDriver::can_frame frame{};
    BaseType_t woken = pdFALSE;

    twai_frame_t rx_frame = {
        .header = {},
        .buffer = frame.data,
        .buffer_len = sizeof(frame.data) / sizeof(uint8_t),
    };
    
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        frame.id = rx_frame.header.id;
        frame.length = rx_frame.header.dlc;
        if (xQueueSendFromISR(driver->rx_queue, &frame, &woken) != pdTRUE) {
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
    if (!rx_queue) {
        return 0;
    }
    return uxQueueMessagesWaiting(rx_queue);
}

esp_err_t CanDriver::flushRxQueue() {
    if (!rx_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xQueueReset(rx_queue);
    return ESP_OK;
}