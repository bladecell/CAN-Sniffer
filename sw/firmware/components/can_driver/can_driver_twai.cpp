#include "can_driver_internal.hpp"

twai_node_status_t CanDriver::getStatus()
{
    twai_node_status_t status{};
    OpGate::Scope      op(opGate_);
    if (!op.admitted() || nodeHdl == NULL)
    {
        return status;
    }
    esp_err_t          ret = twai_node_get_info(nodeHdl, &status, &nodeRecord);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Getting TWAI Controller status failed: %s", esp_err_to_name(ret));
    }

    return status;
}

bool IRAM_ATTR CanDriver::twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx)
{
    CanDriver* driver = static_cast<CanDriver*>(user_ctx);
    if (!driver || !driver->rxQueue)
    {
        return false;
    }

    CanDriver::CanFrame frame{};
    BaseType_t          woken = pdFALSE;

    twai_frame_t rx_frame = {
        .header     = {},
        .buffer     = frame.data,
        .buffer_len = sizeof(frame.data) / sizeof(uint8_t),
    };

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        frame.header = rx_frame.header;
        frame.length = twaifd_dlc2len(rx_frame.header.dlc);
        if (frame.length > sizeof(frame.data))
        {
            frame.length = sizeof(frame.data);  // DLC 9-15 means 12-64 bytes; only 8 fit
        }
        if (driver->debug_mode)
        {
            if (driver->simNotify_)
            {
                driver->simNotify_(frame.header.id, frame.data, static_cast<uint8_t>(frame.length), &woken);
            }
        }
        else
        {
            if (xQueueSendFromISR(driver->rxQueue, &frame, &woken) != pdTRUE)
            {
                ESP_EARLY_LOGW(TAG, "RX queue full, message dropped");
            }
        }
    }

    return woken == pdTRUE;
}

bool IRAM_ATTR CanDriver::twai_tx_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx)
{
    CanDriver* driver                   = static_cast<CanDriver*>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Teardown in progress (twai_node_disable can fail in bus-off, so callbacks may still fire): touch nothing.
    if (driver->stopRequested_.load())
    {
        return false;
    }

    driver->notifyTxDone(edata->done_tx_frame, edata->is_tx_success);

    if (edata->is_tx_success)
    {
        driver->consecutiveStuffErrors.store(0);
        driver->consecutiveAckErrors.store(0);
        if (!driver->isBusConnected())
        {
            if (driver->healthCheckTaskHandle != nullptr)
            {
                vTaskNotifyGiveFromISR(driver->healthCheckTaskHandle, &xHigherPriorityTaskWoken);
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return false;
}

bool IRAM_ATTR CanDriver::twai_state_change_cb(twai_node_handle_t handle, const twai_state_change_event_data_t* edata,
                                               void* user_ctx)
{
    const char* twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGD(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);

    CanDriver* driver                   = static_cast<CanDriver*>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (driver->stopRequested_.load())
    {
        return false;
    }

    // Teardown in progress (twai_node_disable can fail in bus-off, so callbacks may still fire): touch nothing.
    if (driver->stopRequested_.load())
    {
        return false;
    }

    if (edata->new_sta == TWAI_ERROR_ACTIVE)
    {
        ESP_EARLY_LOGI(TAG, "CAN bus on - node recovered");
        driver->canState.store(STATE::NOT_CONNECTED);
        if (driver->healthCheckTaskHandle != nullptr)
        {
            vTaskNotifyGiveFromISR(driver->healthCheckTaskHandle, &xHigherPriorityTaskWoken);
        }
    }

    if (edata->new_sta == TWAI_ERROR_BUS_OFF)
    {
        ESP_EARLY_LOGW(TAG, "CAN bus off - node disconnected");
        driver->canState.store(STATE::BUS_OFF);
        if (driver->healthCheckTaskHandle != nullptr)
        {
            vTaskNotifyGiveFromISR(driver->healthCheckTaskHandle, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return false;
}

bool IRAM_ATTR CanDriver::twai_bus_err_cb(twai_node_handle_t handle, const twai_error_event_data_t* edata,
                                          void* user_ctx)
{
    CanDriver* driver                   = static_cast<CanDriver*>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Teardown in progress (twai_node_disable can fail in bus-off, so callbacks may still fire): touch nothing.
    if (driver->stopRequested_.load())
    {
        return false;
    }

    // Log specific errors (your existing code)
    if (edata->err_flags.arb_lost)
    {
        ESP_EARLY_LOGD(TAG, "Arbitration lost");
    }
    if (edata->err_flags.bit_err)
    {
        ESP_EARLY_LOGD(TAG, "Bit error");
    }
    if (edata->err_flags.form_err)
    {
        ESP_EARLY_LOGD(TAG, "Frame format error");
    }
    if (edata->err_flags.stuff_err)
    {
        ESP_EARLY_LOGD(TAG, "Bit stuffing error");
    }
    if (edata->err_flags.ack_err)
    {
        ESP_EARLY_LOGD(TAG, "No acknowledgment");
    }

    // Stuff errors = likely floating lines (no bus connected)
    if (edata->err_flags.stuff_err)
    {
        driver->consecutiveStuffErrors.fetch_add(1);
        ESP_EARLY_LOGD(TAG, "Consecutive stuff errors: %u", driver->consecutiveStuffErrors.load());

        if (driver->isBusConnected())
        {
            {
                if (driver->healthCheckTaskHandle != nullptr)
                {
                    vTaskNotifyGiveFromISR(driver->healthCheckTaskHandle, &xHigherPriorityTaskWoken);
                }
            }
        }
    }
    // ACK errors = no response (no other devices)
    if (edata->err_flags.ack_err)
    {
        driver->consecutiveAckErrors.fetch_add(1);
        ESP_EARLY_LOGD(TAG, "Consecutive ACK errors: %u", driver->consecutiveAckErrors.load());

        if (driver->isBusConnected())
        {
            if (driver->healthCheckTaskHandle != nullptr)
            {
                vTaskNotifyGiveFromISR(driver->healthCheckTaskHandle, &xHigherPriorityTaskWoken);
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return false;
}
