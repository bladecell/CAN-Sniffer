#include "can_driver_internal.hpp"

esp_err_t CanDriver::transmit(CanDriver::CanFrame& tx_frame, int timeout_ms)
{
    if (!isInitialized() || !isBusConnected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(txQueue, &tx_frame, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send message to tx queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t CanDriver::transmitUrgent(CanDriver::CanFrame& tx_frame)
{
    if (!isInitialized() || !isBusConnected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSendToFront(txQueue, &tx_frame, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "TX queue full, urgent frame dropped");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void CanDriver::notifyTxDone(const twai_frame_t* frame, bool success)
{
    const TxDoneFn fn = txDoneFn_.load();
    if (fn == nullptr || frame == nullptr || frame->buffer == nullptr)
    {
        return;
    }
    const uint8_t length = frame->buffer_len > 8 ? 8 : static_cast<uint8_t>(frame->buffer_len);
    fn(txDoneArg_.load(), frame->header.id, frame->buffer, length, success);
}

esp_err_t CanDriver::receive(CanDriver::CanFrame& frame, int timeout_ms)
{
    if (!isInitialized() || !rxQueue || !isBusConnected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);

    if (xQueueReceive(rxQueue, &frame, ticks) == pdTRUE)
    {
        LOG_CAN_FRAME(TAG, "RX <- ", frame.header.id, frame.data, frame.length);
        rxCb();
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

size_t CanDriver::availableMessages()
{
    if (!rxQueue)
    {
        return 0;
    }
    return uxQueueMessagesWaiting(rxQueue);
}

esp_err_t CanDriver::flushRxQueue()
{
    if (!rxQueue)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xQueueReset(rxQueue);
    return ESP_OK;
}

void CanDriver::setRxCallback(RxCallback_t callback, void* arg)
{
    rxCallback    = callback;
    rxCallbackArg = arg;
}

void CanDriver::rxCb()
{
    if (rxCallback != nullptr)
    {
        rxCallback(rxCallbackArg);
    }
}

void CanDriver::txTaskWrapper(void* param)
{
    CanDriver* driver = static_cast<CanDriver*>(param);
    driver->txTask();
}

void CanDriver::txTask()
{
    CanDriver::CanFrame f;
    size_t              next = 0;
    while (1)
    {
        if (!isBusConnected())
        {
            xSemaphoreTake(canConnectedSemaphore, portMAX_DELAY);
            continue;
        }

        if (xQueueReceive(txQueue, &f, pdMS_TO_TICKS(NO_MESSAGE_SENT_HEALTHCHECK_PING_PERIOD_MS)) == pdTRUE)
        {
            TickType_t startTime = xTaskGetTickCount();

            // A fresh slot per frame: the previous one may still be pending in the driver (lost arbitration, no ACK).
            TxSlot& slot = txRing_[next];
            next         = (next + 1) % txRingSize_;
            memcpy(slot.data, f.data, sizeof(slot.data));
            slot.msg.header     = f.header;
            slot.msg.buffer     = slot.data;
            slot.msg.buffer_len = f.length;

            esp_err_t ret = twai_node_transmit(nodeHdl, &slot.msg, pdMS_TO_TICKS(200));
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to transmit message: %s", esp_err_to_name(ret));
                notifyTxDone(&slot.msg, false);  // the waiting requester learns that nothing went out
            }

            LOG_CAN_FRAME(TAG, "TX -> ", f.header.id, f.data, twaifd_dlc2len(f.header.dlc));

            vTaskDelayUntil(&startTime, pdMS_TO_TICKS(MIN_TRANSMIT_PERIOD_MS));
        }
    }
}
