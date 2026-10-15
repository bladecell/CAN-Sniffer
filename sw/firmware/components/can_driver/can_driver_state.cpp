#include "can_driver_internal.hpp"

bool CanDriver::quickCheckBus()
{
    if (!isInitialized())
    {
        return false;
    }

    consecutiveAckErrors.store(0);
    consecutiveStuffErrors.store(0);

    esp_err_t ret = pingBus();
    if (ret != ESP_OK)
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    if (consecutiveAckErrors.load() == 0 && consecutiveStuffErrors.load() == 0)
    {
        return true;
    }

    return false;
}

esp_err_t CanDriver::pingBus()
{
    if (nodeHdl == NULL)

    {
        return ESP_FAIL;
    }

    static twai_frame_t tx = {};
    // 1. Use the universal OBD ID
    tx.header.id  = 0x7DF;
    tx.header.rtr = 0;
    tx.header.dlc = 8;
    tx.header.ide = 0;  // 11-bit standard ID
    tx.header.fdf = 0;
    tx.header.brs = 0;

    static uint8_t buff[8] = {0};

    buff[0] = 0x02;  // Valid length byte
    buff[1] = 0x01;  // Service 01
    buff[2] = 0x00;  // PID 00

    tx.buffer     = buff;
    tx.buffer_len = sizeof(buff);

    return twai_node_transmit(nodeHdl, &tx, pdMS_TO_TICKS(10));
}

void CanDriver::healthCheckTaskWrapper(void* param)
{
    CanDriver* driver = static_cast<CanDriver*>(param);
    driver->healthCheckTask();
}

void CanDriver::healthCheckTask()
{
    STATE   prevState            = STATE::NOT_INITIALIZED;
    uint8_t successfulPingsCount = 0;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(50));

    while (true)
    {
        switch (canState.load())
        {
            case STATE::NOT_INITIALIZED:
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                break;
            }
            case STATE::NOT_CONNECTED:
            {
                esp_err_t ret = pingBus();
                if (ret != ESP_OK)
                {
                    ESP_LOGW(TAG, "CAN bus ping failed: %s", esp_err_to_name(ret));
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HEALTHCHECK_PING_PERIOD_MS));
                if (consecutiveAckErrors.load() == 0 && consecutiveStuffErrors.load() == 0)
                {
                    successfulPingsCount++;
                    if (successfulPingsCount >= 3)
                    {
                        ESP_LOGI(TAG, "CAN bus connected");
                        canState.store(STATE::CONNECTED);
                        xSemaphoreGive(canConnectedSemaphore);
                        connectionChangeCb(true);
                        successfulPingsCount = 0;
                    }
                }
                else
                {
                    successfulPingsCount = 0;
                }
                break;
            }
            case STATE::CONNECTED:
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                if (consecutiveAckErrors.load() > 3)
                {
                    ESP_LOGW(TAG, "CAN bus disconnected due to ACK errors");
                    canState.store(STATE::NOT_CONNECTED);
                    connectionChangeCb(false);
                }
                else if (consecutiveStuffErrors.load() > 3)
                {
                    ESP_LOGW(TAG, "CAN bus disconnected due to stuffing errors");
                    canState.store(STATE::NOT_CONNECTED);
                    connectionChangeCb(false);
                }
                break;
            }
            case STATE::BUS_OFF:
            {
                if (prevState == STATE::CONNECTED)
                {
                    connectionChangeCb(false);
                }

                ESP_LOGI(TAG, "Attempting CAN bus recovery from BUS OFF");
                esp_err_t ret = twai_node_recover(nodeHdl);
                if (ret != ESP_OK)
                {
                    ESP_LOGW(TAG, "CAN bus recovery failed: %s", esp_err_to_name(ret));
                }
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
                break;
            }
            default:
                ESP_LOGW(TAG, "Unknown state: %d", canState.load());
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
        prevState = canState.load();
    }
    healthCheckTaskHandle = nullptr;
    vTaskDelete(NULL);
}

esp_err_t CanDriver::setConnectionChangeCallback(ConnectionChangeCallback_t callback, void* arg, TickType_t waitTicks)
{
    if (isConnectionChangeCallbackTask())
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (connectionChangeCallbackFence == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(connectionChangeCallbackFence, waitTicks) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    connectionChangeCallback    = callback;
    connectionChangeCallbackArg = arg;
    xSemaphoreGive(connectionChangeCallbackFence);
    return ESP_OK;
}

bool CanDriver::isConnectionChangeCallbackTask() const
{
    const TaskHandle_t executingTask = connectionChangeCallbackTask.load(std::memory_order_acquire);
    return executingTask != nullptr && executingTask == xTaskGetCurrentTaskHandle();
}

void CanDriver::connectionChangeCb(bool connected)
{
    if (connectionChangeCallbackFence == nullptr ||
        xSemaphoreTake(connectionChangeCallbackFence, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    if (connectionChangeCallback != nullptr)
    {
        connectionChangeCallbackTask.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
        connectionChangeCallback(connectionChangeCallbackArg, connected);
        connectionChangeCallbackTask.store(nullptr, std::memory_order_release);
    }
    xSemaphoreGive(connectionChangeCallbackFence);
}
