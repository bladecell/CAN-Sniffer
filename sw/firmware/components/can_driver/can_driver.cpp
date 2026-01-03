#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can_driver.hpp"

static const char *TAG = "CAN_DRIVER";

CanDriver::CanDriver()
{
    canState.store(STATE::NOT_INITIALIZED);
    healthCheckTaskHandle = nullptr;
    rxQueue = nullptr;
    nodeHdl = NULL;
}

CanDriver::~CanDriver()
{
    (void)deinit();

    if (rxQueue != nullptr)
    {
        vQueueDelete(rxQueue);
        rxQueue = nullptr;
    }
}

void CanDriver::setDebugMode(bool enable)
{
    gpio_reset_pin(LBK_PIN);
    gpio_set_direction(LBK_PIN, GPIO_MODE_OUTPUT);
    nodeConfig.flags.enable_self_test = enable ? 1 : 0;
    nodeConfig.flags.enable_loopback = enable ? 1 : 0;
    gpio_set_level(LBK_PIN, enable ? 1 : 0);
    debug_mode = enable ? 1 : 0;
    enable ? start_sim_task(this) : stop_sim_task();
}

esp_err_t CanDriver::init(const Config &config)
{
    if (isInitialized())
    {
        return ESP_OK;
    }

    nodeConfig.io_cfg.tx = config.tx_pin;
    nodeConfig.io_cfg.rx = config.rx_pin;
    nodeConfig.bit_timing.bitrate = static_cast<uint32_t>(config.bitrate);
    nodeConfig.tx_queue_depth = config.tx_queue_depth;
    nodeHdl = NULL;
    nodeRecord.bus_err_num = 0;
    RX_QUEUE_SIZE = config.rx_queue_size;
    LBK_PIN = config.lbk_pin;
    nodeConfig.bit_timing.sp_permill = 800;
    nodeConfig.bit_timing.ssp_permill = 0;

    esp_err_t ret;

    gpio_reset_pin(config.rs_pin);
    gpio_set_direction(config.rs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(config.rs_pin, static_cast<uint32_t>(config.rs_mode));

    setDebugMode(config.debug);

    // Create RX Queue
    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (rxQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    // Create TWAI Instance
    ret = twai_new_node_onchip(&nodeConfig, &nodeHdl);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Creating TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register Callbacks
    twai_event_callbacks_t cbs = {
        .on_tx_done = twai_tx_cb,
        .on_rx_done = twai_rx_cb,
        .on_state_change = twai_state_change_cb,
        .on_error = twai_bus_err_cb,
    };

    ret = twai_node_register_event_callbacks(nodeHdl, &cbs, this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Registering TWAI event callbacks failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure Mask Filter for OBD-II
    twai_mask_filter_config_t mfilter_cfg = {};
    mfilter_cfg.id = 0b011100000000;   // 0b111 11100000
    mfilter_cfg.mask = 0b011100000000; // 0b111 11100000
    mfilter_cfg.is_ext = false;        // Standard 11-bit IDs

    ESP_ERROR_CHECK(twai_node_config_mask_filter(nodeHdl, 0, &mfilter_cfg));

    // Start TWAI Instance
    ret = twai_node_enable(nodeHdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Starting TWAI Controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start health check monitoring task
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        healthCheckTaskWrapper,
        "can_health_check",
        4096,
        this,
        HEALTH_CHECK_TASK_PRIO,
        &healthCheckTaskHandle,
        CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create health check task");
        return ESP_FAIL;
    }
    // Initialization successful
    canState.store(STATE::NOT_CONNECTED);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskNotifyGive(healthCheckTaskHandle);
    return ESP_OK;
}

esp_err_t CanDriver::deinit()
{
    if (!isInitialized())
    {
        return ESP_OK;
    }

    if (healthCheckTaskHandle)
    {
        xTaskNotifyGive(healthCheckTaskHandle);
        vTaskDelete(healthCheckTaskHandle);
        healthCheckTaskHandle = nullptr;
    }
    twai_node_disable(nodeHdl);
    esp_err_t ret = twai_node_delete(nodeHdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Deleting TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }

    flushRxQueue();
    stop_sim_task();
    canState.store(STATE::NOT_INITIALIZED);
    return ESP_OK;
}

twai_node_status_t CanDriver::getStatus()
{
    twai_node_status_t status{};
    esp_err_t ret = twai_node_get_info(nodeHdl, &status, &nodeRecord);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Getting TWAI Controller status failed: %s", esp_err_to_name(ret));
    }

    return status;
}

esp_err_t CanDriver::transmit(twai_frame_t *tx_msg, int timeout_ms)
{
    static uint32_t lastSendTime_ms = 0;

    if (!isInitialized() || !isBusConnected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (tx_msg == nullptr)
    {
        ESP_LOGE(TAG, "tx_msg is NULL!");
        return ESP_ERR_INVALID_ARG;
    }

    LOG_CAN_FRAME(TAG, "TX -> ", tx_msg->header.id, tx_msg->buffer);

    uint32_t currentTime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t timeSinceLastSend = currentTime_ms - lastSendTime_ms;

    if (timeSinceLastSend < MIN_TRANSMIT_PERIOD_MS)
    {
        uint32_t delay_ms = MIN_TRANSMIT_PERIOD_MS - timeSinceLastSend;

        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        currentTime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    lastSendTime_ms = currentTime_ms;

    esp_err_t ret = twai_node_transmit(nodeHdl, tx_msg, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Transmitting message failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t CanDriver::receive(CanDriver::CanFrame &frame, int timeout_ms)
{
    if (!isInitialized() || !rxQueue || !isBusConnected())
    {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);

    if (xQueueReceive(rxQueue, &frame, ticks) == pdTRUE)
    {
        LOG_CAN_FRAME(TAG, "RX <- ", frame.id, frame.data);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

bool IRAM_ATTR CanDriver::twai_rx_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    CanDriver *driver = static_cast<CanDriver *>(user_ctx);
    if (!driver || !driver->rxQueue)
    {
        return false;
    }

    CanDriver::CanFrame frame{};
    BaseType_t woken = pdFALSE;

    twai_frame_t rx_frame = {
        .header = {},
        .buffer = frame.data,
        .buffer_len = sizeof(frame.data) / sizeof(uint8_t),
    };

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
    {
        frame.id = rx_frame.header.id;
        frame.length = rx_frame.header.dlc;
        if (driver->debug_mode)
        {
            if (frame.length < 2)
            {
                return false;
            }
            if (xDataSimTaskHandle != NULL)
            {
                uint32_t sd = (frame.data[1] << 8) | frame.data[2];
                xTaskNotifyFromISR(
                    xDataSimTaskHandle, // Directly use the global handle
                    sd,
                    eSetValueWithOverwrite,
                    &woken);
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

bool IRAM_ATTR CanDriver::twai_tx_cb(twai_node_handle_t handle,
                                     const twai_tx_done_event_data_t *edata,
                                     void *user_ctx)
{
    CanDriver *driver = static_cast<CanDriver *>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

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

bool IRAM_ATTR CanDriver::twai_state_change_cb(twai_node_handle_t handle,
                                               const twai_state_change_event_data_t *edata,
                                               void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGD(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);

    CanDriver *driver = static_cast<CanDriver *>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

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

bool IRAM_ATTR CanDriver::twai_bus_err_cb(twai_node_handle_t handle,
                                          const twai_error_event_data_t *edata,
                                          void *user_ctx)
{
    CanDriver *driver = static_cast<CanDriver *>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Log specific errors (your existing code)
    if (edata->err_flags.arb_lost)
        ESP_EARLY_LOGD(TAG, "Arbitration lost");
    if (edata->err_flags.bit_err)
        ESP_EARLY_LOGD(TAG, "Bit error");
    if (edata->err_flags.form_err)
        ESP_EARLY_LOGD(TAG, "Frame format error");
    if (edata->err_flags.stuff_err)
        ESP_EARLY_LOGD(TAG, "Bit stuffing error");
    if (edata->err_flags.ack_err)
        ESP_EARLY_LOGD(TAG, "No acknowledgment");

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

esp_err_t CanDriver::pingBus()
{
    if (nodeHdl == NULL)
        return ESP_FAIL;

    uint8_t txData[1] = {0x00};
    twai_frame_t tx = {};

    tx.header.id = 0x7FF;
    tx.header.dlc = twaifd_len2dlc(sizeof(txData));
    tx.header.ide = false;
    tx.header.rtr = 0;
    tx.header.fdf = 0;
    tx.header.brs = 0;
    tx.header.esi = 0;
    tx.header.timestamp = 0;
    tx.header.trigger_time = 0;

    tx.buffer = txData;
    tx.buffer_len = sizeof(txData);

    return twai_node_transmit(nodeHdl, &tx, 0);
}

void CanDriver::healthCheckTaskWrapper(void *param)
{
    CanDriver *driver = static_cast<CanDriver *>(param);
    driver->healthCheckTask();
}

void CanDriver::healthCheckTask()
{
    STATE prevState = STATE::NOT_INITIALIZED;
    uint8_t successfulPingsCount = 0;

    // Initial Wait Barrier
    // This ensures the task sits idle until init() calls xTaskNotifyGive()
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(50));

    while (true)
    {
        switch (canState.load())
        {
        case STATE::NOT_INITIALIZED:
        {
            // Wait until initialized
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            break;
        }
        case STATE::NOT_CONNECTED:
        {
            // Ping the bus periodically
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
                    notifyConnectionChange(true);
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
            // Monitor for errors
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (consecutiveAckErrors.load() > 3)
            {
                ESP_LOGW(TAG, "CAN bus disconnected due to ACK errors");
                canState.store(STATE::NOT_CONNECTED);
                notifyConnectionChange(false);
            }
            else if (consecutiveStuffErrors.load() > 3)
            {
                ESP_LOGW(TAG, "CAN bus disconnected due to stuffing errors");
                canState.store(STATE::NOT_CONNECTED);
                notifyConnectionChange(false);
            }
            break;
        }
        case STATE::BUS_OFF:
        {
            if (prevState == STATE::CONNECTED)
            {
                notifyConnectionChange(false);
            }

            // Attempt recovery
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

void CanDriver::setConnectionCallback(ConnectionCallback callback, void *arg)
{
    connectionCallback = callback;
    callbackArg = arg;
}

void CanDriver::notifyConnectionChange(bool connected)
{
    if (connectionCallback != nullptr)
    {
        connectionCallback(callbackArg, connected);
    }
}