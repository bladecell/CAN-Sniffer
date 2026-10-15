#include "can_driver_internal.hpp"

SemaphoreHandle_t canConnectedSemaphore = nullptr;

CanDriver::CanDriver()
{
    canState.store(STATE::NOT_INITIALIZED);
    healthCheckTaskHandle = nullptr;
    rxQueue               = nullptr;
    nodeHdl               = NULL;
    txTaskHandle          = nullptr;
    txQueue               = nullptr;
    connectionChangeCallbackFence = xSemaphoreCreateMutex();
}

CanDriver::~CanDriver()
{
    (void)deinit();
}

void CanDriver::setDebugMode(bool enable)
{
    gpio_reset_pin(LBK_PIN);
    gpio_set_direction(LBK_PIN, GPIO_MODE_OUTPUT);
    nodeConfig.flags.enable_self_test = enable ? 1 : 0;
    nodeConfig.flags.enable_loopback  = enable ? 1 : 0;
    gpio_set_level(LBK_PIN, enable ? 1 : 0);
    debug_mode = enable ? 1 : 0;
    if (enable)
    {
        if (simStart_)
        {
            simStart_(this);
        }
    }
    else
    {
        if (simStop_)
        {
            simStop_();
        }
    }
}

esp_err_t CanDriver::init(const Config& config)
{
    if (isInitialized())
    {
        return ESP_OK;
    }

    nodeConfig.io_cfg.tx              = config.tx_pin;
    nodeConfig.io_cfg.rx              = config.rx_pin;
    nodeConfig.bit_timing.bitrate     = static_cast<uint32_t>(config.bitrate);
    nodeConfig.tx_queue_depth         = config.tx_queue_depth;
    nodeHdl                           = NULL;
    nodeRecord.bus_err_num            = 0;
    RX_QUEUE_SIZE                     = config.rx_queue_size;
    LBK_PIN                           = config.lbk_pin;
    nodeConfig.bit_timing.sp_permill  = 800;
    nodeConfig.bit_timing.ssp_permill = 0;

    esp_err_t ret;

    gpio_reset_pin(config.rs_pin);
    gpio_set_direction(config.rs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(config.rs_pin, static_cast<uint32_t>(config.rs_mode));

    setDebugMode(config.debug);

    this->m_config = config;

    // Create RX Queue
    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (rxQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    // TX frame ring (see TxSlot): one slot more than the driver's queue depth.
    delete[] txRing_;
    txRingSize_ = static_cast<size_t>(config.tx_queue_depth) + 1;
    txRing_     = new (std::nothrow) TxSlot[txRingSize_]();
    if (txRing_ == nullptr)
    {
        txRingSize_ = 0;
        ESP_LOGE(TAG, "Failed to allocate the TX frame ring");
        return ESP_ERR_NO_MEM;
    }

    // Create TX Queue
    txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (txQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create TX queue");
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
        .on_tx_done      = twai_tx_cb,
        .on_rx_done      = twai_rx_cb,
        .on_state_change = twai_state_change_cb,
        .on_error        = twai_bus_err_cb,
    };

    ret = twai_node_register_event_callbacks(nodeHdl, &cbs, this);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Registering TWAI event callbacks failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure Mask Filter for OBD-II
    if (config.filter)
    {
        ret = twai_node_config_mask_filter(nodeHdl, 0, &config.mfilter_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Configuring TWAI filter failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Start TWAI Instance
    ret = twai_node_enable(nodeHdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Starting TWAI Controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    canConnectedSemaphore = xSemaphoreCreateBinary();
    if (!canConnectedSemaphore)
    {
        ESP_LOGE(TAG, "Failed to create canConnectedSemaphore");
        return ESP_FAIL;
    }

    // Start tx task
    BaseType_t taskCreated =
        xTaskCreatePinnedToCore(txTaskWrapper, "can_tx", 4096, this, TX_TASK_PRIO, &txTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create tx task");
        return ESP_FAIL;
    }

    // Start health check monitoring task
    taskCreated = xTaskCreatePinnedToCore(healthCheckTaskWrapper, "can_health_check", 4096, this,
                                          HEALTH_CHECK_TASK_PRIO, &healthCheckTaskHandle, CORE_ID_CAN_TASKS);

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
    if (isConnectionChangeCallbackTask())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (connectionChangeCallbackFence == nullptr ||
        xSemaphoreTake(connectionChangeCallbackFence, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    connectionChangeCallback    = nullptr;
    connectionChangeCallbackArg = nullptr;

    if (!isInitialized())
    {
        xSemaphoreGive(connectionChangeCallbackFence);
        return ESP_OK;
    }

    if (healthCheckTaskHandle != nullptr)
    {
        vTaskDelete(healthCheckTaskHandle);
        healthCheckTaskHandle = nullptr;
    }

    if (txTaskHandle != nullptr)
    {
        vTaskDelete(txTaskHandle);
        txTaskHandle = nullptr;
    }

    if (rxQueue != nullptr)
    {
        vQueueDelete(rxQueue);
        rxQueue = nullptr;
    }

    if (txQueue != nullptr)
    {
        vQueueDelete(txQueue);
        txQueue = nullptr;
    }

    if (canConnectedSemaphore != nullptr)
    {
        vSemaphoreDelete(canConnectedSemaphore);
        canConnectedSemaphore = nullptr;
    }

    twai_node_disable(nodeHdl);
    esp_err_t ret = twai_node_delete(nodeHdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Deleting TWAI Controller instance failed: %s", esp_err_to_name(ret));
        xSemaphoreGive(connectionChangeCallbackFence);
        return ret;
    }
    // The node and the TX task are gone: nothing references the ring any more.
    delete[] txRing_;
    txRing_     = nullptr;
    txRingSize_ = 0;

    if (simStop_)
    {
        simStop_();
    }
    canState.store(STATE::NOT_INITIALIZED);
    xSemaphoreGive(connectionChangeCallbackFence);
    return ESP_OK;
}
