#include "can_driver_internal.hpp"
#include "can_config_limits.hpp"

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
    lifecycleMutex_               = xSemaphoreCreateMutex();
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
}

bool CanDriver::hasResources() const
{
    return txRunning_.load() || healthRunning_.load() || nodeHdl != NULL || rxQueue != nullptr || txQueue != nullptr ||
           txRing_ != nullptr || canConnectedSemaphore != nullptr || txTaskHandle != nullptr ||
           healthCheckTaskHandle != nullptr;
}

esp_err_t CanDriver::init(const Config& config)
{
    if (lifecycleMutex_ == nullptr || isConnectionChangeCallbackTask())
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(lifecycleMutex_, pdMS_TO_TICKS(TASK_STOP_TIMEOUT_MS * 2)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    if (isInitialized())
    {
        // already up
    }
    else if (hasResources())
    {
        ret = ESP_ERR_INVALID_STATE;  // an earlier deinit() could not finish; do not overwrite what is left
    }
    else
    {
        ret = bringUp(config);
        if (ret != ESP_OK)
        {
            (void)teardown();  // one cleanup path for every partial failure
        }
    }
    xSemaphoreGive(lifecycleMutex_);
    return ret;
}

esp_err_t CanDriver::bringUp(const Config& config)
{
    if (!can_limits::canConfigValid(static_cast<double>(config.rx_queue_size), static_cast<double>(config.tx_queue_depth)))
    {
        ESP_LOGE(TAG, "Invalid queue sizes: rx %u (1..256), tx %u (1..64)", static_cast<unsigned>(config.rx_queue_size),
                 static_cast<unsigned>(config.tx_queue_depth));
        return ESP_ERR_INVALID_ARG;
    }
    stopRequested_.store(false);
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

    rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (rxQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_ERR_NO_MEM;
    }

    // TX frame ring (see TxSlot): one slot more than the driver's queue depth.
    txRingSize_ = static_cast<size_t>(config.tx_queue_depth) + 1;
    txRing_     = new (std::nothrow) TxSlot[txRingSize_]();
    if (txRing_ == nullptr)
    {
        txRingSize_ = 0;
        ESP_LOGE(TAG, "Failed to allocate the TX frame ring");
        return ESP_ERR_NO_MEM;
    }

    txQueue = xQueueCreate(TX_QUEUE_SIZE, sizeof(CanDriver::CanFrame));
    if (txQueue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }

    canConnectedSemaphore = xSemaphoreCreateBinary();
    if (!canConnectedSemaphore)
    {
        ESP_LOGE(TAG, "Failed to create canConnectedSemaphore");
        return ESP_ERR_NO_MEM;
    }

    // The simulator injects into rxQueue, so it starts only now that the queues exist.
    if (config.debug)
    {
        if (simStart_)
        {
            simStart_(this);
        }
    }
    else if (simStop_ && !simStop_())
    {
        ESP_LOGE(TAG, "Simulator still running; cannot start without debug mode");
        return ESP_ERR_TIMEOUT;
    }

    ret = twai_new_node_onchip(&nodeConfig, &nodeHdl);
    if (ret != ESP_OK)
    {
        nodeHdl = NULL;
        ESP_LOGE(TAG, "Creating TWAI Controller instance failed: %s", esp_err_to_name(ret));
        return ret;
    }

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

    if (config.filter)
    {
        ret = twai_node_config_mask_filter(nodeHdl, 0, &config.mfilter_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Configuring TWAI filter failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Tasks first, so the node's callbacks (enabled below) always find a live health task.
    txRunning_.store(true);
    BaseType_t taskCreated =
        xTaskCreatePinnedToCore(txTaskWrapper, "can_tx", 4096, this, TX_TASK_PRIO, &txTaskHandle, CORE_ID_CAN_TASKS);
    if (taskCreated != pdPASS)
    {
        txRunning_.store(false);
        txTaskHandle = nullptr;
        ESP_LOGE(TAG, "Failed to create tx task");
        return ESP_ERR_NO_MEM;
    }

    healthRunning_.store(true);
    taskCreated = xTaskCreatePinnedToCore(healthCheckTaskWrapper, "can_health_check", 4096, this,
                                          HEALTH_CHECK_TASK_PRIO, &healthCheckTaskHandle, CORE_ID_CAN_TASKS);
    if (taskCreated != pdPASS)
    {
        healthRunning_.store(false);
        healthCheckTaskHandle = nullptr;
        ESP_LOGE(TAG, "Failed to create health check task");
        return ESP_ERR_NO_MEM;
    }

    ret = twai_node_enable(nodeHdl);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Starting TWAI Controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    canState.store(STATE::NOT_CONNECTED);
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskNotifyGive(healthCheckTaskHandle);
    return ESP_OK;
}

// Releases whatever exists, in dependency order; every step is skipped when its resource is absent, so it is safe after a
// partial init, after a failed earlier teardown and when called twice. Caller holds lifecycleMutex_.
esp_err_t CanDriver::teardown()
{
    canState.store(STATE::NOT_INITIALIZED);  // transmit()/receive() refuse from here on
    stopRequested_.store(true);

    // The simulator task feeds rxQueue: stop it before the queue goes.
    if (simStop_ && !simStop_())
    {
        ESP_LOGE(TAG, "Simulator did not stop; leaving resources in place");
        return ESP_ERR_TIMEOUT;
    }

    // No more TWAI callbacks (they notify the health task) once the node is disabled.
    if (nodeHdl != NULL)
    {
        // INVALID_STATE when init failed before enable or the node is bus-off: callbacks may still fire then, and bail
        // out on stopRequested_.
        const esp_err_t disabled = twai_node_disable(nodeHdl);
        if (disabled != ESP_OK && disabled != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(TAG, "Disabling the TWAI node failed: %s", esp_err_to_name(disabled));
        }
    }

    if (txRunning_.load() || healthRunning_.load())
    {
        if (healthRunning_.load() && healthCheckTaskHandle != nullptr)
        {
            xTaskNotifyGive(healthCheckTaskHandle);
        }
        if (txRunning_.load())
        {
            if (canConnectedSemaphore != nullptr)
            {
                xSemaphoreGive(canConnectedSemaphore);  // TX task parked while the bus was down
            }
            if (txQueue != nullptr)
            {
                CanFrame wake{};
                (void)xQueueSendToFront(txQueue, &wake, 0);  // TX task parked in the queue; it drops this frame on stop
            }
        }
        for (uint32_t waited = 0; (txRunning_.load() || healthRunning_.load()) && waited < TASK_STOP_TIMEOUT_MS;
             waited += 10)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (txRunning_.load() || healthRunning_.load())
        {
            ESP_LOGE(TAG, "CAN tasks did not stop in %d ms; leaving resources in place", TASK_STOP_TIMEOUT_MS);
            return ESP_ERR_TIMEOUT;
        }
    }
    txTaskHandle          = nullptr;
    healthCheckTaskHandle = nullptr;

    if (nodeHdl != NULL)
    {
        esp_err_t ret = twai_node_delete(nodeHdl);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Deleting TWAI Controller instance failed: %s", esp_err_to_name(ret));
            return ret;
        }
        nodeHdl = NULL;
    }

    // The node and both tasks are gone: nothing references the rest any more.
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
    delete[] txRing_;
    txRing_     = nullptr;
    txRingSize_ = 0;
    canState.store(STATE::NOT_INITIALIZED);  // a task or callback woken during teardown may have stored another state
    return ESP_OK;
}

esp_err_t CanDriver::deinit()
{
    if (isConnectionChangeCallbackTask() || lifecycleMutex_ == nullptr)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(lifecycleMutex_, pdMS_TO_TICKS(TASK_STOP_TIMEOUT_MS * 2)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    if (connectionChangeCallbackFence == nullptr ||
        xSemaphoreTake(connectionChangeCallbackFence, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        ret = ESP_ERR_TIMEOUT;
    }
    else
    {
        // Waits out a callback in flight. Released before the tasks are stopped: the health task may be queued on this
        // fence, and must get through it (to a null callback) to see the stop request.
        connectionChangeCallback    = nullptr;
        connectionChangeCallbackArg = nullptr;
        xSemaphoreGive(connectionChangeCallbackFence);

        if (hasResources())
        {
            ret = teardown();
        }
    }
    xSemaphoreGive(lifecycleMutex_);
    return ret;
}
