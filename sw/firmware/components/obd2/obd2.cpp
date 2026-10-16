/**
 * @file obd2.cpp
 * @author bladecell (github@bum.anonaddy.com)
 * @brief
 * @version 0.1
 * @date 2025-12-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "obd2_internal.hpp"

// tinyexpr formula helpers: read other PID values/raw bytes during evaluation.
double get_pid_value(double id)
{
    return (double)OBD2::getInstance().getValueUnsafe((uint16_t)id);
}

double get_pid_raw(double id, double byte_idx)
{
    return (double)OBD2::getInstance().getRawDataByteUnsafe((uint16_t)id, (uint8_t)byte_idx);
}

/**
 * @brief Construct a new OBD2::OBD2 object
 *
 */
OBD2::OBD2()
    : autoPoll(false),
      configurationMtx_(xSemaphoreCreateMutex()),
      xPidConnectedSemaphore(nullptr),
      xBusConnectionSemaphore(nullptr),
      xBusArbitrationMutex(nullptr),
      xRequestNextPIDSemaphore(nullptr)
{
    lifecycleOperationMtx_ = xSemaphoreCreateMutex();
    admissionMtx_ = xSemaphoreCreateMutex();
    lifecycleEvents_ = xEventGroupCreate();
    discoveryMtx_ = xSemaphoreCreateMutex();
    diagnosticRequestMtx_ = xSemaphoreCreateMutex();
    connected_subscribers_mtx_ = xSemaphoreCreateMutex();
    pidMapMtx = xSemaphoreCreateMutex();
    subscribers_mtx_ = xSemaphoreCreateMutex();
    vinData.mtx_ = xSemaphoreCreateMutex();
    dtcData.mtx_ = xSemaphoreCreateMutex();
}

/**
 * @brief Destroy the OBD2::OBD2 object
 *
 */
OBD2::~OBD2()
{
    deinit();
}

bool OBD2::admitOperation()
{
    if (!admissionMtx_ || xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool admitted = lifecycleState_ == LifecycleState::Running;
    if (admitted)
    {
        ++activeOperations_;
    }
    xSemaphoreGive(admissionMtx_);
    return admitted;
}

void OBD2::releaseOperation()
{
    if (!admissionMtx_ || xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    if (activeOperations_)
    {
        --activeOperations_;
    }
    xSemaphoreGive(admissionMtx_);
}

void OBD2::workerExit(EventBits_t bit)
{
    if (!admissionMtx_) { vTaskDelete(nullptr); return; }
    while (xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        taskYIELD();
    }
    if (bit == RECEIVE_EXIT)
    {
        ReceiveTaskHandle = nullptr;
    }
    else if (bit == POLL_EXIT)
    {
        PollTaskHandle = nullptr;
    }
    else if (bit == CALLBACK_EXIT)
    {
        callbackWorkerTaskHandle = nullptr;
    }
    xSemaphoreGive(admissionMtx_);
    if (lifecycleEvents_)
    {
        xEventGroupSetBits(lifecycleEvents_, bit);
    }
    vTaskDelete(nullptr);
}

esp_err_t OBD2::deinit()
{
    const TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
    bool isOwnWorker = false;
    if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE) {
        isOwnWorker = currentTask == ReceiveTaskHandle || currentTask == PollTaskHandle ||
                      currentTask == callbackWorkerTaskHandle;
        xSemaphoreGive(admissionMtx_);
    }
    if (canDriver.isConnectionChangeCallbackTask() || isOwnWorker)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (!lifecycleOperationMtx_ || xSemaphoreTake(lifecycleOperationMtx_, 0) != pdTRUE)
    {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = stopInternal(false);
    xSemaphoreGive(lifecycleOperationMtx_);
    return result;
}

esp_err_t OBD2::stopInternal(bool)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    auto remainingTicks = [&]() { return ticksUntilDeadline(deadline, xTaskGetTickCount()); };
    if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE)
    {
        if (lifecycleState_ == LifecycleState::Stopped) { xSemaphoreGive(admissionMtx_); return ESP_OK; }
        lifecycleState_ = LifecycleState::Stopping;
        stopRequested_ = true;
        autoPoll = false;
        recoveryNeeded_ = false;
        pendingDiscover_ = false;
        pidsInitialized = false;
        if (PollTaskHandle)
        {
            xTaskNotifyGive(PollTaskHandle);
        }
        xSemaphoreGive(admissionMtx_);
    }
    if (xBusConnectionSemaphore)
    {
        xSemaphoreGive(xBusConnectionSemaphore);
    }
    if (xPidConnectedSemaphore)
    {
        xSemaphoreGive(xPidConnectedSemaphore);
    }
    if (xRequestNextPIDSemaphore)
    {
        xSemaphoreGive(xRequestNextPIDSemaphore);
    }
    if (healthCheckSemaphore)
    {
        xSemaphoreGive(healthCheckSemaphore);
    }
    if (event_queue) { bool dummy = false; xQueueSend(event_queue, &dummy, 0); }
    bool callbackInstalled = false;
    if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE) {
        callbackInstalled = canCallbackInstalled_;
        xSemaphoreGive(admissionMtx_);
    }
    if (callbackInstalled) {
        if (!canDriver.setTxDoneHook(nullptr, nullptr))
        {
            return ESP_ERR_TIMEOUT;  // a TX-done callback is still running on this object: nothing may be freed yet
        }
        const TickType_t remaining = remainingTicks();
        if (remaining == 0)
        {
            return ESP_ERR_TIMEOUT;
        }
        esp_err_t callbackFence = canDriver.setConnectionChangeCallback(nullptr, nullptr, remaining);
        if (callbackFence != ESP_OK)
        {
            return callbackFence;
        }
        if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE) {
            canCallbackInstalled_ = false;
            xSemaphoreGive(admissionMtx_);
        }
    }
    for (;;)
    {
        uint32_t active = 0;
        if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE)
        { active = activeOperations_; xSemaphoreGive(admissionMtx_); }
        EventBits_t required = 0;
        if (createdWorkers_ & 1)
        {
            required |= RECEIVE_EXIT;
        }
        if (createdWorkers_ & 2)
        {
            required |= POLL_EXIT;
        }
        if (createdWorkers_ & 4)
        {
            required |= CALLBACK_EXIT;
        }
        EventBits_t got = lifecycleEvents_ ? xEventGroupGetBits(lifecycleEvents_) : 0;
        if ((got & required) == required && active == 0)
        {
            break;
        }
        const TickType_t remaining = remainingTicks();
        if (remaining == 0)
        {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(std::min<TickType_t>(1, remaining));
    }

    // Reset is fallible and precedes all runtime-resource deletion. A failed
    // acquisition leaves the stopped workers' resources intact for retry.
    pollQueue.setConsumerTask(nullptr);
    TickType_t remaining = remainingTicks();
    if (!configurationMtx_ || remaining == 0 || xSemaphoreTake(configurationMtx_, remaining) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    pollQueue.clear();
    supportedPIDsGroup = {};
    discoveryActive_ = false;
    discoveryFailed_ = false;
    discoveryExpectedGroup_ = 0xFF;
    discoverySeenGroups_ = 0;
    remaining = remainingTicks();
    if (!pidMapMtx || remaining == 0 || xSemaphoreTake(pidMapMtx, remaining) != pdTRUE) {
        xSemaphoreGive(configurationMtx_);
        return ESP_ERR_TIMEOUT;
    }
    for (auto& [pid, data] : pidData)
    {
        const auto definition = PID_DEF.find(pid);
        data.id = definition == PID_DEF.end() ? 0 : definition->second.id();
        data.value = 0.0f;
        data.lastUpdated = 0;
        memset(data.data, 0, sizeof(data.data));
        data.isSupported = definition != PID_DEF.end() &&
                           (definition->second.mode() == MODE_READ_DATA_BY_IDENTIFIER ||
                            definition->second.mode() == MODE_DERIVED_DATA);
        data.isValid = false;
    }
    xSemaphoreGive(pidMapMtx);
    xSemaphoreGive(configurationMtx_);

    remaining = remainingTicks();
    if (!vinData.mtx_ || remaining == 0 || xSemaphoreTake(vinData.mtx_, remaining) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    memset(vinData.vin, 0, sizeof(vinData.vin));
    memset(knownVin_, 0, sizeof(knownVin_));
    vinData.lastUpdated = 0;
    vinData.isValid = false;
    xSemaphoreGive(vinData.mtx_);

    remaining = remainingTicks();
    if (!dtcData.mtx_ || remaining == 0 || xSemaphoreTake(dtcData.mtx_, remaining) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    dtcData.confirmed.clear();
    dtcData.pending.clear();
    dtcData.permanent.clear();
    xSemaphoreGive(dtcData.mtx_);

    remaining = remainingTicks();
    if (!subscribers_mtx_ || remaining == 0 || xSemaphoreTake(subscribers_mtx_, remaining) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    subscribers_.clear();
    xSemaphoreGive(subscribers_mtx_);

    remaining = remainingTicks();
    if (!connected_subscribers_mtx_ || remaining == 0 ||
        xSemaphoreTake(connected_subscribers_mtx_, remaining) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    connected_subscribers_.clear();
    xSemaphoreGive(connected_subscribers_mtx_);

    resetDiagCollection();
    pollTaskUtilization = 0.0f;
    nrc_list_size = 0;
    memset(nrc_list, 0, sizeof(nrc_list));
    if (vinData.vinReadySemaphore)
    {
        while (xSemaphoreTake(vinData.vinReadySemaphore, 0) == pdTRUE)
        {
        }
    }
    if (dtcData.confirmedReadySemaphore)
    {
        while (xSemaphoreTake(dtcData.confirmedReadySemaphore, 0) == pdTRUE)
        {
        }
    }
    if (dtcData.pendingReadySemaphore)
    {
        while (xSemaphoreTake(dtcData.pendingReadySemaphore, 0) == pdTRUE)
        {
        }
    }
    if (dtcData.permanentReadySemaphore)
    {
        while (xSemaphoreTake(dtcData.permanentReadySemaphore, 0) == pdTRUE)
        {
        }
    }
    if (dtcData.clearReadySemaphore)
    {
        while (xSemaphoreTake(dtcData.clearReadySemaphore, 0) == pdTRUE)
        {
        }
    }
    if (xPidConnectedSemaphore)
    {
        while (xSemaphoreTake(xPidConnectedSemaphore, 0) == pdTRUE)
        {
        }
    }
    if (xBusConnectionSemaphore)
    {
        while (xSemaphoreTake(xBusConnectionSemaphore, 0) == pdTRUE)
        {
        }
    }
    if (xRequestNextPIDSemaphore)
    {
        while (xSemaphoreTake(xRequestNextPIDSemaphore, 0) == pdTRUE)
        {
        }
    }
    if (healthCheckSemaphore)
    {
        while (xSemaphoreTake(healthCheckSemaphore, 0) == pdTRUE)
        {
        }
    }
    if (event_queue)
    {
        xQueueReset(event_queue);
    }

    if (xPidConnectedSemaphore)
    {
        vSemaphoreDelete(xPidConnectedSemaphore);
        xPidConnectedSemaphore = nullptr;
    }
    if (xBusArbitrationMutex)
    {
        vSemaphoreDelete(xBusArbitrationMutex);
        xBusArbitrationMutex = nullptr;
    }
    if (xBusConnectionSemaphore)
    {
        vSemaphoreDelete(xBusConnectionSemaphore);
        xBusConnectionSemaphore = nullptr;
    }
    if (healthCheckSemaphore)
    {
        vSemaphoreDelete(healthCheckSemaphore);
        healthCheckSemaphore = nullptr;
    }
    if (xRequestNextPIDSemaphore != nullptr)
    {
        vSemaphoreDelete(xRequestNextPIDSemaphore);
        xRequestNextPIDSemaphore = nullptr;
    }
    if (derivedPidQueue_)
    {
        vQueueDelete(derivedPidQueue_);
        derivedPidQueue_ = nullptr;
    }
    if (event_queue)
    {
        vQueueDelete(event_queue);
        event_queue = nullptr;
    }

    pidsInitialized = false;
    busMutexOwnedByReceive_ = false;
    if (lifecycleEvents_)
    {
        xEventGroupClearBits(lifecycleEvents_, STARTUP_GATE | RECEIVE_EXIT | POLL_EXIT | CALLBACK_EXIT);
    }
    stopRequested_ = false;
    if (admissionMtx_ && xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE) {
        createdWorkers_ = 0;
        lifecycleState_ = LifecycleState::Stopped;
        xSemaphoreGive(admissionMtx_);
    }
    return ESP_OK;
}

/**
 * @brief Initialize the OBD2 interface
 *
 * @return esp_err_t ESP_OK if initialization was successful, otherwise an error code
 */
esp_err_t OBD2::init()
{
    if (!lifecycleOperationMtx_ || xSemaphoreTake(lifecycleOperationMtx_, 0) != pdTRUE)
    {
        return ESP_ERR_INVALID_STATE;
    }
    auto finish = [&](esp_err_t err) { xSemaphoreGive(lifecycleOperationMtx_); return err; };
    if (!admissionMtx_ || !lifecycleEvents_ || !configurationMtx_ || !discoveryMtx_ || !diagnosticRequestMtx_ ||
        !connected_subscribers_mtx_ || !pidMapMtx || !subscribers_mtx_ || !vinData.mtx_ || !dtcData.mtx_)
    {
        return finish(ESP_ERR_NO_MEM);
    }
    if (!canDriver.isInitialized())
    {
        return finish(ESP_FAIL);
    }

    if (xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        return finish(ESP_ERR_INVALID_STATE);
    }
    bool isStopped = lifecycleState_ == LifecycleState::Stopped;
    if (isStopped) {
        lifecycleState_ = LifecycleState::Starting;
        stopRequested_ = false;
    }
    xSemaphoreGive(admissionMtx_);
    if (!isStopped)
    {
        return finish(ESP_ERR_INVALID_STATE);
    }

    xEventGroupClearBits(lifecycleEvents_, STARTUP_GATE | RECEIVE_EXIT | POLL_EXIT | CALLBACK_EXIT);
    xSemaphoreTake(admissionMtx_, portMAX_DELAY);
    createdWorkers_ = 0;
    canCallbackInstalled_ = false;
    xSemaphoreGive(admissionMtx_);
    auto failInit = [&](esp_err_t originalError) {
        if (xSemaphoreTake(admissionMtx_, portMAX_DELAY) == pdTRUE) {
            lifecycleState_ = LifecycleState::Stopping;
            stopRequested_ = true;
            xSemaphoreGive(admissionMtx_);
        }
        // Workers are always released only after admission has closed.
        xEventGroupSetBits(lifecycleEvents_, STARTUP_GATE);
        esp_err_t cleanupError = stopInternal(true);
        xSemaphoreGive(lifecycleOperationMtx_);
        return cleanupError == ESP_OK ? originalError : cleanupError;
    };

    if (!xPidConnectedSemaphore)
    {
        xPidConnectedSemaphore = xSemaphoreCreateBinary();
    }
    if (!xBusArbitrationMutex)
    {
        xBusArbitrationMutex = xSemaphoreCreateMutex();
    }
    if (!xBusConnectionSemaphore)
    {
        xBusConnectionSemaphore = xSemaphoreCreateBinary();
    }
    if (!xRequestNextPIDSemaphore)
    {
        xRequestNextPIDSemaphore = xSemaphoreCreateBinary();
    }
    if (!healthCheckSemaphore)
    {
        healthCheckSemaphore = xSemaphoreCreateBinary();
    }
    if (!derivedPidQueue_)
    {
        derivedPidQueue_ = xQueueCreate(10, sizeof(uint16_t));
    }
    esp_err_t initDataError = initDef();
    if (!configurationMtx_ || !xPidConnectedSemaphore || !xBusArbitrationMutex || !xBusConnectionSemaphore ||
        !xRequestNextPIDSemaphore || !healthCheckSemaphore || !derivedPidQueue_)
    {
        return failInit(ESP_ERR_NO_MEM);
    }
    if (initDataError != ESP_OK)
    {
        return failInit(initDataError);
    }

    if (!event_queue)
    {
        event_queue = xQueueCreate(5, sizeof(bool));
    }
    if (!event_queue)
    {
        return failInit(ESP_ERR_NO_MEM);
    }

    TaskHandle_t createdHandle = nullptr;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(callbackWorkerTaskWrapper, "OBD_Connection_callback_task", 4096,
                                                     this, tskIDLE_PRIORITY, &createdHandle, CORE_ID_CAN_TASKS);
    if (taskCreated != pdPASS)
    {
        return failInit(ESP_ERR_NO_MEM);
    }
    xSemaphoreTake(admissionMtx_, portMAX_DELAY);
    callbackWorkerTaskHandle = createdHandle;
    createdWorkers_ |= 4;
    xSemaphoreGive(admissionMtx_);

    createdHandle = nullptr;
    taskCreated = xTaskCreatePinnedToCore(receiveTaskWrapper, "OBD2_receive_task", 8192, this,
                                          tskIDLE_PRIORITY + 2, &createdHandle, CORE_ID_CAN_TASKS);
    if (taskCreated != pdPASS)
    {
        return failInit(ESP_ERR_NO_MEM);
    }
    xSemaphoreTake(admissionMtx_, portMAX_DELAY);
    ReceiveTaskHandle = createdHandle;
    createdWorkers_ |= 1;
    xSemaphoreGive(admissionMtx_);

    createdHandle = nullptr;
    taskCreated = xTaskCreatePinnedToCore(pollTaskWrapper, "OBD2_PollTask", 8192, this,
                                          tskIDLE_PRIORITY + 1, &createdHandle, CORE_ID_CAN_TASKS);
    if (taskCreated != pdPASS)
    {
        return failInit(ESP_ERR_NO_MEM);
    }
    xSemaphoreTake(admissionMtx_, portMAX_DELAY);
    PollTaskHandle = createdHandle;
    createdWorkers_ |= 2;
    pollQueue.setConsumerTask(PollTaskHandle);
    lifecycleState_ = LifecycleState::Running;
    xSemaphoreGive(admissionMtx_);
    xEventGroupSetBits(lifecycleEvents_, STARTUP_GATE);

    esp_err_t callbackError = canDriver.setConnectionChangeCallback(onCanStateChange, this, pdMS_TO_TICKS(1000));
    if (callbackError != ESP_OK)
    {
        return failInit(callbackError);
    }
    xSemaphoreTake(admissionMtx_, portMAX_DELAY);
    canCallbackInstalled_ = true;
    xSemaphoreGive(admissionMtx_);
    if (!canDriver.setTxDoneHook(&OBD2::onTxDone, this))
    {
        return failInit(ESP_ERR_TIMEOUT);  // canCallbackInstalled_ is set, so the rollback clears whatever got installed
    }

    // Register/fence first, then take the bus snapshot to avoid missing a transition.
    if (canDriver.isBusConnected())
    {
        handleCanConnected();
    }
    ESP_LOGI(TAG, "OBD-II interface initialized");
    return finish(ESP_OK);
}

void OBD2::connected_subscribe(OBDIIConnectedCallback cb)
{
    if (!admissionMtx_ || xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    const bool permitted = lifecycleState_ == LifecycleState::Stopped || lifecycleState_ == LifecycleState::Running;
    bool inserted = false;
    if (permitted && connected_subscribers_mtx_ &&
        xSemaphoreTake(connected_subscribers_mtx_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        connected_subscribers_.push_back(std::move(cb));
        inserted = true;
        xSemaphoreGive(connected_subscribers_mtx_);
    }
    xSemaphoreGive(admissionMtx_);
    if (!inserted)
    {
        ESP_LOGW(TAG, "Connection subscription rejected (state/lock unavailable)");
    }
}

void OBD2::subscribe(PidUpdateCallback cb)
{
    if (!admissionMtx_ || xSemaphoreTake(admissionMtx_, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    const bool permitted = lifecycleState_ == LifecycleState::Stopped || lifecycleState_ == LifecycleState::Running;
    bool inserted = false;
    if (permitted && subscribers_mtx_ && xSemaphoreTake(subscribers_mtx_, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        subscribers_.push_back(std::move(cb));
        inserted = true;
        xSemaphoreGive(subscribers_mtx_);
    }
    xSemaphoreGive(admissionMtx_);
    if (!inserted)
    {
        ESP_LOGW(TAG, "PID subscription rejected (state/lock unavailable)");
    }
}

void OBD2::runOBDIIConnectedCallbacks(bool connected)
{
    xQueueSend(event_queue, &connected, 0);
}

void OBD2::callbackWorkerTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->callbackWorkerTask();
    obd2->workerExit(CALLBACK_EXIT);
}

void OBD2::callbackWorkerTask()
{
    bool is_connected;
    if (lifecycleEvents_)
    {
        xEventGroupWaitBits(lifecycleEvents_, STARTUP_GATE, pdFALSE, pdTRUE, portMAX_DELAY);
    }

    while (!stopRequested_)
    {
        if (xQueueReceive(event_queue, &is_connected, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (stopRequested_)
            {
                break;
            }
            std::vector<OBDIIConnectedCallback> callbacks;

            if (connected_subscribers_mtx_ != nullptr &&
                xSemaphoreTake(connected_subscribers_mtx_, portMAX_DELAY) == pdTRUE)
            {
                callbacks = connected_subscribers_;
                xSemaphoreGive(connected_subscribers_mtx_);
            }

            for (const auto& cb : callbacks)
            {
                if (stopRequested_)
                {
                    break;
                }
                cb(is_connected);
            }
        }
        else
        {
            recoveryTick();
        }
    }
}
