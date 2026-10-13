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

    resetMultiFrame();
    last_multiframe_received = 0;
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
    multiframe_state = 99; busMutexOwnedByReceive_ = false;
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

    // Register/fence first, then take the bus snapshot to avoid missing a transition.
    if (canDriver.isBusConnected())
    {
        handleCanConnected();
    }
    ESP_LOGI(TAG, "OBD-II interface initialized");
    return finish(ESP_OK);
}

void OBD2::receiveTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->receiveTask();
    obd2->resetMultiFrame();
    obd2->workerExit(RECEIVE_EXIT);
}

void OBD2::receiveTask()
{
    if (lifecycleEvents_)
    {
        xEventGroupWaitBits(lifecycleEvents_, STARTUP_GATE, pdFALSE, pdTRUE, portMAX_DELAY);
    }
    TickType_t last_ping_ms      = 0;
    uint8_t    failed_pings      = 0;
    bool       awaiting_response = false;
    TickType_t response_deadline = 0;

    while (!stopRequested_)
    {
        if (!canDriver.isBusConnected())
        {
            xSemaphoreTake(xBusConnectionSemaphore, portMAX_DELAY);
            if (stopRequested_)
            {
                break;
            }

            // Reset health-check state on (re)connect.
            last_ping_ms      = xTaskGetTickCount();
            failed_pings      = 0;
            awaiting_response = false;
            continue;
        }

        // Definition-bound derived work is produced by the polling task and
        // consumed here under the configuration lock. A replacement either
        // waits for this whole drain/evaluation or resets the queue before
        // this task can observe the new model.
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (stopRequested_)
            {
                break;
            }
            if (!configurationGuard.isLocked())
            {
                continue;
            }
            uint16_t derivedPid;

            while (!stopRequested_ && xQueueReceive(derivedPidQueue_, &derivedPid, 0) == pdTRUE)
            {
                if (stopRequested_)
                {
                    break;
                }
                CanDriver::CanFrame f{};
                f.header.id = OBD2_FUNCTIONAL_ID;
                f.length    = 4;
                f.data[0]   = 0x03;  // DLC
                f.data[1]   = RESPONSE_MODE_DERIVED_DATA;
                f.data[2]   = (derivedPid >> 8) & 0xFF;
                f.data[3]   = derivedPid & 0xFF;
                parseRecFrame(f);
            }
        }

        CanDriver::CanFrame f{};
        esp_err_t           ret = canDriver.receive(f, 10);
        if (stopRequested_)
        {
            break;
        }

        if (ret == ESP_OK)
        {
            ret = parseRecFrame(f);

            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to parse received frame: %s", esp_err_to_name(ret));
            }
        }
        else if (ret != ESP_ERR_TIMEOUT)
        {
            ESP_LOGE(TAG, "CAN receive driver error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        multiframe_watchdog();

        // Health check: periodically probe Mode 1 (supported PIDs) and clear the
        // poll queue if the ECU stops responding (e.g. ignition turned off).
        TickType_t now = xTaskGetTickCount();

        if (now - last_ping_ms >= pdMS_TO_TICKS(OBD2_HEALTHCHECK_PING_PERIOD_MS))
        {
            last_ping_ms = now;

            while (xSemaphoreTake(healthCheckSemaphore, 0) == pdTRUE)
            {
                ;
            }

            PollRequest r         = {};
            r.id                  = OBD2_FUNCTIONAL_ID;
            r.isRaw               = true;
            r.payload.raw.data[0] = 0x02;
            r.payload.raw.data[1] = MODE_CURRENT_DATA;
            r.payload.raw.data[2] = PID_PIDS_SUPPORTED_01_20;
            r.payload.raw.dlc     = 8;
            r.interval            = 0;
            r.nextWake            = now;
            r.priority            = 0;
            r.isRecurring         = false;
            r.retries_left        = 1;

            esp_err_t enqueueResult = req(r);
            if (enqueueResult == ESP_OK)
            {
                awaiting_response = true;
                response_deadline = now + pdMS_TO_TICKS(HEALTHCHECK_RESPONSE_TIMEOUT_MS);
            }
        }

        if (awaiting_response)
        {
            if (xSemaphoreTake(healthCheckSemaphore, 0) == pdTRUE)
            {
                failed_pings      = 0;
                awaiting_response = false;
                restoreQueueAfterHealthClear();
            }
            else if (now >= response_deadline)
            {
                failed_pings++;
                ESP_LOGW(TAG, "OBD Health Check timeout (%d/%d)", failed_pings, HEALTHCHECK_RETRIES);

                if (failed_pings >= HEALTHCHECK_RETRIES)
                {
                    ESP_LOGW(TAG, "OBD Health Check failed %d times! Clearing poll queue.", HEALTHCHECK_RETRIES);

                    pollQueue.clear();
                    failed_pings = 0;
                }

                awaiting_response = false;
            }
        }
    }
}

esp_err_t OBD2::parseRecFrame(const CanDriver::CanFrame& f)
{
    esp_err_t       ret = ESP_OK;
    obd2frame::IsoTp frame_type = obd2frame::IsoTp::Single;
    ret                         = obd2frame::frameType(f.data, f.length, frame_type);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (frame_type != obd2frame::IsoTp::Single)
    {
        return captureMultiFrame(f);
    }

    obd2frame::SingleFrameInfo info;
    ret = obd2frame::parseSingleFrame(f.data, f.length, info);
    if (ret != ESP_OK)
    {
        return ret;
    }

    uint8_t mode = info.mode;

    switch (mode)
    {
        case RESPONSE_CURRENT_DATA:
            ret = parseCurrentData(f);
            break;
        case RESPONSE_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_CLEAR_DTCS:
            parseClearDTCsAck(f);
            break;
        case RESPONSE_PENDING_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_VEHICLE_INFO:
            // TODO parseVehicleInfo(f);
            break;
        case RESPONSE_PERMANENT_DTCS:
        {
            std::vector<CanDriver::CanFrame> frame = {f};
            ret                                    = parseDTCs(frame, mode);
            break;
        }
        case RESPONSE_MODE_DIAGNOSTIC_SESSION_CONTROL:
            parseRMDSC(f);
            break;
        case RESPONSE_READ_DATA_BY_IDENTIFIER:
            ret = parseRDBI(f);
            break;
        case RESPONSE_MODE_DERIVED_DATA:
            ret = parseDerivedData(f);
            break;
        case RESPONSE_NEGATIVE_RESPONSE_CODE:
            ret = handleNegativeResponse(f);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::handleNegativeResponse(const CanDriver::CanFrame& f)
{
    if (f.length < 4)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t id   = f.header.id;
    uint8_t  mode = f.data[2];
    uint8_t  nrc  = f.data[3];

    bool found = false;
    for (uint8_t i = 0; i < nrc_list_size; i++)
    {
        if (nrc_list[i].can_id == id && nrc_list[i].mode == mode && nrc_list[i].nrc == nrc)
        {
            found = true;
            break;
        }
    }

    if (!found)
    {
        if (nrc_list_size < MAX_NRC_LIST_SIZE)
        {
            nrc_list[nrc_list_size++] = {id, mode, nrc};
        }
        else
        {
            for (uint8_t i = 1; i < MAX_NRC_LIST_SIZE; i++)
            {
                nrc_list[i - 1] = nrc_list[i];
            }
            nrc_list[MAX_NRC_LIST_SIZE - 1] = {id, mode, nrc};
        }
    }

    return ESP_OK;
}

esp_err_t OBD2::parseCurrentData(const CanDriver::CanFrame& f)
{
    uint16_t  pid = f.data[2];
    esp_err_t ret;

    switch (pid)
    {
        case PID_PIDS_SUPPORTED_01_20:
        case PID_PIDS_SUPPORTED_21_40:
        case PID_PIDS_SUPPORTED_41_60:
        case PID_PIDS_SUPPORTED_61_80:
        case PID_PIDS_SUPPORTED_81_A0:
        case PID_PIDS_SUPPORTED_A1_C0:
        case PID_PIDS_SUPPORTED_C1_E0:
            ret = parseSupportedPIDs(f);
            break;
        default:
            ret = updateData(f);
    }

    if (ret == ESP_ERR_INVALID_STATE)
    {
        return ret;
    }

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid, &stopRequested_);

    return ret;
}

esp_err_t OBD2::parseRMDSC(const CanDriver::CanFrame& f)
{
    if (f.data[2] != 0x01)
    {
        ESP_LOGW(TAG, "Failed to request Default Extended Diagnostic session for ID 0x%03X", f.header.id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t OBD2::parseRDBI(const CanDriver::CanFrame& f)
{
    uint16_t  pid = (uint16_t)(f.data[2] << 8) | f.data[3];
    esp_err_t ret;

    ret = updateData(f);

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid, &stopRequested_);

    return ret;
}

esp_err_t OBD2::parseDerivedData(const CanDriver::CanFrame& f)
{
    uint16_t pid = (uint16_t)(f.data[2] << 8) | f.data[3];

    esp_err_t ret = withPidMapLock(
        [&]() -> esp_err_t
        {
            PIDData_t* pdat = nullptr;
            esp_err_t  err  = _getData(pid, pdat);

            if (err != ESP_OK || pdat == nullptr)
            {
                _setDataField(pid, &PIDData_t::isValid, false);
                return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
            }

            const PIDDefinition* pdef = nullptr;
            err                       = _getDef(pid, pdef);

            if (err != ESP_OK || pdef == nullptr)
            {
                _setDataField(pid, &PIDData_t::isValid, false);
                return (err == ESP_OK) ? ESP_ERR_NOT_FOUND : err;
            }

            float result = 0.0f;
            err          = pdef->evaluate(nullptr, 0, result);

            pdat->value       = result;
            pdat->id          = OBD2_FUNCTIONAL_ID;
            pdat->lastUpdated = xTaskGetTickCount();
            pdat->isValid     = (err == ESP_OK);

            return err;
        });

    if (ret == ESP_OK)
    {
        runPidUpdateCallbacks(pid, &stopRequested_);
    }

    return ret;
}

esp_err_t OBD2::parseDTCs(std::vector<CanDriver::CanFrame>& frames, uint8_t mode)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::vector<obd2frame::RawFrame> raw;
    ESP_RETURN_ON_ERROR(toRawFrames(frames, raw), TAG, "bad frame length");

    std::vector<uint16_t> rawDtcs;
    esp_err_t             ret = obd2frame::decodeDtcResponse(raw.data(), raw.size(), mode, rawDtcs);
    if (ret != ESP_OK)
    {
        return ret;
    }

    for (const uint16_t rawDTC : rawDtcs)
    {
        ERR_ACCUMULATE(ret, _setDTC(rawDTC, mode));
    }

    switch (mode)
    {
        case RESPONSE_DTCS:
            xSemaphoreGive(dtcData.confirmedReadySemaphore);
            break;
        case RESPONSE_PENDING_DTCS:
            xSemaphoreGive(dtcData.pendingReadySemaphore);
            break;
        case RESPONSE_PERMANENT_DTCS:
            xSemaphoreGive(dtcData.permanentReadySemaphore);
            break;
        default:
            break;
    }

    return ret;
}

esp_err_t OBD2::parseSupportedPIDs(const CanDriver::CanFrame& f)
{
    uint16_t pidGroup = f.data[2];
    if (pidGroup % 0x20 != 0)
    {
        ESP_LOGE(TAG, "Invalid PID group: 0x%02X", pidGroup);
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t groupIdx = pidGroup >> 5;
    if (groupIdx >= SUPPORTED_PIDS_GROUP_COUNT)
    {
        ESP_LOGE(TAG, "Unsupported PID group: 0x%02X", pidGroup);
        return ESP_ERR_INVALID_ARG;
    }

    if (f.length < 7)
    {
        return ESP_OK;
    }

    uint32_t reportedPIDs = (f.data[3] << 24) | (f.data[4] << 16) | (f.data[5] << 8) | (f.data[6]);

    MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
    if (!configurationGuard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    if (!discoveryActive_ || discoveryExpectedGroup_ != groupIdx)
    {
        // A late response must not mutate an inactive or different discovery
        // generation. Health-check waiters may still use this valid frame.
        if (healthCheckSemaphore != nullptr)
        {
            xSemaphoreGive(healthCheckSemaphore);
        }
        return ESP_OK;
    }

    PollRequest recurringRequests[32];
    size_t      recurringCount = 0;
    const uint16_t firstPID    = pidGroup + 1;
    const uint16_t lastPID     = pidGroup + 32;
    const uint32_t responderID = f.header.id - RESPONSE_ID_OFFSET;
    const uint32_t mergedPIDs  = supportedPIDsGroup.pidGroup[groupIdx] | reportedPIDs;

    esp_err_t ret = withPidMapLock(
        [&]() -> esp_err_t
        {
            // Stage all desired recurring work before changing support flags.
            // The queue operation below is allocation-free and capacity
            // checked, so a failure leaves both the queue and model unchanged.
            for (const auto& [installedPID, definition] : PID_DEF)
            {
                if (definition.mode() != MODE_CURRENT_DATA || installedPID < firstPID || installedPID > lastPID)
                {
                    continue;
                }

                const uint8_t bit = (uint8_t)(installedPID - firstPID);
                if (autoPoll && (mergedPIDs & (1UL << (31 - bit))) &&
                    definition.updateInterval() != UPDATE_DISABLED)
                {
                    recurringRequests[recurringCount++] =
                        makePollRequest(definition.id(), definition.mode(), installedPID, definition.len(),
                                        definition.updateInterval(), definition.priority(), true);
                }
            }

            if (!pollQueue.replaceRecurringPidRange(firstPID, lastPID, MODE_CURRENT_DATA, recurringRequests,
                                                    recurringCount))
            {
                discoveryFailed_ = true;
                return ESP_ERR_INVALID_SIZE;
            }

            for (const auto& [installedPID, definition] : PID_DEF)
            {
                if (definition.mode() != MODE_CURRENT_DATA || installedPID < firstPID || installedPID > lastPID)
                {
                    continue;
                }

                const bool supported = (mergedPIDs & (1UL << (31 - (installedPID - firstPID)))) != 0;
                const bool wasSupported = _getDataField(installedPID, &PIDData_t::isSupported, false);
                _setDataField(installedPID, &PIDData_t::isSupported, supported);
                _setDataField(installedPID, &PIDData_t::isValid, false);
                if (supported && !wasSupported)
                {
                    _setDataField(installedPID, &PIDData_t::id, responderID);
                }
                else if (!supported)
                {
                    _setDataField(installedPID, &PIDData_t::id, definition.id());
                }
            }

            supportedPIDsGroup.pidGroup[groupIdx] = mergedPIDs;
            discoverySeenGroups_ |= (1UL << groupIdx);
            supportedPIDsGroup.numberOfSupportedPIDs = 0;
            for (const auto& group : supportedPIDsGroup.pidGroup)
            {
                supportedPIDsGroup.numberOfSupportedPIDs += __builtin_popcount(group);
            }
            return ESP_OK;
        });

    if (ret != ESP_OK)
    {
        return ret;
    }

    xSemaphoreGive(xRequestNextPIDSemaphore);

    if (healthCheckSemaphore != nullptr)
    {
        xSemaphoreGive(healthCheckSemaphore);
    }

    return ret;
}

esp_err_t OBD2::parseClearDTCsAck(const CanDriver::CanFrame& f)
{
    if (f.length < 3 || f.data[1] != RESPONSE_CLEAR_DTCS)
    {
        ESP_LOGE(TAG, "Invalid frame for Clear DTCs ACK: length=%d, mode=0x%02X", f.length, f.data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    clearDTC(MODE_DTCS);
    if (dtcData.clearReadySemaphore != NULL)
    {
        xSemaphoreGive(dtcData.clearReadySemaphore);
    }
    return ESP_OK;
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
