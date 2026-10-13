#include "obd2_internal.hpp"

class CanLoadTracker
{
private:
    TickType_t  last_slice_tick;
    uint32_t    tx_count_this_slice;
    float       current_ema_utilization;
    const float ALPHA = 0.15f;

public:
    CanLoadTracker()
    {
        last_slice_tick         = xTaskGetTickCount();
        tx_count_this_slice     = 0;
        current_ema_utilization = 0.0f;
    }

    void recordTx()
    {
        tx_count_this_slice++;
    }

    float updateAndGet()
    {
        TickType_t now           = xTaskGetTickCount();
        TickType_t elapsed_ticks = now - last_slice_tick;

        if (elapsed_ticks >= pdMS_TO_TICKS(25))
        {
            float elapsed_ms = pdTICKS_TO_MS(elapsed_ticks);

            float max_possible_frames = elapsed_ms / (float)MIN_TRANSMIT_PERIOD_MS;

            float instant_slice_util = 0.0f;
            if (max_possible_frames > 0.0f)
            {
                instant_slice_util = (float)tx_count_this_slice / max_possible_frames;
            }

            if (instant_slice_util > 1.0f)
            {
                instant_slice_util = 1.0f;
            }

            current_ema_utilization = (instant_slice_util * ALPHA) + (current_ema_utilization * (1.0f - ALPHA));

            tx_count_this_slice = 0;
            last_slice_tick     = now;
        }

        return current_ema_utilization;
    }
};

/**
 * @brief Adds PID to the OBD2 Data Model
 * Function that adds a PID to the OBD2 data model at runtime.
 * This allows dynamic configuration of which PIDs to monitor without needing to hardcode them at compile time.
 * The function checks if the PID is supported based on previously retrieved supported PID bitmaps and then adds it to
 * the polling queue if valid. Calls the addPID function of the OBD2DataModel class to maintain the data model and
 * ensure thread safety when accessing PID data.
 *
 * @param id The CAN ID to use for requests related to this PID (e.g., functional or physical ID)
 * @param mode The OBD-II mode (e.g., current data, data by identifier)
 * @param pid The PID number to add (e.g., 0x0C for engine RPM)
 * @param len The expected length of the PID data in bytes
 * @param name A human-readable name for the PID (e.g., "Engine RPM")
 * @param unit The unit of measurement for the PID value (e.g., "RPM", "°C")
 * @param desc The description of the PID, explaining what it represents and how it can be used (e.g., "Current engine
 * revolutions per minute")
 * @param formula A string representing the formula to calculate the actual value from the raw data bytes (e.g.,
 * "((A*256)+B)/4" for engine RPM)
 * @param minV The minimum valid value for the PID
 * @param maxV The maximum valid value for the PID
 * @param priority The priority level for polling this PID (lower number means higher priority)
 * @param interval The desired update interval for this PID in milliseconds
 * @param color A hexadecimal color code for UI representation of this PID (e.g., 0xFF0000 for red)
 * @param icon A string representing the icon name to use for this PID in the UI (e.g., "tachometer" for engine RPM)
 * @return esp_err_t ESP_OK if the PID was successfully added, or an appropriate error code if it failed (e.g.,
 * ESP_ERR_INVALID_ARG if the PID is not supported or already exists)
 */
esp_err_t OBD2::addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                       std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                       uint16_t interval, uint32_t color, std::string icon)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    if (interval < MIN_TRANSMIT_PERIOD_MS)
    {
        ESP_LOGW(TAG, "Requested interval %d ms is too low, setting to minimum %d ms", interval,
                 MIN_TRANSMIT_PERIOD_MS);
        interval = MIN_TRANSMIT_PERIOD_MS;
    }

    esp_err_t ret = OBD2DataModel::addPID(id, mode, pid, len, name, unit, desc, formula, minV, maxV, priority, interval,
                                          color, icon);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (mode == MODE_CURRENT_DATA)
    {
        if (pid == 0)
        {
            return ESP_ERR_INVALID_ARG;
        }

        uint8_t groupIdx = ((pid - 1) & 0xE0) >> 5;
        if (groupIdx < SUPPORTED_PIDS_GROUP_COUNT)
        {
            uint8_t bitPos = 31 - ((pid - 1) % 32);

            if (supportedPIDsGroup.pidGroup[groupIdx] & (1UL << bitPos))
            {
                withPidMapLock(
                    [&]()
                    {
                        _setDataField(pid, &PIDData_t::isSupported, true);
                        return ESP_OK;
                    });

                if (autoPoll)
                {
                    PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, true);
                    esp_err_t enqueueResult = enqueueDefinitionRequestLocked(request);
                    if (enqueueResult != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Failed to enqueue recurring PID 0x%04X: %s", pid,
                                 esp_err_to_name(enqueueResult));
                    }
                }
            }
        }
    }
    else if (mode == MODE_READ_DATA_BY_IDENTIFIER || mode == MODE_DERIVED_DATA)
    {
        if (autoPoll)
        {
            PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, true);
            esp_err_t enqueueResult = enqueueDefinitionRequestLocked(request);
            if (enqueueResult != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to enqueue recurring PID 0x%04X: %s", pid, esp_err_to_name(enqueueResult));
            }
        }
    }

    return ESP_OK;
}

esp_err_t OBD2::removePID(uint16_t pid)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }
    return OBD2DataModel::removePID(pid);
}

PollRequest OBD2::makePollRequest(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval,
                                  uint8_t priority, bool isRecurring)
{
    static std::atomic<uint32_t> staggerOffsetMs{0};

    PollRequest request = {};
    request.isRaw       = false;
    request.payload.obd.mode = mode;
    request.payload.obd.pid  = (uint16_t)pid;
    request.payload.obd.len  = len;
    request.interval          = interval;
    request.priority          = priority;
    request.isRecurring       = isRecurring;
    request.id                = id;
    request.retries_left      = DEFAULT_NUMER_OF_RETRIES;

    uint32_t maxDelay     = (interval > 0 && interval < 15) ? interval : 15;
    uint32_t initialDelay = staggerOffsetMs.fetch_add(7, std::memory_order_relaxed) % maxDelay;
    request.nextWake      = xTaskGetTickCount() + pdMS_TO_TICKS(initialDelay);
    return request;
}

esp_err_t OBD2::replacePIDDefinitions(const std::vector<PIDDefinitionData>& definitions)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    std::vector<PollRequest> recurringRequests;
    std::vector<uint16_t> supportedCurrentPids;
    recurringRequests.reserve(definitions.size());
    supportedCurrentPids.reserve(definitions.size());
    for (const auto& definition : definitions)
    {
        if (definition.mode == MODE_CURRENT_DATA && definition.pid > 0)
        {
            uint8_t groupIdx = ((definition.pid - 1) & 0xE0) >> 5;
            if (groupIdx < SUPPORTED_PIDS_GROUP_COUNT)
            {
                uint8_t bitPos = 31 - ((definition.pid - 1) % 32);
                if (supportedPIDsGroup.pidGroup[groupIdx] & (1UL << bitPos))
                {
                    supportedCurrentPids.push_back(definition.pid);
                }
            }
        }

        const bool supported =
            definition.mode == MODE_READ_DATA_BY_IDENTIFIER || definition.mode == MODE_DERIVED_DATA ||
            std::find(supportedCurrentPids.begin(), supportedCurrentPids.end(), definition.pid) !=
                supportedCurrentPids.end();
        if (autoPoll && definition.updateInterval_ms != UPDATE_DISABLED && supported)
        {
            recurringRequests.push_back(makePollRequest(definition.id, definition.mode, definition.pid,
                                                        definition.len, definition.updateInterval_ms,
                                                        definition.priority, true));
        }
    }

    esp_err_t result =
        OBD2DataModel::replacePIDDefinitions(definitions, recurringRequests, supportedCurrentPids);
    if (result == ESP_OK && derivedPidQueue_ != nullptr)
    {
        // The receive task takes the same configuration lock while evaluating
        // derived jobs. Resetting here therefore cannot leave a queued job
        // from the previous definition generation to run after the swap.
        xQueueReset(derivedPidQueue_);
    }
    return result;
}

esp_err_t OBD2::requestPID(uint16_t pid)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    return withPidMapLock(
        [&]()
        {
            if (!_getDataField(pid, &PIDData_t::isSupported, false))
            {
                ESP_LOGW(TAG, "PID 0x%02X is not supported or not recognized", pid);
                return ESP_ERR_NOT_SUPPORTED;
            }

            uint32_t id       = _getDataField(pid, &PIDData_t::id, (uint32_t)0);
            uint8_t  mode     = _getDefField(pid, &PIDDefinition::mode, (uint8_t)0);
            uint8_t  len      = _getDefField(pid, &PIDDefinition::len, (uint8_t)0);
            uint8_t  priority = _getDefField(pid, &PIDDefinition::priority, (uint8_t)0);

            PollRequest request = makePollRequest(id, mode, pid, len, 0, priority, false);
            return enqueueDefinitionRequestLocked(request);
        });
}

esp_err_t OBD2::req(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval, uint8_t priority,
               bool isRecurring)
{
    PollRequest request = makePollRequest(id, mode, pid, len, interval, priority, isRecurring);
    return req(request);
}

esp_err_t OBD2::req(PollRequest& req)
{
    if (!admitOperation())
    {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result;
    if (!req.isRaw)
    {
        MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
        result = !configurationGuard.isLocked() ? ESP_ERR_TIMEOUT : enqueueDefinitionRequestLocked(req);
    }
    else
    {
        result = pollQueue.push(req) ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    releaseOperation();
    return result;
}

esp_err_t OBD2::enqueueDefinitionRequestLocked(const PollRequest& request)
{
    return pollQueue.push(request) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

void OBD2::startPollingLocked()
{
    std::set<uint32_t> RequestByDataIdentifierIds;
    pollQueue.clearRecurring();  // refill, not append: the queue does not de-duplicate

    esp_err_t scheduleResult = withPidMapLock(
        [&]()
        {
            for (const auto& [pid, info] : PID_DEF)
            {
                if (!_getDataField(pid, &PIDData_t::isSupported, false))
                {
                    continue;
                }

                uint32_t interval = info.updateInterval();
                if (interval == 0)
                {
                    continue;
                }

                PollRequest request = makePollRequest(info.id(), info.mode(), pid, info.len(), interval,
                                                      info.priority(), true);
                esp_err_t enqueueResult = enqueueDefinitionRequestLocked(request);
                if (enqueueResult != ESP_OK)
                {
                    ESP_LOGW(TAG, "Failed to enqueue polling PID 0x%04X: %s", pid, esp_err_to_name(enqueueResult));
                }

                if (info.mode() == MODE_READ_DATA_BY_IDENTIFIER)
                {
                    RequestByDataIdentifierIds.insert(info.id());
                }
            }
            return ESP_OK;
        });
    if (scheduleResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to schedule polling definitions: %s", esp_err_to_name(scheduleResult));
    }

    // for (const uint32_t id : RequestByDataIdentifierIds)
    // {
    //     requestDefaultDiagnosticSession(id, true);
    // }
}

void OBD2::pollRequestStaticPids()
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked())
    {
        return;
    }

    std::set<uint32_t> RequestByDataIdentifierIds;

    esp_err_t scheduleResult = withPidMapLock(
        [&]()
        {
            for (const auto& [pid, def] : PID_DEF)
            {
                if (def.updateInterval() == UPDATE_DISABLED)
                {
                    PollRequest request = makePollRequest(def.id(), def.mode(), pid, def.len(),
                                                          def.updateInterval(), def.priority(), false);
                    esp_err_t enqueueResult = enqueueDefinitionRequestLocked(request);
                    if (enqueueResult != ESP_OK)
                    {
                        ESP_LOGW(TAG, "Failed to enqueue static PID 0x%04X: %s", pid, esp_err_to_name(enqueueResult));
                    }
                }

                if (def.mode() == MODE_READ_DATA_BY_IDENTIFIER)
                {
                    RequestByDataIdentifierIds.insert(def.id());
                }
            }
            return ESP_OK;
        });
    if (scheduleResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to schedule static PID requests: %s", esp_err_to_name(scheduleResult));
    }

    // for (const uint32_t id : RequestByDataIdentifierIds)
    // {
    //     requestDefaultDiagnosticSession(id, false);
    // }
}

void OBD2::pollTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->pollTask();
    obd2->pollQueue.setConsumerTask(nullptr);
    obd2->workerExit(POLL_EXIT);
}

void OBD2::pollTask()
{
    CanLoadTracker                 busTracker;
    std::map<uint32_t, TickType_t> last_tx_time_per_ecu;

    if (lifecycleEvents_)
    {
        xEventGroupWaitBits(lifecycleEvents_, STARTUP_GATE, pdFALSE, pdTRUE, portMAX_DELAY);
    }
    while (!stopRequested_)
    {
        TickType_t delay = pollQueue.getWait();
        ulTaskNotifyTake(pdTRUE, delay);
        if (stopRequested_)
        {
            break;
        }

        pollTaskUtilization = busTracker.updateAndGet();

        // Keep the lock order bus -> configuration -> PID map/queue. The
        // non-blocking configuration attempt prevents a poll cycle from
        // holding the bus while a replacement is waiting to commit.
        {
        MutexGuard busGuard(xBusArbitrationMutex, pdMS_TO_TICKS(100));
        if (!busGuard.isLocked())
        { ulTaskNotifyTake(pdTRUE, 1); continue; }

        MutexGuard configurationGuard(configurationMtx_, 0);
        if (!configurationGuard.isLocked() || !canDriver.isBusConnected())
        {
            // Leave both RAII scopes before notification-aware backoff.
            goto poll_busy_backoff;
        }
        if (stopRequested_)
        {
            break;
        }

        // 2. Take the most urgent appointment
        PollRequest current = {};
        if (!pollQueue.tryPopDue(current))
        {
            continue;
        }

        if (stopRequested_)
        {
            break;
        }

        TickType_t now = xTaskGetTickCount();

        // if (!current.isRaw && current.isRecurring)
        // {
        //     uint16_t pid = current.payload.obd.pid;

        if (!current.isRaw && current.isRecurring)
        {
            uint16_t pid = current.payload.obd.pid;

            withPidMapLock(
                [&]()
                {
                    if (_pidExists(pid))
                    {
                        uint32_t last_updated = _getDataField(pid, &PIDData_t::lastUpdated, (uint32_t)0);
                        uint32_t now_ms       = pdTICKS_TO_MS(now);

                        uint32_t threshold = current.interval * 3;
                        if (threshold < 1000)
                        {
                            threshold = 1000;
                        }

                        if (last_updated == 0 || (now_ms - last_updated > threshold))
                        {
                            if (current.retries_left > 0)
                            {
                                current.retries_left--;
                            }
                            else
                            {
                                _setDataField(pid, &PIDData_t::isValid, false);
                                current.isRecurring = false;
                            }
                        }
                        else
                        {
                            current.retries_left = DEFAULT_NUMER_OF_RETRIES;
                        }
                    }
                    return ESP_OK;
                });
        }

        bool scheduled = false;
        auto schedule  = [&](PollRequest& request, const char* reason)
        {
            const bool queued = (request.isRecurring && !request.isRaw) ? pollQueue.pushRecurringUnique(request)
                                                                        : pollQueue.push(request);
            if (!queued)
            {
                ESP_LOGW(TAG, "Poll queue full; dropped %s PID 0x%04X", reason, request.payload.obd.pid);
            }
            scheduled = true;
        };

        if (last_tx_time_per_ecu.count(current.id) > 0)
        {
            TickType_t time_since_last = now - last_tx_time_per_ecu[current.id];
            if (time_since_last < pdMS_TO_TICKS(30))
            {
                current.nextWake = last_tx_time_per_ecu[current.id] + pdMS_TO_TICKS(30);
                schedule(current, "deferred");
                continue;
            }
        }

        // 3. Process based on the Mode stored in the request
        if (stopRequested_)
        {
            break;
        }
        if (!current.isRaw && current.payload.obd.mode == MODE_DERIVED_DATA)
        {
            xQueueSend(derivedPidQueue_, &current.payload.obd.pid, pdMS_TO_TICKS(10));
        }
        else
        {
            esp_err_t err = queryMsg(current);

            if (err == ESP_OK)
            {
                busTracker.recordTx();
                last_tx_time_per_ecu[current.id] = xTaskGetTickCount();
            }
            else if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL)
            {
                if (current.retries_left > 0)
                {
                    current.retries_left--;
                    current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(current.interval);
                    schedule(current, "retry");
                }
                else
                {
                    current.isRecurring = false;
                }
            }

        }

        // 4. Reschedule recurring tasks
        if (!stopRequested_ && !scheduled && current.isRecurring && autoPoll)
        {
            current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(current.interval);
            schedule(current, "recurring");
        }

        taskYIELD();
        }
        continue;

    poll_busy_backoff:
        if (!stopRequested_)
        {
            ulTaskNotifyTake(pdTRUE, 1);
        }
    }
}
