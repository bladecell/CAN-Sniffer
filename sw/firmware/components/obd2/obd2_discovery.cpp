#include "obd2_internal.hpp"

bool OBD2::lockDiscovery()
{
    if (!discoveryMtx_)
    {
        return false;
    }
    while (!stopRequested_)
    {
        if (xSemaphoreTake(discoveryMtx_, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (!stopRequested_)
            {
                return true;
            }
            xSemaphoreGive(discoveryMtx_);
            return false;
        }
    }
    return false;
}

bool OBD2::lockDiagnosticRequest()
{
    if (!diagnosticRequestMtx_)
    {
        return false;
    }
    while (!stopRequested_)
    {
        if (xSemaphoreTake(diagnosticRequestMtx_, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (!stopRequested_)
            {
                return true;
            }
            xSemaphoreGive(diagnosticRequestMtx_);
            return false;
        }
    }
    return false;
}

void OBD2::requestSuppPids()
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return;
    }
    if (!lockDiscovery())
    {
        return;
    }
    requestSuppPidsInternal();
    xSemaphoreGive(discoveryMtx_);
}

void OBD2::requestSuppPidsInternal()
{
    if (stopRequested_ || !canDriver.isBusConnected())
    {
        return;
    }
    esp_err_t resetResult = ESP_OK;
    memset(knownVin_, 0, sizeof(knownVin_));  // the supported list is about to change; it no longer matches any VIN
    {
        MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
        if (!configurationGuard.isLocked())
        {
            resetResult = ESP_ERR_TIMEOUT;
        }
        else
        {
            resetResult = withPidMapLock(
                [&]() -> esp_err_t
                {
                    pollQueue.clearRecurringMode(MODE_CURRENT_DATA);
                    for (auto& [pid, definition] : PID_DEF)
                    {
                        if (definition.mode() != MODE_CURRENT_DATA)
                        {
                            continue;
                        }
                        _setDataField(pid, &PIDData_t::isSupported, false);
                        _setDataField(pid, &PIDData_t::isValid, false);
                        _setDataField(pid, &PIDData_t::id, definition.id());
                    }
                    supportedPIDsGroup = {};
                    discoveryActive_ = false;
                    discoveryFailed_ = false;
                    discoveryExpectedGroup_ = 0xFF;
                    discoverySeenGroups_ = 0;
                    pidsInitialized = false;
                    return ESP_OK;
                });
        }
    }

    while (xSemaphoreTake(xRequestNextPIDSemaphore, 0) == pdTRUE)
    {
        ;
    }
    while (xSemaphoreTake(xPidConnectedSemaphore, 0) == pdTRUE)
    {
        ;
    }

    if (resetResult != ESP_OK)
    {
        pidsInitialized = false;
        xSemaphoreGive(xPidConnectedSemaphore);
        return;
    }

    bool terminalSuccess = false;
    bool transitionFailed = false;

    for (uint8_t group = 0; group < SUPPORTED_PIDS_GROUP_COUNT && !terminalSuccess && !stopRequested_ &&
                           canDriver.isBusConnected(); ++group)
    {
        while (xSemaphoreTake(xRequestNextPIDSemaphore, 0) == pdTRUE)
        {
            ;
        }

        bool opened = false;
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (!configurationGuard.isLocked())
            {
                transitionFailed = true;
            }
            else
            {
                // The active generation and its expected group are published
                // together. Response waits never hold this lock.
                discoveryActive_        = true;
                discoveryExpectedGroup_ = group;
                opened                  = true;
            }
        }

        if (!opened)
        {
            break;
        }

        PollRequest probe         = {};
        probe.id                  = OBD2_FUNCTIONAL_ID;
        probe.isRaw               = true;
        obd2frame::buildCurrentDataRequest((uint8_t)(group << 5), probe.payload.raw.data);
        probe.payload.raw.dlc     = 8;
        probe.interval            = 0;
        probe.nextWake            = xTaskGetTickCount();
        probe.priority            = 0;
        probe.isRecurring         = false;
        probe.retries_left        = 1;

        if (!pollQueue.push(probe))
        {
            ESP_LOGW(TAG, "Poll queue full; supported-PID probe group %u dropped", group);
            bool closed = false;
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                discoveryFailed_        = true;
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                closed                  = true;
            }
            if (!closed)
            {
                transitionFailed = true;
            }
            break;
        }
        bool receivedExpected = false;
        TickType_t responseDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SUPPORTED_PID_RESPONSE_TIMEOUT_MS);
        while (!stopRequested_ && canDriver.isBusConnected() && !receivedExpected)
        {
            TickType_t remaining = responseDeadline - xTaskGetTickCount();
            if ((int32_t)remaining <= 0)
            {
                break;
            }
            receivedExpected = xSemaphoreTake(xRequestNextPIDSemaphore, std::min(remaining, pdMS_TO_TICKS(50))) == pdTRUE;
        }
        if (stopRequested_ || !canDriver.isBusConnected())
        {
            break;
        }
        if (!receivedExpected)
        {
            bool closed = false;
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                discoveryFailed_        = true;
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                closed                  = true;
            }
            if (!closed)
            {
                transitionFailed = true;
            }
            break;
        }

        // Keep collecting valid responses for this group so multiple ECUs
        // contribute to the union before the next raw probe is sent.
        TickType_t windowDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(SUPPORTED_PID_RESPONSE_WINDOW_MS);
        while (!stopRequested_ && canDriver.isBusConnected())
        {
            TickType_t remaining = windowDeadline - xTaskGetTickCount();
            if ((int32_t)remaining <= 0 || xSemaphoreTake(xRequestNextPIDSemaphore, remaining) != pdTRUE)
            {
                break;
            }
        }
        if (stopRequested_ || !canDriver.isBusConnected())
        {
            break;
        }

        bool windowClosed    = false;
        bool continueDiscovery = false;
        {
            MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
            if (configurationGuard.isLocked())
            {
                const bool groupSeen = (discoverySeenGroups_ & (1UL << group)) != 0;
                const bool continuation = (supportedPIDsGroup.pidGroup[group] & 0x00000001UL) != 0;

                if (!groupSeen)
                {
                    discoveryFailed_ = true;
                }

                // Close the response generation before deciding whether a
                // subsequent group may be requested.
                discoveryActive_        = false;
                discoveryExpectedGroup_ = 0xFF;
                continueDiscovery       = groupSeen && continuation && group + 1 < SUPPORTED_PIDS_GROUP_COUNT;
                terminalSuccess         = groupSeen && (!continuation || group + 1 == SUPPORTED_PIDS_GROUP_COUNT);
                windowClosed             = true;
            }
        }

        if (!windowClosed)
        {
            transitionFailed = true;
            break;
        }

        if (terminalSuccess || !continueDiscovery)
        {
            break;
        }
    }

    {
        MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
        if (configurationGuard.isLocked())
        {
            discoveryActive_        = false;
            discoveryExpectedGroup_ = 0xFF;
            supportedPIDsGroup.numberOfSupportedPIDs = 0;
            for (const auto& group : supportedPIDsGroup.pidGroup)
            {
                supportedPIDsGroup.numberOfSupportedPIDs += __builtin_popcount(group);
            }

            if (transitionFailed)
            {
                discoveryFailed_ = true;
            }

            const bool complete = terminalSuccess && !discoveryFailed_ && !stopRequested_ && canDriver.isBusConnected();
            pidsInitialized = complete;
            if (stopRequested_ || !canDriver.isBusConnected())
            {
                pidsInitialized = false;
            }
            if (!complete)
            {
                ESP_LOGW(TAG, "Supported-PID discovery incomplete (seen=0x%08lX)", (unsigned long)discoverySeenGroups_);
            }
        }
        else
        {
            pidsInitialized = false;
        }
    }

    // Wake the connection path on both complete and incomplete terminal
    // outcomes; it must not wait indefinitely for discovery.
    xSemaphoreGive(xPidConnectedSemaphore);
}

void OBD2::getSupportedPids(supportedPIDsGroup_t& supportedPIDsGroup)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (configurationGuard.isLocked())
    {
        supportedPIDsGroup = this->supportedPIDsGroup;
    }
};
