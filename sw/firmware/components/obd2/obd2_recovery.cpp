#include "obd2_internal.hpp"

bool OBD2::isPidInit() const
{
    return pidsInitialized;
}

void OBD2::onCanStateChange(void* arg, bool connected)
{
    OBD2* instance = static_cast<OBD2*>(arg);

    if (connected)
    {
        instance->handleCanConnected();
    }
    else
    {
        instance->handleCanDisconnected();
    }
}

void OBD2::handleCanConnected()
{
    OperationGuard operation(this);
    if (!operation.admitted() || stopRequested_)
    {
        return;
    }
    if (!lockDiscovery())
    {
        return;
    }
    if (stopRequested_ || !canDriver.isBusConnected()) { xSemaphoreGive(discoveryMtx_); return; }
    ESP_LOGI(TAG, "CAN bus connected event received");
    xSemaphoreGive(xBusConnectionSemaphore);
    recoveryNeeded_ = false;
    recoveryBackoffMs_ = 0;
    pendingDiscover_ = false;
    if (autoPoll)
    {
        if (!recoverOnce())
        {
            armRecovery();
        }
    }
    else if (!pidsInitialized)
    {
        ESP_LOGI(TAG, "Retrieving supported PIDs...");
        requestSuppPidsInternal();
    }

    if (!stopRequested_ && canDriver.isBusConnected())
    {
        runOBDIIConnectedCallbacks(true);
    }
    xSemaphoreGive(discoveryMtx_);
}

// One recovery attempt with autoPoll set. Caller holds discoveryMtx_. Returns true when the queue is (re)filled.
// Also returns true, doing nothing, when OBD2 is stopping or the bus is down: there is nothing to retry then, and a
// later connect event restarts recovery.
// A cold ECU misses the first discovery window; the caller retries with backoff (see recoveryTick).
bool OBD2::recoverOnce()
{
    if (stopRequested_ || !canDriver.isBusConnected())
    {
        return true;
    }
    std::string vin;
    if (requestVinInternal() == ESP_OK)
    {
        vin = getVIN();
        if (!obd2frame::isValidVin(vin.c_str()))
        {
            vin.clear();
        }
    }
    if (obd2frame::planOnConnect(vin.c_str(), knownVin_) == obd2frame::ConnectPlan::RestoreKnown)
    {
        ESP_LOGI(TAG, "Same VIN as last discovery; refilling poll queue without discovery");
        restoreFromKnown(portMAX_DELAY);
        return true;
    }
    return discoverAndFill(vin);
}

// Keep the supported list, mark OBD initialised and refill the queue if the user wants polling. Caller holds discoveryMtx_.
bool OBD2::restoreFromKnown(TickType_t wait)
{
    MutexGuard configurationGuard(configurationMtx_, wait);
    if (!configurationGuard.isLocked() || stopRequested_)
    {
        return false;
    }
    pidsInitialized = true;
    if (autoPoll)
    {
        startPollingLocked();
    }
    return true;
}

// Supported-PID discovery for `vin` (empty if unknown), then record the VIN and refill if autoPoll. Caller holds
// discoveryMtx_. False means discovery missed and the caller may retry.
bool OBD2::discoverAndFill(const std::string& vin)
{
    ESP_LOGI(TAG, "Retrieving supported PIDs...");
    requestSuppPidsInternal();
    if (stopRequested_ || !canDriver.isBusConnected())
    {
        return true;
    }
    if (!pidsInitialized)
    {
        return false;
    }
    strncpy(knownVin_, vin.c_str(), VIN_LENGTH);
    knownVin_[VIN_LENGTH] = '\0';
    // Discovery enqueues Mode 01 work per group; rebuild the whole queue so Mode 22 / derived entries are present too.
    MutexGuard configurationGuard(configurationMtx_, portMAX_DELAY);
    if (configurationGuard.isLocked() && !stopRequested_ && autoPoll)
    {
        startPollingLocked();
    }
    return true;
}

void OBD2::armRecovery()
{
    const uint32_t backoff = obd2frame::nextDiscoveryBackoffMs(recoveryBackoffMs_);
    recoveryBackoffMs_ = backoff;
    recoveryDueTick_   = (uint32_t)(xTaskGetTickCount() + pdMS_TO_TICKS(backoff));
    recoveryNeeded_    = true;
}

// Called from the callback worker when idle. Never holds discoveryMtx_ while waiting.
void OBD2::recoveryTick()
{
    if (!recoveryNeeded_ || stopRequested_)
    {
        return;
    }
    if (!canDriver.isBusConnected())
    {
        recoveryNeeded_ = false;
        return;
    }
    if ((int32_t)((uint32_t)xTaskGetTickCount() - recoveryDueTick_.load()) < 0)
    {
        return;
    }
    OperationGuard operation(this);
    if (!operation.admitted() || !lockDiscovery())
    {
        return;
    }
    bool done = true;
    if (recoveryNeeded_)
    {
        if (pendingDiscover_)
        {
            done = discoverAndFill(pendingVin_);
            if (!done && !autoPoll)
            {
                done = true;  // a user VIN read without autoPoll gets one attempt
            }
            pendingDiscover_ = !done;
        }
        else if (autoPoll && !pidsInitialized)
        {
            done = recoverOnce();
        }
        if (done)
        {
            recoveryNeeded_    = false;
            recoveryBackoffMs_ = 0;
        }
        else
        {
            armRecovery();
        }
    }
    xSemaphoreGive(discoveryMtx_);
}

// Health probe answered after the health check cleared the queue: refill from the known supported list.
void OBD2::restoreQueueAfterHealthClear()
{
    if (!obd2frame::shouldRestoreQueue(autoPoll, canDriver.isBusConnected(), pidsInitialized,
                                       pollQueue.hasRecurring()))
    {
        return;
    }
    MutexGuard configurationGuard(configurationMtx_, pdMS_TO_TICKS(100));
    if (!configurationGuard.isLocked() || stopRequested_ || !autoPoll)
    {
        return;
    }
    ESP_LOGI(TAG, "ECU answered again; restoring poll queue");
    startPollingLocked();
}

void OBD2::handleCanDisconnected()
{
    OperationGuard operation(this);
    if (!operation.admitted() || stopRequested_)
    {
        return;
    }
    if (!lockDiscovery())
    {
        return;
    }
    if (stopRequested_ || canDriver.isBusConnected()) { xSemaphoreGive(discoveryMtx_); return; }
    ESP_LOGW(TAG, "CAN bus disconnected event received");
    pidsInitialized = false;
    recoveryNeeded_ = false;
    pendingDiscover_ = false;
    memset(pendingVin_, 0, sizeof(pendingVin_));
    runOBDIIConnectedCallbacks(false);
    xSemaphoreGive(discoveryMtx_);
}

bool OBD2::isAutoPoll() const
{
    return autoPoll;
}

bool OBD2::isPolling()
{
    return autoPoll && canDriver.isBusConnected() && pollQueue.hasRecurring();
}

esp_err_t OBD2::setAutoPoll(bool on)
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
    if (stopRequested_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    autoPoll = on;
    if (on)
    {
        startPollingLocked();
        if (!pidsInitialized && canDriver.isBusConnected())
        {
            recoveryBackoffMs_ = 0;
            armRecovery();
        }
    }
    else
    {
        recoveryNeeded_ = false;
        pendingDiscover_ = false;  // flag only: discoveryMtx_ is never taken under configurationMtx_
        pollQueue.clearRecurring();
    }
    return ESP_OK;
}
