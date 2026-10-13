#include "obd2_internal.hpp"

// A user VIN request succeeded. Never waits on discovery: takes discoveryMtx_ without blocking, applies cheap actions
// inline and hands a rediscovery to the callback worker. Never turns autoPoll on.
void OBD2::onVinConfirmed()
{
    const std::string vin = getVIN();
    if (!obd2frame::isValidVin(vin.c_str()) || !discoveryMtx_ || xSemaphoreTake(discoveryMtx_, 0) != pdTRUE)
    {
        return;  // discovery or another recovery is running and will settle the state itself
    }
    if (!stopRequested_ && canDriver.isBusConnected())
    {
        switch (obd2frame::planOnVinConfirmed(vin.c_str(), knownVin_, pidsInitialized))
        {
            case obd2frame::VinAction::Adopt:
                strncpy(knownVin_, vin.c_str(), VIN_LENGTH);
                knownVin_[VIN_LENGTH] = '\0';
                pendingDiscover_ = false;
                break;
            case obd2frame::VinAction::Restore:
                pendingDiscover_ = false;
                // Bounded wait on the HTTP / callback thread; on a busy lock the next connect or VIN read retries.
                if (!restoreFromKnown(pdMS_TO_TICKS(100)))
                {
                    ESP_LOGW(TAG, "VIN matched but the configuration lock is busy; restore skipped");
                }
                break;
            case obd2frame::VinAction::Discover:
                strncpy(pendingVin_, vin.c_str(), VIN_LENGTH);
                pendingVin_[VIN_LENGTH] = '\0';
                pendingDiscover_        = true;
                recoveryBackoffMs_      = 0;
                recoveryDueTick_        = (uint32_t)xTaskGetTickCount();
                recoveryNeeded_         = true;
                break;
            case obd2frame::VinAction::None:
                pendingDiscover_ = false;
                break;
        }
    }
    xSemaphoreGive(discoveryMtx_);
}

void OBD2::requestDefaultDiagnosticSession(uint32_t id, bool isRecurring)
{
    PollRequest r         = {};
    r.id                  = id;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_DIAGNOSTIC_SESSION_CONTROL;
    r.payload.raw.data[2] = 0x01;
    r.payload.raw.dlc     = 8;
    r.interval            = isRecurring ? 2000 : 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = isRecurring;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;
    esp_err_t enqueueResult = req(r);
    if (enqueueResult != ESP_OK)
    {
        ESP_LOGW(TAG, "Default diagnostic-session request failed: %s", esp_err_to_name(enqueueResult));
    }
}

// User-initiated VIN read. Success proves the ECU answers, so it also acts as a recovery trigger (onVinConfirmed).
esp_err_t OBD2::requestVIN()
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t result = requestVinInternal();
    if (result == ESP_OK)
    {
        onVinConfirmed();
    }
    return result;
}

esp_err_t OBD2::requestVinInternal()
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    DiagnosticRequestGuard diagnostic(this);
    if (!diagnostic.locked())
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (vinData.vinReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "VIN semaphore not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_VEHICLE_INFO;
    r.payload.raw.data[2] = PID_VIN;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    xSemaphoreTake(vinData.vinReadySemaphore, 0);
    esp_err_t enqueueResult = req(r);
    if (enqueueResult != ESP_OK)
    {
        return enqueueResult;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (!stopRequested_)
    {
        TickType_t remaining = deadline - xTaskGetTickCount();
        if ((int32_t)remaining <= 0)
        {
            return ESP_ERR_TIMEOUT;
        }
        if (xSemaphoreTake(vinData.vinReadySemaphore, std::min(remaining, pdMS_TO_TICKS(50))) == pdTRUE)
        {
            return stopRequested_ ? ESP_ERR_INVALID_STATE : ESP_OK;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t OBD2::requestDTC(uint8_t mode)
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t* sem = NULL;

    switch (mode)
    {
        case MODE_DTCS:
            sem = &dtcData.confirmedReadySemaphore;
            break;
        case MODE_PENDING_DTCS:
            sem = &dtcData.pendingReadySemaphore;
            break;
        case MODE_PERMANENT_DTCS:
            sem = &dtcData.permanentReadySemaphore;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    DiagnosticRequestGuard diagnostic(this);
    if (!diagnostic.locked())
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (*sem == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for mode %02x not initialized!", mode);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(clearDTC(mode), TAG, "Failed to clear DTCs");

    // Drain any stale tokens before requesting
    while (xSemaphoreTake(*sem, 0) == pdTRUE)
    {
        ;
    }

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x01;
    r.payload.raw.data[1] = mode;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    esp_err_t enqueueResult = req(r);
    if (enqueueResult != ESP_OK)
    {
        return enqueueResult;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (!stopRequested_)
    {
        TickType_t remaining = deadline - xTaskGetTickCount();
        if ((int32_t)remaining <= 0) { ESP_LOGW(TAG, "DTC Request %s timed out", OBD2_MODE_TO_STR(mode)); return ESP_ERR_TIMEOUT; }
        if (xSemaphoreTake(*sem, std::min(remaining, pdMS_TO_TICKS(50))) == pdTRUE)
        {
            return stopRequested_ ? ESP_ERR_INVALID_STATE : ESP_OK;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t OBD2::requestClearDTCs()
{
    OperationGuard operation(this);
    if (!operation.admitted())
    {
        return ESP_ERR_INVALID_STATE;
    }
    DiagnosticRequestGuard diagnostic(this);
    if (!diagnostic.locked())
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (dtcData.clearReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for clear DTCs not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    // Drain any stale tokens
    while (xSemaphoreTake(dtcData.clearReadySemaphore, 0) == pdTRUE)
    {
        ;
    }

    PollRequest r         = {};
    r.id                  = OBD2_FUNCTIONAL_ID;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x02;
    r.payload.raw.data[1] = MODE_CLEAR_DTCS;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    esp_err_t enqueueResult = req(r);
    if (enqueueResult != ESP_OK)
    {
        return enqueueResult;
    }
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
    while (!stopRequested_)
    {
        TickType_t remaining = deadline - xTaskGetTickCount();
        if ((int32_t)remaining <= 0) { ESP_LOGW(TAG, "Clear DTCs Request timed out"); return ESP_ERR_TIMEOUT; }
        if (xSemaphoreTake(dtcData.clearReadySemaphore, std::min(remaining, pdMS_TO_TICKS(50))) == pdTRUE)
        {
            return stopRequested_ ? ESP_ERR_INVALID_STATE : ESP_OK;
        }
    }
    return ESP_ERR_INVALID_STATE;
}
