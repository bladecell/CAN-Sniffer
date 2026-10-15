#include "obd2_internal.hpp"

esp_err_t OBD2::queryMsg(PollRequest& req)
{
    if (!canDriver.isBusConnected())
    {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }

    if (req.payload.raw.dlc > 8)
    {
        return ESP_ERR_INVALID_ARG;
    }

    CanDriver::CanFrame tx = {};
    uint8_t             dlc;

    // memset(tx.data, 0x55, sizeof(tx.data));

    if (req.isRaw)
    {
        memcpy(tx.data, req.payload.raw.data, req.payload.raw.dlc);
        dlc = req.payload.raw.dlc;
    }
    else
    {
        if (req.payload.obd.mode == MODE_READ_DATA_BY_IDENTIFIER)
        {
            tx.data[0] = req.payload.obd.len;
            tx.data[1] = req.payload.obd.mode;
            tx.data[2] = (req.payload.obd.pid >> 8) & 0xFF;
            tx.data[3] = req.payload.obd.pid & 0xFF;
            dlc        = 8;
        }
        else
        {
            tx.data[0] = req.payload.obd.len;
            tx.data[1] = req.payload.obd.mode;
            tx.data[2] = (uint8_t)req.payload.obd.pid;
            dlc        = 8;
        }
    }

    tx.header.id           = req.id;  // OBD-II  request ID
    tx.header.dlc          = twaifd_len2dlc(dlc);
    tx.header.ide          = false;  // Standard Frame Format (11-bit ID)
    tx.header.rtr          = 0;      // Data frame (not remote frame)
    tx.header.fdf          = 0;      // Classic CAN format
    tx.header.brs          = 0;      // No bit rate switching
    tx.header.esi          = 0;      // No error state indicator
    tx.header.timestamp    = 0;      // Not used for TX
    tx.header.trigger_time = 0;      // Not used for immediate transmission

    tx.length = dlc;

    esp_err_t ret = canDriver.transmit(tx, 100);

    if (ret == ESP_OK && req.isRaw && dlc >= 3 && req.id == OBD2_FUNCTIONAL_ID)
    {
        // The running DTC / VIN request is on the bus: the receive task starts the first-response wait from here.
        const uint8_t mode = diagRequest_.load() & 0xFF;
        if (mode != 0 && tx.data[1] == mode && (mode != MODE_VEHICLE_INFO || tx.data[2] == PID_VIN))
        {
            diagSentMs_.store(pdTICKS_TO_MS(xTaskGetTickCount()));
            diagSent_.store(true);
        }
    }

    return ret;
}

void OBD2::releaseDiagBus()
{
    if (busMutexOwnedByReceive_ && xBusArbitrationMutex)
    {
        xSemaphoreGive(xBusArbitrationMutex);
    }
    busMutexOwnedByReceive_ = false;
}

void OBD2::resetDiagCollection()
{
    releaseDiagBus();
    diagCollector_.cancel();
    diagPending_.store(false);
    diagReassembling_.store(false);
    diagSeenRequest_ = 0;
}

// Receive task: follow the requester's start / end of a collection.
void OBD2::syncDiagRequest()
{
    const uint32_t request = diagRequest_.load();
    if (request == diagSeenRequest_)
    {
        return;
    }
    diagSeenRequest_ = request;
    releaseDiagBus();
    const uint8_t mode = request & 0xFF;
    if (mode == 0)
    {
        diagCollector_.cancel();
        diagPending_.store(false);
    diagReassembling_.store(false);
        return;
    }
    diagCollector_.begin(mode == MODE_VEHICLE_INFO ? obd2frame::DiagKind::Vin : obd2frame::DiagKind::Dtc, mode,
                         pdTICKS_TO_MS(xTaskGetTickCount()));
}

// Receive task: a frame on a response ID while a collection runs belongs to the collector, one session per ECU.
bool OBD2::handleDiagFrame(const CanDriver::CanFrame& f)
{
    if (f.header.ide || !diagCollector_.active())
    {
        return false;
    }
    const obd2frame::DiagFrameResult r =
        diagCollector_.onFrame(f.header.id, f.data, f.length, pdTICKS_TO_MS(xTaskGetTickCount()));
    if (r.err != ESP_OK)
    {
        ESP_LOGW(TAG, "ECU 0x%03X: multi-frame response dropped: %s", static_cast<unsigned>(f.header.id),
                 esp_err_to_name(r.err));
    }
    if (r.sendFlowControl)
    {
        // Keep the poll task off the bus while any ECU is mid-message; released when the last session ends.
        if (!busMutexOwnedByReceive_ && xBusArbitrationMutex)
        {
            if (xSemaphoreTake(xBusArbitrationMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                busMutexOwnedByReceive_ = true;
            }
            else
            {
                // Flow control still goes out (the ECU would abort otherwise); at most one warning per first frame.
                ESP_LOGW(TAG, "Bus arbitration busy: sending flow control to 0x%03X without holding the bus",
                         static_cast<unsigned>(r.flowControlId));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Wait before sending Flow Control
        if (!stopRequested_)
        {
            sendFlowControlFrame(r.flowControlId);
        }
    }
    return r.consumed;
}

// Receive task, every loop: expire stale sessions and publish the result once all ECUs have answered.
void OBD2::serviceDiagCollection()
{
    if (!diagCollector_.active())
    {
        releaseDiagBus();
        return;
    }
    if (diagSent_.exchange(false))
    {
        diagCollector_.markSent(diagSentMs_.load());
    }
    const obd2frame::DiagStatus status = diagCollector_.poll(pdTICKS_TO_MS(xTaskGetTickCount()));
    diagPending_.store(diagCollector_.anyPending());
    diagReassembling_.store(diagCollector_.progressing());
    if (diagCollector_.activeSessions() == 0)
    {
        releaseDiagBus();
    }
    if (status == obd2frame::DiagStatus::Complete)
    {
        commitDiagResult();
        diagCollector_.cancel();
        diagPending_.store(false);
    diagReassembling_.store(false);
    }
    else if (status == obd2frame::DiagStatus::Failed)
    {
        ESP_LOGW(TAG, "No ECU answered the 0x%02X request", static_cast<unsigned>(diagSeenRequest_ & 0xFF));
        diagCollector_.cancel();
        diagPending_.store(false);
    diagReassembling_.store(false);
    }
}

void OBD2::commitDiagResult()
{
    const uint8_t mode = diagSeenRequest_ & 0xFF;
    if (mode == MODE_VEHICLE_INFO)
    {
        char vin[VIN_LENGTH] = {};
        const bool valid     = diagCollector_.vin(vin);
        if (diagCollector_.conflictingVins() > 0)
        {
            ESP_LOGW(TAG, "%u ECU(s) reported a different VIN; keeping the first valid one",
                     static_cast<unsigned>(diagCollector_.conflictingVins()));
        }
        if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            // Do not signal: the waiting request then times out with an error instead of returning a stale VIN as success.
            return;
        }
        if (valid)
        {
            memcpy(vinData.vin, vin, VIN_LENGTH);
        }
        vinData.isValid     = valid;
        vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xSemaphoreGive(vinData.vinReadySemaphore);
        xSemaphoreGive(vinData.mtx_);
        return;
    }

    for (size_t i = 0; i < diagCollector_.dtcCount(); i++)
    {
        _setDTC(diagCollector_.dtcs()[i], mode);
    }
    switch (mode)
    {
        case MODE_DTCS:
            xSemaphoreGive(dtcData.confirmedReadySemaphore);
            break;
        case MODE_PENDING_DTCS:
            xSemaphoreGive(dtcData.pendingReadySemaphore);
            break;
        case MODE_PERMANENT_DTCS:
            xSemaphoreGive(dtcData.permanentReadySemaphore);
            break;
        default:
            break;
    }
}

inline void OBD2::sendFlowControlFrame(uint32_t id)
{
    PollRequest r         = {};
    r.id                  = id;
    r.isRaw               = true;
    r.payload.raw.data[0] = 0x30;
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    queryMsg(r);
}

// Requester side: start the collection, enqueue the request, wait for the merged result, end the collection.
// Waits OBD2_DIAG_WAIT_MS, longer while a multi-frame message is arriving (up to OBD2_DIAG_REASSEMBLY_CAP_MS) or an ECU has a
// response pending (NRC 0x78, up to OBD2_DIAG_PENDING_CAP_MS); see obd2frame::diagWaitExpired.
esp_err_t OBD2::runDiagRequest(uint8_t mode, SemaphoreHandle_t sem, PollRequest& r)
{
    diagSent_.store(false);
    const uint32_t generation = ++diagGen_;
    diagRequest_.store((generation << 8) | mode);

    esp_err_t result = req(r);
    if (result == ESP_OK)
    {
        const uint32_t startMs = pdTICKS_TO_MS(xTaskGetTickCount());
        result                     = ESP_ERR_INVALID_STATE;
        while (!stopRequested_)
        {
            if (xSemaphoreTake(sem, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                result = stopRequested_ ? ESP_ERR_INVALID_STATE : ESP_OK;
                break;
            }
            if (!canDriver.isBusConnected())
            {
                ESP_LOGW(TAG, "Bus dropped during diagnostic request 0x%02X", static_cast<unsigned>(mode));
                result = ESP_ERR_INVALID_STATE;
                break;
            }
            const obd2frame::DiagWaitState wait = {pdTICKS_TO_MS(xTaskGetTickCount()),
                                                   startMs,
                                                   OBD2_DIAG_WAIT_MS,
                                                   OBD2_DIAG_REASSEMBLY_CAP_MS,
                                                   OBD2_DIAG_PENDING_CAP_MS,
                                                   diagReassembling_.load(),
                                                   diagPending_.load()};
            if (obd2frame::diagWaitExpired(wait))
            {
                ESP_LOGW(TAG, "Diagnostic request 0x%02X timed out", static_cast<unsigned>(mode));
                result = ESP_ERR_TIMEOUT;
                break;
            }
        }
    }
    diagRequest_.store(generation << 8);  // mode 0: the receive task drops the collection
    return result;
}
