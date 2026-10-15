#include "obd2_internal.hpp"

esp_err_t OBD2::queryMsg(PollRequest& req, bool urgent)
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
        if (!obd2frame::buildPidRequest(req.payload.obd.mode, req.payload.obd.pid, tx.data))
        {
            ESP_LOGW(TAG, "Refusing to send PID 0x%04X: invalid mode 0x%02X", static_cast<unsigned>(req.payload.obd.pid),
                     static_cast<unsigned>(req.payload.obd.mode));
            return ESP_ERR_INVALID_ARG;
        }
        dlc = 8;
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

    // The response timer does not start here: the frame is only queued. onTxDone() starts it when the frame left the controller.
    esp_err_t ret = urgent ? canDriver.transmitUrgent(tx) : canDriver.transmit(tx, 100);

    return ret;
}

// TWAI TX-done (ISR) or a refused send (can_tx task). Atomics only: no locks, no logging, no allocation.
void IRAM_ATTR OBD2::onTxDone(void* arg, uint32_t id, const uint8_t* data, uint8_t length, bool success)
{
    OBD2* self = static_cast<OBD2*>(arg);
    if (self == nullptr)
    {
        return;
    }
    const uint32_t nowMs = pdTICKS_TO_MS(xTaskGetTickCountFromISR());
    if (obd2frame::isDiagRequestFrame(self->diagRequest_.load() & 0xFF, id, data, length))
    {
        if (success)
        {
            self->diagSentMs_.store(nowMs);
            self->diagSent_.store(true);
        }
        else
        {
            self->diagTxFailed_.store(true);
        }
    }
    else if (success && obd2frame::isFlowControlFrame(id, data, length))
    {
        const uint32_t sinceMs = self->fcFirstFrameMs_[id - (OBD2_RESPONSE_BASE_ID - RESPONSE_ID_OFFSET)].load();
        const uint32_t latency = static_cast<uint32_t>(nowMs - sinceMs);
        self->fcLastLatencyMs_.store(latency);
        if (latency > self->fcMaxLatencyMs_.load())
        {
            self->fcMaxLatencyMs_.store(latency);
        }
        if (obd2frame::isoTpFlowControlLate(sinceMs, nowMs))
        {
            self->fcLateCount_.fetch_add(1);
        }
    }
}

// Keep the poll task off the bus while an ECU is mid-message. Never blocks: if the poll task holds the bus right now the
// attempt is repeated every receive-loop pass while a session is open.
void OBD2::tryHoldDiagBus()
{
    if (!busMutexOwnedByReceive_ && xBusArbitrationMutex && xSemaphoreTake(xBusArbitrationMutex, 0) == pdTRUE)
    {
        busMutexOwnedByReceive_ = true;
    }
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
    const uint32_t rxMs = pdTICKS_TO_MS(xTaskGetTickCount());
    const obd2frame::DiagFrameResult r = diagCollector_.onFrame(f.header.id, f.data, f.length, rxMs);
    if (r.err != ESP_OK)
    {
        ESP_LOGW(TAG, "ECU 0x%03X: multi-frame response dropped: %s", static_cast<unsigned>(f.header.id),
                 esp_err_to_name(r.err));
    }
    if (r.sendFlowControl)
    {
        // Flow control first: stamp the first frame, then queue the frame at the front of the TX queue without waiting
        // (N_Ar). Only afterwards try to keep the poll task off the bus; failing to get it is harmless.
        fcFirstFrameMs_[f.header.id - obd2frame::DIAG_RESPONSE_ID_FIRST].store(rxMs);
        if (!stopRequested_ && sendFlowControlFrame(r.flowControlId) != ESP_OK)
        {
            // Not queued (TX queue full, bus down): the ECU will abort the message. Count it with the late ones (the
            // receive loop logs the count) and fail the waiting request instead of letting it run into its timeout.
            fcLateCount_.fetch_add(1);
            diagTxFailed_.store(true);
        }
        tryHoldDiagBus();
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
    else
    {
        tryHoldDiagBus();
    }
    const uint32_t late = fcLateCount_.load();
    if (late != fcLateLogged_)
    {
        fcLateLogged_ = late;
        ESP_LOGW(TAG, "Flow control missed N_Ar (%u ms): last %u ms, max %u ms, %u late so far",
                 static_cast<unsigned>(ISOTP_N_AR_MS), static_cast<unsigned>(fcLastLatencyMs_.load()),
                 static_cast<unsigned>(fcMaxLatencyMs_.load()), static_cast<unsigned>(late));
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
        if (diagCollector_.result() != ESP_OK)
        {
            // ECUs answered but none sent a usable VIN: report that, keep the previous VIN, isValid and lastUpdated.
            ESP_LOGW(TAG, "VIN response without a valid 17-character VIN");
            diagResult_.store(diagCollector_.result());
            xSemaphoreGive(vinData.vinReadySemaphore);
            return;
        }
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
        memcpy(vinData.vin, vin, VIN_LENGTH);
        vinData.isValid     = valid;
        vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
        diagResult_.store(ESP_OK);
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

inline esp_err_t OBD2::sendFlowControlFrame(uint32_t id)
{
    PollRequest r         = {};
    r.id                  = id;
    r.isRaw               = true;
    obd2frame::buildFlowControl(r.payload.raw.data);
    r.payload.raw.dlc     = 8;
    r.interval            = 0;
    r.nextWake            = xTaskGetTickCount();
    r.priority            = 0;
    r.isRecurring         = false;
    r.retries_left        = DEFAULT_NUMER_OF_RETRIES;

    return queryMsg(r, true);
}

// Requester side: start the collection, enqueue the request, wait for the merged result, end the collection.
// Waits OBD2_DIAG_WAIT_MS, longer while a multi-frame message is arriving (up to OBD2_DIAG_REASSEMBLY_CAP_MS) or an ECU has a
// response pending (NRC 0x78, up to OBD2_DIAG_PENDING_CAP_MS); see obd2frame::diagWaitExpired.
esp_err_t OBD2::runDiagRequest(uint8_t mode, SemaphoreHandle_t sem, PollRequest& r)
{
    diagSent_.store(false);
    diagTxFailed_.store(false);
    diagResult_.store(ESP_OK);
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
                result = stopRequested_ ? ESP_ERR_INVALID_STATE : diagResult_.load();
                break;
            }
            if (diagTxFailed_.load())
            {
                ESP_LOGW(TAG, "Diagnostic request 0x%02X could not be sent", static_cast<unsigned>(mode));
                result = ESP_FAIL;
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
