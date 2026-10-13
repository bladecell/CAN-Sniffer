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

    return ret;
}

void OBD2::multiframe_watchdog()
{
    if (multiframe_state != 99)
    {
        if ((xTaskGetTickCount() - last_multiframe_received) > pdMS_TO_TICKS(250))
        {
            ESP_LOGE(TAG, "Multiframe session timeout");
            resetMultiFrame();
        }
    }
}

void OBD2::resetMultiFrame()
{
    if (busMutexOwnedByReceive_ && xBusArbitrationMutex) {
        xSemaphoreGive(xBusArbitrationMutex);
        busMutexOwnedByReceive_ = false;
    }
    multiframe_state = 99;
    last_multiframe_received = 0;
    multiframe_totalLength_ = multiframe_consecutiveFrameIndex_ = multiframe_consecutiveFramesNeeded_ = 0;
    multiFrameBuffer_.clear();
}

esp_err_t OBD2::captureMultiFrame(const CanDriver::CanFrame& f)
{
    uint8_t                                 frameType;
    esp_err_t                               ret = ESP_OK;

    if (f.length == 0 || f.length > obd2frame::CAN_DATA_MAX)
    {
        resetMultiFrame();
        return ESP_ERR_INVALID_SIZE;
    }

    last_multiframe_received = xTaskGetTickCount();

    switch (multiframe_state)
    {
        case 99:  // Idle State
        {
            frameType = (f.data[0] >> 4) & 0x0F;
            if (frameType == 1)
            {
                multiframe_state = 0;  // Go to First Frame state
            }
            else
            {
                ESP_LOGE(TAG, "Unexpected multi-frame type: %d", frameType);
                resetMultiFrame();
                return ESP_ERR_INVALID_RESPONSE;
            }
            ret = captureMultiFrame(f);
            break;
        }
        case 0:  // First Frame
        {
            const esp_err_t ffErr = obd2frame::parseFirstFrame(f.data, f.length, multiframe_totalLength_,
                                                               multiframe_consecutiveFramesNeeded_);
            if (ffErr != ESP_OK)
            {
                resetMultiFrame();
                return ffErr;
            }
            multiframe_consecutiveFrameIndex_   = 0;
            multiFrameBuffer_.clear();
            multiFrameBuffer_.push_back(f);
            if (stopRequested_ || !xBusArbitrationMutex || xSemaphoreTake(xBusArbitrationMutex, pdMS_TO_TICKS(100)) != pdTRUE) { resetMultiFrame(); return ESP_ERR_TIMEOUT; }
            busMutexOwnedByReceive_ = true;
            vTaskDelay(pdMS_TO_TICKS(10));  // Wait before sending Flow Control
            if (stopRequested_) { resetMultiFrame(); return ESP_ERR_INVALID_STATE; }
            sendFlowControlFrame(f.header.id - RESPONSE_ID_OFFSET);
            multiframe_state = 1;  // Go to Consecutive Frame state
            break;
        }
        case 1:  // Consecutive Frames
        {
            multiframe_consecutiveFrameIndex_++;
            if (!obd2frame::consecutiveSequenceOk(f.data, f.length, multiframe_consecutiveFrameIndex_))
            {
                ESP_LOGW(TAG, "Unexpected consecutive frame. Expected seq: %d, got: 0x%02X",
                         multiframe_consecutiveFrameIndex_ & 0x0F, f.data[0]);
                resetMultiFrame();
                return ESP_ERR_INVALID_RESPONSE;
            }
            multiFrameBuffer_.push_back(f);
            if (multiframe_consecutiveFrameIndex_ >= multiframe_consecutiveFramesNeeded_)
            {
                if (busMutexOwnedByReceive_) { xSemaphoreGive(xBusArbitrationMutex); busMutexOwnedByReceive_ = false; }
                ret              = parseMultiFrame(multiFrameBuffer_);
                resetMultiFrame();
            }
            break;
        }
        default:
            resetMultiFrame();
            ret = ESP_ERR_INVALID_STATE;
            break;
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to parse multi-frame: %s", esp_err_to_name(ret));
        resetMultiFrame();
    }
    return ret;
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

esp_err_t OBD2::parseMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint8_t mode = frames[0].data[2];

    switch (mode)
    {
        case RESPONSE_CURRENT_DATA:
            // NOT IMPLEMENTED
            break;
        case RESPONSE_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        case RESPONSE_CLEAR_DTCS:
            // NOT IMPLEMENTED
            break;
        case RESPONSE_PENDING_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        case RESPONSE_VEHICLE_INFO:
            ret = parseVehicleInfoMultiFrame(frames);
            break;
        case RESPONSE_PERMANENT_DTCS:
            ret = parseDTCs(frames, mode);
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint16_t pid = frames[0].data[3];

    switch (pid)
    {
        case PID_VIN:
            ret = parseVINMultiFrame(frames);
            break;
        default:
            ret = ESP_ERR_NOT_SUPPORTED;
            break;
    }
    return ret;
}

esp_err_t OBD2::parseVINMultiFrame(std::vector<CanDriver::CanFrame>& frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::vector<obd2frame::RawFrame> raw;
    ESP_RETURN_ON_ERROR(toRawFrames(frames, raw), TAG, "bad frame length");

    char      vinBuffer[VIN_LENGTH] = {0};
    esp_err_t ret                   = obd2frame::decodeVinResponse(raw.data(), raw.size(), vinBuffer);

    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        // Do not signal: the waiting request then times out with an error instead of returning a stale VIN as success.
        return ESP_ERR_TIMEOUT;
    }

    if (ret == ESP_OK)
    {
        memcpy(vinData.vin, vinBuffer, VIN_LENGTH);
        vinData.isValid = true;
    }
    else
    {
        vinData.isValid = false;
    }

    vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(vinData.vinReadySemaphore);
    xSemaphoreGive(vinData.mtx_);

    return ret;
}
