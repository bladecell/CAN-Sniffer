#include "obd2_internal.hpp"

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
