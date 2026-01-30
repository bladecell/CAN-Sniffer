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

#include "obd2.hpp"

static const char *TAG = "OBD2";

/**
 * @brief Construct a new OBD2::OBD2 object
 *
 * @param canDriver
 */
OBD2::OBD2()
    : continuousRunning(false),
      xPidConnectedSemaphore(xSemaphoreCreateBinary()),
      xBusConnectionSemaphore(xSemaphoreCreateBinary()),
      xRequestNextPIDSemaphore(xSemaphoreCreateBinary())
{
}

OBD2::~OBD2()
{
    if (xPidConnectedSemaphore)
    {
        vSemaphoreDelete(xPidConnectedSemaphore);
        xPidConnectedSemaphore = nullptr;
    }
    if (xBusConnectionSemaphore)
    {
        vSemaphoreDelete(xBusConnectionSemaphore);
        xBusConnectionSemaphore = nullptr;
    }
    if (xRequestNextPIDSemaphore != nullptr)
    {
        vSemaphoreDelete(xRequestNextPIDSemaphore);
        xRequestNextPIDSemaphore = nullptr;
    }
}

esp_err_t OBD2::init()
{
    if (!canDriver.isInitialized())
    {
        ESP_LOGE(TAG, "CAN driver not initialized");
        return ESP_FAIL;
    }

    initDef();

    canDriver.setConnectionCallback(onCanStateChange, this);
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        receiveTaskWrapper,
        "OBD2_receive_task",
        8192,
        this,
        tskIDLE_PRIORITY + 2,
        &ReceiveTaskHandle,
        CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }

    if (canDriver.isBusConnected())
    {
        ESP_LOGI(TAG, "CAN bus already connected, getting supported PIDs");
        requestSuppPids();
    }
    else
    {
        ESP_LOGW(TAG, "CAN bus not connected, waiting for connection...");
    }

    ESP_LOGI(TAG, "OBD-II interface initialized");
    return ESP_OK;
}

void OBD2::requestSuppPids()
{
    esp_err_t ret;
    xSemaphoreTake(xRequestNextPIDSemaphore, 0);
    for (uint16_t pid_marker = 0; pid_marker <= PID_PIDS_SUPPORTED_C1_E0; pid_marker += 0x20)
    {
        ret = queryMsg(OBD2_FUNCTIONAL_ID, MODE_CURRENT_DATA, pid_marker);

        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to queue PID support query 0x%02X. Error: %d", pid_marker, ret);
        }
        if (xSemaphoreTake(xRequestNextPIDSemaphore, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            break;
        }
    }

    pidsInitialized = true;
    generatePollingGroups();
    xSemaphoreGive(xPidConnectedSemaphore);
}

esp_err_t OBD2::req(uint16_t pid)
{
    if (!isSup(pid))
    {
        ESP_LOGW(TAG, "PID 0x%02X is not supported or not recognized", pid);
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret = queryMsg(getId(pid), PID_DEF.at(pid)->mode(), pid);

    return ret;
}

/**
 * @brief Sends an OBD-II query message over the CAN bus.
 *
 * This function constructs and transmits a standard OBD-II diagnostic request
 * frame with an 11-bit identifier (Standard Frame Format). It is typically used
 * to request data identified by a specific Mode and PID from a vehicle's ECU.
 *
 * The OBD-II request message data format is structured as:
 * - **Byte 0:** Data Length Code (DLC), indicating the number of subsequent data bytes.
 * - **Byte 1:** Diagnostic Test Mode (Service ID).
 * - **Byte 2:** Parameter Identifier (PID).
 * - **Bytes 3-7:** Reserved/Zero-padded.
 *
 *
 *
 * @param id The 11-bit CAN ID for the request. For OBD-II PIDs, this is usually 0x7DF
 * for a functional broadcast request, or a specific physical request ID
 * (e.g., 0x7E0, 0x7E1, etc.).
 * @param mode The OBD-II Diagnostic Test Mode (Service ID) being requested (e.g., 0x01 for Show Current Data).
 * @param pid The Parameter Identifier (PID) within the specified mode (e.g., 0x0C for Engine RPM).
 * @param len The actual number of data bytes to be transmitted (Byte 0 of the payload). For standard PID requests,
 * this is typically 0x02 (Mode + PID).
 * @return esp_err_t
 * - **ESP_ERR_INVALID_STATE:** If the CAN bus driver is not connected.
 * - **ESP_OK:** If the message was successfully queued for transmission.
 * - Other error codes from the underlying `canDriver.transmit` function.
 */
esp_err_t OBD2::queryMsg(uint32_t id, uint8_t mode, uint16_t pid)
{
    if (!canDriver.isBusConnected())
    {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t txData[8] = {0};

    if (mode == MODE_READ_DATA_BY_IDENTIFIER)
    {
        txData[0] = 3;
        txData[1] = mode;
        txData[2] = (pid >> 8) & 0xFF;
        txData[3] = pid & 0xFF;
    }
    else
    {
        txData[0] = 2;
        txData[1] = mode;
        txData[2] = (uint8_t)pid;
    }
    twai_frame_t tx = {};

    tx.header.id = id; // OBD-II  request ID
    tx.header.dlc = twaifd_len2dlc(sizeof(txData));
    tx.header.ide = false;      // Standard Frame Format (11-bit ID)
    tx.header.rtr = 0;          // Data frame (not remote frame)
    tx.header.fdf = 0;          // Classic CAN format
    tx.header.brs = 0;          // No bit rate switching
    tx.header.esi = 0;          // No error state indicator
    tx.header.timestamp = 0;    // Not used for TX
    tx.header.trigger_time = 0; // Not used for immediate transmission

    tx.buffer = txData;
    tx.buffer_len = sizeof(txData);

    esp_err_t ret = canDriver.transmit(&tx, 100);

    return ret;
}

bool OBD2::isPidInit() const
{
    return pidsInitialized;
}

void OBD2::onCanStateChange(void *arg, bool connected)
{
    OBD2 *instance = static_cast<OBD2 *>(arg);

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
    ESP_LOGI(TAG, "CAN bus connected event received");
    xSemaphoreGive(xBusConnectionSemaphore);
    if (!pidsInitialized)
    {
        ESP_LOGI(TAG, "Retrieving supported PIDs...");
        requestSuppPids();
    }
}

void OBD2::handleCanDisconnected()
{
    ESP_LOGW(TAG, "CAN bus disconnected event received");
    pidsInitialized = false;
}

void OBD2::pollTaskWrapper(void *param)
{
    OBD2 *obd2 = static_cast<OBD2 *>(param);
    obd2->pollTask();
}

bool OBD2::isContinuousRunning() const
{
    return continuousRunning;
}

void OBD2::startContinuousMode()
{
    if (continuousRunning)
        return;

    continuousRunning = true;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        pollTaskWrapper,
        "OBD2_PollTask",
        8192,
        this,
        tskIDLE_PRIORITY + 1,
        &PollTaskHandle,
        CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create polling task");
        continuousRunning = false;
    }
}

void OBD2::stopContinuousMode()
{
    if (!continuousRunning)
        return;

    continuousRunning = false;

    if (PollTaskHandle != nullptr)
    {
        vTaskDelete(PollTaskHandle);
        PollTaskHandle = nullptr;
    }
}

void OBD2::pollStatic()
{
    pollStaticGroup.store(true);
}

void OBD2::pollTask()
{
    const TickType_t POLL_PERIOD_TICKS = pdMS_TO_TICKS(POLL_TASK_PERIOD_MS);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    std::size_t groupStatic_size = 0, groupSlow_size = 0, groupMedium_size = 0, groupFast_size = 0;
    uint32_t groupStatic_idx = 0, groupSlow_idx = 0, groupMedium_idx = 0, groupFast_idx = 0;

    uint32_t taskCnt = 0;

    if (!pidsInitialized)
    {
        ESP_LOGI(TAG, "PIDs not initialized.");

        xSemaphoreTake(xPidConnectedSemaphore, portMAX_DELAY);
    }

    groupStatic_size = vGroupStatic.size();
    groupSlow_size = vGroupSlow.size();
    groupMedium_size = vGroupMedium.size();
    groupFast_size = vGroupFast.size();

    const uint32_t intervalFast = UPDATE_FAST / POLL_TASK_PERIOD_MS;
    const uint32_t intervalMedium = UPDATE_MEDIUM / POLL_TASK_PERIOD_MS;
    const uint32_t intervalSlow = UPDATE_SLOW / POLL_TASK_PERIOD_MS;
    const uint32_t intervalStatic = UPDATE_STATIC / POLL_TASK_PERIOD_MS;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, POLL_PERIOD_TICKS);
        if (!canDriver.isBusConnected())
        {
            ESP_LOGW(TAG, "Bus disconnected. Waiting for connection...");
            xSemaphoreTake(xBusConnectionSemaphore, portMAX_DELAY);
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        if (!pidsInitialized)
        {
            ESP_LOGI(TAG, "PIDs not initialized.");

            xSemaphoreTake(xPidConnectedSemaphore, portMAX_DELAY);
            groupStatic_size = vGroupStatic.size();
            groupSlow_size = vGroupSlow.size();
            groupMedium_size = vGroupMedium.size();
            groupFast_size = vGroupFast.size();
            groupStatic_idx = groupSlow_idx = groupMedium_idx = groupFast_idx = 0;
            // taskCnt = 0;
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        // Poll Slow group
        if (groupSlow_size > 0 && ((taskCnt + 2) % intervalSlow == 0))
        {
            req(vGroupSlow[groupSlow_idx]);
            groupSlow_idx = (groupSlow_idx + 1) % groupSlow_size;
        }

        // Poll Medium group
        if (groupMedium_size > 0 && ((taskCnt + 1) % intervalMedium == 0))
        {
            req(vGroupMedium[groupMedium_idx]);
            groupMedium_idx = (groupMedium_idx + 1) % groupMedium_size;
        }

        // Poll Fast group
        if (groupFast_size > 0 && (taskCnt % intervalFast == 0))
        {
            req(vGroupFast[groupFast_idx]);
            groupFast_idx = (groupFast_idx + 1) % groupFast_size;
        }

        // Poll Static group
        if (pollStaticGroup.load() && groupStatic_size > 0 && ((taskCnt + 3) % intervalStatic == 0))
        {
            req(vGroupStatic[groupStatic_idx]);
            groupStatic_idx = (groupStatic_idx + 1);
            if (groupStatic_idx >= groupStatic_size)
            {
                pollStaticGroup.store(false);
                groupStatic_idx = 0;
            }
        }

        taskCnt++;
    }
}

void OBD2::receiveTaskWrapper(void *param)
{
    OBD2 *obd2 = static_cast<OBD2 *>(param);
    obd2->receiveTask();
}

void OBD2::receiveTask()
{
    while (1)
    {
        if (!canDriver.isBusConnected())
        {
            xSemaphoreTake(xBusConnectionSemaphore, portMAX_DELAY);
            continue;
        }

        CanDriver::CanFrame f{};
        esp_err_t ret = canDriver.receive(
            f,
            portMAX_DELAY);

        if (ret == ESP_OK)
        {
            ret = parseRecFrame(f);

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to parse received frame: %s", esp_err_to_name(ret));
            }
        }
        else
        {
            ESP_LOGE(TAG, "CAN receive driver error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

esp_err_t OBD2::parseRecFrame(const CanDriver::CanFrame &f)
{
    if (f.id < OBD2_RESPONSE_BASE_ID || f.id > (OBD2_RESPONSE_BASE_ID + 7))
        return ESP_ERR_INVALID_RESPONSE;

    if (f.length < 3)
        return ESP_ERR_INVALID_SIZE;

    esp_err_t ret = ESP_OK;
    uint8_t frame_type = (f.data[0] >> 4) & 0x0F; // High Nibble - 0 - Sigle Frame, 1 - First Frame, 2 - Consecutive Frame (3 - Flow Control)

    if (frame_type > 0)
    {
        ret = captureMultiFrame(f);
        return ret;
    }

    uint8_t mode = f.data[1];

    switch (mode)
    {
    case RESPONSE_CURRENT_DATA:
        ret = parseCurrentData(f);
        break;
    case RESPONSE_DTCS:
    {
        std::vector<CanDriver::CanFrame> frame = {f};
        ret = parseDTCs(frame, mode);
        break;
    }
    case RESPONSE_CLEAR_DTCS:
        // TODO parseClearDTCsAck(f);
        break;
    case RESPONSE_PENDING_DTCS:
    {
        std::vector<CanDriver::CanFrame> frame = {f};
        ret = parseDTCs(frame, mode);
        break;
    }
    case RESPONSE_VEHICLE_INFO:
        // TODO parseVehicleInfo(f);
        break;
    case RESPONSE_PERMANENT_DTCS:
    {
        std::vector<CanDriver::CanFrame> frame = {f};
        ret = parseDTCs(frame, mode);
        break;
    }
    case RESPONSE_READ_DATA_BY_IDENTIFIER:
        parseRDBI(f);
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::parseCurrentData(const CanDriver::CanFrame &f)
{
    uint16_t pid = f.data[2];
    esp_err_t ret;

    switch (pid)
    {
    case PID_PIDS_SUPPORTED_1_20:
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

    setValid(pid, ret == ESP_OK);

    for (const auto &callback : subscribers_)
    {
        callback(pid);
    }

    return ret;
}

esp_err_t OBD2::parseRDBI(const CanDriver::CanFrame &f)
{
    uint16_t pid = f.data[2];
    esp_err_t ret = updateData(f);
    setValid(pid, ret == ESP_OK);

    for (const auto &callback : subscribers_)
    {
        callback(pid);
    }

    return ret;
}

esp_err_t OBD2::parseDTCs(std::vector<CanDriver::CanFrame> &frames, uint8_t mode)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t dtc_rem = 0, hi_half = 0, idx = 0;
    bool halved = false;
    esp_err_t ret = ESP_OK;
    uint16_t rawDTC;

    for (const auto &frame : frames)
    {
        uint8_t frame_type = (frame.data[0] >> 4) & 0x0F;
        switch (frame_type)
        {
        case 0:
            dtc_rem = frame.data[2];
            idx = 3;
            break;
        case 1:
            dtc_rem = frame.data[3];
            idx = 4;
            break;
        case 2:
            idx = 1;
            break;
        default:
            return ESP_ERR_INVALID_RESPONSE;
        }

        while (dtc_rem > 0 && idx < frame.length)
        {
            if (halved)
            {
                rawDTC = (hi_half << 8) | frame.data[idx++];
                ERR_ACCUMULATE(ret, setDTC(rawDTC, mode));
                dtc_rem--;
                halved = false;
            }
            if (frame.length - idx == 1)
            {
                hi_half = frame.data[idx++];
                halved = true;
                break;
            }

            if (frame.length - idx >= 2)
            {
                uint8_t dtc_high = frame.data[idx++];
                uint16_t rawDTC = (dtc_high << 8) | frame.data[idx++];
                ERR_ACCUMULATE(ret, setDTC(rawDTC, mode));
                dtc_rem--;
            }
        }

        if (dtc_rem == 0)
        {
            break;
        }
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

    return ret;
}

esp_err_t OBD2::parseSupportedPIDs(const CanDriver::CanFrame &f)
{
    uint16_t pidGroup = f.data[2];
    esp_err_t ret = ESP_OK;
    if (pidGroup % 0x20 != 0)
    {
        ESP_LOGE(TAG, "Invalid PID group in supported PIDs response: 0x%02X", pidGroup);
        return ESP_ERR_INVALID_ARG;
    }

    if (f.length >= 4)
    {
        uint32_t supportedPIDs = (f.data[3] << 24) |
                                 (f.data[4] << 16) |
                                 (f.data[5] << 8) |
                                 (f.data[6]);

        for (uint8_t i = 0; i < 32; ++i)
        {
            if (supportedPIDs & (1UL << (31 - i)))
            {
                uint8_t supportedPID = pidGroup + 1 + i;

                if (!pidExists(supportedPID))
                {
                    ESP_LOGD(TAG, "PID 0x%02X supported by vehicle", supportedPID);
                    continue;
                }

                ret = setIsSupported(supportedPID, true);
                if (ret != ESP_OK)
                {
                    ESP_LOGD(TAG, "Failed to set PID 0x%02X as supported: %s",
                             supportedPID, esp_err_to_name(ret));
                }
                esp_err_t ret2 = setId(supportedPID, f.id - RESPONSE_ID_OFFSET);
                if (ret2 != ESP_OK)
                {
                    ESP_LOGD(TAG, "Failed to set PID 0x%02X request ID: %s",
                             supportedPID, esp_err_to_name(ret));
                    ret = ret2;
                }
            }
        }
        if (supportedPIDs << 31)
        {
            xSemaphoreGive(xRequestNextPIDSemaphore);
        }
    }
    return ret;
}

esp_err_t OBD2::requestVIN()
{
    if (vinData.vinReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "VIN semaphore not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(queryMsg(OBD2_FUNCTIONAL_ID, MODE_VEHICLE_INFO, PID_VIN), TAG, "Failed to query request VIN message");

    xSemaphoreTake(vinData.vinReadySemaphore, 0);

    if (xSemaphoreTake(vinData.vinReadySemaphore, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestDTC(uint8_t mode)
{
    SemaphoreHandle_t sem = NULL; // Local handle to work with

    // 1. Assign the handle based on mode
    switch (mode)
    {
    case MODE_DTCS:
        sem = dtcData.confirmedReadySemaphore;
        break;
    case MODE_PENDING_DTCS:
        sem = dtcData.pendingReadySemaphore;
        break;
    case MODE_PERMANENT_DTCS:
        sem = dtcData.permanentReadySemaphore;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    if (sem == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for mode %02x not initialized!", mode);
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(sem, 0);

    ESP_RETURN_ON_ERROR(queryMsg(OBD2_FUNCTIONAL_ID, mode, 0x00), TAG, "Failed to query DTCs");

    if (xSemaphoreTake(sem, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "DTC Request %s timed out", OBD2_MODE_TO_STR(mode));
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestConfirmedDTCs()
{
    esp_err_t ret = queryMsg(OBD2_FUNCTIONAL_ID, MODE_DTCS, 0x00);
    if (ret == ESP_OK)
    {
        clearDTC(MODE_DTCS);
    }
    return ret;
}

esp_err_t OBD2::requestPendingDTCs()
{
    esp_err_t ret = queryMsg(OBD2_FUNCTIONAL_ID, MODE_PENDING_DTCS, 0x00);
    if (ret == ESP_OK)
    {
        clearDTC(MODE_PENDING_DTCS);
    }
    return ret;
}

//
esp_err_t OBD2::requestPermanentDTCs()
{
    esp_err_t ret = queryMsg(OBD2_FUNCTIONAL_ID, MODE_PERMANENT_DTCS, 0x00);
    if (ret == ESP_OK)
    {
        clearDTC(MODE_PERMANENT_DTCS);
    }
    return ret;
}

esp_err_t OBD2::requestClearDTCs()
{
    esp_err_t ret = queryMsg(OBD2_FUNCTIONAL_ID, MODE_CLEAR_DTCS, 0x00);
    return ret;
}

esp_err_t OBD2::captureMultiFrame(const CanDriver::CanFrame &f)
{
    static uint8_t MULTIDRAME_STATE = 99;
    uint8_t frameType;
    static uint16_t totalLength = 0, consecutiveFrameIndex = 0, consecutiveFramesNeeded = 0;
    constexpr uint8_t CONSECUTIVE_FRAME_DATA_BYTES = 7, FIRST_FRAME_DATA_BYTES = 6;
    static std::vector<CanDriver::CanFrame> multiFrameBuffer;
    esp_err_t ret = ESP_OK;

    switch (MULTIDRAME_STATE)
    {
    case 99: // Idle State
    {
        frameType = (f.data[0] >> 4) & 0x0F;
        if (frameType == 1)
        {
            MULTIDRAME_STATE = 0; // Go to First Frame state
        }
        else
        {
            ESP_LOGE(TAG, "Unexpected multi-frame type: %d", frameType);
            return ESP_ERR_INVALID_RESPONSE;
        }
        captureMultiFrame(f);
        break;
    }
    case 0: // First Frame
    {
        frameType = (f.data[0] >> 4) & 0x0F;   // High nibble = 1
        uint8_t lengthHigh = f.data[0] & 0x0F; // Low nibble = 0
        uint8_t lengthLow = f.data[1];
        totalLength = (lengthHigh << 8) | lengthLow;
        consecutiveFramesNeeded = (totalLength - FIRST_FRAME_DATA_BYTES + CONSECUTIVE_FRAME_DATA_BYTES - 1) / CONSECUTIVE_FRAME_DATA_BYTES;
        consecutiveFrameIndex = 0;
        multiFrameBuffer.clear();
        multiFrameBuffer.push_back(f);
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait before sending Flow Control
        ret = sendFlowControlFrame(f.id - 8);
        MULTIDRAME_STATE = 1; // Go to Consecutive Frame state
        break;
    }
    case 1: // Consecutive Frames
    {
        consecutiveFrameIndex++;
        frameType = (f.data[0] >> 4) & 0x0F;
        uint8_t sequence = f.data[0] & 0x0F;
        if (frameType != 2 || sequence != consecutiveFrameIndex)
        {
            ESP_LOGE(TAG, "Unexpected consecutive frame. Expected seq: %d, got: %d", consecutiveFrameIndex & 0x0F, sequence);
            MULTIDRAME_STATE = 99;
            return ESP_ERR_INVALID_RESPONSE;
        }
        multiFrameBuffer.push_back(f);
        if (consecutiveFrameIndex >= consecutiveFramesNeeded)
        {
            MULTIDRAME_STATE = 99;
            ret = parseMultiFrame(multiFrameBuffer);
        }
        break;
    }
    default:
        MULTIDRAME_STATE = 99;
        ret = ESP_ERR_INVALID_STATE;
        break;
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to parse multi-frame: %s", esp_err_to_name(ret));
        MULTIDRAME_STATE = 99;
    }
    return ret;
}

inline esp_err_t OBD2::sendFlowControlFrame(uint32_t id)
{
    return queryMsg(id, 0x00, 0x00, 0x30);
}

esp_err_t OBD2::parseMultiFrame(std::vector<CanDriver::CanFrame> &frames)
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

esp_err_t OBD2::parseVehicleInfoMultiFrame(std::vector<CanDriver::CanFrame> &frames)
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

esp_err_t OBD2::parseVINMultiFrame(std::vector<CanDriver::CanFrame> &frames)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    uint8_t vinBuffer[VIN_LENGTH] = {0};
    size_t vinIndex = 0;

    for (const auto &frame : frames)
    {
        uint8_t startIdx = (frame.data[0] >> 4) == 1 ? 5 : 1; // First frame starts data at index 5
        for (uint8_t i = startIdx; i < frame.length && vinIndex < VIN_LENGTH; i++)
        {
            vinBuffer[vinIndex++] = frame.data[i];
        }
    }

    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        xSemaphoreGive(vinData.vinReadySemaphore);
        return ESP_ERR_TIMEOUT;
    }

    if (vinIndex == VIN_LENGTH)
    {

        memcpy(vinData.vin, vinBuffer, VIN_LENGTH);
        vinData.isValid = true;
    }
    else
    {
        vinData.isValid = false;
        ret = ESP_ERR_INVALID_SIZE;
    }

    vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(vinData.vinReadySemaphore);
    xSemaphoreGive(vinData.mtx_);

    return ret;
}

// TODO: PID support responds with multiple IDS - different ECUs - add to PIDdata ECU which supports it
//      then add querying specific ECU for specific PIDs
//      VIN request should be sent to specific ECU as well - the flow control
//      The rate for sendingmessages should me lower - 100ms per message works now