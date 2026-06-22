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

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "can_driver.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "obd2_data_model.hpp"
#include "obd2_utils.hpp"

static const char* TAG = "OBD2";

/**
 * @brief Construct a new OBD2::OBD2 object
 *
 */
OBD2::OBD2()
    : continuousRunning(false),
      xPidConnectedSemaphore(xSemaphoreCreateBinary()),
      xBusConnectionSemaphore(xSemaphoreCreateBinary()),
      xRequestNextPIDSemaphore(xSemaphoreCreateBinary())
{
}

/**
 * @brief Destroy the OBD2::OBD2 object
 *
 */
OBD2::~OBD2()
{
    if (ReceiveTaskHandle)
    {
        vTaskDelete(ReceiveTaskHandle);
        ReceiveTaskHandle = nullptr;
    }
    if (PollTaskHandle)
    {
        vTaskDelete(PollTaskHandle);
        PollTaskHandle = nullptr;
    }
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

/**
 * @brief Initialize the OBD2 interface
 *
 * @return esp_err_t ESP_OK if initialization was successful, otherwise an error code
 */
esp_err_t OBD2::init()
{
    if (!canDriver.isInitialized())
    {
        ESP_LOGE(TAG, "CAN driver not initialized");
        return ESP_FAIL;
    }

    derivedPidQueue_ = xQueueCreate(10, sizeof(uint16_t));

    initDef();

    canDriver.setConnectionChangeCallback(onCanStateChange, this);
    BaseType_t taskCreated = xTaskCreatePinnedToCore(receiveTaskWrapper, "OBD2_receive_task", 8192, this,
                                                     tskIDLE_PRIORITY + 2, &ReceiveTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }

    taskCreated = xTaskCreatePinnedToCore(pollTaskWrapper, "OBD2_PollTask", 8192, this, tskIDLE_PRIORITY + 1,
                                          &PollTaskHandle, CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create polling task");
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
    xSemaphoreTake(xRequestNextPIDSemaphore, 0);
    supportedPIDsGroup = {};
    for (uint16_t pid_marker = 0; pid_marker <= PID_PIDS_SUPPORTED_C0_DF; pid_marker += 0x20)
    {
        req(OBD2_FUNCTIONAL_ID, MODE_CURRENT_DATA, pid_marker, 2, 0, 0);

        if (xSemaphoreTake(xRequestNextPIDSemaphore, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            break;
        }
    }

    pidsInitialized = true;
    xSemaphoreGive(xPidConnectedSemaphore);
}

void OBD2::getSupportedPids(supportedPIDsGroup_t& supportedPIDsGroup)
{
    supportedPIDsGroup = this->supportedPIDsGroup;
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
            return ESP_ERR_INVALID_ARG;

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

                if (continuousRunning)
                {
                    req(id, mode, pid, len, interval, priority, true);
                }
            }
        }
    }
    else if (mode == MODE_READ_DATA_BY_IDENTIFIER)
    {
        if (continuousRunning)
        {
            req(id, mode, pid, len, interval, priority, true);
        }
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestPID(uint16_t pid)
{
    withPidMapLock(
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

            req(id, mode, pid, len, 0, priority);
            return ESP_OK;
        });

    return ESP_OK;
}

void OBD2::req(uint32_t id, uint8_t mode, uint32_t pid, uint8_t len, uint32_t interval, uint8_t priority,
               bool isRecurring)
{
    PollRequest req;
    req.pid         = pid;
    req.interval    = interval;
    req.nextWake    = xTaskGetTickCount();
    req.priority    = priority;
    req.isRecurring = isRecurring;
    req.id          = id;
    req.mode        = mode;
    req.len         = len;

    pollQueue.push(req);

    if (PollTaskHandle != nullptr)
    {
        xTaskNotifyGive(PollTaskHandle);
    }
}

esp_err_t OBD2::queryMsg(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len)
{
    if (!canDriver.isBusConnected())
    {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }
    CanDriver::CanFrame tx = {};

    if (mode == MODE_READ_DATA_BY_IDENTIFIER)
    {
        tx.data[0] = len;
        tx.data[1] = mode;
        tx.data[2] = (pid >> 8) & 0xFF;
        tx.data[3] = pid & 0xFF;
    }
    else
    {
        tx.data[0] = len;
        tx.data[1] = mode;
        tx.data[2] = (uint8_t)pid;
    }

    tx.header.id           = id;  // OBD-II  request ID
    tx.header.dlc          = twaifd_len2dlc(sizeof(tx.data) / sizeof(uint8_t));
    tx.header.ide          = false;  // Standard Frame Format (11-bit ID)
    tx.header.rtr          = 0;      // Data frame (not remote frame)
    tx.header.fdf          = 0;      // Classic CAN format
    tx.header.brs          = 0;      // No bit rate switching
    tx.header.esi          = 0;      // No error state indicator
    tx.header.timestamp    = 0;      // Not used for TX
    tx.header.trigger_time = 0;      // Not used for immediate transmission

    tx.length = sizeof(tx.data) / sizeof(uint8_t);  // TODO - needs to be set correctly

    esp_err_t ret = canDriver.transmit(tx, 100);

    return ret;
}

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

bool OBD2::isContinuousRunning() const
{
    return continuousRunning;
}

void OBD2::startContinuousMode()
{
    if (continuousRunning)
        return;

    continuousRunning = true;
    startPolling();
}

void OBD2::stopContinuousMode()
{
    if (!continuousRunning)
        return;

    continuousRunning = false;
    pollQueue.clearRecurring();
}

void OBD2::pollRequestStaticPids()
{
    withPidMapLock(
        [&]()
        {
            for (const auto& [pid, def] : PID_DEF)
            {
                if (def.updateInterval() == UPDATE_STATIC)
                {
                    req(def.id(), def.mode(), pid, def.len(), def.updateInterval(), def.priority(), false);
                }
            }
            return ESP_OK;
        });
}

void OBD2::pollTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
    obd2->pollTask();
}

void OBD2::pollTask()
{
    const float ALPHA = 0.1f;

    while (1)
    {
        TickType_t delay = pollQueue.getWait();
        ulTaskNotifyTake(pdTRUE, delay);

        // If disconnected or empty, naturally decay the utilization down to 0
        // instead of snapping it instantly to 0.0f.
        if (pollQueue.isEmpty() || !canDriver.isBusConnected())
        {
            pollTaskUtilization = pollTaskUtilization * (1.0f - ALPHA);  // Smooth decay
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float   fill      = pollQueue.getFillFactor();
        int32_t latencyMs = pdTICKS_TO_MS(pollQueue.getTopLatency());

        // Normalize latency. Consider changing 100.0f to your highest acceptable lag.
        float latencyFactor = (float)latencyMs / 100.0f;
        if (latencyFactor > 1.0f)
            latencyFactor = 1.0f;
        if (latencyFactor < 0.0f)
            latencyFactor = 0.0f;  // Guard against negative ticks clock rolls

        // Calculate the raw snapshot for this specific loop
        float instantUtilization = (latencyFactor * 0.7f) + (fill * 0.3f);

        // Apply Exponential Moving Average filter
        pollTaskUtilization = (instantUtilization * ALPHA) + (pollTaskUtilization * (1.0f - ALPHA));

        // 2. Take the most urgent appointment
        PollRequest current = pollQueue.pop();

        // 3. Process based on the Mode stored in the request
        if (current.mode == MODE_DERIVED_DATA)
        {
            xQueueSend(derivedPidQueue_, &current.pid, pdMS_TO_TICKS(10));
        }
        else
        {
            esp_err_t err = queryMsg(current.id, current.mode, current.pid, current.len);

            if (err != ESP_OK && (err == ESP_ERR_TIMEOUT || err == ESP_FAIL))
            {
                current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(5);
                pollQueue.push(current);
                vTaskDelay(1);  // Small yield before retry
                continue;
            }
        }

        // 4. Reschedule recurring tasks
        if (current.isRecurring && continuousRunning)
        {
            current.nextWake = xTaskGetTickCount() + pdMS_TO_TICKS(current.interval);
            pollQueue.push(current);
        }

        vTaskDelay(1);
    }
}

void OBD2::receiveTaskWrapper(void* param)
{
    OBD2* obd2 = static_cast<OBD2*>(param);
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

        uint16_t derivedPid;
        while (xQueueReceive(derivedPidQueue_, &derivedPid, 0) == pdTRUE)
        {
            CanDriver::CanFrame f{};
            f.header.id = OBD2_FUNCTIONAL_ID;
            f.length    = 3;
            f.data[0]   = 0x03;  // DLC
            f.data[1]   = RESPONSE_MODE_DERIVED_DATA;
            f.data[2]   = (derivedPid >> 8) & 0xFF;
            f.data[3]   = derivedPid & 0xFF;
            parseRecFrame(f);
        }

        CanDriver::CanFrame f{};
        esp_err_t           ret = canDriver.receive(f, pdMS_TO_TICKS(10));

        if (ret == ESP_OK)
        {
            ret = parseRecFrame(f);

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to parse received frame: %s", esp_err_to_name(ret));
            }
        }
        else if (ret != ESP_ERR_TIMEOUT)
        {
            ESP_LOGE(TAG, "CAN receive driver error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

esp_err_t OBD2::parseRecFrame(const CanDriver::CanFrame& f)
{
    if (f.length < 3)
        return ESP_ERR_INVALID_SIZE;

    esp_err_t ret = ESP_OK;
    uint8_t   frame_type =
        (f.data[0] >> 4) &
        0x0F;  // High Nibble - 0 - Sigle Frame, 1 - First Frame, 2 - Consecutive Frame (3 - Flow Control)

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
        case RESPONSE_READ_DATA_BY_IDENTIFIER:
            ret = parseRDBI(f);
            break;
        case RESPONSE_MODE_DERIVED_DATA:
            ret = parseDerivedData(f);
            break;
        case RESPONSE_NEGATIVE_RESPONSE_CODE:
            // TODO
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::parseCurrentData(const CanDriver::CanFrame& f)
{
    uint16_t  pid = f.data[2];
    esp_err_t ret;

    switch (pid)
    {
        case PID_PIDS_SUPPORTED_0_19:
        case PID_PIDS_SUPPORTED_20_39:
        case PID_PIDS_SUPPORTED_40_59:
        case PID_PIDS_SUPPORTED_60_79:
        case PID_PIDS_SUPPORTED_80_99:
        case PID_PIDS_SUPPORTED_A0_BF:
        case PID_PIDS_SUPPORTED_C0_DF:
            ret = parseSupportedPIDs(f);
            break;
        default:
            ret = updateData(f);
    }

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid);

    return ret;
}

esp_err_t OBD2::parseRDBI(const CanDriver::CanFrame& f)
{
    uint16_t  pid = (uint16_t)(f.data[2] << 8) | f.data[3];
    esp_err_t ret;

    if (f.data[1] == 0x7F)
    {
        uint8_t rejectedService = f.data[2];
        uint8_t nrcCode         = f.data[3];

        ESP_LOGE(TAG, "NRC Received! Service: 0x%02X, Error: 0x%02X", rejectedService, nrcCode);

        ret = ESP_FAIL;
    }
    else
    {
        ret = updateData(f);
    }

    _setDataFieldWithLock(pid, &PIDData_t::isValid, ret == ESP_OK);

    runPidUpdateCallbacks(pid);

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
            pdat->isValid     = true;

            return err;
        });

    if (ret == ESP_OK)
    {
        runPidUpdateCallbacks(pid);
    }

    return ret;
}

esp_err_t OBD2::parseDTCs(std::vector<CanDriver::CanFrame>& frames, uint8_t mode)
{
    if (frames.empty())
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t   dtc_rem = 0, hi_half = 0, idx = 0;
    bool      halved = false;
    esp_err_t ret    = ESP_OK;
    uint16_t  rawDTC;

    for (const auto& frame : frames)
    {
        uint8_t frame_type = (frame.data[0] >> 4) & 0x0F;
        switch (frame_type)
        {
            case 0:
                dtc_rem = frame.data[2];
                idx     = 3;
                break;
            case 1:
                dtc_rem = frame.data[3];
                idx     = 4;
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
                ERR_ACCUMULATE(ret, _setDTC(rawDTC, mode));
                dtc_rem--;
                halved = false;
            }
            if (frame.length - idx == 1)
            {
                hi_half = frame.data[idx++];
                halved  = true;
                break;
            }

            if (frame.length - idx >= 2)
            {
                uint8_t  dtc_high = frame.data[idx++];
                uint16_t rawDTC   = (dtc_high << 8) | frame.data[idx++];
                ERR_ACCUMULATE(ret, _setDTC(rawDTC, mode));
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

    if (f.length < 4)
        return ESP_OK;

    uint32_t supportedPIDs = (f.data[3] << 24) | (f.data[4] << 16) | (f.data[5] << 8) | (f.data[6]);
    uint8_t  groupIdx      = (pidGroup >> 5);

    supportedPIDsGroup.pidGroup[groupIdx] = supportedPIDs;

    esp_err_t ret = withPidMapLock(
        [&]()
        {
            for (uint8_t i = 0; i < 32; ++i)
            {
                if (supportedPIDs & (1UL << (31 - i)))
                {
                    uint16_t supportedPID = pidGroup + 1 + i;
                    supportedPIDsGroup.numberOfSupportedPIDs++;

                    if (!_pidExists(supportedPID))
                        continue;

                    _setDataField(supportedPID, &PIDData_t::isSupported, true);
                    _setDataField(supportedPID, &PIDData_t::id, f.header.id - RESPONSE_ID_OFFSET);
                }
            }
            return ESP_OK;
        });

    if (supportedPIDs & 0x00000001)
    {
        xSemaphoreGive(xRequestNextPIDSemaphore);
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
    return ESP_OK;
}

esp_err_t OBD2::requestVIN()
{
    if (vinData.vinReadySemaphore == NULL)
    {
        ESP_LOGE(TAG, "VIN semaphore not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    req(OBD2_FUNCTIONAL_ID, MODE_VEHICLE_INFO, PID_VIN, 2, 0, 0);

    xSemaphoreTake(vinData.vinReadySemaphore, 0);

    if (xSemaphoreTake(vinData.vinReadySemaphore, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t OBD2::requestDTC(uint8_t mode)
{
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

    if (*sem == NULL)
    {
        ESP_LOGE(TAG, "Semaphore for mode %02x not initialized!", mode);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(clearDTC(mode), TAG, "Failed to clear DTCs");

    // Drain any stale tokens before requesting
    while (xSemaphoreTake(*sem, 0) == pdTRUE)
        ;

    req(OBD2_FUNCTIONAL_ID, mode, 0x00, 2, 0, 0);

    if (xSemaphoreTake(*sem, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "DTC Request %s timed out", OBD2_MODE_TO_STR(mode));
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

void OBD2::requestClearDTCs()
{
    req(OBD2_FUNCTIONAL_ID, MODE_CLEAR_DTCS, 0x00, 2, 0, 0);
}

esp_err_t OBD2::captureMultiFrame(const CanDriver::CanFrame& f)
{
    static uint8_t                          MULTIDRAME_STATE = 99;
    uint8_t                                 frameType;
    static uint16_t                         totalLength = 0, consecutiveFrameIndex = 0, consecutiveFramesNeeded = 0;
    constexpr uint8_t                       CONSECUTIVE_FRAME_DATA_BYTES = 7, FIRST_FRAME_DATA_BYTES = 6;
    static std::vector<CanDriver::CanFrame> multiFrameBuffer;
    esp_err_t                               ret = ESP_OK;

    switch (MULTIDRAME_STATE)
    {
        case 99:  // Idle State
        {
            frameType = (f.data[0] >> 4) & 0x0F;
            if (frameType == 1)
            {
                MULTIDRAME_STATE = 0;  // Go to First Frame state
            }
            else
            {
                ESP_LOGE(TAG, "Unexpected multi-frame type: %d", frameType);
                return ESP_ERR_INVALID_RESPONSE;
            }
            captureMultiFrame(f);
            break;
        }
        case 0:  // First Frame
        {
            frameType               = (f.data[0] >> 4) & 0x0F;  // High nibble = 1
            uint8_t lengthHigh      = f.data[0] & 0x0F;         // Low nibble = 0
            uint8_t lengthLow       = f.data[1];
            totalLength             = (lengthHigh << 8) | lengthLow;
            consecutiveFramesNeeded = (totalLength - FIRST_FRAME_DATA_BYTES + CONSECUTIVE_FRAME_DATA_BYTES - 1) /
                                      CONSECUTIVE_FRAME_DATA_BYTES;
            consecutiveFrameIndex   = 0;
            multiFrameBuffer.clear();
            multiFrameBuffer.push_back(f);
            vTaskDelay(pdMS_TO_TICKS(10));  // Wait before sending Flow Control
            sendFlowControlFrame(f.header.id - 8);
            MULTIDRAME_STATE = 1;  // Go to Consecutive Frame state
            break;
        }
        case 1:  // Consecutive Frames
        {
            consecutiveFrameIndex++;
            frameType        = (f.data[0] >> 4) & 0x0F;
            uint8_t sequence = f.data[0] & 0x0F;
            if (frameType != 2 || sequence != consecutiveFrameIndex)
            {
                ESP_LOGE(TAG, "Unexpected consecutive frame. Expected seq: %d, got: %d", consecutiveFrameIndex & 0x0F,
                         sequence);
                MULTIDRAME_STATE = 99;
                return ESP_ERR_INVALID_RESPONSE;
            }
            multiFrameBuffer.push_back(f);
            if (consecutiveFrameIndex >= consecutiveFramesNeeded)
            {
                MULTIDRAME_STATE = 99;
                ret              = parseMultiFrame(multiFrameBuffer);
            }
            break;
        }
        default:
            MULTIDRAME_STATE = 99;
            ret              = ESP_ERR_INVALID_STATE;
            break;
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to parse multi-frame: %s", esp_err_to_name(ret));
        MULTIDRAME_STATE = 99;
    }
    return ret;
}

inline void OBD2::sendFlowControlFrame(uint32_t id)
{
    req(id, 0x00, 0x00, 0x30, 0, 0);
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

    esp_err_t ret = ESP_OK;

    uint8_t vinBuffer[VIN_LENGTH] = {0};
    size_t  vinIndex              = 0;

    for (const auto& frame : frames)
    {
        uint8_t startIdx = (frame.data[0] >> 4) == 1 ? 5 : 1;  // First frame starts data at index 5
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
        ret             = ESP_ERR_INVALID_SIZE;
    }

    vinData.lastUpdated = xTaskGetTickCount() * portTICK_PERIOD_MS;
    xSemaphoreGive(vinData.vinReadySemaphore);
    xSemaphoreGive(vinData.mtx_);

    return ret;
}