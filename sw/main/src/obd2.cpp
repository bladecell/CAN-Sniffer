// obd2.cpp
#include "obd2.hpp"

static const char *TAG = "OBD2";

OBD2::OBD2(CanDriver &canDriver)
    : canDriver(canDriver),
      continuousRunning(false),
      mtx_(xSemaphoreCreateMutex()) {}

OBD2::~OBD2()
{
    if (mtx_)
    {
        vSemaphoreDelete(mtx_);
        mtx_ = nullptr;
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
        4096,
        this,
        tskIDLE_PRIORITY + 2,
        &ReceiveTaskHandle,
        CORE_ID_CAN_TASKS);

    if (taskCreated != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }
    xBusConnectionSemaphore = xSemaphoreCreateBinary();
    configASSERT(xBusConnectionSemaphore);

    if (canDriver.isBusConnected())
    {
        ESP_LOGI(TAG, "CAN bus already connected, getting supported PIDs");
        getSuppPids();
    }
    else
    {
        ESP_LOGW(TAG, "CAN bus not connected, waiting for connection...");
    }

    ESP_LOGI(TAG, "OBD-II interface initialized");
    return ESP_OK;
}

const std::map<uint8_t, PIDInfo_t> OBD2::PID_DEF = {
    {PID_ENGINE_LOAD,
     PIDInfo_t{
         .mode = MODE_CURRENT_DATA,
         .pid = PID_ENGINE_LOAD,
         .name = "Engine Load",
         .unit = PERCENTAGE,
         .description = "Calculated engine load",
         .formula = OBDFormulas::engineLoad,
         .minValue = 0.0f,
         .maxValue = 100.0f,
         .priority = 2,
         .updateInterval_ms = UPDATE_FAST}},
    {PID_COOLANT_TEMP,
     PIDInfo_t{
         .mode = MODE_CURRENT_DATA,
         .pid = PID_COOLANT_TEMP,
         .name = "Coolant Temp",
         .unit = DEGREES_CELCIUS,
         .description = "Engine coolant temperature",
         .formula = OBDFormulas::coolantTemp,
         .minValue = -40.0f,
         .maxValue = 215.0f,
         .priority = 3,
         .updateInterval_ms = UPDATE_MEDIUM}},
    {PID_ENGINE_RPM,
     PIDInfo_t{
         .mode = MODE_CURRENT_DATA,
         .pid = PID_ENGINE_RPM,
         .name = "Engine RPM",
         .unit = RPM,
         .description = "Engine speed",
         .formula = OBDFormulas::engineRPM,
         .minValue = 0.0f,
         .maxValue = 16383.75f,
         .priority = 1,
         .updateInterval_ms = UPDATE_FAST}},
};

void OBD2::initDef()
{
    for (const auto &[pid, info] : PID_DEF)
    {
        pidData[pid] = {
            .value = 0.0f,
            .lastUpdated = 0,
            .data = {0},
            .isSupported = false,
            .isValid = false,
            .updateInterval_ms = info.updateInterval_ms,
            .semaphore = xSemaphoreCreateBinary()};
    }
}

esp_err_t OBD2::getSuppPids()
{
    esp_err_t ret = ESP_OK;

    for (OBDPID pid_marker : SUPPORTED_PID_MAKERS)
    {
        esp_err_t ret_q = queryMsg(MODE_CURRENT_DATA, (uint8_t)pid_marker);

        if (ret_q != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to queue PID support query 0x%02X. Error: %d", pid_marker, ret_q);
            ret = ret_q;
        }
    }

    return ret;
}

esp_err_t OBD2::req(uint8_t pid)
{
    if (!isSup(pid))
    {
        ESP_LOGW(TAG, "PID 0x%02X is not supported or not recognized", pid);
        return ESP_ERR_NOT_SUPPORTED;
    }

    setValid(pid, false); // Invalidate data until updated
    esp_err_t ret = queryMsg(PID_DEF.at(pid).mode, pid);

    return ret;
}

esp_err_t OBD2::getData(uint8_t pid, PIDData_t &pd) const
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    auto it = pidData.find(pid);
    if (it != pidData.end())
    {
        pd = it->second;
    }
    xSemaphoreGive(mtx_);
    return it != pidData.end() ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t OBD2::updateData(const CanDriver::CanFrame &frame)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    uint8_t pid = frame.data[2];

    pidData.at(pid).value = PID_DEF.at(pid).formula(frame.data, frame.length);
    pidData.at(pid).lastUpdated = xTaskGetTickCount();
    if (frame.length > PID_DATA_LENGTH)
    {
        ESP_LOGW(TAG, "Frame length %d exceeds PID data length %d, truncating", frame.length, PID_DATA_LENGTH);
    }
    memcpy(pidData.at(pid).data, frame.data, PID_DATA_LENGTH < frame.length ? PID_DATA_LENGTH : frame.length);

    xSemaphoreGive(mtx_);
    return ESP_OK;
}

esp_err_t OBD2::queryMsg(uint8_t mode, uint8_t pid, uint8_t len)
{
    if (!canDriver.isBusConnected())
    {
        ESP_LOGE(TAG, "OBD-II interface not connected");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t txData[8] = {len, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
    twai_frame_t tx = {};

    tx.header.id = OBD2_FUNCTIONAL_ID; // OBD-II functional request ID
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

    esp_err_t ret = canDriver.transmit(&tx, 1000);

    return ret;
}

bool OBD2::isSup(uint8_t pid) const
{
    if (!pidExists(pid))
    {
        return false;
    }

    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return false;
    }

    bool supported = pidData.at(pid).isSupported;

    xSemaphoreGive(mtx_);
    return supported;
}

uint8_t OBD2::getmode(uint8_t pid) const
{
    return !pidExists(pid) ? 0 : PID_DEF.at(pid).mode;
}

const char *OBD2::getName(uint8_t pid) const
{
    return !pidExists(pid) ? "Unknown PID" : PID_DEF.at(pid).name;
}

const char *OBD2::getUnit(uint8_t pid) const
{
    return !pidExists(pid) ? "Unknown PID" : PID_DEF.at(pid).unit;
}

const char *OBD2::getDescription(uint8_t pid) const
{
    return !pidExists(pid) ? "Unknown PID" : PID_DEF.at(pid).description;
}

float OBD2::getMinValue(uint8_t pid) const
{
    return !pidExists(pid) ? NAN : PID_DEF.at(pid).minValue;
}

float OBD2::getMaxValue(uint8_t pid) const
{
    return !pidExists(pid) ? NAN : PID_DEF.at(pid).maxValue;
}
uint8_t OBD2::getPriority(uint8_t pid) const
{
    return !pidExists(pid) ? 0 : PID_DEF.at(pid).priority;
}

float OBD2::getValue(uint8_t pid, uint32_t timeout_ms) const
{
    if (!pidExists(pid))
    {
        return NAN;
    }

    // Block until receiver task signals response arrival
    if (xSemaphoreTake(pidData.at(pid).semaphore, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return NAN; // Timeout waiting for CAN response
    }

    if (!isValid(pid))
    {
        return NAN;
    }

    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return NAN;
    }

    float value = pidData.at(pid).value;
    xSemaphoreGive(mtx_);

    return value;
}

uint32_t OBD2::getLastUpdated(uint8_t pid) const
{
    if (!pidExists(pid))
    {
        return 0;
    }

    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return 0;
    }

    uint32_t lastUpdated = pidData.at(pid).lastUpdated;

    xSemaphoreGive(mtx_);
    return lastUpdated;
}

esp_err_t OBD2::getRawData(uint8_t pid, uint8_t *outData) const
{
    if (!pidExists(pid))
        return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(outData, pidData.at(pid).data, PID_DATA_LENGTH);

    xSemaphoreGive(mtx_);
    return ESP_OK;
}

uint16_t OBD2::getUpdateInterval(uint8_t pid) const
{
    return !pidExists(pid) ? 0 : pidData.at(pid).updateInterval_ms;
}

bool OBD2::isValid(uint8_t pid) const
{
    if (!pidExists(pid))
    {
        return false;
    }

    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return false;
    }

    bool valid = pidData.at(pid).isValid;

    xSemaphoreGive(mtx_);
    return valid;
}

esp_err_t OBD2::setValid(uint8_t pid, bool valid)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    if (pidExists(pid))
    {
        pidData[pid].isValid = valid;
        xSemaphoreGive(pidData[pid].semaphore);
        xSemaphoreGive(mtx_);
        return ESP_OK;
    }
    xSemaphoreGive(mtx_);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t OBD2::setUpdateInterval(uint8_t pid, UpdateRate interval_ms)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    if (pidExists(pid))
    {
        pidData[pid].updateInterval_ms = interval_ms;
        xSemaphoreGive(mtx_);
        return ESP_OK;
    }
    xSemaphoreGive(mtx_);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t OBD2::setIsSupported(uint8_t pid, bool supported)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    if (pidExists(pid))
    {
        pidData[pid].isSupported = supported;
        xSemaphoreGive(mtx_);
        return ESP_OK;
    }
    xSemaphoreGive(mtx_);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t OBD2::setLastUpdated(uint8_t pid, uint32_t lastUpdated)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }
    if (pidExists(pid))
    {
        pidData[pid].lastUpdated = lastUpdated;
        xSemaphoreGive(mtx_);
        return ESP_OK;
    }
    xSemaphoreGive(mtx_);
    return ESP_ERR_NOT_FOUND;
}

bool OBD2::pidExists(uint8_t pid) const
{
    return PID_DEF.find(pid) != PID_DEF.end();
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
        getSuppPids();
    }
}

void OBD2::handleCanDisconnected()
{
    ESP_LOGW(TAG, "CAN bus disconnected event received");
    pidsInitialized = false;
    pidGroupStatus.clear();
}

void OBD2::pollTaskWrapper(void *param)
{
    OBD2 *obd2 = static_cast<OBD2 *>(param);
    obd2->pollTask();
}

void OBD2::startContinuousMode()
{
    if (continuousRunning)
        return;

    continuousRunning = true;
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        pollTaskWrapper,
        "OBD2_PollTask",
        4096,
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

void OBD2::pollTask()
{
    const int MAT_PENDING_REQUESTS = 10;

    while (1)
    {
        if (!canDriver.isBusConnected())
        {
            xSemaphoreTake(xBusConnectionSemaphore, portMAX_DELAY);
            continue;
        }

        if (!pidsInitialized)
        {
            xSemaphoreTake(xPidConnectedSemaphore, portMAX_DELAY);
            continue;
        }

        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        struct Candidate
        {
            uint8_t pid;
            uint8_t priority;
            uint32_t timeSinceLastUpdate;
        };

        std::vector<Candidate> candidates;
        // 1. Identify Overdue Candidates
        for (auto &[pid, _] : pidData)
        {
            if (!isSup(pid))
                continue;

            uint16_t lastUpdated = getLastUpdated(pid);
            uint16_t updateInterval = getUpdateInterval(pid);

            if (now - lastUpdated >= updateInterval)
            {
                candidates.push_back({pid,
                                      getPriority(pid),
                                      (now - lastUpdated) - updateInterval});
            }
        }
        // 2. Prioritize Candidates
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate &a, const Candidate &b)
                  {
                      // Sort by Priority (High to Low)
                      if (a.priority != b.priority)
                      {
                          return a.priority > b.priority;
                      }

                      // Sort by Time Since Last Update
                      return a.timeSinceLastUpdate > b.timeSinceLastUpdate;
                  });
        // 3. Process Top Candidates
        int requestsSent = 0;
        for (const auto &candidate : candidates)
        {
            if (requestsSent >= MAT_PENDING_REQUESTS)
            {
                break;
            }

            esp_err_t res = req(candidate.pid);

            if (res == ESP_OK)
            {
                requestsSent++;

                setLastUpdated(candidate.pid, now);
            }
            else
            {
                break;
            }
        }

        // 4. Yield/Delay
        vTaskDelay(pdMS_TO_TICKS(5));
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

    if (f.data[0] > 0x08)
    {
        // TODO parseMultiFrame(f);
    }

    switch (f.data[1])
    {
    case MODE_CURRENT_DATA | 0x40:
        ret = parseCurrentData(f);
        break;
    case MODE_DTCS | 0x40:
        // TODO parseDTCs(f);
        break;
    case MODE_CLEAR_DTCS | 0x40:
        // TODO parseClearDTCsAck(f);
        break;
    case MODE_PENDING_DTCS | 0x40:
        // TODO parsePendingDTCs(f);
        break;
    case MODE_VEHICLE_INFO | 0x40:
        // TODO parseVehicleInfo(f);
        break;
    case MODE_PERMANENT_DTCS | 0x40:
        // TODO parsePermanentDTCs(f);
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

esp_err_t OBD2::parseCurrentData(const CanDriver::CanFrame &f)
{
    uint8_t pid = f.data[2];
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

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update %s data: %s", getName(pid), esp_err_to_name(ret));
        setValid(pid, false);
        return ret;
    }
    setValid(pid, true);

    return ESP_OK;
}

esp_err_t OBD2::parseSupportedPIDs(const CanDriver::CanFrame &f)
{
    uint8_t pidGroup = f.data[2] - 1;
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

                esp_err_t ret = setIsSupported(supportedPID, true);
                if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND)
                {
                    ESP_LOGD(TAG, "Failed to set PID 0x%02X as supported: %s",
                             supportedPID, esp_err_to_name(ret));
                }
            }
        }
    }
    else
    {
        pidGroupStatus[pidGroup] = false;
        return ESP_ERR_INVALID_SIZE;
    }

    pidGroupStatus[pidGroup] = true;

    bool allGroupsComplete = true;
    for (auto const &[marker, received] : pidGroupStatus)
    {
        if (!received)
        {
            allGroupsComplete = false;
            break;
        }
    }

    if (allGroupsComplete)
    {
        pidsInitialized = true;
        ESP_LOGI(TAG, "All %zu PID support groups initialized successfully!", SUPPORTED_PID_MAKERS.size());
        xSemaphoreGive(xPidConnectedSemaphore);
    }
    else
    {
        pidsInitialized = false;
    }

    return ESP_OK;
}