#include "esp_log.h"
#include "esp_err.h"
#include "obd2.hpp"
#include "freertos/FreeRTOS.h"

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

    if (canDriver.isBusConnected())
    {
        ESP_LOGI(TAG, "CAN bus already connected, getting supported PIDs");
        getSuppPids(PID_PIDS_SUPPORTED_1_20);
        pidsInitialized = true;
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
         .priority = 2}},
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
         .priority = 3}},
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
         .priority = 1}},
};

void OBD2::initDef()
{
    for (const auto &[pid, _] : PID_DEF)
    {
        pidData[pid] = {
            .value = 0.0f,
            .lastUpdated = 0,
            .data = {0},
            .isSupported = false,
            .isValid = false,
            .updateInterval_ms = 100};
    }
}

esp_err_t OBD2::getSuppPids(uint8_t pidGroup)
{
    CanDriver::CanFrame responseFrame;

    esp_err_t ret = queryMsg(MODE_CURRENT_DATA, pidGroup, 0x02, responseFrame, 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to query supported PIDs for group 0x%02X: %s", pidGroup, esp_err_to_name(ret));
        return ret;
    }

    if (responseFrame.length >= 4)
    {
        uint32_t supportedPIDs = (responseFrame.data[3] << 24) |
                                 (responseFrame.data[4] << 16) |
                                 (responseFrame.data[5] << 8) |
                                 (responseFrame.data[6]);

        ESP_LOGD(TAG, "Supported PIDs for group 0x%02X: 0x%08X", pidGroup, supportedPIDs);

        for (uint8_t i = 0; i < 32; ++i)
        {
            if (supportedPIDs & (1UL << (31 - i)))
            {
                uint8_t supportedPID = pidGroup + 1 + i;

                ret = setIsSupported(supportedPID, true);
                if (ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "PID 0x%02X (%s) is supported",
                             supportedPID, getName(supportedPID));
                }
                else if (ret == ESP_ERR_NOT_FOUND)
                {
                    ESP_LOGD(TAG, "PID 0x%02X is supported but not in database", supportedPID);
                }
                else
                {
                    ESP_LOGD(TAG, "Failed to set PID 0x%02X as supported: %s",
                             supportedPID, esp_err_to_name(ret));
                }
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "Response frame too short to determine supported PIDs");
    }

    return ESP_OK;
}

esp_err_t OBD2::req(uint8_t pid)
{
    if (!isSup(pid))
    {
        ESP_LOGW(TAG, "PID 0x%02X is not supported or not recognized", pid);
        return ESP_ERR_NOT_SUPPORTED;
    }

    CanDriver::CanFrame responseFrame;

    esp_err_t ret = queryMsg(PID_DEF.at(pid).mode, pid, 0x02, responseFrame, 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to query %s: %s", getName(pid), esp_err_to_name(ret));
        setValid(pid, false);
        return ret;
    }

    ret = updateData(pid, responseFrame);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update %s data: %s", getName(pid), esp_err_to_name(ret));
        setValid(pid, false);
        return ret;
    }
    setValid(pid, true);
    return ESP_OK;
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

esp_err_t OBD2::updateData(uint8_t pid, const CanDriver::CanFrame &frame)
{
    if (xSemaphoreTake(mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

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

esp_err_t OBD2::queryMsg(uint8_t mode, uint8_t pid, uint8_t len, CanDriver::CanFrame &rxFrame, uint32_t timeout_ms)
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

    ESP_ERROR_CHECK_WITHOUT_ABORT(canDriver.flushRxQueue());

    esp_err_t ret = canDriver.transmit(&tx, 1000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send OBD request: %s", esp_err_to_name(ret));
    }

    const uint32_t start = xTaskGetTickCount();
    const uint8_t expectedMode = 0x40 | mode;

    while (pdTICKS_TO_MS(xTaskGetTickCount() - start) < timeout_ms)
    {
        CanDriver::CanFrame f{};
        ret = canDriver.receive(f, 50);
        if (ret != ESP_OK)
            continue;

        if (f.id < OBD2_RESPONSE_BASE_ID || f.id > (OBD2_RESPONSE_BASE_ID + 7))
            continue;

        if (f.length < 3)
            continue;

        if (f.data[1] == expectedMode && f.data[2] == pid)
        {
            rxFrame = f;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

bool OBD2::isSup(uint8_t pid)
{
    if (!pidExists(pid))
        return false;
    return pidData.at(pid).isSupported;
}

uint8_t OBD2::getmode(uint8_t pid) const
{
    if (!pidExists(pid))
        return 0;
    return PID_DEF.at(pid).mode;
}

const char *OBD2::getName(uint8_t pid) const
{
    if (!pidExists(pid))
        return "Unknown PID";
    return PID_DEF.at(pid).name;
}

const char *OBD2::getUnit(uint8_t pid) const
{
    if (!pidExists(pid))
        return "Unknown PID";
    return PID_DEF.at(pid).unit;
}

const char *OBD2::getDescription(uint8_t pid) const
{
    if (!pidExists(pid))
        return "Unknown PID";
    return PID_DEF.at(pid).description;
}

float OBD2::getMinValue(uint8_t pid) const
{
    if (!pidExists(pid))
        return NAN;
    return PID_DEF.at(pid).minValue;
}

float OBD2::getMaxValue(uint8_t pid) const
{
    if (!pidExists(pid))
        return NAN;
    return PID_DEF.at(pid).maxValue;
}
uint8_t OBD2::getPriority(uint8_t pid) const
{
    if (!pidExists(pid))
        return 0;
    return PID_DEF.at(pid).priority;
}

float OBD2::getValue(uint8_t pid) const
{
    if (!pidExists(pid))
        return 0;
    return pidData.at(pid).value;
}

uint32_t OBD2::getLastUpdated(uint8_t pid) const
{
    if (!pidExists(pid))
        return 0;
    return pidData.at(pid).lastUpdated;
}

esp_err_t OBD2::getData(uint8_t pid, uint8_t *outData) const
{
    if (!pidExists(pid))
        return ESP_ERR_NOT_FOUND;
    memcpy(outData, pidData.at(pid).data, PID_DATA_LENGTH);
    return ESP_OK;
}

uint16_t OBD2::getUpdateInterval(uint8_t pid) const
{
    if (!pidExists(pid))
        return 0;
    return pidData.at(pid).updateInterval_ms;
}

bool OBD2::isValid(uint8_t pid) const
{
    if (!pidExists(pid))
        return false;
    return pidData.at(pid).isValid;
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
        xSemaphoreGive(mtx_);
        return ESP_OK;
    }
    xSemaphoreGive(mtx_);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t OBD2::setUpdateInterval(uint8_t pid, uint16_t interval_ms)
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

    if (!pidsInitialized)
    {
        ESP_LOGI(TAG, "Retrieving supported PIDs...");
        getSuppPids(PID_PIDS_SUPPORTED_1_20);
        pidsInitialized = true;
    }
}

void OBD2::handleCanDisconnected()
{
    ESP_LOGW(TAG, "CAN bus disconnected event received");
    pidsInitialized = false;
}