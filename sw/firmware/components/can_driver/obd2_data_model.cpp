// obd2_dtb.cpp

#include "obd2_data_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "obd2_utils.hpp"
#include "pid_def.hpp"

static const char* TAG = "OBD2DataModel";

void OBD2DataModel::initDef()
{
    memset(vinData.vin, 0, sizeof(vinData.vin));
    vinData.lastUpdated = 0;
    vinData.isValid     = false;

    dtcData.confirmed.clear();
    dtcData.pending.clear();
    dtcData.permanent.clear();

    if (vinData.vinReadySemaphore == nullptr)
        vinData.vinReadySemaphore = xSemaphoreCreateBinary();
    if (vinData.mtx_ == nullptr)
        vinData.mtx_ = xSemaphoreCreateMutex();

    if (dtcData.confirmedReadySemaphore == nullptr)
        dtcData.confirmedReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.pendingReadySemaphore == nullptr)
        dtcData.pendingReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.permanentReadySemaphore == nullptr)
        dtcData.permanentReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.clearReadySemaphore == nullptr)
        dtcData.clearReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.mtx_ == nullptr)
        dtcData.mtx_ = xSemaphoreCreateMutex();

    if (pidMapMtx == nullptr)
        pidMapMtx = xSemaphoreCreateMutex();

    if (subscribers_mtx_ == nullptr)
        subscribers_mtx_ = xSemaphoreCreateMutex();
};

esp_err_t OBD2DataModel::addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name,
                                std::string unit, std::string desc, std::string formula, float minV, float maxV,
                                uint8_t priority, uint16_t interval, uint32_t color, std::string icon)
{
    return withPidMapLock(
        [&]() -> esp_err_t
        {
            if (PID_DEF.find(pid) != PID_DEF.end())
            {
                return ESP_ERR_INVALID_ARG;
            }

            PID_DEF.try_emplace(pid, id, mode, pid, len, name, unit, desc, formula, minV, maxV, priority, interval,
                                color, icon);

            bool defaultSupported = (mode == MODE_READ_DATA_BY_IDENTIFIER || mode == MODE_DERIVED_DATA);

            pidData[pid] = {.id                = id,
                            .value             = 0.0f,
                            .lastUpdated       = 0,
                            .data              = {0},
                            .isSupported       = defaultSupported,
                            .isValid           = false,
                            .updateInterval_ms = interval};

            return ESP_OK;
        });
}

esp_err_t OBD2DataModel::removePID(uint16_t pid)
{
    return withPidMapLock(
        [&]() -> esp_err_t
        {
            if (!_pidExists(pid))
            {
                return ESP_ERR_NOT_FOUND;
            }

            pidData.erase(pid);
            PID_DEF.erase(pid);

            pollQueue.removePID(pid);

            return ESP_OK;
        });
}

esp_err_t OBD2DataModel::updateData(const CanDriver::CanFrame& frame)
{
    const uint8_t  mode          = frame.data[1];
    const bool     isRDBI        = (mode == RESPONSE_READ_DATA_BY_IDENTIFIER);
    const uint16_t pid           = isRDBI ? (uint16_t)(frame.data[2] << 8) | frame.data[3] : frame.data[2];
    const uint8_t  payloadOffset = isRDBI ? 4 : 3;

    return withPidMapLock(
        [&]() -> esp_err_t
        {
            PIDData_t*           pdat = nullptr;
            const PIDDefinition* pdef = nullptr;

            if (_getData(pid, pdat) != ESP_OK || pdat == nullptr)
                return ESP_ERR_NOT_FOUND;
            if (_getDef(pid, pdef) != ESP_OK || pdef == nullptr)
                return ESP_ERR_NOT_FOUND;

            float     val = 0.f;
            esp_err_t ret = pdef->evaluate(&frame.data[payloadOffset], frame.length - payloadOffset, val);

            if (ret == ESP_OK)
            {
                pdat->value       = val;
                pdat->lastUpdated = pdTICKS_TO_MS(xTaskGetTickCount());

                if (mode == RESPONSE_CURRENT_DATA && pdat->id != OBD2_FUNCTIONAL_ID)
                {
                    pdat->id = frame.header.id - 8;
                }

                memcpy(pdat->data, frame.data, PID_DATA_LENGTH < frame.length ? PID_DATA_LENGTH : frame.length);
            }

            return ret;
        });
}

bool OBD2DataModel::_pidExists(uint16_t pid) const
{
    return PID_DEF.find(pid) != PID_DEF.end();
}

bool OBD2DataModel::pidExists(uint16_t pid) const
{
    bool exists = false;

    withPidMapLock(
        [&]()
        {
            exists = _pidExists(pid);
            return ESP_OK;
        });

    return exists;
}

esp_err_t OBD2DataModel::_getData(uint16_t pid, PIDData_t*& pd) const
{
    auto it = pidData.find(pid);

    if (it == pidData.end())
    {
        pd = nullptr;
        return ESP_ERR_NOT_FOUND;
    }

    pd = const_cast<PIDData_t*>(&(it->second));
    return ESP_OK;
}

esp_err_t OBD2DataModel::getData(uint16_t pid, PIDData_t& pd) const
{
    return withPidMapLock(
        [&]()
        {
            PIDData_t* internalPtr = nullptr;
            esp_err_t  ret         = _getData(pid, internalPtr);

            if (ret == ESP_OK && internalPtr != nullptr)
            {
                pd = *internalPtr;
            }

            return ret;
        });
}

esp_err_t OBD2DataModel::_getDef(uint16_t pid, const PIDDefinition*& outDef) const
{
    auto it = PID_DEF.find(pid);
    if (it == PID_DEF.end())
    {
        outDef = nullptr;
        return ESP_ERR_NOT_FOUND;
    }

    outDef = &(it->second);

    return ESP_OK;
}

esp_err_t OBD2DataModel::getDef(uint16_t pid, PIDDefinitionData& outDef) const
{
    return withPidMapLock(
        [&]()
        {
            const PIDDefinition* internalPtr = nullptr;
            esp_err_t            ret         = _getDef(pid, internalPtr);

            if (ret == ESP_OK && internalPtr != nullptr)
            {
                outDef.id                = internalPtr->id_;
                outDef.mode              = internalPtr->mode_;
                outDef.pid               = internalPtr->pid_;
                outDef.len               = internalPtr->len_;
                outDef.name              = internalPtr->name_;
                outDef.unit              = internalPtr->unit_;
                outDef.description       = internalPtr->description_;
                outDef.formula           = internalPtr->formula_;
                outDef.minValue          = internalPtr->minValue_;
                outDef.maxValue          = internalPtr->maxValue_;
                outDef.priority          = internalPtr->priority_;
                outDef.updateInterval_ms = internalPtr->updateInterval_ms_;
                outDef.color             = internalPtr->color_;
                outDef.icon              = internalPtr->icon_;
            }

            return ret;
        });
}

// PID_DEF Getters

std::vector<uint16_t> OBD2DataModel::getPIDs() const
{
    std::vector<uint16_t> keys;

    withPidMapLock(
        [&]()
        {
            keys.reserve(PID_DEF.size());

            for (const auto& [pid, info] : PID_DEF)
            {
                keys.push_back(pid);
            }
            return ESP_OK;
        });

    return keys;
}

// Array / Buffer Getters

uint8_t OBD2DataModel::_getRawDataByte(uint16_t pid, uint8_t idx) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    if (idx < PID_DATA_LENGTH)
    {
        return pd->data[idx];
    }
    else
    {
        return 0;
    }
}

uint8_t OBD2DataModel::getRawDataByte(uint16_t pid, uint8_t idx) const
{
    uint8_t byte = 0;
    withPidMapLock(
        [&]()
        {
            PIDData_t* pd = nullptr;
            if (_getData(pid, pd) == ESP_OK && pd != nullptr && idx < PID_DATA_LENGTH)
            {
                byte = pd->data[idx];
            }
            return ESP_OK;
        });
    return byte;
}

esp_err_t OBD2DataModel::_getRawData(uint16_t pid, uint8_t* outData) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    memcpy(outData, pd->data, PID_DATA_LENGTH);

    return ESP_OK;
}

esp_err_t OBD2DataModel::getRawData(uint16_t pid, uint8_t* outData) const
{
    if (outData == nullptr)
        return ESP_ERR_INVALID_ARG;

    return withPidMapLock(
        [&]()
        {
            PIDData_t* pd  = nullptr;
            esp_err_t  err = _getData(pid, pd);
            if (err == ESP_OK && pd != nullptr)
            {
                memcpy(outData, pd->data, 8);
            }
            return err;
        });
}

float OBD2DataModel::getValueUnsafe(uint16_t pid) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return NAN;
    };

    return pd->value;
}

uint8_t OBD2DataModel::getRawDataByteUnsafe(uint16_t pid, uint8_t idx) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    if (idx < PID_DATA_LENGTH)
    {
        return pd->data[idx];
    }
    else
    {
        return 0;
    }
}

std::string OBD2DataModel::getVIN() const
{
    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return "";
    }

    std::string result;

    if (vinData.isValid)
    {
        result = std::string(vinData.vin);
    }

    xSemaphoreGive(vinData.mtx_);
    return result;
}

esp_err_t OBD2DataModel::_setDTC(uint16_t rawDTC, uint8_t mode)
{
    if (rawDTC == 0)
    {
        return ESP_OK;
    }

    if (xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    std::string               dtcCode = decodeDTC(rawDTC);
    std::vector<std::string>* target  = nullptr;

    switch (mode)
    {
        case MODE_DTCS:
        case RESPONSE_DTCS:
            target = &dtcData.confirmed;
            break;
        case MODE_PENDING_DTCS:
        case RESPONSE_PENDING_DTCS:
            target = &dtcData.pending;
            break;
        case MODE_PERMANENT_DTCS:
        case RESPONSE_PERMANENT_DTCS:
            target = &dtcData.permanent;
            break;
        default:
            xSemaphoreGive(dtcData.mtx_);
            return ESP_ERR_INVALID_ARG;
    }

    if (std::find(target->begin(), target->end(), dtcCode) == target->end())
    {
        target->push_back(dtcCode);
    }

    xSemaphoreGive(dtcData.mtx_);
    return ESP_OK;
}

esp_err_t OBD2DataModel::clearDTC(uint8_t mode)
{
    if (xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    switch (mode)
    {
        case MODE_DTCS:
        case RESPONSE_DTCS:
            dtcData.confirmed.clear();
            break;
        case MODE_PENDING_DTCS:
        case RESPONSE_PENDING_DTCS:
            dtcData.pending.clear();
            break;
        case MODE_PERMANENT_DTCS:
        case RESPONSE_PERMANENT_DTCS:
            dtcData.permanent.clear();
            break;
    }
    xSemaphoreGive(dtcData.mtx_);
    return ESP_OK;
}

std::string OBD2DataModel::decodeDTC(uint16_t rawDTC)
{
    if (rawDTC == 0)
    {
        return "No DTC";
    }
    char    dtc[6];
    uint8_t type_bits = (rawDTC >> 14) & 0x03;

    uint8_t first_digit = (rawDTC >> 12) & 0x03;

    uint16_t last_digits = rawDTC & 0x0FFF;

    char prefix;
    switch (type_bits)
    {
        case 0:
            prefix = 'P';
            break;  // Powertrain
        case 1:
            prefix = 'C';
            break;  // Chassis
        case 2:
            prefix = 'B';
            break;  // Body
        case 3:
            prefix = 'U';
            break;  // Network
        default:
            return "No DTC";
    }

    // Format the DTC string: Letter + 4 digits
    snprintf(dtc, sizeof(dtc), "%c%01X%03X", prefix, first_digit, last_digits);

    return std::string(dtc);
}

std::vector<std::string> OBD2DataModel::getDTC(uint8_t mode) const
{
    std::vector<std::string> result;

    if (xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Failed to get dtc");
        return result;
    }

    switch (mode)
    {
        case MODE_DTCS:
        case RESPONSE_DTCS:
            result = dtcData.confirmed;
            break;
        case MODE_PENDING_DTCS:
        case RESPONSE_PENDING_DTCS:
            result = dtcData.pending;
            break;
        case MODE_PERMANENT_DTCS:
        case RESPONSE_PERMANENT_DTCS:
            result = dtcData.permanent;
            break;
    }

    xSemaphoreGive(dtcData.mtx_);
    return result;
}

void OBD2DataModel::subscribe(PidUpdateCallback cb)
{
    bool locked = (subscribers_mtx_ != nullptr) && (xSemaphoreTake(subscribers_mtx_, portMAX_DELAY) == pdTRUE);
    subscribers_.push_back(std::move(cb));
    if (locked)
        xSemaphoreGive(subscribers_mtx_);
}

void OBD2DataModel::runPidUpdateCallbacks(uint16_t pid)
{
    std::vector<PidUpdateCallback> callbacks;

    if (subscribers_mtx_ != nullptr && xSemaphoreTake(subscribers_mtx_, portMAX_DELAY) == pdTRUE)
    {
        callbacks = subscribers_;
        xSemaphoreGive(subscribers_mtx_);
    }

    for (const auto& cb : callbacks)
    {
        cb(pid);
    }
}
