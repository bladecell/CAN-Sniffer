// obd2_dtb.cpp

#include "obd2_data_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "obd2_common.hpp"
#include "pid_def.hpp"

static const char* TAG = "OBD2DataModel";

esp_err_t OBD2DataModel::initDef()
{
    if (vinData.vinReadySemaphore == nullptr)
        vinData.vinReadySemaphore = xSemaphoreCreateBinary();

    if (dtcData.confirmedReadySemaphore == nullptr)
        dtcData.confirmedReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.pendingReadySemaphore == nullptr)
        dtcData.pendingReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.permanentReadySemaphore == nullptr)
        dtcData.permanentReadySemaphore = xSemaphoreCreateBinary();
    if (dtcData.clearReadySemaphore == nullptr)
        dtcData.clearReadySemaphore = xSemaphoreCreateBinary();
    const bool allocated = vinData.vinReadySemaphore && vinData.mtx_ && dtcData.confirmedReadySemaphore &&
                   dtcData.pendingReadySemaphore && dtcData.permanentReadySemaphore && dtcData.clearReadySemaphore &&
                   dtcData.mtx_ && pidMapMtx && subscribers_mtx_;
    if (!allocated) return ESP_ERR_NO_MEM;
    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(100)) != pdTRUE) return ESP_ERR_TIMEOUT;
    if (xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(100)) != pdTRUE) {
        xSemaphoreGive(vinData.mtx_);
        return ESP_ERR_TIMEOUT;
    }
    memset(vinData.vin, 0, sizeof(vinData.vin));
    vinData.lastUpdated = 0;
    vinData.isValid = false;
    dtcData.confirmed.clear(); dtcData.pending.clear(); dtcData.permanent.clear();
    xSemaphoreGive(dtcData.mtx_);
    xSemaphoreGive(vinData.mtx_);
    while (xSemaphoreTake(vinData.vinReadySemaphore, 0) == pdTRUE) {}
    while (xSemaphoreTake(dtcData.confirmedReadySemaphore, 0) == pdTRUE) {}
    while (xSemaphoreTake(dtcData.pendingReadySemaphore, 0) == pdTRUE) {}
    while (xSemaphoreTake(dtcData.permanentReadySemaphore, 0) == pdTRUE) {}
    while (xSemaphoreTake(dtcData.clearReadySemaphore, 0) == pdTRUE) {}
    return ESP_OK;
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

esp_err_t OBD2DataModel::getDefinitionSnapshot(std::vector<PIDDefinitionData>& out) const
{
    return withPidMapLock(
        [&]() -> esp_err_t
        {
            out.clear();
            out.reserve(PID_DEF.size());
            for (const auto& [pid, definition] : PID_DEF)
            {
                PIDDefinitionData snapshot = {};
                snapshot.id                = definition.id_;
                snapshot.mode              = definition.mode_;
                snapshot.pid               = pid;
                snapshot.len               = definition.len_;
                snapshot.name              = definition.name_;
                snapshot.unit              = definition.unit_;
                snapshot.description       = definition.description_;
                snapshot.formula           = definition.formula_;
                snapshot.minValue          = definition.minValue_;
                snapshot.maxValue          = definition.maxValue_;
                snapshot.priority          = definition.priority_;
                snapshot.updateInterval_ms = definition.updateInterval_ms_;
                snapshot.color             = definition.color_;
                snapshot.icon              = definition.icon_;
                out.push_back(std::move(snapshot));
            }
            return ESP_OK;
        });
}

esp_err_t OBD2DataModel::getDataSnapshot(std::vector<std::pair<uint16_t, PIDData_t>>& out) const
{
    return withPidMapLock(
        [&]() -> esp_err_t
        {
            out.clear();
            out.reserve(pidData.size());
            for (const auto& [pid, data] : pidData)
                out.emplace_back(pid, data);
            return ESP_OK;
        });
}

uint32_t OBD2DataModel::getPIDDataSize() const
{
    uint32_t result = 0;
    withPidMapLock(
        [&]()
        {
            result = static_cast<uint32_t>(pidData.size());
            return ESP_OK;
        });
    return result;
}

uint32_t OBD2DataModel::getPIDDEFSize() const
{
    uint32_t result = 0;
    withPidMapLock(
        [&]()
        {
            result = static_cast<uint32_t>(PID_DEF.size());
            return ESP_OK;
        });
    return result;
}

esp_err_t OBD2DataModel::replacePIDDefinitions(const std::vector<PIDDefinitionData>& definitions,
                                               const std::vector<PollRequest>& recurringRequests,
                                               const std::vector<uint16_t>& supportedCurrentPids)
{
    if (definitions.size() > NUMBER_OF_ITEMS || recurringRequests.size() > NUMBER_OF_ITEMS)
        return ESP_ERR_INVALID_SIZE;

    std::map<uint16_t, PIDDefinition> stagedDefinitions;
    std::map<uint16_t, PIDData_t> stagedData;

    for (const auto& definition : definitions)
    {
        auto result = stagedDefinitions.emplace(
            std::piecewise_construct, std::forward_as_tuple(definition.pid),
            std::forward_as_tuple(definition.id, definition.mode, definition.pid, definition.len, definition.name,
                                  definition.unit, definition.description, definition.formula, definition.minValue,
                                  definition.maxValue, definition.priority, definition.updateInterval_ms,
                                  definition.color, definition.icon));
        if (!result.second || !result.first->second.formulaValid())
            return ESP_ERR_INVALID_ARG;

        const bool currentSupported =
            definition.mode == MODE_CURRENT_DATA &&
            std::find(supportedCurrentPids.begin(), supportedCurrentPids.end(), definition.pid) !=
                supportedCurrentPids.end();
        const bool defaultSupported = currentSupported || definition.mode == MODE_READ_DATA_BY_IDENTIFIER ||
                                      definition.mode == MODE_DERIVED_DATA;

        stagedData.emplace(definition.pid,
                           PIDData_t{definition.id, 0.0f, 0, {0}, defaultSupported, false,
                                     definition.updateInterval_ms});
    }

    return withPidMapLock(
        [&]() -> esp_err_t
        {
            // Queue capacity is checked and changed before either active map
            // is touched. A failed capacity check therefore leaves both the
            // old maps and the old queue intact.
            if (!pollQueue.replaceRecurring(recurringRequests.data(), recurringRequests.size()))
                return ESP_ERR_INVALID_SIZE;

            // Responder ownership is the only telemetry state that survives
            // a complete definition replacement. Carry it only when the same
            // PID remains a supported Mode-1 entry; all value, validity,
            // timestamp, and raw-data fields stay freshly initialized.
            for (const auto& definition : definitions)
            {
                if (definition.mode != MODE_CURRENT_DATA ||
                    std::find(supportedCurrentPids.begin(), supportedCurrentPids.end(), definition.pid) ==
                        supportedCurrentPids.end())
                    continue;

                const auto oldDefinition = PID_DEF.find(definition.pid);
                const auto oldData       = pidData.find(definition.pid);
                if (oldDefinition == PID_DEF.end() || oldData == pidData.end() ||
                    oldDefinition->second.mode() != MODE_CURRENT_DATA || !oldData->second.isSupported)
                    continue;

                auto staged = stagedData.find(definition.pid);
                if (staged != stagedData.end())
                    staged->second.id = oldData->second.id;
            }

            PID_DEF.swap(stagedDefinitions);
            pidData.swap(stagedData);
            return ESP_OK;
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
    if (!vinData.mtx_ || xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
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

    if (!dtcData.mtx_ || xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(100)) != pdTRUE)
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
    if (!dtcData.mtx_ || xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
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

    if (!dtcData.mtx_ || xSemaphoreTake(dtcData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
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
    if (!subscribers_mtx_ || xSemaphoreTake(subscribers_mtx_, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "PID subscriber lock unavailable; subscription rejected");
        return;
    }
    subscribers_.push_back(std::move(cb));
    xSemaphoreGive(subscribers_mtx_);
}

void OBD2DataModel::runPidUpdateCallbacks(uint16_t pid, const std::atomic<bool>* stopRequested)
{
    std::vector<PidUpdateCallback> callbacks;

    if (subscribers_mtx_ != nullptr && xSemaphoreTake(subscribers_mtx_, portMAX_DELAY) == pdTRUE)
    {
        callbacks = subscribers_;
        xSemaphoreGive(subscribers_mtx_);
    }

    for (const auto& cb : callbacks)
    {
        if (stopRequested && stopRequested->load()) break;
        cb(pid);
    }
}
