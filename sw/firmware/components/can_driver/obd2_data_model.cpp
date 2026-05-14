// obd2_dtb.cpp

#include "obd2_data_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <set>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "obd2_utils.hpp"
#include "pid_def.hpp"

static const char* TAG = "OBD2DataModel";

void OBD2DataModel::initDef()
{
    vinData = {
        .vin               = {0},
        .lastUpdated       = 0,
        .isValid           = false,
        .vinReadySemaphore = xSemaphoreCreateBinary(),
        .mtx_              = xSemaphoreCreateMutex(),
    };

    dtcData = {
        .confirmed               = {},
        .pending                 = {},
        .permanent               = {},
        .confirmedReadySemaphore = xSemaphoreCreateBinary(),
        .pendingReadySemaphore   = xSemaphoreCreateBinary(),
        .permanentReadySemaphore = xSemaphoreCreateBinary(),
        .mtx_                    = xSemaphoreCreateMutex(),
    };

    pidMapMtx = xSemaphoreCreateMutex();
};

esp_err_t OBD2DataModel::addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name,
                                std::string unit, std::string desc, std::string formula, float minV, float maxV,
                                uint8_t priority, UpdateRate interval, uint32_t color, std::string icon)
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    if (PID_DEF.find(pid) != PID_DEF.end())
    {
        return ESP_ERR_INVALID_ARG;
    }

    PID_DEF.try_emplace(pid, id, mode, pid, len, name, unit, desc, formula, minV, maxV, priority, interval, color,
                        icon);

    pidData[pid] = {.id          = id,
                    .value       = 0.0f,
                    .lastUpdated = 0,
                    .data        = {0},
                    .isSupported = mode == MODE_READ_DATA_BY_IDENTIFIER || mode == MODE_DERIVED_DATA ? true : false,
                    .isValid     = false,
                    .updateInterval_ms = interval};

    return ESP_OK;
}

esp_err_t OBD2DataModel::removePID(uint16_t pid)
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }

    pidData.erase(pid);
    PID_DEF.erase(pid);

    pollQueue.removePID(pid);

    return ESP_OK;
}

void OBD2DataModel::startPolling()
{
    pollQueue.clear();
    TickType_t         now = xTaskGetTickCount();
    std::set<uint32_t> RequestByDataIdentifierIds;

    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return;
    }

    for (const auto& [pid, info] : PID_DEF)
    {
        if (!_isSup(pid))
            continue;

        uint32_t interval = info.updateInterval();
        if (interval == 0)
            continue;

        PollRequest req;
        req.pid         = pid;
        req.interval    = interval;
        req.nextWake    = now;
        req.priority    = info.priority();
        req.isRecurring = true;
        req.id          = info.id();
        req.mode        = info.mode();
        req.len         = info.len();

        pollQueue.push(req);

        if (info.mode() == MODE_READ_DATA_BY_IDENTIFIER)
        {
            RequestByDataIdentifierIds.insert(info.id());
        }
    }

    for (const auto& id : RequestByDataIdentifierIds)
    {
        PollRequest req;
        req.pid         = 1;
        req.interval    = 2000;  // Arbitrary interval for session maintenance
        req.nextWake    = now;
        req.priority    = 1;
        req.isRecurring = true;
        req.id          = id;
        req.mode        = MODE_DIAGNOSTIC_SESSION_CONTROL;
        req.len         = 2;
        pollQueue.push(req);
    }
}

esp_err_t OBD2DataModel::updateData(const CanDriver::CanFrame& frame)
{
    uint16_t mode = frame.data[1];
    uint16_t pid =
        mode == RESPONSE_READ_DATA_BY_IDENTIFIER ? (uint16_t)(frame.data[2] << 8) | frame.data[3] : frame.data[2];
    uint32_t  id = frame.header.id;
    esp_err_t ret;

    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    PIDData_t* pdat = nullptr;
    ret             = _getData(pid, pdat);
    if (ret != ESP_OK || pdat == nullptr)
    {
        return (ret == ESP_OK) ? ESP_ERR_NOT_FOUND : ret;
    }

    const PIDDefinition* pdef = nullptr;
    ret                       = _getDef(pid, pdef);

    if (ret != ESP_OK || pdef == nullptr)
    {
        return (ret == ESP_OK) ? ESP_ERR_NOT_FOUND : ret;
    }

    float val = 0.f;

    if (mode == RESPONSE_READ_DATA_BY_IDENTIFIER)
    {
        ret = pdef->evaluate(&frame.data[4], frame.length - 4, val);
    }
    else
    {
        ret = pdef->evaluate(&frame.data[3], frame.length - 3, val);
    }

    pdat->value = val;

    pdat->lastUpdated = pdTICKS_TO_MS(xTaskGetTickCount());
    if (mode == RESPONSE_CURRENT_DATA && pdat->id != OBD2_FUNCTIONAL_ID)
    {
        pdat->id = id - 8;
    }
    if (frame.length > PID_DATA_LENGTH)
    {
        ESP_LOGW(TAG, "Frame length %d exceeds PID data length %d, truncating", frame.length, PID_DATA_LENGTH);
    }
    memcpy(pdat->data, frame.data, PID_DATA_LENGTH < frame.length ? PID_DATA_LENGTH : frame.length);

    return ret;
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

esp_err_t OBD2DataModel::getData(uint16_t pid, PIDData_t*& pd) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = _getData(pid, pd);

    return ret;
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

esp_err_t OBD2DataModel::getDef(uint16_t pid, const PIDDefinition*& outDef) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = _getDef(pid, outDef);

    return ret;
}

// PID_DEF Getters

std::vector<uint16_t> OBD2DataModel::getPIDs() const
{
    std::vector<uint16_t> keys;

    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return keys;
    }

    keys.reserve(PID_DEF.size());

    for (const auto& pair : PID_DEF)
    {
        keys.push_back(pair.first);
    }

    return keys;
}

bool OBD2DataModel::_pidExists(uint16_t pid) const
{
    return PID_DEF.find(pid) != PID_DEF.end();
}

bool OBD2DataModel::pidExists(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return false;
    }

    return _pidExists(pid);
}

uint32_t OBD2DataModel::_getId_Def(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->id();
}

uint32_t OBD2DataModel::getId_Def(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->id();
}

uint8_t OBD2DataModel::_getMode(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->mode();
}

uint8_t OBD2DataModel::getMode(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->mode();
}

uint8_t OBD2DataModel::_getLen(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->len();
}

uint8_t OBD2DataModel::getLen(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->len();
}

std::string OBD2DataModel::_getName(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->name();
}

std::string OBD2DataModel::getName(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return "Mutex Timeout";
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->name();
}

std::string OBD2DataModel::_getUnit(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->unit();
}

std::string OBD2DataModel::getUnit(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return "Mutex Timeout";
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->unit();
}

std::string OBD2DataModel::_getDescription(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->description();
}

std::string OBD2DataModel::getDescription(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return "Mutex Timeout";
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->description();
}

float OBD2DataModel::_getMinValue(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return NAN;
    };

    return def->maxValue();
}

float OBD2DataModel::getMinValue(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return NAN;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return NAN;
    };

    return def->minValue();
}
float OBD2DataModel::_getMaxValue(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return NAN;
    };

    return def->maxValue();
}

float OBD2DataModel::getMaxValue(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return NAN;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return NAN;
    };

    return def->maxValue();
}

uint8_t OBD2DataModel::_getPriority(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->priority();
}

uint8_t OBD2DataModel::getPriority(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->priority();
}

uint16_t OBD2DataModel::_getUpdateInterval(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->updateInterval();
}

uint16_t OBD2DataModel::getUpdateInterval(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0;
    };

    return def->updateInterval();
}

uint32_t OBD2DataModel::_getColor(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0xFFFFFF;
    };

    return def->color();
}

uint32_t OBD2DataModel::getColor(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0xFFFFFF;
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return 0xFFFFFF;
    };

    return def->color();
}

std::string OBD2DataModel::_getIcon(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->icon();
}

std::string OBD2DataModel::getIcon(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return "Mutex Timeout";
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->icon();
}

std::string OBD2DataModel::_getFormula(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->formula();
}

std::string OBD2DataModel::getFormula(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return "Mutex Timeout";
    }

    const PIDDefinition* def = nullptr;
    esp_err_t            ret = _getDef(pid, def);
    if (ret != ESP_OK || def == nullptr)
    {
        return "Unknown PID";
    };

    return def->formula();
}

float OBD2DataModel::_getValue(uint16_t pid, uint32_t timeout_ms)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return NAN;
    };

    return pd->value;
}

float OBD2DataModel::getValueUnsafe(uint16_t pid, uint32_t timeout_ms)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return NAN;
    };

    return pd->value;
}

float OBD2DataModel::getValue(uint16_t pid, uint32_t timeout_ms)
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return NAN;
    }
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    return pd->value;
}

uint32_t OBD2DataModel::_getLastUpdated(uint16_t pid) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    return pd->lastUpdated;
}

uint32_t OBD2DataModel::getLastUpdated(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    return pd->lastUpdated;
}

uint32_t OBD2DataModel::_getId(uint16_t pid) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    return pd->id;
}

uint32_t OBD2DataModel::getId(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return 0;
    };

    return pd->id;
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
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return ESP_ERR_TIMEOUT;
    }
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    memcpy(outData, pd->data, PID_DATA_LENGTH);

    return ESP_OK;
}

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

uint8_t OBD2DataModel::getRawDataByte(uint16_t pid, uint8_t idx) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return 0;
    }

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

bool OBD2DataModel::_isValid(uint16_t pid) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return false;
    };

    return pd->isValid;
}

bool OBD2DataModel::isValid(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return false;
    }

    PIDData_t* pd = nullptr;
    if (_getData(pid, pd) != ESP_OK && pd == nullptr)
    {
        return false;
    };

    return pd->isValid;
}

bool OBD2DataModel::_isSup(uint16_t pid) const
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return false;
    };

    return pd->isSupported;
}

bool OBD2DataModel::isSup(uint16_t pid) const
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return false;
    }

    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return false;
    };

    return pd->isSupported;
}

esp_err_t OBD2DataModel::setValid(uint16_t pid, bool valid)
{
    MutexGuard guard(pidMapMtx, pdMS_TO_TICKS(100));

    if (!guard.isLocked())
    {
        return false;
    }

    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->isValid = valid;
    return ESP_OK;
}

esp_err_t OBD2DataModel::_setValid(uint16_t pid, bool valid)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->isValid = valid;
    return ESP_OK;
}

esp_err_t OBD2DataModel::_setUpdateInterval(uint16_t pid, UpdateRate interval_ms)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->updateInterval_ms = interval_ms;

    return ESP_OK;
}

esp_err_t OBD2DataModel::_setIsSupported(uint16_t pid, bool supported)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->isSupported = supported;
    return ESP_OK;
}

esp_err_t OBD2DataModel::_setLastUpdated(uint16_t pid, uint32_t lastUpdated)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->lastUpdated = lastUpdated;
    return ESP_OK;
}

esp_err_t OBD2DataModel::_setId(uint16_t pid, uint32_t id)
{
    PIDData_t* pd  = nullptr;
    esp_err_t  ret = _getData(pid, pd);
    if (ret != ESP_OK || pd == nullptr)
    {
        return ret;
    };

    pd->id = id;
    return ESP_OK;
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
            dtcData.confirmed.clear();
            break;
        case MODE_PENDING_DTCS:
        case RESPONSE_PENDING_DTCS:
            dtcData.pending.clear();
            dtcData.pending.clear();
            break;
        case MODE_PERMANENT_DTCS:
        case RESPONSE_PERMANENT_DTCS:
            dtcData.permanent.clear();
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

std::vector<std::string> OBD2DataModel::getDTC(uint8_t mode)
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
    subscribers_.push_back(cb);
}

void OBD2DataModel::runPidUpdateCallbacks(uint16_t pid)
{
    for (const auto& cb : subscribers_)
    {
        cb(pid);
    }
}
