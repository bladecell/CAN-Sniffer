// obd2_dtb.cpp

#include "obd2_dtb.hpp"

#include "obd2_utils.hpp"

static const char* TAG = "OBD2DTB";

void OBD2DTB::initDef()
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
};

esp_err_t OBD2DTB::addPID(uint32_t id, uint8_t mode, uint16_t pid, uint8_t len, std::string name, std::string unit,
                          std::string desc, std::string formula, float minV, float maxV, uint8_t priority,
                          UpdateRate interval, uint32_t color, std::string icon)
{
    if (pidExists(pid))
    {
        return ESP_ERR_INVALID_ARG;  // PID already exists
    }

    PID_DEF.try_emplace(pid, id, mode, pid, len, name, unit, desc, formula, minV, maxV, priority, interval, color,
                        icon);

    pidData[pid] = {.id          = id,
                    .value       = 0.0f,
                    .lastUpdated = 0,
                    .data        = {0},
                    .isSupported = mode == MODE_READ_DATA_BY_IDENTIFIER || mode == MODE_DERIVED_DATA ? true : false,
                    .isValid     = false,
                    .updateInterval_ms = interval,
                    .mtx_              = xSemaphoreCreateMutex()};

    return ESP_OK;
}

esp_err_t OBD2DTB::getData(uint16_t pid, PIDData_t& pd) const
{
    auto it = pidData.find(pid);
    if (it == pidData.end())
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (xSemaphoreTake(it->second.mtx_, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        pd = it->second;

        xSemaphoreGive(it->second.mtx_);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t OBD2DTB::getDef(uint16_t pid, const PIDDefinition*& outDef) const
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }
    auto it = PID_DEF.find(pid);
    if (it != PID_DEF.end())
    {
        outDef = &it->second;
    }
    return it != PID_DEF.end() ? ESP_OK : ESP_ERR_NOT_FOUND;
}

void OBD2DTB::startPolling()
{
    pollQueue.clear();
    TickType_t         now = xTaskGetTickCount();
    std::set<uint32_t> RequestByDataIdentifierIds;

    for (const auto& [pid, info] : PID_DEF)
    {
        if (!isSup(pid))
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

esp_err_t OBD2DTB::updateData(const CanDriver::CanFrame& frame)
{
    uint16_t mode = frame.data[1];
    uint16_t pid =
        mode == RESPONSE_READ_DATA_BY_IDENTIFIER ? (uint16_t)(frame.data[2] << 8) | frame.data[3] : frame.data[2];
    uint32_t id = frame.header.id;

    esp_err_t ret;
    auto      it = pidData.find(pid);
    if (it != pidData.end())
    {
        if (xSemaphoreTake(it->second.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
            return ESP_ERR_TIMEOUT;

        float val = 0.f;
        if (mode == RESPONSE_READ_DATA_BY_IDENTIFIER)
        {
            ret = PID_DEF.at(pid).evaluate(&frame.data[4], frame.length - 4, val);
        }
        else
        {
            ret = PID_DEF.at(pid).evaluate(&frame.data[3], frame.length - 3, val);
        }
        it->second.value = val;

        it->second.lastUpdated = pdTICKS_TO_MS(xTaskGetTickCount());
        if (mode == RESPONSE_CURRENT_DATA && it->second.id != OBD2_FUNCTIONAL_ID)
        {
            it->second.id = id - 8;
        }
        if (frame.length > PID_DATA_LENGTH)
        {
            ESP_LOGW(TAG, "Frame length %d exceeds PID data length %d, truncating", frame.length, PID_DATA_LENGTH);
        }
        memcpy(it->second.data, frame.data, PID_DATA_LENGTH < frame.length ? PID_DATA_LENGTH : frame.length);

        xSemaphoreGive(it->second.mtx_);
    }
    else
    {
        ret = ESP_ERR_NOT_FOUND;
    }

    return ret;
}

bool OBD2DTB::isSup(uint16_t pid) const
{
    auto it        = pidData.find(pid);
    bool supported = false;
    if (it != pidData.end())
    {
        if (xSemaphoreTake(it->second.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
        {
            return false;
        }

        supported = it->second.isSupported;

        xSemaphoreGive(it->second.mtx_);
    }

    return supported;
}

// PID_DEF Getters

std::vector<uint16_t> OBD2DTB::getPIDs() const
{
    std::vector<uint16_t> keys;

    // Reserve memory upfront for performance (avoids multiple re-allocations)
    keys.reserve(PID_DEF.size());

    for (const auto& pair : PID_DEF)
    {
        keys.push_back(pair.first);
    }

    return keys;
}

bool OBD2DTB::pidExists(uint16_t pid) const
{
    return PID_DEF.find(pid) != PID_DEF.end();
}

uint32_t OBD2DTB::getId_Def(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->id() : 0;
}

uint8_t OBD2DTB::getMode(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->mode() : 0;
}

uint8_t OBD2DTB::getLen(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->len() : 0;
}

const char* OBD2DTB::getName(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->name().c_str() : "Unknown PID";
}

const char* OBD2DTB::getUnit(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->unit().c_str() : "Unknown PID";
}

const char* OBD2DTB::getDescription(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->description().c_str() : "Unknown PID";
}

float OBD2DTB::getMinValue(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->minValue() : NAN;
}

float OBD2DTB::getMaxValue(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->maxValue() : NAN;
}

uint8_t OBD2DTB::getPriority(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->priority() : 0;
}

uint16_t OBD2DTB::getUpdateInterval(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->updateInterval() : 0;
}

uint32_t OBD2DTB::getColor(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->color() : 0xFFFFFF;
}

const char* OBD2DTB::getIcon(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->icon().c_str() : "Unknown PID";
}

const char* OBD2DTB::getFormula(uint16_t pid) const
{
    const PIDDefinition* def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->formula().c_str() : "Unknown PID";
}

float OBD2DTB::getValue(uint16_t pid, uint32_t timeout_ms)
{
    if (!pidExists(pid))
    {
        return NAN;
    }

    if (!isValid(pid))
    {
        ESP_LOGI(TAG, "PID 0x%02X data is not valid", pid);
        return NAN;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        ESP_LOGI(TAG, "Timeout acquiring mutex to get PID 0x%02X value", pid);
        return NAN;
    }

    float value = pidData.at(pid).value;
    xSemaphoreGive(pidData.at(pid).mtx_);

    return value;
}

uint32_t OBD2DTB::getLastUpdated(uint16_t pid) const
{
    if (!pidExists(pid))
    {
        return 0;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return 0;
    }

    uint32_t lastUpdated = pidData.at(pid).lastUpdated;

    xSemaphoreGive(pidData.at(pid).mtx_);
    return lastUpdated;
}

uint32_t OBD2DTB::getId(uint16_t pid) const
{
    if (!pidExists(pid))
    {
        return 0;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return 0;
    }

    uint32_t id = pidData.at(pid).id;

    xSemaphoreGive(pidData.at(pid).mtx_);
    return id;
}

esp_err_t OBD2DTB::getRawData(uint16_t pid, uint8_t* outData) const
{
    if (!pidExists(pid))
        return ESP_ERR_NOT_FOUND;

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    memcpy(outData, pidData.at(pid).data, PID_DATA_LENGTH);

    xSemaphoreGive(pidData.at(pid).mtx_);
    return ESP_OK;
}

uint8_t OBD2DTB::getRawDataByte(uint16_t pid, uint8_t idx) const
{
    if (!pidExists(pid))
        return 0;

    if (idx >= PID_DATA_LENGTH)
    {
        return 0;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return 0;
    }

    uint8_t value = pidData.at(pid).data[idx];

    xSemaphoreGive(pidData.at(pid).mtx_);
    return value;
}

std::string OBD2DTB::getVIN() const
{
    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return "";  // Return empty string on timeout
    }

    std::string result;

    if (vinData.isValid)
    {
        result = std::string(vinData.vin);  // Copy while protected
    }

    xSemaphoreGive(vinData.mtx_);
    return result;  // Return copy (thread-safe)
}

bool OBD2DTB::isValid(uint16_t pid) const
{
    if (!pidExists(pid))
    {
        return false;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return false;
    }

    bool valid = pidData.at(pid).isValid;

    xSemaphoreGive(pidData.at(pid).mtx_);
    return valid;
}

esp_err_t OBD2DTB::setValid(uint16_t pid, bool valid)
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    pidData[pid].isValid = valid;
    xSemaphoreGive(pidData.at(pid).mtx_);
    return ESP_OK;
}

esp_err_t OBD2DTB::setUpdateInterval(uint16_t pid, UpdateRate interval_ms)
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    pidData[pid].updateInterval_ms = interval_ms;
    return ESP_OK;

    xSemaphoreGive(pidData.at(pid).mtx_);
}

esp_err_t OBD2DTB::setIsSupported(uint16_t pid, bool supported)
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    pidData[pid].isSupported = supported;
    xSemaphoreGive(pidData.at(pid).mtx_);
    return ESP_OK;
}

esp_err_t OBD2DTB::setLastUpdated(uint16_t pid, uint32_t lastUpdated)
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    pidData[pid].lastUpdated = lastUpdated;
    xSemaphoreGive(pidData.at(pid).mtx_);
    return ESP_OK;
}

esp_err_t OBD2DTB::setId(uint16_t pid, uint32_t id)
{
    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }
    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    pidData[pid].id = id;
    xSemaphoreGive(pidData.at(pid).mtx_);
    return ESP_OK;
}

esp_err_t OBD2DTB::setDTC(uint16_t rawDTC, uint8_t mode)
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

esp_err_t OBD2DTB::clearDTC(uint8_t mode)
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

std::string OBD2DTB::decodeDTC(uint16_t rawDTC)
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

std::vector<std::string> OBD2DTB::getDTC(uint8_t mode)
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

void OBD2DTB::subscribe(PidUpdateCallback cb)
{
    subscribers_.push_back(cb);
}

void OBD2DTB::runPidUpdateCallbacks(uint16_t pid)
{
    for (const auto& cb : subscribers_)
    {
        cb(pid);
    }
}

// TODO Rename to OBD2DATAMODEL