// obd2_dtb.cpp

#include "obd2_dtb.hpp"

static const char *TAG = "OBD2DTB";

void OBD2DTB::initDef()
{
    vinData = {
        .vin = {0},
        .lastUpdated = 0,
        .isValid = false,
        .vinReadySemaphore = xSemaphoreCreateBinary(),
        .mtx_ = xSemaphoreCreateMutex(),
    };

    dtcData = {
        .confirmed = {},
        .pending = {},
        .permanent = {},
        .confirmedReadySemaphore = xSemaphoreCreateBinary(),
        .pendingReadySemaphore = xSemaphoreCreateBinary(),
        .permanentReadySemaphore = xSemaphoreCreateBinary(),
        .mtx_ = xSemaphoreCreateMutex(),
    };
};

// const std::map<uint8_t, PIDDef_t> OBD2DTB::PID_DEF = {
//     {PID_ENGINE_LOAD,
//      PIDDef_t{
//          .mode = MODE_CURRENT_DATA,
//          .pid = PID_ENGINE_LOAD,
//          .name = "Engine Load",
//          .unit = PERCENTAGE,
//          .description = "Calculated engine load",
//          .formula = OBDFormulas::engineLoad,
//          .minValue = 0.0f,
//          .maxValue = 100.0f,
//          .priority = 2,
//          .updateInterval_ms = UPDATE_FAST,
//          .color = 0xf59e0b,
//          .icon = "gauge"}},
//     {PID_COOLANT_TEMP,
//      PIDDef_t{
//          .mode = MODE_CURRENT_DATA,
//          .pid = PID_COOLANT_TEMP,
//          .name = "Coolant Temp",
//          .unit = DEGREES_CELCIUS,
//          .description = "Engine coolant temperature",
//          .formula = OBDFormulas::coolantTemp,
//          .minValue = -40.0f,
//          .maxValue = 215.0f,
//          .priority = 3,
//          .updateInterval_ms = UPDATE_SLOW,
//          .color = 0xef4444,
//          .icon = "thermometer"}},
//     {PID_ENGINE_RPM,
//      PIDDef_t{
//          .mode = MODE_CURRENT_DATA,
//          .pid = PID_ENGINE_RPM,
//          .name = "Engine RPM",
//          .unit = RPM,
//          .description = "Engine speed",
//          .formula = OBDFormulas::engineRPM,
//          .minValue = 0.0f,
//          .maxValue = 16383.75f,
//          .priority = 1,
//          .updateInterval_ms = UPDATE_FAST,
//          .color = 0x3b82f6,
//          .icon = "droplet"}},
// };

void OBD2DTB::addPID(uint8_t mode, uint8_t pid, std::string name, std::string unit,
                     std::string desc, std::string formula, float minV, float maxV,
                     uint8_t priority, UpdateRate interval, uint32_t color, std::string icon)
{
    PID_DEF[pid] = std::make_unique<PIDDefinition>(
        mode, pid, name, unit, desc, formula, minV, maxV, priority, interval, color, icon);

    pidData[pid] = {
        .id = pid,
        .value = 0.0f,
        .lastUpdated = 0,
        .data = {0},
        .isSupported = true,
        .isValid = false,
        .updateInterval_ms = interval,
        .mtx_ = xSemaphoreCreateMutex()};
}

esp_err_t OBD2DTB::getData(uint8_t pid, PIDData_t &pd) const
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

esp_err_t OBD2DTB::getDef(uint8_t pid, const PIDDefinition *&outDef) const
{
    auto it = PID_DEF.find(pid);
    if (it == PID_DEF.end())
    {
        outDef = nullptr;
        return ESP_ERR_NOT_FOUND;
    }

    outDef = it->second.get();
    return ESP_OK;
}

void OBD2DTB::generatePollingGroups()
{
    vGroupFast.clear();
    vGroupMedium.clear();
    vGroupSlow.clear();
    vGroupStatic.clear();

    for (const auto &[pid, info] : PID_DEF)
    {
        if (!isSup(pid))
        {
            continue;
        }

        switch (info->updateInterval())
        {
        case UPDATE_FAST:
            vGroupFast.push_back(pid);
            break;
        case UPDATE_MEDIUM:
            vGroupMedium.push_back(pid);
            break;
        case UPDATE_SLOW:
            vGroupSlow.push_back(pid);
            break;
        case UPDATE_STATIC:
            vGroupStatic.push_back(pid);
            break;
        default:
            break;
        }
    }
}

esp_err_t OBD2DTB::updateData(const CanDriver::CanFrame &frame)
{
    uint8_t pid = frame.data[2];

    if (!pidExists(pid))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    float val = 0.f;
    esp_err_t ret = PID_DEF.at(pid)->evaluate(&frame.data[3], frame.length - 3, val);
    pidData.at(pid).value = val;

    pidData.at(pid).lastUpdated = xTaskGetTickCount();
    if (frame.length > PID_DATA_LENGTH)
    {
        ESP_LOGW(TAG, "Frame length %d exceeds PID data length %d, truncating", frame.length, PID_DATA_LENGTH);
    }
    memcpy(pidData.at(pid).data, frame.data, PID_DATA_LENGTH < frame.length ? PID_DATA_LENGTH : frame.length);

    xSemaphoreGive(pidData.at(pid).mtx_);
    return ret;
}

bool OBD2DTB::isSup(uint8_t pid) const
{
    if (!pidExists(pid))
    {
        return false;
    }

    if (xSemaphoreTake(pidData.at(pid).mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return false;
    }

    bool supported = pidData.at(pid).isSupported;

    xSemaphoreGive(pidData.at(pid).mtx_);
    return supported;
}

// PID_DEF Getters

std::vector<uint8_t> OBD2DTB::getPIDs() const
{
    std::vector<uint8_t> keys;

    // Reserve memory upfront for performance (avoids multiple re-allocations)
    keys.reserve(PID_DEF.size());

    for (const auto &pair : PID_DEF)
    {
        keys.push_back(pair.first);
    }

    return keys;
}

bool OBD2DTB::pidExists(uint8_t pid) const
{
    return PID_DEF.find(pid) != PID_DEF.end();
}

uint8_t OBD2DTB::getmode(uint8_t pid) const
{
    PIDDefinition *def = getDef(pid);
    return def ? def->mode() : 0;
}

const char *OBD2DTB::getName(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->name().c_str() : "Unknown PID";
}

const char *OBD2DTB::getUnit(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->unit().c_str() : "Unknown PID";
}

const char *OBD2DTB::getDescription(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->description().c_str() : "Unknown PID";
}

float OBD2DTB::getMinValue(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->minValue() : NAN;
}

float OBD2DTB::getMaxValue(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->maxValue() : NAN;
}

uint8_t OBD2DTB::getPriority(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->priority() : 0;
}

uint16_t OBD2DTB::getUpdateInterval(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->updateInterval() : 0;
}

uint32_t OBD2DTB::getColor(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->color() : 0xFFFFFF;
}

const char *OBD2DTB::getIcon(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->icon().c_str() : "Unknown PID";
}

const char *OBD2DTB::getFormula(uint8_t pid) const
{
    const PIDDefinition *def = nullptr;
    return (getDef(pid, def) == ESP_OK) ? def->formula().c_str() : "Unknown PID";
}

float OBD2DTB::getValue(uint8_t pid, uint32_t timeout_ms) const
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

uint32_t OBD2DTB::getLastUpdated(uint8_t pid) const
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

uint32_t OBD2DTB::getId(uint8_t pid) const
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

esp_err_t OBD2DTB::getRawData(uint8_t pid, uint8_t *outData) const
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

std::string OBD2DTB::getVIN() const
{
    if (xSemaphoreTake(vinData.mtx_, pdMS_TO_TICKS(10)) != pdTRUE)
    {
        return ""; // Return empty string on timeout
    }

    std::string result;

    if (vinData.isValid)
    {
        result = std::string(vinData.vin); // Copy while protected
    }

    xSemaphoreGive(vinData.mtx_);
    return result; // Return copy (thread-safe)
}

bool OBD2DTB::isValid(uint8_t pid) const
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

esp_err_t OBD2DTB::setValid(uint8_t pid, bool valid)
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

esp_err_t OBD2DTB::setUpdateInterval(uint8_t pid, UpdateRate interval_ms)
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

esp_err_t OBD2DTB::setIsSupported(uint8_t pid, bool supported)
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

esp_err_t OBD2DTB::setLastUpdated(uint8_t pid, uint32_t lastUpdated)
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

esp_err_t OBD2DTB::setId(uint8_t pid, uint32_t id)
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

    std::string dtcCode = decodeDTC(rawDTC);
    std::vector<std::string> *target = nullptr;

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
    char dtc[6];
    uint8_t type_bits = (rawDTC >> 14) & 0x03;

    uint8_t first_digit = (rawDTC >> 12) & 0x03;

    uint16_t last_digits = rawDTC & 0x0FFF;

    char prefix;
    switch (type_bits)
    {
    case 0:
        prefix = 'P';
        break; // Powertrain
    case 1:
        prefix = 'C';
        break; // Chassis
    case 2:
        prefix = 'B';
        break; // Body
    case 3:
        prefix = 'U';
        break; // Network
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

void OBD2DTB::subscribe(DataUpdateCallback cb)
{
    subscribers_.push_back(cb);
}