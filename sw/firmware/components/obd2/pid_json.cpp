#include "pid_json.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_set>

#include "can_driver.hpp"
#include "obd2_common.hpp"
#include "obd2_frame.hpp"
#include "pid_priority_queue.hpp"

namespace pidjson
{
namespace
{
constexpr size_t PID_NAME_MAX    = 64;
constexpr size_t PID_UNIT_MAX    = 32;
constexpr size_t PID_DESC_MAX    = 256;
constexpr size_t PID_FORMULA_MAX = 1024;
constexpr size_t PID_ICON_MAX    = 64;

template <typename T>
bool jsonInteger(const cJSON* object, const char* key, T minValue, T maxValue, T& output)
{
    uint32_t parsed = 0;
    if (!integerFromJson(cJSON_GetObjectItemCaseSensitive(object, key), static_cast<uint32_t>(minValue),
                         static_cast<uint32_t>(maxValue), parsed))
    {
        return false;
    }
    output = static_cast<T>(parsed);
    return true;
}

bool jsonFloat(const cJSON* object, const char* key, float& output)
{
    return floatFromJson(cJSON_GetObjectItemCaseSensitive(object, key), output);
}

// Absent or null means "unset" and is stored as NaN; anything else must be a finite float.
bool jsonOptionalFloat(const cJSON* object, const char* key, float& output)
{
    const cJSON* value = cJSON_GetObjectItemCaseSensitive(object, key);
    if (value == nullptr || cJSON_IsNull(value))
    {
        output = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    return jsonFloat(object, key, output);
}

bool jsonString(const cJSON* object, const char* key, size_t maxLength, std::string& output)
{
    const cJSON* value = cJSON_GetObjectItemCaseSensitive(object, key);
    if (!cJSON_IsString(value) || value->valuestring == nullptr || strlen(value->valuestring) > maxLength)
    {
        return false;
    }
    output = value->valuestring;
    return true;
}

ParseResult fail(int status, const char* reason)
{
    return {false, status, reason};
}
}  // namespace

bool integerFromJson(const cJSON* value, uint32_t minValue, uint32_t maxValue, uint32_t& output)
{
    if (!cJSON_IsNumber(value))
    {
        return false;
    }
    const double d = value->valuedouble;
    if (!std::isfinite(d) || std::floor(d) != d || d < static_cast<double>(minValue) || d > static_cast<double>(maxValue))
    {
        return false;
    }
    output = static_cast<uint32_t>(d);  // integral and inside [minValue, maxValue]: no wrap, no UB
    return true;
}

bool floatFromJson(const cJSON* value, float& output)
{
    if (!cJSON_IsNumber(value) || !std::isfinite(value->valuedouble) ||
        value->valuedouble < -static_cast<double>(std::numeric_limits<float>::max()) ||
        value->valuedouble > static_cast<double>(std::numeric_limits<float>::max()))
    {
        return false;
    }
    output = static_cast<float>(value->valuedouble);
    return true;
}

bool lengthFromJson(const cJSON* object, uint8_t& output, bool required)
{
    const cJSON* length = cJSON_GetObjectItemCaseSensitive(object, "length");
    const cJSON* len    = cJSON_GetObjectItemCaseSensitive(object, "len");

    if (length == nullptr && len == nullptr)
    {
        return !required;
    }

    uint8_t parsedLength = 0;
    uint8_t parsedLen    = 0;
    if (length != nullptr && !jsonInteger(object, "length", uint8_t(0), uint8_t(PID_DATA_LENGTH), parsedLength))
    {
        return false;
    }
    if (len != nullptr && !jsonInteger(object, "len", uint8_t(0), uint8_t(PID_DATA_LENGTH), parsedLen))
    {
        return false;
    }
    if (length != nullptr && len != nullptr && parsedLength != parsedLen)
    {
        return false;
    }

    output = length != nullptr ? parsedLength : parsedLen;
    return true;
}

bool parseAddItem(const cJSON* item, PIDDefinitionData& out)
{
    if (!cJSON_IsObject(item))
    {
        return false;
    }
    const auto get      = [item](const char* key) { return cJSON_GetObjectItem(item, key); };
    const auto present  = [](const cJSON* v) { return v != nullptr && !cJSON_IsNull(v); };
    const auto text     = [&](const char* key, size_t maxLength, bool required, std::string& dst)
    {
        const cJSON* v = get(key);
        if (!required && (v == nullptr || cJSON_IsNull(v)))
        {
            return true;  // absent or null keeps the default ""
        }
        if (!cJSON_IsString(v) || v->valuestring == nullptr)
        {
            return false;
        }
        if (strlen(v->valuestring) > maxLength)
        {
            return false;
        }
        dst = v->valuestring;
        return true;
    };

    PIDDefinitionData d = {};
    uint32_t          id = 0, mode = 0, pid = 0, interval = 0, priority = 0, color = 0x4EB31B;
    d.minValue = d.maxValue = std::numeric_limits<float>::quiet_NaN();

    bool ok = integerFromJson(get("pid"), 0, std::numeric_limits<uint16_t>::max(), pid) &&
              integerFromJson(get("id"), 0, 0x7FF, id) &&
              integerFromJson(get("mode"), 0, std::numeric_limits<uint8_t>::max(), mode) &&
              integerFromJson(get("interval"), 0, std::numeric_limits<uint16_t>::max(), interval) &&
              text("name", PID_NAME_MAX, true, d.name) && text("formula", PID_FORMULA_MAX, true, d.formula) &&
              !d.formula.empty() && text("unit", PID_UNIT_MAX, false, d.unit) &&
              text("desc", PID_DESC_MAX, false, d.description) && text("icon", PID_ICON_MAX, false, d.icon);
    if (ok && present(get("priority")))
    {
        ok = integerFromJson(get("priority"), 0, std::numeric_limits<uint8_t>::max(), priority);
    }
    if (ok && present(get("color")))
    {
        ok = integerFromJson(get("color"), 0, 0xFFFFFF, color);
    }
    if (ok && present(get("minV")))
    {
        ok = floatFromJson(get("minV"), d.minValue);
    }
    if (ok && present(get("maxV")))
    {
        ok = floatFromJson(get("maxV"), d.maxValue);
    }
    if (ok && std::isfinite(d.minValue) && std::isfinite(d.maxValue) && d.minValue > d.maxValue)
    {
        ok = false;
    }
    uint8_t length = mode == MODE_READ_DATA_BY_IDENTIFIER ? 3 : (mode == MODE_DERIVED_DATA ? 0 : 2);
    if (ok)
    {
        ok = lengthFromJson(item, length, false) &&
             obd2frame::isValidPidSpec(static_cast<uint8_t>(mode), static_cast<uint16_t>(pid), length);
    }
    if (!ok)
    {
        return false;
    }
    d.id                = id;
    d.mode              = static_cast<uint8_t>(mode);
    d.pid               = static_cast<uint16_t>(pid);
    d.len               = length;
    d.priority          = static_cast<uint8_t>(priority);
    d.updateInterval_ms = static_cast<uint16_t>(interval);
    d.color             = color;
    out                 = std::move(d);
    return true;
}

ParseResult parse(const cJSON* array, std::vector<PIDDefinitionData>& out)
{
    out.clear();
    if (array == nullptr || !cJSON_IsArray(array))
    {
        return fail(400, "Payload must be a JSON array");
    }

    const int itemCount = cJSON_GetArraySize(array);
    if (itemCount < 0 || itemCount > NUMBER_OF_ITEMS)
    {
        return fail(422, "PID definition count exceeds capacity");
    }

    std::vector<PIDDefinitionData> definitions;
    std::unordered_set<uint16_t>   pids;
    definitions.reserve(static_cast<size_t>(itemCount));
    pids.reserve(static_cast<size_t>(itemCount));

    const cJSON* item = nullptr;
    cJSON_ArrayForEach(item, array)
    {
        if (!cJSON_IsObject(item))
        {
            return fail(400, "Each PID definition must be a JSON object");
        }

        PIDDefinitionData definition = {};
        uint32_t          id = 0, color = 0;
        uint8_t           mode = 0, length = 0, priority = 0;
        uint16_t          pid = 0, interval = 0;
        if (!jsonInteger(item, "id", uint32_t(0), uint32_t(0x7FF), id) ||
            !jsonInteger(item, "mode", uint8_t(0), std::numeric_limits<uint8_t>::max(), mode) ||
            !jsonInteger(item, "pid", uint16_t(0), std::numeric_limits<uint16_t>::max(), pid) ||
            !lengthFromJson(item, length, true) ||
            !jsonInteger(item, "priority", uint8_t(0), std::numeric_limits<uint8_t>::max(), priority) ||
            !jsonInteger(item, "interval", uint16_t(0), std::numeric_limits<uint16_t>::max(), interval) ||
            !jsonInteger(item, "color", uint32_t(0), uint32_t(0xFFFFFF), color) ||
            !jsonString(item, "name", PID_NAME_MAX, definition.name) ||
            !jsonString(item, "unit", PID_UNIT_MAX, definition.unit) ||
            !jsonString(item, "desc", PID_DESC_MAX, definition.description) ||
            !jsonString(item, "formula", PID_FORMULA_MAX, definition.formula) ||
            !jsonString(item, "icon", PID_ICON_MAX, definition.icon))
        {
            return fail(422, "Missing or invalid PID definition field");
        }

        const bool validModeAndPayload = obd2frame::isValidPidSpec(mode, pid, length);
        if ((interval != 0 && interval < MIN_TRANSMIT_PERIOD_MS) || !validModeAndPayload || definition.formula.empty())
        {
            return fail(422, "Invalid PID interval, mode, PID, or length");
        }

        if (!jsonOptionalFloat(item, "minV", definition.minValue) ||
            !jsonOptionalFloat(item, "maxV", definition.maxValue) ||
            (std::isfinite(definition.minValue) && std::isfinite(definition.maxValue) &&
             definition.minValue > definition.maxValue))
        {
            return fail(422, "Invalid PID value range");
        }

        if (!pids.insert(pid).second)
        {
            return fail(422, "Duplicate PID definition");
        }

        definition.id                = id;
        definition.mode              = mode;
        definition.pid               = pid;
        definition.len               = length;
        definition.priority          = priority;
        definition.updateInterval_ms = interval;
        definition.color             = color;
        definitions.push_back(std::move(definition));
    }

    out = std::move(definitions);
    return {true, 204, nullptr};
}

cJSON* serialize(const std::vector<PIDDefinitionData>& definitions)
{
    cJSON* rootArray = cJSON_CreateArray();
    if (rootArray == nullptr)
    {
        return nullptr;
    }

    for (const auto& def : definitions)
    {
        cJSON* item = cJSON_CreateObject();
        if (item == nullptr)
        {
            cJSON_Delete(rootArray);
            return nullptr;
        }
        cJSON_AddItemToArray(rootArray, item);
        if (cJSON_AddNumberToObject(item, "id", def.id) == nullptr ||
            cJSON_AddNumberToObject(item, "mode", def.mode) == nullptr ||
            cJSON_AddNumberToObject(item, "pid", def.pid) == nullptr ||
            cJSON_AddNumberToObject(item, "length", def.len) == nullptr ||
            cJSON_AddStringToObject(item, "name", def.name.c_str()) == nullptr ||
            cJSON_AddStringToObject(item, "unit", def.unit.c_str()) == nullptr ||
            cJSON_AddStringToObject(item, "desc", def.description.c_str()) == nullptr ||
            cJSON_AddStringToObject(item, "formula", def.formula.c_str()) == nullptr ||
            cJSON_AddNumberToObject(item, "minV", def.minValue) == nullptr ||
            cJSON_AddNumberToObject(item, "maxV", def.maxValue) == nullptr ||
            cJSON_AddNumberToObject(item, "priority", def.priority) == nullptr ||
            cJSON_AddNumberToObject(item, "interval", def.updateInterval_ms) == nullptr ||
            cJSON_AddNumberToObject(item, "color", def.color) == nullptr ||
            cJSON_AddStringToObject(item, "icon", def.icon.c_str()) == nullptr)
        {
            cJSON_Delete(rootArray);
            return nullptr;
        }
    }
    return rootArray;
}

}  // namespace pidjson
