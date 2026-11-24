#include "pid_core.hpp"

#include <algorithm>
#include <new>

namespace pidcore
{
namespace
{
constexpr uint32_t kObdId  = 0x7DF;
constexpr float    kZero    = 0.0f;  // a real minimum of 0, not "unset"

PIDDefinitionData make(uint16_t pid, const char* name, const char* unit, const char* desc, const char* formula,
                       float minV, float maxV, uint8_t priority, uint16_t interval, uint32_t color, const char* icon)
{
    PIDDefinitionData d = {};
    d.id                = kObdId;
    d.mode              = MODE_CURRENT_DATA;
    d.pid               = pid;
    d.len               = 2;
    d.name              = name;
    d.unit              = unit;
    d.description       = desc;
    d.formula           = formula;
    d.minValue          = minV;
    d.maxValue          = maxV;
    d.priority          = priority;
    d.updateInterval_ms = interval;
    d.color             = color;
    d.icon              = icon;
    return d;
}
}  // namespace

const std::vector<PIDDefinitionData>& definitions()
{
    static const std::vector<PIDDefinitionData> core = {
        make(0x04, "Engine Load", "%", "Calculated engine load", "A*100/255", 0.0f, 100.0f, 2, 500, 0xF59F0B, "gauge"),
        make(0x05, "Coolant Temp", "C", "Engine coolant temperature", "A-40", -40.0f, 215.0f, 3, 2000, 0x3B82F6,
             "thermometer"),
        make(0x0C, "Engine RPM", "rpm", "Engine speed", "(A*256+B)/4", kZero, 8000.0f, 1, 250, 0xEF4444, "engine"),
        make(0x0D, "Vehicle Speed", "km/h", "Vehicle speed", "A", kZero, 255.0f, 1, 250, 0x22C55E, "dial"),
        make(0x0F, "Intake Air Temp", "C", "Intake air temperature", "A-40", -40.0f, 215.0f, 3, 2000, 0x06B6D4,
             "thermometer"),
        make(0x10, "MAF", "g/s", "Mass air flow rate", "(A*256+B)/100", kZero, 655.35f, 2, 500, 0xA855F7, "fan"),
    };
    return core;
}

bool isCore(uint16_t pid)
{
    const auto& core = definitions();
    return std::any_of(core.begin(), core.end(), [pid](const PIDDefinitionData& d) { return d.pid == pid; });
}

bool isIdentical(const PIDDefinitionData& d)
{
    for (const auto& c : definitions())
    {
        if (c.pid != d.pid)
        {
            continue;
        }
        return c.id == d.id && c.mode == d.mode && c.len == d.len && c.name == d.name && c.unit == d.unit &&
               c.description == d.description && c.formula == d.formula && c.minValue == d.minValue &&
               c.maxValue == d.maxValue && c.updateInterval_ms == d.updateInterval_ms && c.color == d.color &&
               c.priority == d.priority && c.icon == d.icon;
    }
    return false;
}

esp_err_t fill(std::vector<PIDDefinitionData>& defs, size_t maxItems)
{
    for (const auto& d : defs)
    {
        if (isCore(d.pid) && !isIdentical(d))
        {
            return kBuiltin;
        }
    }
    std::vector<PIDDefinitionData> missing;
    for (const auto& c : definitions())
    {
        const bool present = std::any_of(defs.begin(), defs.end(), [&c](const PIDDefinitionData& d) { return d.pid == c.pid; });
        if (!present)
        {
            missing.push_back(c);
        }
    }
    if (defs.size() + missing.size() > maxItems)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    defs.insert(defs.end(), missing.begin(), missing.end());
    return ESP_OK;
}

esp_err_t merged(const std::vector<PIDDefinitionData>& requested, std::vector<PIDDefinitionData>& out, size_t maxItems)
{
    std::vector<PIDDefinitionData> copy = requested;
    const esp_err_t                err  = fill(copy, maxItems);
    if (err == ESP_OK)
    {
        out.swap(copy);
    }
    return err;
}

void overrideChanged(std::vector<PIDDefinitionData>& defs, std::vector<uint16_t>& replaced)
{
    for (auto& d : defs)
    {
        if (!isCore(d.pid) || isIdentical(d))
        {
            continue;
        }
        for (const auto& c : definitions())
        {
            if (c.pid == d.pid)
            {
                d = c;
                replaced.push_back(d.pid);
                break;
            }
        }
    }
}

esp_err_t guardChange(uint16_t pid)
{
    return isCore(pid) ? kBuiltin : ESP_OK;
}

}  // namespace pidcore
