#pragma once

#include "power_settings.hpp"
#include "cJSON.h"
#include "settings_validation.hpp"
#include <cstddef>
#include <cstring>

namespace power_settings
{
struct PatchResult
{
  size_t count;
  const char* error;
};

inline PatchResult patch(const cJSON* json, Config& config)
{
  if (!cJSON_IsObject(json)) { return {0, "system settings must be an object"}; }
  Config candidate = config;
  bool seen[3]{};
  size_t count = 0;
  for (const cJSON* item = json->child; item != nullptr; item = item->next)
  {
    if (item->string == nullptr) { continue; }
    int field = -1;
    if (strcmp(item->string, "stay_awake_voltage_mv") == 0) { field = 0; }
    else if (strcmp(item->string, "low_battery_voltage_mv") == 0) { field = 1; }
    else if (strcmp(item->string, "sleep_enabled") == 0) { field = 2; }
    if (field < 0) { continue; }
    if (seen[field]) { return {0, "duplicate power setting"}; }
    seen[field] = true;
    if (field == 2)
    {
      if (!cJSON_IsBool(item)) { return {0, "sleep_enabled must be a boolean"}; }
      candidate.sleep_enabled = cJSON_IsTrue(item);
    }
    else
    {
      int64_t value = 0;
      if (!cJSON_IsNumber(item) || !settings_validation::checkedInt(item->valuedouble, 9000, 16000, value))
      {
        return {0, "voltage thresholds must be integers in 9000..16000 mV"};
      }
      if (field == 0) { candidate.stay_awake_voltage_mv = static_cast<uint32_t>(value); }
      else { candidate.low_battery_voltage_mv = static_cast<uint32_t>(value); }
    }
    ++count;
  }
  if (const char* error = check(candidate)) { return {0, error}; }
  config = candidate;
  return {count, nullptr};
}

inline bool append(cJSON* object, const Config& c)
{
  return cJSON_AddNumberToObject(object, "stay_awake_voltage_mv", c.stay_awake_voltage_mv) != nullptr &&
         cJSON_AddNumberToObject(object, "low_battery_voltage_mv", c.low_battery_voltage_mv) != nullptr &&
         cJSON_AddBoolToObject(object, "sleep_enabled", c.sleep_enabled) != nullptr;
}
}
