#pragma once

#include "power_settings.hpp"
#include "nvs.h"

namespace power_settings
{
inline bool useDefault(esp_err_t error)
{
  return error == ESP_ERR_NVS_NOT_FOUND || error == ESP_ERR_NVS_TYPE_MISMATCH || error == ESP_ERR_NVS_INVALID_LENGTH;
}

// Getters return actual typed NVS results. New keys are optional for legacy devices;
// malformed values never trigger a write or erase from this read path.
template <class GetU32, class GetU8>
esp_err_t read(Config& c, GetU32 getU32, GetU8 getU8)
{
  Config candidate;
  uint32_t awake = 0, low = 0;
  uint8_t enabled = 0;
  const esp_err_t a = getU32("p_awake_mv", awake);
  if (a != ESP_OK && !useDefault(a)) { return a; }
  if (a == ESP_OK && voltageValid(awake)) { candidate.stay_awake_voltage_mv = awake; }
  const esp_err_t l = getU32("p_low_mv", low);
  if (l != ESP_OK && !useDefault(l)) { return l; }
  if (l == ESP_OK && voltageValid(low)) { candidate.low_battery_voltage_mv = low; }
  const esp_err_t e = getU8("p_sleep", enabled);
  if (e != ESP_OK && !useDefault(e)) { return e; }
  if (e == ESP_OK && enabled <= 1) { candidate.sleep_enabled = enabled != 0; }
  if (check(candidate) != nullptr)
  {
    const Config defaults;
    candidate.stay_awake_voltage_mv = defaults.stay_awake_voltage_mv;
    candidate.low_battery_voltage_mv = defaults.low_battery_voltage_mv;
  }
  c = candidate;
  return ESP_OK;
}

// The containing settings section commits only after paths and power fields succeed.
template <class SetU32, class SetU8>
esp_err_t write(const Config& c, SetU32 setU32, SetU8 setU8)
{
  if (check(c) != nullptr) { return ESP_ERR_INVALID_ARG; }
  esp_err_t error = setU32("p_awake_mv", c.stay_awake_voltage_mv);
  if (error != ESP_OK) { return error; }
  error = setU32("p_low_mv", c.low_battery_voltage_mv);
  if (error != ESP_OK) { return error; }
  return setU8("p_sleep", c.sleep_enabled ? 1 : 0);
}
}
