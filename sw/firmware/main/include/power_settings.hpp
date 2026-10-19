#pragma once

#include <cstdint>

namespace power_settings
{
struct Config
{
  uint32_t stay_awake_voltage_mv = 13200;
  uint32_t low_battery_voltage_mv = 11400;
  bool sleep_enabled = true;
};

constexpr bool voltageValid(uint32_t mv) { return mv >= 9000 && mv <= 16000; }

inline const char* check(const Config& c)
{
  if (!voltageValid(c.stay_awake_voltage_mv) || !voltageValid(c.low_battery_voltage_mv))
  {
    return "voltage thresholds must be integers in 9000..16000 mV";
  }
  if (c.low_battery_voltage_mv >= c.stay_awake_voltage_mv)
  {
    return "low_battery_voltage_mv must be below stay_awake_voltage_mv";
  }
  return nullptr;
}

}
