#pragma once

// OBD voltage from ADC samples (P2.9h, BAT-1). No ESP-IDF includes, so host tests can drive it with a fake reader.
// A failed or out-of-range sample makes the result NaN ("unknown"): the supervisor must never act on a stale or
// uninitialised count (it decides between staying awake and sleeping).

#include <cmath>
#include <limits>

namespace battery_calc
{
constexpr int   kAdcMax = 4095;
constexpr float kVrefMv = 3300.0f;

// read(int& raw) -> true when `raw` holds a valid conversion.
template <typename Read>
float averageVolts(Read&& read, int samples, float dividerRatio)
{
  constexpr float unknown = std::numeric_limits<float>::quiet_NaN();
  if (samples <= 0)
  {
    return unknown;
  }
  long sum = 0;
  for (int i = 0; i < samples; i++)
  {
    int raw = 0;
    if (!read(raw) || raw < 0 || raw > kAdcMax)
    {
      return unknown;
    }
    sum += raw;
  }
  const float mv = (static_cast<float>(sum) / static_cast<float>(samples) / static_cast<float>(kAdcMax)) * kVrefMv;
  const float v  = (mv / 1000.0f) * dividerRatio;
  return std::round(v * 10.0f) / 10.0f;
}
}  // namespace battery_calc
