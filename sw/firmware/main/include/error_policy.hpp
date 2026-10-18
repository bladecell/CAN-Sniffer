#pragma once

// Supervisor ERROR-state policy (P2.9e, SUP-1). No ESP-IDF includes, so host tests include it directly.
// ERROR blinks the LED for kBlinkS, then sleeps for backoffS(n) with the PM lock released and restarts. n is the number of
// consecutive ERROR restarts, kept in RTC memory; it grows the sleep so a permanent fault is retried at most once per
// kMaxBackoffS instead of looping and draining the car battery.

#include <cstdint>

namespace error_policy
{
constexpr uint32_t kBlinkS       = 30;
constexpr uint32_t kBaseBackoffS = 60;
constexpr uint32_t kMaxBackoffS  = 3600;
constexpr uint32_t kMaxCount     = 16;
constexpr uint32_t kMagic        = 0xE220B007u;

constexpr uint32_t backoffS(uint32_t count)
{
  if (count >= kMaxCount)
  {
    return kMaxBackoffS;
  }
  const uint32_t s = kBaseBackoffS << count;  // count < 16: at most 60 << 15, no overflow
  return s > kMaxBackoffS ? kMaxBackoffS : s;
}

// RTC memory holds garbage after a power-on: trust the count only when the magic matches, and bound it.
constexpr uint32_t validCount(uint32_t magic, uint32_t count)
{
  if (magic != kMagic)
  {
    return 0;
  }
  return count > kMaxCount ? kMaxCount : count;
}
}  // namespace error_policy
