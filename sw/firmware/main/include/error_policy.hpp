#pragma once

// Supervisor ERROR-state policy (P2.9e, SUP-1). No ESP-IDF includes, so host tests include it directly.
// ERROR blinks the LED for kBlinkS, shuts the modules down, then deep-sleeps backoffS(n) and reboots. n is the number of
// recovery episodes, kept in RTC memory; it grows the sleep so a permanent fault is retried at most once per
// kMaxBackoffS instead of looping and draining the car battery.
// Completed startup (STARTING to NOT_CONNECTED) and successful orderly teardown before normal sleep clear it; ERROR cleanup
// never does (docs/decisions.md, 2026-10-20).

#include <cstdint>

namespace error_policy
{
constexpr uint32_t kBlinkS       = 30;
constexpr uint32_t kBaseBackoffS = 60;
constexpr uint32_t kMaxBackoffS  = 3600;
constexpr uint32_t kMaxCount     = 16;
constexpr uint32_t kMagic        = 0xE220B007u;
constexpr uint32_t kMaxNvsResets = 3;  // factory resets in a row before the supervisor goes to ERROR instead

constexpr uint32_t nextCount(uint32_t count)
{
  return count < kMaxCount ? count + 1 : kMaxCount;
}

// Factory reset + reboot on a corrupt NVS key partition shares the consecutive-failure count with the ERROR restarts, so
// a reset that does not cure the corruption cannot loop forever.
constexpr bool nvsResetAllowed(uint32_t count)
{
  return count < kMaxNvsResets;
}

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
